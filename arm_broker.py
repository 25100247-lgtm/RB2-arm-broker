import csv
import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from arm_broker_interfaces.action import MoveArm
from arm_broker_interfaces.msg import QueueState
from arm_broker import fk
from arm_broker.politicas import POLITICAS, Pedido


class ArmBroker(Node):
    def __init__(self):
        super().__init__('arm_broker')
        self.declare_parameter('politica', 'fifo')
        self.declare_parameter('cola_max', 20)
        self.declare_parameter('tau', 8.0)
        self.declare_parameter('paso_max', 1.2)
        self.declare_parameter('archivo_rechazos', 'rechazos.csv')
        self.declare_parameter('archivo_atendidos', 'atendidos.csv')

        self.nombre_politica = self.get_parameter('politica').value
        clase = POLITICAS[self.nombre_politica]
        if self.nombre_politica == 'prioridad':
            self.politica = clase(self.get_parameter('tau').value)
        else:
            self.politica = clase()

        self.cola_max = self.get_parameter('cola_max').value
        self.paso_max = self.get_parameter('paso_max').value
        self.archivo_rechazos = self.get_parameter('archivo_rechazos').value
        self.archivo_atendidos = self.get_parameter('archivo_atendidos').value

        self.lock = threading.Lock()
        self.pendientes = []
        self.por_goal_id = {}
        self.ejecutando = None
        self.q_actual = [0.0] * 6      # TODO: leer la pose real del robot
        self.n_aceptados = 0
        self.n_rechazados = 0
        self.n_completados = 0

        with open(self.archivo_rechazos, 'w', newline='') as f:
            csv.writer(f).writerow(['t', 'client_id', 'categoria', 'motivo'])
        with open(self.archivo_atendidos, 'w', newline='') as f:
            csv.writer(f).writerow(
                ['t_inicio', 'client_id', 'priority', 'espera_s', 'goal_id'])

        self.grupo_entrada = ReentrantCallbackGroup()
        self.grupo_worker = MutuallyExclusiveCallbackGroup()

        self.pub_joint = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_cola = self.create_publisher(QueueState, '/arm/queue_state', 10)

        self.servidor = ActionServer(
            self,
            MoveArm,
            'move_arm',
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=self.grupo_entrada,
        )

        self.create_timer(0.2, self.publicar_estado_cola,
                          callback_group=self.grupo_worker)

        self.worker = threading.Thread(target=self._worker, daemon=True)
        self.worker.start()
        self.get_logger().info(
            f'ArmBroker iniciado | politica={self.nombre_politica} '
            f'| cola_max={self.cola_max}')

    # ---------------- admision ----------------
    def goal_callback(self, goal_request):
        q = list(goal_request.joint_positions)
        cliente = goal_request.client_id

        ok, motivo = fk.dentro_de_limites(q)
        if not ok:
            return self.rechazar(cliente, 'limite_articular', motivo)

        ok, motivo = fk.dentro_del_workspace(q)
        if not ok:
            return self.rechazar(cliente, 'workspace', motivo)

        with self.lock:
            llena = len(self.pendientes) >= self.cola_max
            if self.pendientes:
                ref = self.pendientes[-1].joint_positions
            elif self.ejecutando is not None:
                ref = self.ejecutando.joint_positions
            else:
                ref = self.q_actual
            ref = list(ref)

        if llena:
            return self.rechazar(
                cliente, 'cola_llena', f'cola llena ({self.cola_max})')

        paso = fk.paso_articular(ref, q)
        if paso > self.paso_max:
            return self.rechazar(
                cliente, 'paso_excesivo',
                f'paso articular {paso:.2f} rad > {self.paso_max:.2f} rad')

        return GoalResponse.ACCEPT

    def rechazar(self, cliente, categoria, motivo):
        with self.lock:
            self.n_rechazados += 1
            with open(self.archivo_rechazos, 'a', newline='') as f:
                csv.writer(f).writerow(
                    [f'{time.time():.3f}', cliente, categoria, motivo])
        self.get_logger().warn(f'RECHAZADO {cliente} [{categoria}]: {motivo}')
        return GoalResponse.REJECT

    # ---------------- encolado ----------------
    def handle_accepted_callback(self, goal_handle):
        pedido = Pedido(
            goal_handle,
            goal_handle.request.client_id,
            goal_handle.request.priority,
            goal_handle.request.joint_positions,
        )
        with self.lock:
            self.pendientes.append(pedido)
            self.por_goal_id[pedido.goal_id] = pedido
            self.n_aceptados += 1

    # ---------------- worker unico ----------------
    def _worker(self):
        while rclpy.ok():
            pedido = None
            with self.lock:
                if self.pendientes and self.ejecutando is None:
                    i = self.politica.siguiente(self.pendientes)
                    pedido = self.pendientes.pop(i)
                    self.ejecutando = pedido
            if pedido:
                try:
                    self._registrar_atendido(pedido)
                    pedido.goal_handle.execute()
                    pedido.fin.wait(timeout=60.0)
                finally:
                    with self.lock:
                        self.ejecutando = None
            time.sleep(0.02)

    def _registrar_atendido(self, pedido):
        with open(self.archivo_atendidos, 'a', newline='') as f:
            csv.writer(f).writerow([
                f'{time.time():.3f}', pedido.client_id, pedido.priority,
                f'{pedido.espera_s:.3f}', pedido.goal_id])

    # ---------------- ejecucion ----------------
    def execute_callback(self, goal_handle):
        resultado = MoveArm.Result()
        gid = bytes(goal_handle.goal_id.uuid).hex()[:12]
        with self.lock:
            pedido = self.por_goal_id.get(gid)
        if pedido is None:
            goal_handle.abort()
            resultado.success = False
            resultado.message = 'goal desconocido'
            return resultado
        try:
            q0 = list(self.q_actual)
            destino = pedido.joint_positions
            for i in range(1, 11):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    resultado.success = False
                    resultado.message = 'Cancelado'
                    return resultado
                q = [a + (b - a) * i / 10 for a, b in zip(q0, destino)]
                self.mover(q)
                time.sleep(0.3)
            goal_handle.succeed()
            with self.lock:
                self.n_completados += 1
            resultado.success = True
            resultado.message = 'Movimiento completado'
            return resultado
        except Exception as e:
            self.get_logger().error(f'Error ejecutando goal {gid}: {e}')
            goal_handle.abort()
            resultado.success = False
            resultado.message = f'error: {e}'
            return resultado
        finally:
            pedido.fin.set()
            with self.lock:
                self.por_goal_id.pop(gid, None)

    def mover(self, q):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = fk.JOINT_NAMES
        msg.position = [float(v) for v in q]
        self.pub_joint.publish(msg)
        self.q_actual = list(q)

    # ---------------- estado de la cola (5 Hz) ----------------
    def publicar_estado_cola(self):
        msg = QueueState()
        msg.stamp = self.get_clock().now().to_msg()
        with self.lock:
            msg.policy = self.nombre_politica
            msg.queue_length = len(self.pendientes)
            msg.executing_client = (
                self.ejecutando.client_id if self.ejecutando else '')
            msg.pending_clients = [x.client_id for x in self.pendientes]
            msg.pending_priorities = [int(x.priority) for x in self.pendientes]
            msg.pending_wait_s = [float(x.espera_s) for x in self.pendientes]
            msg.accepted = self.n_aceptados
            msg.rejected = self.n_rechazados
            msg.completed = self.n_completados
        self.pub_cola.publish(msg)

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT


def main():
    rclpy.init()
    nodo = ArmBroker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(nodo)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
