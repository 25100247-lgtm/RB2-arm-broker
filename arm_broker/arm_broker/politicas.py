import time
import threading


class Pedido:


    def __init__(self,goal_handle,client_id,priority,joints):

        self.goal_handle=goal_handle

        self.client_id=client_id

        self.priority=priority

        self.joint_positions=list(joints)

        self.goal_id=bytes(
            goal_handle.goal_id.uuid
        ).hex()[:12]


        self.t_inicio=time.time()

        self.fin=threading.Event()



    @property
    def espera_s(self):

        return time.time()-self.t_inicio





class FIFO:


    nombre="fifo"


    def siguiente(self,cola):

        return 0



    def atendido(self,pedido):

        pass





class Prioridad:


    nombre="prioridad"


    def __init__(self,tau=8):

        self.tau=tau



    def siguiente(self,cola):

        mejor=0

        valor=-1


        for i,p in enumerate(cola):

            score=p.priority+p.espera_s/self.tau


            if score>valor:

                valor=score

                mejor=i


        return mejor



    def atendido(self,pedido):

        pass





POLITICAS={

"fifo":FIFO,

"prioridad":Prioridad

}
