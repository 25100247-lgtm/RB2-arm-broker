import math


JOINT_NAMES=[
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6"
]


LIMITES=[
(-3.14,3.14),
(-1.57,1.57),
(-1.57,1.57),
(-3.14,3.14),
(-1.57,1.57),
(-3.14,3.14)
]



def dentro_de_limites(q):

    if len(q)!=6:
        return False,"faltan articulaciones"


    for i,v in enumerate(q):

        mi,ma=LIMITES[i]

        if v<mi or v>ma:

            return False,f"joint {i+1} fuera de limite"


    return True,"ok"



def fk(q):

    q1,q2,q3,q4,q5,q6=q


    L1=150
    L2=250
    L3=200


    r=L2*math.cos(q2)+L3*math.cos(q2+q3)


    x=r*math.cos(q1)

    y=r*math.sin(q1)

    z=L1+L2*math.sin(q2)+L3*math.sin(q2+q3)


    return x,y,z



def dentro_del_workspace(q):

    x,y,z=fk(q)


    d=math.sqrt(x*x+y*y+z*z)


    if d>700:
        return False,"fuera del workspace"


    return True,"ok"



def paso_articular(a,b):

    return max(
        abs(x-y)
        for x,y in zip(a,b)
    )
