import roboticstoolbox as rtb
from spatialmath import *
from spatialmath.base import *
import numpy as np
from sympy import print_ccode
import matplotlib.pyplot as plt
import time
from smooth import smooth

def deg2rad(value):
    return value*pi/180

class Generator:

    def __init__(self) -> None:
        self.totalTrayectory = []

    def generateTrayectory(self,oldqt,inicial, final,name,mask=[1,1,1,0,0,1]):
        steps = 20
        MTHtraj = rtb.ctraj(inicial,final, steps)
        print(oldqt)
        i=0
        points = np.zeros((steps,4))
        for traj in MTHtraj:
            #print(i)
            #print(traj)
            qinv_lm=robot.ikine_LM(traj,oldqt,mask)
            points[i,:] = (qinv_lm.q)
            self.totalTrayectory.append(qinv_lm.q)
            print(qinv_lm)
            #robot.plot(qinv_lm.q)
            #print(qinv_lm.q*180/np.pi)
            oldqt=qinv_lm.q
            i=i+1
        points = np.around(points, 3)
        np.savetxt(f"scripts/trajectories/{name}.csv", points, delimiter=",")
        return points

if __name__ == "__main__":
    l = np.array([4.075, 10.6, 10.6, 10.97])
    limit = 5*np.pi/6
    #qlims = np.array([[-np.pi, np.pi],[-np.pi, np.pi],[-np.pi, np.pi],[-np.pi, np.pi]])
    qlims = np.array([[-limit, limit],[-limit, limit],[-limit, limit],[-limit, limit]])
    robot = rtb.DHRobot(
        [rtb.RevoluteDH(alpha=np.pi/2, d=l[0], qlim=qlims[0,:]),
        rtb.RevoluteDH(a=l[1], offset=np.pi/2, qlim=qlims[0,:]),
        rtb.RevoluteDH(a=l[2], qlim=qlims[0,:]),
        rtb.RevoluteDH(a=l[3], qlim=qlims[0,:])],       
        name="Px_DH_std")
    robot.tool = np.array([[0,0,1,0], [-1,0,0,0],[0,-1,0,0], [0,0,0,1]])
    print(robot)

    qt = np.deg2rad(np.array([0, 0, 0, 0]))    
    print('qt = ', qt*180/np.pi)
    Tt = robot.fkine(qt)
   

    np.set_printoptions(suppress=True)
    T = Tt.A  
    #T = np.array([[1,0,0,5], [0,-1,0,6],[0,0,-1,25], [0,0,0,1]])
    #print(T.T)
    Tw = T-(l[3]*T[0:4,2]).reshape(4,1)
    #print(Tw)

    #  Solucion q1
    q1 = np.arctan2(Tw[1,3],Tw[0,3])
    #print('q1= '+ str(q1)+', '+str(np.rad2deg(q1))) 

    # Solucion 2R
    h = Tw[2,3] - l[0]
    r = np.sqrt(Tw[0,3]**2 + Tw[1,3]**2)

    # Codo arriba
    the3 = np.arccos((r**2+h**2-l[1]**2-l[2]**2)/(2*l[1]*l[2]))
    the2 = np.arctan2(h,r) + np.arctan2(l[2]*np.sin(the3),l[1]+l[2]*np.cos(the3))
    q2 = -(np.pi/2-the2)
    q3 = -the3

    # Solucion q4
    M0T3 = robot.A(2,[q1,q2,q3])
    M3TT = np.matmul(np.linalg.inv(M0T3.A),(T))
    M3T4 = np.matmul(M3TT,np.linalg.inv(robot.tool.A))
    q4= np.arctan2(M3T4[1,0],M3T4[0,0])
    

    qinv = np.empty((1,4))
    qinv[:] =np.NaN
    #q1=q1+np.pi;
    #q2=-q2;
    #q3=-q3;
    #q4=-q4;
    qinv[0,:] = np.array([q1, q2, q3, q4])    
    
    #robot.plot(qt,block=False)
    #trplot( transl(0,0,0), color='rgb', width=1, frame='0', length=5)
    #print(robot.fkine(qt))
    
    #print('qinv = ',qinv*180/np.pi)
    #robot.plot(qinv_lm,block=False)
    #trplot( transl(0,0,0), color='rgb', width=1, frame='0', length=5)    
    #robot.teach(qinv)       
    
    #qinv_lm=robot.ikine_LM(Tt)
    #print(qinv_lm)
    #print('qinv_lm = ', qinv_lm.q*180/np.pi)
    #print(robot.fkine(qinv_lm.q))
    #robot.teach(qinv_lm.q)
    home = Tt
    primera = SE3(np.array([[0,0,1,25.46], [0,1,0,0],[-1,0,0,18.53], [0,0,0,1]]))
    segunda = SE3(np.array([[1,0,0,11], [0,1,0,0],[0,0,-1,5], [0,0,0,1]]))
    tercera = SE3(np.array([[1,0,0,11], [0,1,0,15.5],[0,0,-1,-6.6], [0,0,0,1]]))
    cuarta = SE3(np.array([[1,0,0,11], [0,1,0,15.5],[0,0,-1,5], [0,0,0,1]]))
    quinta = SE3(np.array([[1,0,0,14], [0,1,0,0],[0,0,-1,5], [0,0,0,1]]))
    sexta = SE3(np.array([[1,0,0,14], [0,1,0,0],[0,0,-1,-3], [0,0,0,1]]))
    septima = SE3(np.array([[1,0,0,18], [0,1,0,0],[0,0,-1,-3], [0,0,0,1]]))
    octava = SE3(np.array([[1,0,0,18], [0,1,0,0],[0,0,-1,5], [0,0,0,1]]))

    generator = Generator()
    
    #Para definir trayectoria 1
    points1 = generator.generateTrayectory(qt,home,primera,'primera')

    #Para definir trayectoria 2
    points2 = generator.generateTrayectory(points1[-1],primera,segunda,'segunda')

    #Para definir trayectoria 3
    points3 = generator.generateTrayectory(points2[-1],segunda,tercera,'tercera')

    #Para definir trayectoria 4
    points4 = generator.generateTrayectory(points3[-1],tercera,cuarta,'cuarta')

    #Para definir trayectoria 5
    points5 = generator.generateTrayectory(points4[-1],cuarta,quinta,'quinta')

    #Para definir trayectoria 6
    points6 = generator.generateTrayectory(points5[-1],quinta,sexta,'sexta')

    #Para definir trayectoria 7
    points7 = generator.generateTrayectory(points6[-1],sexta,septima,'septima')
    
    #Para definir trayectoria 8
    points8 = generator.generateTrayectory(points7[-1],septima,octava,'octava')

    generator.totalTrayectory = np.array(generator.totalTrayectory)
    robot.plot(generator.totalTrayectory,'pyplot')
    print("Posicion final")
    print(robot.fkine(generator.totalTrayectory[-1]))