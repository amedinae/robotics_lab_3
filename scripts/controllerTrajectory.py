import rospy
import time
import termios, sys, os
import numpy as np
#import genpy
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

TERMIOS = termios
home = 512

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c


def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def setTorquesLimit():
    jointCommand('', 1, 'Torque_Limit', 500, 0)
    jointCommand('', 2, 'Torque_Limit', 500, 0)
    jointCommand('', 3, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Torque_Limit', 400, 0)

def rad2bin(value):
    fromLow = -5/6*np.pi
    fromHigh = 5/6*np.pi
    toLow = 0
    toHigh = 1023
    return int(np.around((value-fromLow) * (toHigh - toLow) / (fromHigh - fromLow)+ toLow,0))

def runTrajectory(primera):
    for point in primera:
        for index,q in enumerate(point):
            toPosition = rad2bin(point[index])
            index += 1
            print(f"Current ID: {index}")
            jointCommand('', index, 'Goal_Position', toPosition, 0.05)
            print(f"Joint {index} moved to {toPosition}")

def moveToHome():
    for i in range(5):
        jointCommand('', i+1, 'Goal_Position', home, 1)

def closeTool():
    jointCommand('', 5, 'Goal_Position', 650, 0.05)

def openTool():
    jointCommand('', 5, 'Goal_Position', 512, 0.05)

if __name__ == '__main__':
    primera = np.genfromtxt('scripts/trajectories/primera.csv', delimiter=',')
    segunda = np.genfromtxt('scripts/trajectories/segunda.csv', delimiter=',')
    tercera = np.genfromtxt('scripts/trajectories/tercera.csv', delimiter=',')
    cuarta = np.genfromtxt('scripts/trajectories/cuarta.csv', delimiter=',')
    quinta = np.genfromtxt('scripts/trajectories/quinta.csv', delimiter=',')
    sexta = np.genfromtxt('scripts/trajectories/sexta.csv', delimiter=',')
    septima = np.genfromtxt('scripts/trajectories/septima.csv', delimiter=',')
    octava = np.genfromtxt('scripts/trajectories/octava.csv', delimiter=',')
    try:
        setTorquesLimit()
        while(not rospy.is_shutdown()):
            moveToHome()
            runTrajectory(primera)
            runTrajectory(segunda)
            runTrajectory(tercera)
            closeTool()
            runTrajectory(cuarta)
            runTrajectory(quinta)
            runTrajectory(sexta)
            runTrajectory(septima)
            openTool() 
            break
    except rospy.ROSInterruptException:
        pass