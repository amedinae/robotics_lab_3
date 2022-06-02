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

if __name__ == '__main__':
    primera = np.genfromtxt('scripts/trayectories/primera.csv', delimiter=',')
    segunda = np.genfromtxt('scripts/trayectories/segunda.csv', delimiter=',')
    tercera = np.genfromtxt('scripts/trayectories/tercera.csv', delimiter=',')
    cuarta = np.genfromtxt('scripts/trayectories/cuarta.csv', delimiter=',')
    quinta = np.genfromtxt('scripts/trayectories/quinta.csv', delimiter=',')
    sexta = np.genfromtxt('scripts/trayectories/sexta.csv', delimiter=',')
    septima = np.genfromtxt('scripts/trayectories/septima.csv', delimiter=',')
    octava = np.genfromtxt('scripts/trayectories/octava.csv', delimiter=',')
    try:
        setTorquesLimit()
        while(not rospy.is_shutdown()):
            for i in range(5):
                jointCommand('', i+1, 'Goal_Position', home, 1)
            for point in primera:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            for point in segunda:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            for point in tercera:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            jointCommand('', 5, 'Goal_Position', 650, 0.05)
            for point in cuarta:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            for point in quinta:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            for point in sexta:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            for point in septima:
                for index,q in enumerate(point):
                    toPosition = rad2bin(point[index])
                    index += 1
                    print(f"Current ID: {index}")
                    jointCommand('', index, 'Goal_Position', toPosition, 0.05)
                    print(f"Joint {index} moved to {toPosition}")
            jointCommand('', 5, 'Goal_Position', 512, 0.05) 
            break
    except rospy.ROSInterruptException:
        pass