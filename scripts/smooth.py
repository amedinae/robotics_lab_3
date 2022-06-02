import numpy as np
import matplotlib.pyplot as plt

def smooth(trayectory):
    trayectory = np.transpose(trayectory)
    q1 = trayectory[0]
    q2 = trayectory[1]
    q3 = trayectory[2]
    q4 = trayectory[3]
    for q in trayectory:
        #langs = [str(i) for i in range(20)]
        #plt.bar(langs,q)
        #plt.show()
        for index,value in enumerate(q):
            print(f'index = {index} value = {value}')
            if index<=0:
                continue
            diff = (abs(value-q[index-1]))
            if diff > 0.3:
                print(f'index = {index} diff = {diff}')
                print(index)
                if index+1 == len(q):
                    q[index] = q[index-1]
                    continue
                q[index] = (q[index-1]+q[index+1])/2
        #langs = [str(i) for i in range(20)]
        #plt.bar(langs,q)
        #plt.show()
    return np.transpose(trayectory)