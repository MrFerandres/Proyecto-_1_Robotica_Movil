"""
    Proyecto 1: Control de un robot movil
    Equipo:

    Codigo modificado de Juan-Pablo Ramirez-Paredes <jpi.ramirez@ugto.mx>
"""

import numpy as np
import scipy.interpolate as spi
import matplotlib.pyplot as plt

import numpy as np
import time
import math as m
import sim as vrep  # access all the VREP elements
import random as rnd


def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1) * m.cos(t2) + m.sin(t1) * m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1) * m.sin(t2) - m.sin(t1) * m.cos(t2)
    return m.copysign(angmag, angdir)

vrep.simxFinish(-1)  # just in case, close all opened connections
clientID = vrep.simxStart('127.0.0.1', -1, True, True, 5000, 5)  # start a connection
if clientID != -1:
    print('Connected to remote API server')
else:
    print('Not connected to remote API server')
    sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1, 17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor' + str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i],
                                                                                       vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

# Controller gains (linear and heading)
Kv = 0.5
Kh = 2.5

# xd and yd are the coordinates of the desired setpoint
xd = np.zeros(10) #si se modifica este 10 se tiene que modificar el de yd y el del siguiente for
yd = np.zeros(10)

for i in range(10):
    xd[i]=(rnd.randrange(-3, 3))
    yd[i]=(rnd.randrange(-3, 3))

print(xd,yd)

hd = 0
r = 0.5 * 0.195
L = 0.311


for i in range(len(xd)):
    errp = 10
    while errp > 1.5:

        ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
        ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
        errp = m.sqrt((xd[i] - carpos[0]) ** 2 + (yd[i] - carpos[1]) ** 2)
        angd = m.atan2(yd[i] - carpos[1], xd[i] - carpos[0])
        errh = angdiff(carrot[2], angd)
        print('Distance to goal: {}   Heading error: {}'.format(errp, errh))

        # Uncomment for switched control
        # if errh > 2.0*m.pi/180.0:
        #     v = 0
        #     omega = -Kh*errh
        # else:
        #     v = Kv*err
        #     omega = 0
        # Uncomment for continuous control
        v = Kv * errp
        omega = Kh * errh

        ul = (v / r - L * omega / (2 * r))*0.5 # se multiplica por 0.5 porque la velocidad es mucha y cuando calcula otra vez el ERRP se pasa haciendo que este en un ciclo infinito
        ur = (v / r + L * omega / (2 * r))*0.5
        errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
        errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
        # time.sleep(0.1)

for i in range(10):
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0, vrep.simx_opmode_streaming)
    # time.sleep(0.1)

vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)


        #Aqui inicia la parte de la ruta a trazar


ttime = 10 #tiempo total entre el primer punto y el ultimo punto

tarr = np.linspace(0,10, xd.shape[0])

tnew = np.linspace(0, 10, 200) #division de puntos entre el segundo 0 y el segundo 10 con una division de 200 instantes
xc = spi.splrep(tarr, xd, s=0)
yc = spi.splrep(tarr, yd, s=0)

xnew = spi.splev(tnew, xc, der=0)
ynew = spi.splev(tnew, yc, der=0)
plt.plot(xnew, ynew)
plt.plot(xd, yd, '.')
plt.show()


print(xd,yd)