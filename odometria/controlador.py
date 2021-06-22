#Hecho por Juan Miguel Regal Llamas y Saúl Díaz García

from controller import Robot, Motor, DistanceSensor
from controller import Camera
import time
import threading
import numpy as np
import math

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Maximun velocidad para este experimento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 32
NUMBER_OF_ULTRASONIC_SENSORS = 5
NUMBER_OF_IR_SENSORS = 8
#Odometry
WHEEL_RADIO = 0.021
WHEEL_SPACE = 0.10829
WHEELS_RADIO = WHEEL_SPACE/2
DESIRED_DISTANCE = 0.25
TURN_ANGLE = (90 * (math.pi/180))
#Camera
MIN_VERDE = 180
MAX_AZUL = 60
MAX_ROJO = 60
LIM_GREEN = 5000

#ORIENTACION
ABAJO = 0
IZQUIERDA = 1
ARRIBA = 2
DERECHA = 3

#Movements
LINEAR_MOVE = DESIRED_DISTANCE / WHEEL_RADIO
TURN_MOVE = TURN_ANGLE*WHEELS_RADIO/WHEEL_RADIO

#MAP
STARTX = 15
STARTY = 15
START = STARTX,STARTY
MAP_SIZE = 30,30
FREE_POS = 1
WALL_POS = 0
MAX_VALUE = 99

#POSITION CHECK
LIM_FINAL = 0.01
LIM_SENSORS = 180

c = 0
semaforo = threading.Semaphore(1)
global prioridad
prioridad = 0
global stop
stop = False

#el mapa es un array 30x30 y comenzamos en el medio (15x15)
mapa = np.zeros((MAP_SIZE))
distMap = np.zeros((MAP_SIZE))
finalMap = np.zeros((MAP_SIZE))

position = START
positionx,positiony = START
mapa[positionx] [positiony] = 0
#abajo = 0, izquierda = 1, derecha = 2, arriba = 3
orientacion = 0
stopMap = False
mapaDone = False
objectFound = False
startSearch = False
ObjectFound = False
comeback = False
diag = False
wall = False
lap = True
firstLap = False
findWall = False
back = False
startingOr = 0

barrier = threading.Barrier(5)

for i in range(mapa.shape[0]):
        for j in range(mapa.shape[1]):
            distMap[i][j] = (abs(i-STARTX)+abs(j-STARTY))
count = 0
final = True

ultrasonic_sensors_names = [
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"];

#define NUMBER_OF_INFRARED_SENSORS 12
infrared_sensors_names = [
  # turret sensors
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
  # ground sensors
  "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
  "ground right infrared sensor"];   

def init_devices(timeStep):
    """
    Obtener y configurar los dispositivos necesarios.
    """
    robot = Robot()
    
    # Obtener dispositivos correspondientes a las ruedas.
    leftWheel = robot.getDevice('left wheel motor')
    rightWheel = robot.getDevice('right wheel motor')

    #habilitamos encodersy ponemos movimiento por posición
    encoderL = robot.getDevice("left wheel sensor")
    encoderR = robot.getDevice("right wheel sensor")
    encoderL.enable(timeStep)
    encoderR.enable(timeStep)
    leftWheel.setPosition(0)
    rightWheel.setPosition(0)
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)
    # Obtener y activar el dispositivo de la cámara    
    camera = robot.getDevice('camera')
    camera.enable(timeStep*10)
    
    # Activar otros sensores necesarios
    # ...
    #get and enable the ultrasonic sensors
    ultrasonic_sensors = []
    i = 0
    for sensor in  ultrasonic_sensors_names:      
        ultrasonic_sensors = ultrasonic_sensors + [robot.getDevice(sensor)]
        ultrasonic_sensors[i].enable(timeStep)
        i = i + 1
    print(ultrasonic_sensors_names)

     #get and enable the infrared sensors
    infrared_sensors = []
    i = 0
    for sensor in  infrared_sensors_names:      
         infrared_sensors = infrared_sensors + [robot.getDevice(sensor)]
         infrared_sensors[i].enable(timeStep)
         i = i + 1
    
    return robot, camera, leftWheel, rightWheel , infrared_sensors, ultrasonic_sensors,encoderL, encoderR



def getSensorValues():
    back_left = DistanceSensor.getValue(infrared_sensors[0]) > LIM_SENSORS
    left= DistanceSensor.getValue(infrared_sensors[1]) > LIM_SENSORS
    front_left = DistanceSensor.getValue(infrared_sensors[2]) > LIM_SENSORS
    front = DistanceSensor.getValue(infrared_sensors[3]) > LIM_SENSORS
    front_right = DistanceSensor.getValue(infrared_sensors[4]) > LIM_SENSORS
    right = DistanceSensor.getValue(infrared_sensors[5]) > LIM_SENSORS
    back_right = DistanceSensor.getValue(infrared_sensors[6]) > LIM_SENSORS
    back = DistanceSensor.getValue(infrared_sensors[7]) > LIM_SENSORS
    return back_left, back_right, front,back, front_left, front_right, left, right

def turnLeft():
    final_posL = posL - TURN_MOVE
    final_posR = posR + TURN_MOVE
    final = False
    leftWheel.setPosition(encoderL.getValue() - TURN_MOVE)
    rightWheel.setPosition(encoderR.getValue() + TURN_MOVE)
    turned = True
    return final_posL, final_posR, turned

def turnRight():
    final_posL = posL + TURN_MOVE
    final_posR = posR - TURN_MOVE
    final = False
    leftWheel.setPosition(encoderL.getValue() + TURN_MOVE)
    rightWheel.setPosition(encoderR.getValue() - TURN_MOVE)
    turned = True
    return final_posL, final_posR, turned

def moveFront():
    final_posL = posL + LINEAR_MOVE
    final_posR = posR + LINEAR_MOVE
    final = False
    leftWheel.setPosition(encoderL.getValue() + LINEAR_MOVE)
    rightWheel.setPosition(encoderR.getValue() + LINEAR_MOVE)
    turned = False
    return final_posL, final_posR, turned

def getGreenObject():
    W = camera.getWidth()
    H = camera.getHeight()  
    image = bytes(np.frombuffer(cameraData, np.uint8).reshape((H, W, 4)))
    c = 0
    for y in range(1*H//3,2*H//3):
        for x in range(2*W//5,3*W//5):
            green = Camera.imageGetGreen(image,W,x,y) 
            red = Camera.imageGetRed(image,W,x,y) 
            blue = Camera.imageGetBlue(image,W,x,y)
            if (green > MIN_VERDE and blue < MAX_AZUL and red < MAX_ROJO):
                c+=1
    #print("verde", c)
    return  c > LIM_GREEN



def get_casillas(position,orientacion):
    x,y = position
    if orientacion == ABAJO:
        front = (x+1),(y+0) 
        back = (x-1),(y+0) 
        left = (x+0),(y+1) 
        right = (x+0),(y-1) 
    elif orientacion == ARRIBA:
        front = (x-1),(y+0) 
        back = (x+1),(y+0) 
        left = (x+0),(y-1) 
        right = (x+0),(y+1) 
    elif orientacion == DERECHA:
        front = (x+0),(y-1) 
        back = (x+0),(y+1) 
        left = (x+1),(y+0) 
        right = (x-1),(y+0) 
    elif orientacion == IZQUIERDA:
        front = (x+0),(y+1) 
        back = (x+0),(y-1) 
        left = (x-1),(y+0) 
        right = (x+1),(y+0) 
    return front,back,left,right

def get_diag_casillas(position,orientacion):
    x,y = position
    if orientacion == ABAJO:
        front_left = (x+1),(y+1) 
        back_left = (x-1),(y+1) 
        front_right = (x+1),(y-1) 
        back_right = (x-1),(y-1) 
    elif orientacion == ARRIBA:
        front_left = (x-1),(y-1) 
        back_left = (x+1),(y-1) 
        front_right = (x-1),(y+1) 
        back_right = (x+1),(y+1)
    elif orientacion == IZQUIERDA:
        front_left = (x-1),(y+1) 
        back_left = (x-1),(y-1) 
        front_right = (x+1),(y+1) 
        back_right = (x+1),(y-1)
    elif orientacion == DERECHA:
        front_left = (x+1),(y-1) 
        back_left = (x+1),(y+1) 
        front_right = (x-1),(y-1) 
        back_right = (x-1),(y+1)
    return front_left,front_right,back_left,back_right

def restar_orientacion(orientacion):
    orientacion = orientacion - 1
    if orientacion < 0:
        orientacion = 3
    return orientacion

def sumar_orientacion(orientacion):
    orientacion = orientacion + 1
    if orientacion > 3:
        orientacion = 0
    return orientacion
def addvalues(casillas,valores,mapa):
    #simplemente por motivos de legibilidad del mapa se pondrán false como 0 y true como 1
    (frontx,fronty),(leftx,lefty),(rightx,righty) = casillas
    (front,left,right) = valores
    #mapa[frontx] [fronty] = front
    #mapa[leftx] [lefty] = left
    #mapa[rightx] [righty] = right

    if front: mapa[frontx] [fronty] = WALL_POS
    else: mapa[frontx] [fronty] = FREE_POS
    
    if left: mapa[leftx] [lefty] = WALL_POS
    else: mapa[leftx] [lefty] = FREE_POS

    if right: mapa[rightx] [righty] = WALL_POS
    else: mapa[rightx] [righty] = FREE_POS
    return mapa

def getValues(leftpos,frontpos,rightpos,backpos,mapa):
    (leftx,lefty) = leftpos
    (frontx,fronty) = frontpos
    (rightx,righty) = rightpos
    (backx,backy) = backpos
    leftVal = mapa[leftx][lefty]
    frontVal= mapa[frontx][fronty]
    rightVal = mapa[rightx][righty]
    backVal = mapa[backx][backy]
    return leftVal, frontVal, rightVal,backVal

def getDiagValues(fl,fr,bl,br,mapa):
    (flx,fly) = fl
    (frx,fry) = fr
    (blx,bly) = bl
    (brx,bry) = br
    flVal = mapa[flx][fly]
    frVal= mapa[frx][fry]
    blVal = mapa[blx][bly]
    brVal = mapa[brx][bry]
    return flVal,frVal,blVal,brVal

def getMinimumValue(left,front,right,back,fl,fr):
    return min(left,front,right,back,fl,fr)

def getMaximumValue(left,front,right):
    return max(left,front,right,back)

def stopMapping(mapa):
    with open('wallmap.txt','wb') as f:
        np.savetxt(f, (mapa), fmt='%i')
    print("map explored")
    finalMap = np.multiply(mapa,distMap)
    with open('dist.txt','wb') as d:
        np.savetxt(d,finalMap, fmt='%i')
    return finalMap
    #startSearch = True
    #return startSearch

def stopSearching():
    
    comeback = True
    return comeback

def behaviour04(prior_propia):
    """
    Buscar pared
    """
    global stop
    global mapaDone,position, orientacion,count,mapa,stopMap,final_posL, final_posR, turned,posL,posR,final,startingPos,wall, findWall, startingOr
    while not stop:
        #time.sleep(0.01)
        if not wall:
            semaforo.acquire()
            
            posL = encoderL.getValue()
            posR = encoderR.getValue()
            final = (abs(final_posL-posL)) <= LIM_FINAL
            
            if not wall and final:
                #time.sleep(0.1)
                global prioridad
                if prioridad <= prior_propia:
                    prioridad = prior_propia
                print("looking for a wall")
                back_left, back_right, front,back, front_left, front_right, left, right = getSensorValues()
                frontpos,backpos,leftpos,rightpos = get_casillas(position,orientacion)
                isWall = front or back  or left or right
                if isWall:
                    if front:
                        print("hey")
                        orientacion = restar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnRight()
                    elif left:
                        wall = True
                        startingPos = position
                        startingOr = orientacion
                        if prioridad <= prior_propia:
                            prioridad = 0
                    else:
                        orientacion = sumar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnLeft()
                else:
                    posx, posy = position
                    mapa[posx][posy] = FREE_POS
                    mapa[posx+1][posy] = FREE_POS
                    mapa[posx-1][posy] = FREE_POS
                    mapa[posx][posy-1] = FREE_POS
                    mapa[posx][posy+1] = FREE_POS
                    position = frontpos
                    final_posL, final_posR, turned = moveFront()
                    findWall = True
                    
                semaforo.release()
            else:
                semaforo.release()

        barrier.wait()

def behaviour03(prior_propia):
    """
    Explorar mapa
    """
    global stop
    global mapaDone,position, orientacion,count,mapa,stopMap,final_posL, final_posR, turned,posL,posR,final,startingPos, wall,lap,firstLap, finalMap,second, startingOr
    while not stop:
        #time.sleep(0.01)
        if not mapaDone and wall:
            semaforo.acquire()

            posL = encoderL.getValue()
            posR = encoderR.getValue()
            final = (abs(final_posL-posL)) <= LIM_FINAL
            if not mapaDone and final:
                #time.sleep(0.1)
                global prioridad
                if prioridad <= prior_propia:
                    prioridad = prior_propia
                print("explorando mapa")
                back_left, back_right, front,back, front_left, front_right, left, right = getSensorValues()
                frontpos,backpos,leftpos,rightpos = get_casillas(position,orientacion)
                green = False
                green = getGreenObject()
                if count > 0: count = count-1
                if green and not right: count = 3
                print("count is ",count)
                front = green or front
                mapa = addvalues((frontpos,leftpos,rightpos),(front,left,right),mapa)
                if (position == startingPos): 
                    lap = False
                if (stopMap):
                    finalMap = stopMapping(mapa)
                    mapaDone = True
                    second = True
                    if prioridad <= prior_propia:
                        prioridad = 0
                elif (front and  not turned):
                    #position = rightpos
                    #si count es 1 significa que rodeó un objeto verde y giramos a la izquierda si podemos para seguir correctamente la forma de la habitación
                    if not left and (count == 1):
                        orientacion = sumar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnLeft()
                    else:
                        orientacion = restar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnRight()
                elif (not left and not turned):
                    orientacion = sumar_orientacion(orientacion)
                    final_posL, final_posR, turned = turnLeft()
                elif (not front):
                    position = frontpos
                    final_posL, final_posR, turned = moveFront()
                else:
                    turned = False
                if ((position == startingPos and lap) or (position == startingPos and firstLap) and orientacion == startingOr):
                    stopMap = True
                if not lap:
                    firstLap = True

                semaforo.release()
            else:
                semaforo.release()

        barrier.wait()

def behaviour02(prior_propia):
    """
    Buscar objeto
    """
    global stop
    global mapaDone,position, orientacion,count,mapa,stopMap,final,final_posL, final_posR,posL,posR, turned,ObjectFound,startSearch,finalMap,findWall
    while not stop:
        #time.sleep(0.01)
    #Thread02 buscarObjeto

        if startSearch and mapaDone:
            semaforo.acquire()

            posL = encoderL.getValue()
            posR = encoderR.getValue()
            final = (abs(final_posL-posL)) <= LIM_FINAL

            if startSearch and mapaDone and final:
                #time.sleep(0.1)
                print("buscando objeto")
                global prioridad
                if prioridad <= prior_propia:
                    prioridad = prior_propia
                frontpos,backpos,leftpos,rightpos = get_casillas(position,orientacion)
                objectFound = getGreenObject()
                leftVal, frontVal, rightVal ,backVal= getValues(leftpos,frontpos,rightpos,backpos,mapa)
                if (objectFound):
                    comeback = stopSearching()
                    if prioridad <= prior_propia:
                        prioridad = 0
                    startSearch = False
                    ObjectFound = True
                elif startingPos != (START) and findWall:
                    if (frontVal == 1):
                        position = frontpos
                        final_posL, final_posR, turned = moveFront()
                    else:
                        orientacion = restar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnRight()
                        findWall = False
                elif (frontVal==0):
                    orientacion = restar_orientacion(orientacion)
                    final_posL, final_posR, turned = turnRight()
                elif (leftVal ==1 and  not turned ):
                    orientacion = sumar_orientacion(orientacion)
                    final_posL, final_posR, turned = turnLeft()
                elif (frontVal == 1):
                    position = frontpos
                    final_posL, final_posR, turned = moveFront()
                else:
                    turned = 0
                semaforo.release()
            else:
                semaforo.release()

        barrier.wait()



def behaviour01(prior_propia):
    """
    volver a la base utilizando el mapa
    """
    global stop
    global mapaDone,position, orientacion,count,mapa,stopMap,final,final_posL, final_posR,posL,posR, turned,ObjectFound,comeback, back,second,destination
    diag = False
    while not stop:
        #time.sleep(0.01)
        #Thread03 volverBase
        if comeback:
            global finalMap
            semaforo.acquire()

            posL = encoderL.getValue()
            posR = encoderR.getValue()
            final = (abs(final_posL-posL)) <= LIM_FINAL

            if comeback and final:
                #time.sleep(0.1)
                print("volviendo a base")
                global prioridad
                if prioridad <= prior_propia:
                    prioridad = prior_propia
                if not diag and final:
                    frontpos,backpos,leftpos,rightpos = get_casillas(position,orientacion)
                    front_leftpos,front_rightpos,back_leftpos,back_rightpos = get_diag_casillas(position,orientacion)
                    objectFound = getGreenObject()
                    (posx,posy) = position
                    leftVal, frontVal, rightVal, backVal = getValues(leftpos,frontpos,rightpos,backpos,finalMap)
                    front_leftVal,front_rightVal,back_leftVal,back_rightVal = getDiagValues(front_leftpos,front_rightpos,back_leftpos,back_rightpos, finalMap) 
                    
                    if leftVal == WALL_POS and leftpos != (START): leftVal = MAX_VALUE
                    if frontVal == WALL_POS and frontpos != (START): frontVal = MAX_VALUE
                    if rightVal == WALL_POS and rightpos != (START): rightVal = MAX_VALUE
                    if backVal == WALL_POS and backpos != (START): backVal = MAX_VALUE
                    if front_leftVal == WALL_POS and front_leftpos != (START): front_leftVal = MAX_VALUE
                    if front_rightVal == WALL_POS and front_rightpos != (START): front_rightVal = MAX_VALUE
                    if back_leftVal == WALL_POS and back_leftpos != (START): back_leftVal = MAX_VALUE
                    if back_rightVal == WALL_POS and back_rightpos != (START): back_rightVal = MAX_VALUE
                    minVal = getMinimumValue(leftVal, frontVal, rightVal, backVal,front_leftVal,front_rightVal)
                    maxVal = getMaximumValue(leftVal,frontVal,rightVal)
                    if not second and finalMap[posx][posy] <= minVal and maxVal == MAX_VALUE: finalMap[posx][posy] = MAX_VALUE
                    
                    if (position == (START)):
                        if second:
                            comeback = False
                            back = True
                        else:
                            stop = True
                        if prioridad <= prior_propia:
                            prioridad = 0
                    elif (minVal == frontVal) and not getGreenObject():
                        position = frontpos
                        final_posL, final_posR, turned = moveFront()
                    elif (minVal == rightVal):
                        orientacion = restar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnRight()  
                    elif (minVal == front_leftVal):
                        diag = True
                        desiredPos = "front_left"
                        destination = front_leftpos
                    elif (minVal == front_rightVal):
                        diag = True
                        desiredPos = "front_right"
                        destination = front_rightpos
                    else:
                        orientacion = sumar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnLeft()

                else:
                    frontpos,backpos,leftpos,rightpos = get_casillas(position,orientacion)
                    if (frontpos == destination):
                        if not getGreenObject():
                            position = frontpos
                            final_posL, final_posR, turned = moveFront()
                            diag = False
                        else:
                            frontposx,frontposy = frontpos
                            finalMap[frontposx][frontposy] = WALL_POS
                            orientacion = sumar_orientacion(orientacion)
                            final_posL, final_posR, turned = turnLeft()
                            diag = False

                    elif (rightpos == destination):
                        orientacion = restar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnRight() 

                    elif (leftpos == destination):
                        orientacion = sumar_orientacion(orientacion)
                        final_posL, final_posR, turned = turnLeft()

                    else:
                        leftVal, frontVal, rightVal, backVal = getValues(leftpos,frontpos,rightpos,backpos,finalMap)
                        if (desiredPos == "front_left"):
                            if frontVal != WALL_POS and not getGreenObject():
                                position = frontpos
                                final_posL, final_posR, turned = moveFront()
                            else:
                                orientacion = sumar_orientacion(orientacion)
                                final_posL, final_posR, turned = turnLeft()
                        elif (desiredPos == "front_right"):
                            if frontVal != WALL_POS and not getGreenObject():
                                position = frontpos
                                final_posL, final_posR, turned = moveFront()
                            else:
                                orientacion = restar_orientacion(orientacion)
                                final_posL, final_posR, turned = turnRight()
                semaforo.release()
            else:
                semaforo.release()

        barrier.wait()


"""
HILO PRINCIPAL
- Se inicializan los comportamientos como threads.
- Se ejecuta el bucle de sincronización de la simulación.
- La API de Webots es "Thread safe". 
- ¡ATENCIÓN! Puede ser necesario sincronizar acceso concurrente a variables con bloqueos.
"""

# Activamos los dispositivos necesarios y obtenemos referencias a ellos.
robot, camera, leftWheel, rightWheel, infrared_sensors, ultrasonic_sensors, encoderL, encoderR = init_devices(TIME_STEP)

# Parámetros para las tareas que se ejecutan en threads.
# "stop" controla el final de la ejecución de la tarea.
global params
params = {"stop": False}

# Ejecutamos una sincronización para poder cargar el primer frame de la cámara.
robot.step(TIME_STEP)
final_posL = 0
final_posR = 0
leftWheel.setVelocity(CRUISE_SPEED)
rightWheel.setVelocity(CRUISE_SPEED)
turned = False
cameraData = camera.getImage()

posL = encoderL.getValue()
posR = encoderR.getValue()

# Ejecutamos los threads de las tareas
thread01 = threading.Thread(target=behaviour01, args=(1,))
thread01.start()
thread02 = threading.Thread(target = behaviour02, args=(2,))
thread02.start()
thread03 = threading.Thread(target=behaviour03, args=(3,))
thread03.start()
thread04 = threading.Thread(target = behaviour04, args=(4,))
thread04.start()

#mapa
#search
#primera tarea (hacer mapa)
first = True
#segunda tarea(volver a casilla inicial)
second = False
#tercera tarea (buscar objeto)
third = True
#cuarta tarea (volver de nuevo a casilla inicial)
fourth = True

while robot.step(TIME_STEP) != -1:

    #time.sleep(0.005)
    posL = encoderL.getValue()
    posR = encoderR.getValue()
    final = (abs(final_posL-posL)) <= LIM_FINAL
    #Valores de los sensores

    # Si vamos a utilizar la cámara actualizamos el frame actual en la variable global.
    cameraData = camera.getImage()
    #time.sleep(0.01)
    
    if mapaDone and first:
        first = False
        comeback = True
    if second and back:
        comeback = False
        first = False
        startSearch = True
        second = False
    if ObjectFound and third:
        comeback = True
        third = False
    if stop:
        thread01.join()
        thread02.join()
        thread03.join()
        thread04.join()
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(0)
        print("FIN")
    barrier.wait()