#Hecho por Juan Miguel Regal Llamas y Saúl Díaz García
"""qlearn controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot,DistanceSensor,Motor
import numpy as np
import random
import math

# Maximun velocidad para este experimento.
CRUISE_SPEED = 10
# Time step por defecto para el controlador.
TIME_STEP = 32
NUMBER_OF_ULTRASONIC_SENSORS = 5
NUMBER_OF_IR_SENSORS = 8
LIMITE_BLANCO = 750
LIMITE_NEGRO = 500
LIMITE_PARED = 250
LIMITE_PARED_DELANTE = 180
ESTADOS = 6
ACCIONES = 3
MAP_SIZE = ESTADOS,ACCIONES
GIRO = 4
MAX_ITERACIONES = 700      
#POSITION CHECK
LIM_FINAL = 0.01
N = 3


#Odometry
WHEEL_RADIO = 0.021
WHEEL_SPACE = 0.10829
WHEELS_RADIO = WHEEL_SPACE/2
DESIRED_DISTANCE = 0.05
TURN_ANGLE = (50 * (math.pi/180))/2
#Movements
LINEAR_MOVE = (DESIRED_DISTANCE / WHEEL_RADIO)/2
TURN_MOVE = TURN_ANGLE*WHEELS_RADIO/WHEEL_RADIO




ultrasonic_sensors_names = [
  "left ultrasonic sensor", "front left ultrasonic sensor", "front ultrasonic sensor", "front right ultrasonic sensor",
  "right ultrasonic sensor"]

#define NUMBER_OF_INFRARED_SENSORS 12
infrared_sensors_names = [
  # turret sensors
  "rear left infrared sensor", "left infrared sensor", "front left infrared sensor", "front infrared sensor",
  "front right infrared sensor", "right infrared sensor", "rear right infrared sensor", "rear infrared sensor",
  # ground sensors
  "ground left infrared sensor", "ground front left infrared sensor", "ground front right infrared sensor",
  "ground right infrared sensor"]




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

def izquierda():
    return DistanceSensor.getValue(infrared_sensors[8])
def izquierda_delante():
    return DistanceSensor.getValue(infrared_sensors[9])
def derecha_delante():
    return DistanceSensor.getValue(infrared_sensors[10])
def derecha():
    return DistanceSensor.getValue(infrared_sensors[11])
def pared_izquierda():
    return DistanceSensor.getValue(infrared_sensors[2])
def pared_delante():
    return DistanceSensor.getValue(infrared_sensors[4])
def pared_derecha():
    return DistanceSensor.getValue(infrared_sensors[6])

def estado():
    if (pared_izquierda() > LIMITE_PARED):
        return 3
    elif (pared_delante() > LIMITE_PARED_DELANTE):
        return 4
    elif (pared_derecha() > LIMITE_PARED):
        return 5
    elif (izquierda_delante() > LIMITE_BLANCO) and (derecha() < LIMITE_NEGRO): 
        return 1 #saliendo por la izquierda
    elif (derecha_delante() > LIMITE_BLANCO) and (izquierda() < LIMITE_NEGRO):
        return 2 #saliendo por la derecha
    return 0 #resto de casos

def accion(iteracion, estado_previo):

    global posL, posR, final_posL, final_posR, accion_previa, matriz, l_previa,r_previa,fl_previa,fr_previa,pfr_previa,pl_previa,pr_previa
    
    estado_actual = estado()
    l = izquierda()
    r = derecha()
    fl = izquierda_delante()
    fr = derecha_delante()
    pfr = pared_delante()
    pl = pared_izquierda()
    pr = pared_derecha()

    probabilidad = 1 - iteracion/MAX_ITERACIONES
    if (random.random() <= probabilidad):
        indice = random.randint(0,2)
        if (indice == 0):
            final_posL, final_posR = turnLeft(posL,posR)
            accion_actual = 0
        if (indice == 1):
            final_posL, final_posR = turnRight(posL,posR)
            accion_actual = 1
        if (indice == 2):
            final_posL, final_posR = moveFront(posL,posR)
            accion_actual = 2
    else:
        maximo = max(matriz[estado_actual,0],matriz[estado_actual,1], matriz[estado_actual,2])
        if (maximo == matriz[estado_actual,0]):
            final_posL, final_posR = turnLeft(posL,posR)
            accion_actual = 0
        elif (maximo == matriz[estado_actual,1]):
            final_posL, final_posR = turnRight(posL,posR)
            accion_actual = 1
        elif (maximo == matriz[estado_actual,2]):
            final_posL, final_posR = moveFront(posL,posR)
            accion_actual = 2
    visitas[estado_actual,accion_actual] = visitas[estado_actual,accion_actual] +1
    if iteracion != 0 : 
        if (estado_actual > 2):
            refuerzo = calcularRefuerzoPared(pl,pr,pfr,pl_previa,pr_previa,pfr_previa)
        else :refuerzo = calcularRefuerzo(l,r,fl,fr,l_previa,r_previa,fl_previa,fr_previa)
    else:
        refuerzo = 0
    veces_visitado = 1/(1+ visitas[estado_actual,accion_actual])
    matriz[estado_previo,accion_previa] = (1-veces_visitado)*matriz[estado_previo,accion_previa] + veces_visitado*(refuerzo + 0.5 *(max(matriz[estado_actual,0],matriz[estado_actual,1],matriz[estado_actual,2])))
    
    accion_previa = accion_actual
    l_previa = l
    r_previa = r
    fl_previa = fl
    fr_previa = fr
    pfr_previa = pfr
    pl_previa = pl
    pr_previa = pr
    iteracion = iteracion+1
    if iteracion % 20 == 0:
        print("iteracion: ",iteracion)
        print("matriz q:")
        print(matriz)
        print("matriz nº de visitas:")
        print(visitas)
        print("_________________\n")
    return estado_actual, iteracion

def calcularRefuerzo(l,r,fl,fr,lp,rp,flp,frp):
    return calcRefuerzoSensor(l,lp) + calcRefuerzoSensor(r,rp) + calcRefuerzoSensorFrente(fl,flp) + calcRefuerzoSensorFrente(fr,frp)

def calcRefuerzoSensor(sens, sens_previo):
    ref = 0
    if sens_previo < LIMITE_NEGRO:
        if sens < LIMITE_NEGRO:
            ref = ref + 0.5
        elif sens > LIMITE_BLANCO:
            ref = ref - 1
    else:
        if sens < LIMITE_NEGRO:
            ref = ref + 1
    return ref

def calcRefuerzoSensorFrente(sens, sens_previo):
    ref = 0
    if sens_previo < LIMITE_NEGRO:
        if sens < LIMITE_NEGRO:
            ref = ref + 0.7
        elif sens > LIMITE_BLANCO:
            ref = ref - 1.2
    else:
        if sens < LIMITE_NEGRO:
            ref = ref + 1.5
    return ref
    
def calcularRefuerzoPared(pl,pr,pfr,plp,prp,pfrp):
    return calcRefuerzoSensorPared(pl,plp) + calcRefuerzoSensorPared(pr,prp) + calcRefuerzoSensorParedFrente(pfr,pfrp)

def calcRefuerzoSensorPared(sens, sens_previo):
    ref = 0
    if sens_previo < LIMITE_PARED:
        if sens < LIMITE_PARED:
            ref = ref + 0.5 #+0.1 *(LIMITE_PARED - sens)
        elif sens > LIMITE_PARED:
            ref = ref - 1 
    else:
        if sens < LIMITE_PARED:
            ref = ref + 1 + 0.5 #+0.1 *(LIMITE_PARED - sens)
    return ref

def calcRefuerzoSensorParedFrente(sens, sens_previo):
    ref = 0
    if sens_previo < LIMITE_PARED:
        if sens < LIMITE_PARED:
            ref = ref + 0.7 + 0.5 +0.01 *(LIMITE_PARED - sens)
        elif sens > LIMITE_PARED:
            ref = ref - 1.2
    else:
        if sens < LIMITE_PARED:
            ref = ref + 1.5 + 0.5 +0.01 *(LIMITE_PARED - sens)
    return ref

def turnLeft(posL,posR):
    final_posL = posL - TURN_MOVE
    final_posR = posR + TURN_MOVE+N
    final = False
    leftWheel.setPosition(encoderL.getValue() - TURN_MOVE )
    rightWheel.setPosition(encoderR.getValue() + TURN_MOVE+N )
    turned = True
    return final_posL, final_posR

def turnRight(posL, posR):
    final_posL = posL + TURN_MOVE+N
    final_posR = posR - TURN_MOVE 
    final = False
    leftWheel.setPosition(encoderL.getValue() + TURN_MOVE+N )
    rightWheel.setPosition(encoderR.getValue() - TURN_MOVE )
    turned = True
    return final_posL, final_posR

def moveFront(posL, posR):
    final_posL = posL + LINEAR_MOVE
    final_posR = posR + LINEAR_MOVE
    final = False
    leftWheel.setPosition(encoderL.getValue() + LINEAR_MOVE)
    rightWheel.setPosition(encoderR.getValue() + LINEAR_MOVE)
    turned = False
    return final_posL, final_posR

# create the Robot instance.
#robot = Robot()
robot, camera, leftWheel, rightWheel, infrared_sensors, ultrasonic_sensors, encoderL, encoderR = init_devices(TIME_STEP)

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
robot.step(TIME_STEP)
final_posL = encoderL.getValue()
final_posR = encoderR.getValue()
posL = encoderL.getValue()
posR = encoderR.getValue()
matriz = np.zeros((MAP_SIZE))
visitas = np.zeros((MAP_SIZE))

print(posL,posR,final_posL,final_posR)
# - perform simulation steps until Webots is stopping the controller
estado_previo = estado()
accion_previa = 0
refuerzo = 0
iteracion = 0
l_previa = 0
r_previa = 0
fl_previa = 0
fr_previa = 0
pfr_previa = 0
pl_previa = 0
pr_previa = 0
while robot.step(timestep) != -1:

    posL = encoderL.getValue()
    posR = encoderR.getValue()
    final = ((abs(final_posL-posL)) <= LIM_FINAL)
    if final:
        estado_previo, iteracion = accion(iteracion, estado_previo)
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(CRUISE_SPEED)