#Hecho por Juan Miguel Regal Llamas y Saúl Díaz García

from controller import Robot, Motor, DistanceSensor
from controller import Camera
import time
import threading
import numpy as np

# Máxima velocidad de las ruedas soportada por el robot (khepera4).
MAX_SPEED = 47.6
# Maximun velocidad para este experimento.
CRUISE_SPEED = 8
# Time step por defecto para el controlador.
TIME_STEP = 24
NUMBER_OF_ULTRASONIC_SENSORS = 5
NUMBER_OF_IR_SENSORS = 8

#Valores límite detección pared ultrasonidos
L_WALL_US = 0.7
F_WALL_US = 0.25
#Valores límite detección pared infrarrojos
F_WALL_IR = 165
L_WALL_IR = 120
L_WALL_BEHIND_IR = 105
#Valores ajuste giro
KP = 2.75
DESIRED_DISTANCE = 0.10
AVOID_EXCEED_CRUISE_SPEED = KP*DESIRED_DISTANCE
#Valores comparación imagenes
LIM_SAME_IMG = 130  

#constantes usadas solo en el comportamiento de encontrar pared
SPIRAL_SPEED = 2
MAX_SPIRAL_SPEED = 6
TASA_AUMENTO_DE_GIRO_SPIRAL = 0.1
INF_DIST_MIN = 200 #valor mínimo de los infrarrojos para que el comportamiento buscar pared considere que hay una pared
ULT_DIST_MAX = 1 #distancia mínima detectada por los ultrasonidos para que el comportamiento de buscar pared considere que hay una pared

#constantes usadas para el comportamiento de buscar el objeto de color verde
MIN_VERDE = 220
MAX_AZUL = 50
MAX_ROJO = 50
MIN_PIXELES_VERDES = 100
MAX_PIXELES_VERDES = 15000
KP_COLOR = 2

semaforo = threading.Semaphore(1)
global prioridad
prioridad = 0
global stop
stop = False

#define NUMBER_OF_ULTRASONIC_SENSORS 5
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
    # Utilizamos velocidad, establecemos posición a infinito.
    leftWheel.setPosition(float('inf'))
    rightWheel.setPosition(float('inf'))    
    leftWheel.setVelocity(0)
    rightWheel.setVelocity(0)

    # Obtener y activar el dispositivo de la cámara    
    camera = robot.getDevice('camera')
    camera.enable(timeStep*10)


    #activar sensores ultrasónicos
    ultrasonic_sensors = []
    i = 0
    for sensor in  ultrasonic_sensors_names:      
        ultrasonic_sensors = ultrasonic_sensors + [robot.getDevice(sensor)]
        ultrasonic_sensors[i].enable(timeStep)
        i = i + 1
    print(ultrasonic_sensors_names)

     #activar sensores infrarojos
    infrared_sensors = []
    i = 0
    for sensor in  infrared_sensors_names:      
         infrared_sensors = infrared_sensors + [robot.getDevice(sensor)]
         infrared_sensors[i].enable(timeStep)
         i = i + 1
    
    return robot, camera, leftWheel, rightWheel , infrared_sensors, ultrasonic_sensors


def process_image(cameraData,prior_propia):
    """
    Procesamiento de imagen a partir del último frame capturado en el hilo principal.
    """
    W = camera.getWidth()
    H = camera.getHeight()  

    # Creamos una copia del frame actual de la cámara utilizando Numpy.
    image = np.frombuffer(cameraData, np.uint8).reshape((H, W, 4))

    g1 = 0
    g2 = 0
    g = 0
    for x in range(220,340):  #En vez de range(H), optimizamos con un rango central de líneas
        for y in range(W // 2):
            blue = image[x,y,0]
            green = image[x,y,1]
            red = image[x,y,2]
            if green > MIN_VERDE and blue < MAX_AZUL and red < MAX_ROJO:
                g1 = g1 +1
        for y in range(W//2,W):
            blue = image[x,y,0]
            green = image[x,y,1]
            red = image[x,y,2]
            if green > MIN_VERDE and blue < MAX_AZUL and red < MAX_ROJO:
                g2 = g2 +1

    g = g1 + g2

    if g > MIN_PIXELES_VERDES and g < MAX_PIXELES_VERDES:
        error_der = ((g1) /g )
        error_iz = ((g2) /g )
        ajuste_der = KP_COLOR * error_der
        ajuste_iz = KP_COLOR * error_iz
        
        semaforo.acquire()
        global prioridad
        if prioridad <= prior_propia:
            prioridad = prior_propia
            leftWheel.setVelocity(CRUISE_SPEED + ajuste_iz)
            rightWheel.setVelocity(CRUISE_SPEED + ajuste_der)
            semaforo.release()
        else:
            semaforo.release()
            return (g1, g2, g) 

    elif g >= MAX_PIXELES_VERDES:
        if prioridad <= prior_propia:
            prioridad = prior_propia
            print("acabado")
            leftWheel.setVelocity(0)
            rightWheel.setVelocity(0)
            semaforo.release()
        else:
            semaforo.release()
            return (g1, g2, g) 
        global stop
        stop = True
    else:
        if prioridad <= prior_propia:
            prioridad = 0
            semaforo.release()
        else:
            semaforo.release()
    return (g1, g2, g)              

def behaviour03(prior_propia):
    """
    Comportamiento basado en procesamiento de imagen.
    """   
    count = 0
    global stop
    while not stop:
        results = process_image(cameraData,prior_propia)
        if count > 3:
            count = 0
            print("hay estos pixeles verdes")
            print(results)
        else:
            count = count + 1

def behaviour02(prior_propia):
    """
    Seguir muros"
    """
    global stop
    while not stop:
        seguir_pared(prior_propia)

def seguir_pared(prior_propia):

    #Valores de los sensores
    izquierda = DistanceSensor.getValue(ultrasonic_sensors[0])
    delante = DistanceSensor.getValue(ultrasonic_sensors[2])
    diag_izquierda = DistanceSensor.getValue(ultrasonic_sensors[1])
    inf_izquierda = DistanceSensor.getValue(infrared_sensors[1])
    inf_delante = DistanceSensor.getValue(infrared_sensors[3])
    inf_diag_izquierda = DistanceSensor.getValue(infrared_sensors[2])
    inf_diag_atras_izquierda = DistanceSensor.getValue(infrared_sensors[0])
    
    #Detección de paredes
    close_l_wall = izquierda < L_WALL_US or diag_izquierda < L_WALL_US or inf_izquierda > L_WALL_IR or inf_diag_atras_izquierda > L_WALL_BEHIND_IR
    l_corner = (diag_izquierda < F_WALL_US or inf_diag_izquierda > F_WALL_IR)
    f_wall = (delante < F_WALL_US or inf_delante > F_WALL_IR)
    ajuste_giro = KP*(DESIRED_DISTANCE-(izquierda + diag_izquierda)/2)
    
    #Valores velocidad
    l_speed = CRUISE_SPEED
    r_speed = CRUISE_SPEED
    
    if not (close_l_wall):#Si no hay pared cerca, no se ejecuta el comportamiento de seguir pared
        semaforo.acquire()
        global prioridad
        if prioridad <= prior_propia:
               prioridad = 0
               semaforo.release()
        else:
               semaforo.release()
        return
    if f_wall or l_corner:#Si detecta pared en frente, gira a la derecha
       l_speed = CRUISE_SPEED
       r_speed = -CRUISE_SPEED 
    else:
       l_speed = CRUISE_SPEED + ajuste_giro
       r_speed = CRUISE_SPEED - AVOID_EXCEED_CRUISE_SPEED
           
    
    semaforo.acquire()
    if prioridad <= prior_propia:
        prioridad = prior_propia
        leftWheel.setVelocity(l_speed)
        rightWheel.setVelocity(r_speed)
        semaforo.release()
    else:
        semaforo.release()
        return
    
    
    return


def behaviour01(prior_propia):
    """
    Buscar pared
    """
    c = 0
    r = 0
    global stop
    while not stop:
        semaforo.acquire(1)
        global prioridad
        if prioridad <= prior_propia:
            prioridad = prior_propia
            c = encontrar_pared(c)
            c = c + (Robot.getTime(robot) - r) * TASA_AUMENTO_DE_GIRO_SPIRAL
            r = Robot.getTime(robot)
            semaforo.release()
        else:
            semaforo.release()


def encontrar_pared(c):

    delante = DistanceSensor.getValue(ultrasonic_sensors[2])
    izquierda = DistanceSensor.getValue(ultrasonic_sensors[0])
    derecha = DistanceSensor.getValue(ultrasonic_sensors[4])
    inf_delante = DistanceSensor.getValue(infrared_sensors[3])
    inf_izq = DistanceSensor.getValue(infrared_sensors[1])
    inf_der = DistanceSensor.getValue(infrared_sensors[5])
    inf_atras = DistanceSensor.getValue(infrared_sensors[7])

    #infrarojos
    if inf_delante > INF_DIST_MIN : #delante
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(CRUISE_SPEED)
    elif (inf_izq > INF_DIST_MIN or inf_atras > INF_DIST_MIN) : #izquierda o atrás
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(CRUISE_SPEED)
    elif inf_der > INF_DIST_MIN: #derecha
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(0)

        #ultrasonidos
    elif delante < ULT_DIST_MAX and (delante == min(delante,izquierda,derecha)) : #delante
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(CRUISE_SPEED)
    elif izquierda < ULT_DIST_MAX and (izquierda == min(izquierda,derecha)): #izquierda o atrás
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(CRUISE_SPEED)
    elif derecha < ULT_DIST_MAX: #derecha
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(0)

    else: #si no detecta nada gira en espiral
        leftWheel.setVelocity(CRUISE_SPEED)
        rightWheel.setVelocity(SPIRAL_SPEED + c)
        if c > MAX_SPIRAL_SPEED:
            c = 0
    return c
    
def behaviour04(prior_propia):
    """
    Escapar
    """
    global stop
    while not stop:
        escapar_pared(prior_propia)


def escapar_pared(prior_propia):
    global prioridad
    camData = camera.getImage()
    W = camera.getWidth()
    H = camera.getHeight()  

    # Creamos una copia del frame actual de la cámara utilizando Numpy.
    image = np.frombuffer(camData, np.uint8).reshape((H, W, 4))
    time.sleep(1)
    img = np.frombuffer(cameraData, np.uint8).reshape((H, W, 4))
    #Calculate mse of the 2 images to compare them, if the mse is less than 100 the escape behaviour will take effect
    
    err = np.sum((img - image)*2)
    err /= (img.shape[0] * img.shape[1])
    if  err < LIM_SAME_IMG:
        semaforo.acquire(1)
        if prioridad <= prior_propia:
            prioridad = prior_propia
            leftWheel.setVelocity(-CRUISE_SPEED)
            rightWheel.setVelocity(-CRUISE_SPEED)
            semaforo.release()
        else:
            semaforo.release()
            return  
    else: 
        semaforo.acquire(1)
        if prioridad <= prior_propia:
               prioridad = 0
               semaforo.release()
        else:
               semaforo.release()
    return 
        
        
"""
HILO PRINCIPAL
"""

# Activamos los dispositivos necesarios y obtenemos referencias a ellos.
robot, camera, leftWheel, rightWheel, infrared_sensors, ultrasonic_sensors = init_devices(TIME_STEP)

# Ejecutamos una sincronización para poder cargar el primer frame de la cámara.
robot.step(TIME_STEP)
cameraData = camera.getImage()

# Ejecutamos los threads de las tareas
thread03 = threading.Thread(target=behaviour01, args=(1,))
thread03.start()
thread02 = threading.Thread(target = behaviour02, args=(2,))
thread02.start()
thread01 = threading.Thread(target=behaviour03, args=(3,))
thread01.start()
thread04 = threading.Thread(target = behaviour04, args=(4,))
thread04.start()

# Bucle de sincronización en el hilo princiapl.
while robot.step(TIME_STEP) != -1:
    # Si vamos a utilizar la cámara actualizamos el frame actual en la variable global.
    cameraData = camera.getImage()
    time.sleep(0.01)

    if stop:
        thread01.join()
        thread02.join()
        thread03.join()
        thread04.join()
        leftWheel.setVelocity(0)
        rightWheel.setVelocity(0)
        print("FIN")
