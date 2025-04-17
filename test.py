import RPyLIDAR
import threading
import queue
import random
import time
import pygame
import copy

BLACK = pygame.Color(0,0,0)
WHITE = pygame.Color(255,255,255)

# Queue pour recevoir les données depuis le thread de scan
scan_queue = []

def fakeScan():
    scanIndex = 0
    currentScan = []
    while True:
        for i in range(180):
            time.sleep(0.01)
            currentScan.append((scanIndex, random.random(), random.random(), 0, 47))
        scan_queue.append(copy.deepcopy(currentScan))
        currentScan.clear()
        scanIndex += 1
screenSize = 700
pygame.init()
screen = pygame.display.set_mode([screenSize,screenSize])
screen.fill(BLACK)
pygame.display.update()

scan_thread = threading.Thread(target=fakeScan, daemon=True)
scan_thread.start()

# sensor.start_express_scan()
running = True

while (running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    
    # Récupération des données disponibles dans la queue
    while not len(scan_queue)==0:
        print(len(scan_queue[0]))
        points = scan_queue.pop()
        screen.fill(BLACK)
        for point in points:
            rect = pygame.Rect(point[1]*screenSize, point[2]*screenSize, 2, 2)
            pygame.draw.rect(screen, WHITE, rect, 0)

    # Affichage
    pygame.display.update()


def trueScan():
    
    try:
        sensor = RPyLIDAR.RPyLIDAR('COM3',    115200,       1,         700,      False,       True)
        sensor.connect()
        sensor.stop_scan()
        sensor.stop_motor()

        sensor.get_device_info()
        sensor.get_device_health()
        sensor.get_sample_rate()

        #sensor.start_motor_slow()
        #sensor.start_motor_14()
        #sensor.start_motor_half()
        sensor.start_motor_34()
        #sensor.start_motor_max()

        yield sensor.start_standard_scan() # Count, X, Y, Z, quality




        sensor.stop_scan()
        sensor.stop_motor()
        sensor.disconnect()

    except:
        print(" *** AN UNKNOWN ERROR OCCURRED *** ")
        sensor.stop_scan()
        sensor.stop_motor()
        sensor.disconnect()
    