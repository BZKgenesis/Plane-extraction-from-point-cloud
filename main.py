import threading
import pygame
import copy
from utils import getCenterVar, lerpVector
from extractPointScan import fakeScan, oneScan

BLACK = pygame.Color(0,0,0)
WHITE = pygame.Color(255,255,255)

# Queue pour recevoir les données depuis le thread de scan
scan_queue = []

def packScans():
    currentScan = []
    scanIndex = 0
    scan = fakeScan()
    #scan = oneScan()
    for scan in scan:
        if scanIndex != scan[0]:
            scan_queue.append(copy.deepcopy(currentScan))
            currentScan.clear()
            scanIndex = scan[0]
        currentScan.append((scan[0], scan[1], scan[2], scan[3], scan[4]))

SCREEN_SIZE = 700
pygame.init()
screen = pygame.display.set_mode([SCREEN_SIZE,SCREEN_SIZE])
screen.fill(BLACK)
pygame.display.update()

scan_thread = threading.Thread(target=packScans, daemon=True)
scan_thread.start()

running = True

centre = (.5, .5)

SCALE = 1.25

points = []

while (running):
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # Récupération des données disponibles dans la queue
    while not len(scan_queue)==0:
        points = scan_queue.pop()
    screen.fill(BLACK)
    if len(points) > 0:
        trueCentre, var = getCenterVar(points)
        centre = lerpVector(trueCentre, centre, 0.99)

        for point in points:
            centeredPoint = (point[1]-centre[0], point[2]-centre[1])
            reducedPoint = (centeredPoint[0]/(var*SCALE), centeredPoint[1]/(var*SCALE))
            rect = pygame.Rect((reducedPoint[0])*SCREEN_SIZE + SCREEN_SIZE/2, (reducedPoint[1])*SCREEN_SIZE + SCREEN_SIZE/2, 1, 1)
            pygame.draw.rect(screen, WHITE, rect, 0)

    # Affichage
    pygame.display.update()