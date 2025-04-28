import threading
import pygame
import copy
from utils import getCenterVar, lerpVector, BLACK, WHITE
from extractPointScan import fakeScan, oneScan


# Queue pour recevoir les données depuis le thread de scan
scan_queue = []

def packScans():
    """
    Fonction qui va recevoir les scans et les mettre dans une queue regrouper par groupe de scan 
    pour être traités par le thread principal
    """
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

# initialisation
WINDOW_SIZE = 700
pygame.init()
screen = pygame.display.set_mode([WINDOW_SIZE,WINDOW_SIZE])
screen.fill(BLACK)
pygame.display.update()
running = True
centre = (.5, .5)
SCALE = 1.25
points = []

# On crée un thread pour le scan pour séparer de l'affichage
scan_thread = threading.Thread(target=packScans, daemon=True)
scan_thread.start()

while (running):
    # Récupération des événements si on clique sur la croix de la fenêtre ou sur echap on quitte
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                running = False

    # Récupération des données disponibles dans la queue
    while not len(scan_queue)==0: # On récupère tous les scans disponibles dans la queue
        points = scan_queue.pop() # On récupère le dernier scan de la queue
    screen.fill(BLACK) # On vide l'écran
    if len(points) > 0: # Si on a des points à afficher
        trueCentre, var = getCenterVar(points) # On récupère le centre et la variance des points pour centrer l'affichage
        centre = lerpVector(trueCentre, centre, 0.99) # On interpole le centre pour avoir un affichage plus fluide

        for point in points: # On parcourt tous les points du scan
            centeredPoint = (point[1]-centre[0], point[2]-centre[1]) # On centre le point par rapport au centre
            reducedPoint = (centeredPoint[0]/(var*SCALE), centeredPoint[1]/(var*SCALE))
            rect = pygame.Rect( (reducedPoint[0])*WINDOW_SIZE + WINDOW_SIZE/2,
                                (reducedPoint[1])*WINDOW_SIZE + WINDOW_SIZE/2,
                                1,
                                1) # On crée un rectangle pour le point
            pygame.draw.rect(screen, WHITE, rect, 0) # On dessine le point

    # Affichage
    pygame.display.update()