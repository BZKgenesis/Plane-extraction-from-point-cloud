import random
import math
import pyautogui
import screeninfo
from pygame import Color 

MONITOR_INDEX = 0
BLACK = Color(0,0,0)
WHITE = Color(255,255,255)

monitor = screeninfo.get_monitors()[MONITOR_INDEX]

SCREEN_SIZE = (monitor.width, monitor.height)


def getCenterVar(points):
    minX = float("inf")
    minY = float("inf")
    maxX = float("-inf")
    maxY = float("-inf")
    for point in points:
        if point[1] < minX:
            minX = point[1]
        if point[1] > maxX:
            maxX = point[1]
        if point[2] < minY:
            minY = point[2]
        if point[2] > maxY:
            maxY = point[2]
    var = max(max(minX-(minX + maxX) /2,maxX-(minX + maxX) /2),max(minY-(minY + maxY) /2,maxY-(minY + maxY) /2))
    return (((minX + maxX) /2, (minY + maxY) /2), var*2)

def lerpVector(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)

def getRandomPoint(center, radius, radiusMaxOffset):
    MOUSE_OFFSET_SIZE = 1
    angle = random.uniform(0, 2 * 3.14)  # Angle in radians
    posMouse = ((pyautogui.position()[0]-(SCREEN_SIZE[0]/2))/(SCREEN_SIZE[0]/2), (pyautogui.position()[1]-(SCREEN_SIZE[1]/2))/(SCREEN_SIZE[1]/2))
    angleMouse = math.atan2(posMouse[1], posMouse[0]) % (2 * math.pi)
    #print(angleMouse, angle)
    raw_r=max(- 2*min(abs(angleMouse - angle),abs(angleMouse + math.pi*2 - angle),abs(angleMouse-math.pi*2 - angle))+radius+MOUSE_OFFSET_SIZE,radius)
    r = raw_r + random.uniform(-radiusMaxOffset, radiusMaxOffset)  # Random distance from the center
    return (center[0] + r * math.cos(angle), center[1] + r * math.sin(angle))


