from os import chdir
from typing import List

try:
	import dill as pickle
except:
	import pickle
import pygame

from networktables import NetworkTables
import socket

import sys
sys.path.append('.')
sys.path.append('..')
from subsystems.indexer import Ball, BallLocation

local_ip = socket.gethostbyname(socket.gethostname())
remote_ip = '10.40.96.2' if '40.96' in local_ip else '127.0.0.1'
NetworkTables.initialize(server=remote_ip)
nt_robot = NetworkTables.getTable("Robot")

pygame.init()

# Define the colors we will use in RGB format
BLACK = (  0,   0,   0)
WHITE = (255, 255, 255)
GRAY = (160, 160, 160)
BLUE =  (  0,   0, 255)
RED =   (255,   0,   0)

colors = {
    "red": RED,
    "blue": BLUE,
}

size = [400, 300]
screen = pygame.display.set_mode(size)

pygame.display.set_caption("Indexer balls")

clock = pygame.time.Clock()

X = 100
Y = 100

prev_r_balls = None

while True:
    pygame.display.flip()
    clock.tick(10)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

    screen.fill(GRAY)

    pygame.draw.line(screen, BLACK, [X+ 0, Y+0], [X+0,Y+160], 5)
    pygame.draw.line(screen, BLACK, [X+ 80, Y+0], [X+80,Y+80], 5)
    pygame.draw.line(screen, BLACK, [X+ 80, Y+80], [X+160,Y+80], 5)
    pygame.draw.line(screen, BLACK, [X+ 0, Y+160], [X+160,Y+160], 5)

    r_balls = nt_robot.getRaw("indexer balls", None)
    if r_balls is None:
        continue


    balls: List[Ball] = pickle.loads(r_balls)

    for ball in balls:
        if ball.location == BallLocation.LOWEST:
            # low
            pygame.draw.circle(screen, colors[ball.color], [X+120,Y+120], 30, 5)

        if ball.location == BallLocation.FLOATING:

            #float
            pygame.draw.circle(screen, colors[ball.color], [X+40,Y+120], 30, 5)

        if ball.location == BallLocation.TOP:
            #top
            pygame.draw.circle(screen, colors[ball.color], [X+40,Y+40], 30, 5)


