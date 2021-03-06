

import random
import time
import argparse

from E160_environment import *
from E160_graphics import *
from pynput.keyboard import Key, Listener

def main():  
    args = parse_args()

    mode = "SIMULATION MODE"
    if args.robot:
        mode = "HARDWARE MODE"

    # instantiate robot navigation classes
    environment = E160_environment(mode)
    graphics = E160_graphics(environment)

    #with Listener(on_press=lambda x: on_press(x, graphics),
    #              on_release=lambda x: on_release(x, graphics)) as l:
    # set time step size in seconds
    deltaT = 0.07
    # loop over time
    while True:
        # update graphics, but stop the thread if user stopped the gui
        if not graphics.update():
            break
        # update robots
        environment.update_robots(deltaT)
        # log all the robot data
        environment.log_data()

        # maintain timing
        time.sleep(deltaT)

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r",
                        "--robot",
                        action="store_true",
                        help="Turn hardware mode on")
    return parser.parse_args()

def on_press(key, env):
    if key == Key.right:
        env.turn(-20)
    elif key == Key.left:
        env.turn(20)
    elif key == Key.up:
        env.forward(20)
    elif key == Key.down:
        env.forward(-20)


def on_release(key, env):
    if key == Key.right:
        env.turn(0)
    elif key == Key.left:
        env.turn(0)
    elif key == Key.up:
        env.forward(0)
    elif key == Key.down:
        env.forward(0)

main()
