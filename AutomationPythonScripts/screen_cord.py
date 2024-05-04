#!/usr/bin/env python3

import pyautogui
import time

try:
    while True:
        x, y = pyautogui.position()
        position_str = f'X: {x} Y: {y}'
        print(position_str, end='\r')
        time.sleep(0.1)
except KeyboardInterrupt:
    print('\nDone')
