import time
import colorsys
from loguru import logger as log
from drover import Drone
from threading import Thread
from pymavlink.dialects.v20 import ardupilotmega as mavlink

try:
    import board
    import neopixel
except NotImplementedError:
    class board():
        D10 = 0
    class mock_neopixel():
        fill = lambda *a: None
    class neopixel():
        NeoPixel = lambda *a: mock_neopixel()
        

class DRoverLEDs():
    """" Displays drone's state and other stuff """
    
    BLACK   = (  0,   0,   0)
    WHITE   = (255, 255, 255)
    RED     = (255,   0,   0)
    GREEN   = (  0, 255,   0)
    BLUE    = (  0,   0, 255)
    YELLOW  = (255, 200,   0)
    PINK    = (255,   0, 255)
    ORANGE  = (255, 100,   0)
    
    GUIDED_COLOR  = GREEN
    MANUAL_COLOR  = ORANGE
    RTL_COLOR     = RED
    IDLE_COLOR    = YELLOW
    UNKNOWN_COLOR = PINK
    
    def __init__(self, drone: Drone, pin=board.D10, count=18, speed=1):
        self.main_color = self.IDLE_COLOR
        self.secondary_color = self.BLACK
        self.speed = speed
        
        self._drone = drone
        self._last_flash_call = 0
        self._flash_duration = 2
        
        self.pixels = neopixel.NeoPixel(pin, count)
        self.pixels.fill(self.main_color)
        
        self._main_thread = Thread(target=self._run, daemon=True)
        self._main_thread.start()
                
    def _run(self):
        while True:
            # determine main color
            state = self._drone.state()
            if state.armed:
                if state.mode == "GUIDED":
                    self.main_color = self.GUIDED_COLOR
                elif state.mode == "RTL":
                    self.main_color = self.RTL_COLOR
                elif state.mode in ["STABILIZE", "ACRO", "ALT_HOLD", "LOITER"]:
                    self.main_color = self.MANUAL_COLOR
                else:
                    self.main_color = self.UNKNOWN_COLOR
            
            else:
                self.main_color = self.IDLE_COLOR
            
            # check if we should reset secondary color
            if (time.time()-self._last_flash_call) > self._flash_duration:
                self.secondary_color = self.BLACK
                self._last_flash_call = float('inf')
            
            # set color
            self.pixels.fill(self.main_color)
            time.sleep(self.speed*1.00)
            self.pixels.fill(self.secondary_color)
            time.sleep(self.speed*0.25)
            self.pixels.fill(self.main_color)
            time.sleep(self.speed*0.25)
            self.pixels.fill(self.secondary_color)
            time.sleep(self.speed*0.25)
            
    def flash_color(self, color, duration=2):
        self._last_flash_call = time.time()
        self._flash_duration = duration
        self.secondary_color = color
