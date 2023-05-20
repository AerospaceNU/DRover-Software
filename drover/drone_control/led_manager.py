import time
import colorsys
from loguru import logger as log
from drover import Drone
from threading import Thread, Lock
from pymavlink.dialects.v20 import ardupilotmega as mavlink

try:
    import board
    import neopixel
except:
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
    ORANGE  = (255,  80,   0)
    
    GUIDED_COLOR  = RED
    MANUAL_COLOR  = BLUE
    UNKNOWN_COLOR = RED
    RTL_COLOR     = ORANGE
    
    def __init__(self, drone: Drone, pin=board.D10, count=18, speed=2):
        self.main_color = self.WHITE
        self.secondary_color = self.BLACK
        self.speed = speed
        
        self._drone = drone
        self._count = count
        self._last_flash_call = 0
        self._flash_duration = 2
        self._lock = Lock()
        self._rgb_mode = False
        
        self.pixels = neopixel.NeoPixel(pin, count)
        self.pixels.fill(self.main_color)
        
        self._main_thread = Thread(target=self._run, daemon=True)
        self._main_thread.start()

        self._rgb_thread = Thread(target=self._run_rgb, daemon=True)
        self._rgb_thread.start()
                
    def _run(self):
        """ Handle main and secondary color flashing """
        while True:
            # determine main color
            state = self._drone.state()
            if state.mode == "GUIDED":
                self.main_color = self.GUIDED_COLOR
            elif state.mode in ["STABILIZE", "ACRO", "ALT_HOLD", "LOITER"]:
                self.main_color = self.MANUAL_COLOR
            else:
                self.main_color = self.UNKNOWN_COLOR

            if state.mode in ["RTL", "LAND"]:
                self.secondary_color = self.RTL_COLOR

            # check if we should reset secondary color
            if (time.time()-self._last_flash_call) > self._flash_duration:
                self.secondary_color = self.BLACK
                self._last_flash_call = float('inf')

            # if idle have funsies
            if state.armed:
                pix_min, pix_max = 0, self._count
                self._rgb_mode = True
            else:
                pix_min, pix_max = self._count//3, self._count*2//3
                self._rgb_mode = False

            # set main/secondary color
            with self._lock:
                 self.pixels[pix_min:pix_max] = self.main_color
            time.sleep(1.25/self.speed)
            with self._lock:
                self.pixels[pix_min:pix_max] = self.secondary_color
            time.sleep(0.25/self.speed)
            with self._lock:
                self.pixels[pix_min:pix_max] = self.main_color
            time.sleep(0.25/self.speed)
            with self._lock:
                self.pixels[pix_min:pix_max] = self.secondary_color
            time.sleep(0.25/self.speed)

    def _run_rgb(self):
        """ RGB flashies """
        while True:
            if not self._rgb_mode:
                time.sleep(0.1)
            
            r, g, b = colorsys.hsv_to_rgb( (time.time()/2/self.speed) % 1 )
            self.pixels[0:self._count//3] = (int(r*255),int(g*255),int(b*255))
            self.pixels[self._count*2//3:-1] = (int(r*255),int(g*255),int(b*255))
            time.sleep(1.0/255) 
            
                

    def flash_color(self, color, duration=1):
        self._last_flash_call = time.time()
        self._flash_duration = duration
        self.secondary_color = color
