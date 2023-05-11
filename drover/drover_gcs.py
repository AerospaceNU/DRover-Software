import sys
import cmd
from typing import IO
from drover import GCSComms, Waypoint
from loguru import logger as log


# init pretty logging
logger_format = (
    # "<green>{time:HH:mm:ss.SSS}</green> | "
    # "<level>{level: <8}</level> | "
    "<level>{message}</level>"
)
log.remove()
log.add(sys.stderr, level="DEBUG", format=logger_format)


# command line shell thing, see https://docs.python.org/3/library/cmd.html
class GCShell(cmd.Cmd):
    intro = 'Welcome to DRover GCS.   Type help or ? to list commands.\n'
    prompt = 'DRover> '
    file = None
    
    def __init__(self,
                 comms: GCSComms = GCSComms(),
                 completekey: str = "tab", 
                 stdin: IO[str] | None = None, 
                 stdout: IO[str] | None = None):
        super().__init__(completekey, stdin, stdout)
        self._comms = comms
        
        self._waypoints = {}

    def do_upload(self, args):
        """ Upload the stored mission to DRover (no args) """
        # sort waypoints by number and make into a list
        wps = [v for k, v in sorted(self._waypoints.items())]
        self._comms.send_waypoints(wps)

    def do_reset(self, args):
        """ Resets the stored waypoints (no args)"""
        self._waypoints = {}

    def do_remove(self, args):
        """ Removes a waypoint (remove <wp_num>)"""
        wp = self._waypoints.pop(int(args.split()[0]), None)
        if wp is None:
            log.error("WP not found")

    def do_lla(self, args):
        """Adds an LLA waypoint to the list\n(add <wp_num> <lat> <lon> <alt> [aruco1_id] [aruco2_id])"""
        args = args.split()
        wp = Waypoint(float(args[1]),
                      float(args[2]),
                      float(args[3]),
                      aruco_id=None if len(args)<5 else int(args[4]),
                      aruco2_id=None if len(args)<6 else int(args[5]),
                      use_latlon=True)
        
        self._waypoints[int(args[0])] = wp
        log.info(wp)

    def do_neu(self, args):
        """Adds an NEU waypoint to the list\n(add <wp_num> <north> <east> <alt> [aruco1_id] [aruco2_id])"""
        args = args.split()
        wp = Waypoint(float(args[1]),
                      float(args[2]),
                      float(args[3]),
                      aruco_id=None if len(args)<5 else int(args[4]),
                      aruco2_id=None if len(args)<6 else int(args[5]),
                      use_latlon=False)
        
        self._waypoints[int(args[0])] = wp
        log.info(wp)

    def do_start(self, args):
        """Starts the mission with configurable values\n(start [start_radius] [end_radius] [speed] [laps] [max_dps])"""
        args = [float(x) for x in args.split()]
        
        if len(args) == 0:
            self._comms.send_start_signal(*args)
        elif len(args) == 5:
            self._comms.send_start_signal()
        else:
            log.error("start command needs 0 or 5 args")


    def do_print(self, args):
        """ Prints the list of waypoints """
        for key, wp in self._waypoints.items():
            print(f"{key}:  {wp}")

    def do_EOF(self, args):
        exit()
    def do_quit(self, args):
        exit()

    #TODO PLAY_TUNE_V2        

if __name__ == "__main__":
    try:
        GCShell().cmdloop()
    except Exception as e:
        log.exception(e)
    except KeyboardInterrupt as e:
        log.error("Keyboard interrupt")