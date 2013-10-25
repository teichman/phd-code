#!/usr/bin/python

from rocket_backend import RocketManager
import thread
import time

class RocketDriver:
    def __init__(self):
        self.rm_ = RocketManager()
        self.rm_.acquire_devices()
        assert(len(self.rm_.launchers) == 1)
        self.launcher_ = self.rm_.launchers[0]

    def left(self, seconds):
        thread.start_new_thread(self.launcher_.start_movement, (2,))
        time.sleep(seconds)
        self.launcher_.stop_movement()
    
    def right(self, seconds):
        thread.start_new_thread(self.launcher_.start_movement, (3,))
        time.sleep(seconds)
        self.launcher_.stop_movement()

    def up(self, seconds):
        thread.start_new_thread(self.launcher_.start_movement, (1,))
        time.sleep(seconds)
        self.launcher_.stop_movement()

    def down(self, seconds):
        thread.start_new_thread(self.launcher_.start_movement, (0,))
        time.sleep(seconds)
        self.launcher_.stop_movement()

    def fire(self):
        self.launcher_.start_movement(4)
        time.sleep(4)


if __name__ == "__main__":
    rd = RocketDriver()
    rd.left(0.3)
    time.sleep(0.2)
    rd.right(0.3)
    time.sleep(0.2)
    rd.up(0.3)
    time.sleep(0.2)
    rd.down(0.3)
    time.sleep(0.2)
    rd.fire()
