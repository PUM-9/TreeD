"""
This module contains the class LinearDrive, an abstraction layer over linear
drives built upon technosoft servos.
"""
import math
import time
import struct

import serial as s

from technosoftlineardrive.assemblyprogram import create_linear_drive_program

# Constants
TRANSFER_OK = 79
TRANSFER_BAD = 13
SYNC_DATA = 13
SYNC_OK = 13
MAX_RESYNC_ATTEMPTS = 10
UNITS_PER_MM = 167.52
UNITS_PER_MM_PER_S = 0.16752
UNITS_PER_MM_PER_S_SQUARED = 0.00016752
RAIL_LENGTH = 744.0 # mm

RESET_SPEED = 74 # mm/s
RESET_DISTANCE = 800 # mm

def assert_numerical(x):
    """
    Assert that a number is numeric. Throws an exception otherwise.
    
    :param x: object to check.
    """
    if not (type(x) is float or type(x) is int):
        raise Exception("LinearDrive was given a non-numerical value. Please send an int or a float.")

class LinearDrive:
    """
    Abstraction layer over a linear drive built upon technosoft servos.
    
    :Example:

    >>> ld = LinearDrive()
    >>> ld.goto_position(500)

    """
    def __init__(self, device_path='/dev/ttyUSB0'):
        """
        Constructor for the Linear Drive.
        
        :param device_path: serial device to use
        """
        self.device = s.Serial(device_path,
                            115200,
                            timeout=1.0)
        self.__current_position = 0
        self.speed = 200 # mm/s
        self.acceleration = 500 # mm/s^2
        self.reset_position()

    def __calculate_movement_time(self, distance):
        distance /= UNITS_PER_MM
        accel_time = self.speed / self.acceleration
        accel_dist = self.acceleration * (accel_time**2.0) / 2
        return 2*accel_time + (abs(distance) - 2*accel_dist)/self.speed

    def __send_data(self, data):
        for line in data:
            self.__send_data_line(line)


    def __send_data_line(self, data):
        data_package = struct.pack("")
        for byte in data:
            data_package += struct.pack("B", byte)

        # Send data package
        send = lambda: self.device.write(data_package)
        send()
        while self.__transfer_response() != TRANSFER_OK:
            self.__sync()
            send()


    def __sync(self):
        for _ in range(MAX_RESYNC_ATTEMPTS):
            self.device.write(struct.pack("B", SYNC_DATA))
            time.sleep(0.1)
            if self.__transfer_response() == SYNC_OK:
                return True

        raise Exception("Could not resync to linear drive")

    def __transfer_response(self):
        return struct.unpack("B", self.device.read(1))[0]

    def __move_to_unit_position(self, desired_position):
        move_amount = desired_position - self.__current_position
        self.__current_position = desired_position
        self.__move(move_amount)
        return self.__calculate_movement_time(move_amount)

    def __move(self, move_amount):
        speed = self.speed * UNITS_PER_MM_PER_S
        acceleration = self.acceleration * UNITS_PER_MM_PER_S_SQUARED
        self.__send_data(create_linear_drive_program(-move_amount,
                                                     acceleration=acceleration,
                                                     speed=speed))
    def set_speed(self, speed):
        """
        Set the speed of the linear drives movement in millimeters per second.
        
        :param speed: to move the linear drive with. 
        """
        assert_numerical(speed)
        if speed <= 0:
            raise Exception("LinearDrive cannot do non-positive speeds.")
        self.speed = speed

    def get_speed(self):
        """
        Get the currently set speed of the linear drive in millimeters per second.
        
        :return speed: the linear drive is set to move with.
        """
        
        return self.speed

    def reset_position(self):
        """
        Reset the linear drive to it's zero position. This will move the linear
        drive to the leftmost position and reset the internal counter.

        :return seconds: The number of seconds until it's safe to assume the
        camera has been reset.

        """
        # Set a separate reset speed to increase accuracy.
        pre_reset_speed = self.speed
        self.speed = RESET_SPEED

        self.__move(math.floor(-RESET_DISTANCE*UNITS_PER_MM)) # Move to negative max
        self.__current_position = 0

        maximum_time = self.__calculate_movement_time(RESET_DISTANCE*UNITS_PER_MM)

        self.speed = pre_reset_speed
        return maximum_time

    def goto_position(self, position):
        """
        Move the linear drive to the specified position. The position is specified
        in millimeters.

        :param position: to move linear drive to.
        :return time: it will take to move to position.
        """
        assert_numerical(position)
        position = min(position, RAIL_LENGTH)
        position = max(position, 0)
        position = math.floor(position * UNITS_PER_MM)
        return self.__move_to_unit_position(position)

    def goto_relative_position(self, position):
        """
        Move the linear drive to a position relative the current position. Distance
        is given in millimeters.

        :param position: to move from current position.
        :return time: it will take to move to position.
        """
        assert_numerical(position)
        return self.goto_position(self.get_current_position() + position)


    def get_current_position(self):
        """
        Return the current position of the linear drive. The position is given in
        millimeters from the linear drives leftmost position.
        
        :return postition: of the linear drive. 
        """
        return self.__current_position / UNITS_PER_MM

