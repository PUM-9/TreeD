import math
from technosoftlineardrive.tml import *
import struct

SERVO_DEFAULT_ACCELERATION = 0.1751 # IU
SERVO_DEFAULT_SPEED = 31.831 # IU

MAX_ACCELERATION = 0.3 # IU
MAX_SPEED = 170 # IU

# Here be some dragons. Kind and friendly dragons, but dragons nonetheless.

def create_linear_drive_program(pos_amount,
                                acceleration=SERVO_DEFAULT_ACCELERATION,
                                speed=SERVO_DEFAULT_SPEED):
    """
    Create a assembly tml program for the linear drive.
    
    :param pos_amount: amount to move linear drives position relatively.
    :param acceleration: how fast the linear drive will accelerate to it's desired speed.
    :param speed: how fast the linear drive will move to desired position.
    :return assemble_program: a complete assembly program to run on the linear drive.
    """
    assert (acceleration <= MAX_ACCELERATION)
    assert (speed <= MAX_SPEED)

    def pack_int(num):
        """
        Pack a integer to a struct for using in serial communication.
        
        :param num: integer to pack.
        :return num: as a packed integer for use in serial communication. 
        """
        return struct.unpack("HH", struct.pack("i", num))

    def to_fixed_point(num):
        """
        Pack a number to a fixed point type and pack it as a struct.
        
        :param num: number to pack to a fixed point type.
        :return num: as a packed fixed point for use in serial communication. 
        """
        return pack_int(math.floor(num * 0x10000)) # Magic? No.

    # Unpack into word16
    (pos_low, pos_hi) = pack_int(pos_amount)
    (accel_low, accel_hi) = to_fixed_point(acceleration)
    (speed_low, speed_hi) = to_fixed_point(speed)
    # Return the program
    return assemble_program([
        CACC(accel_low, accel_hi),
        CSPD(speed_low, speed_hi),
        CPOS(pos_low, pos_hi),
        CSET(0xDFFF, 0x0000),
        CSET(0xBFC1, 0x8701),
        CSET(0xFFFF, 0x4000),
        UPD()#,
       # NOTMC(),
       # WAIT()
    ])

