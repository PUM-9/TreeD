__author__ = 'Fabian Petersen'

from ctypes import *
import os
from time import sleep


class PhotoBoard():
    def __init__(self, dll_path, dll_name):
        self._photoboard_dll = windll.LoadLibrary(os.path.join(dll_path, dll_name))
        self.is_open = False
        self.rotation = 0
        self.curvature = 0

    def __enter__(self):
        """
        Function that is called at the beginning when using the 'with' statement.
        The 'with' statement help the users to write better code

        :return: A Photoboard object with a established connection to the hardware
        """
        self.open()
        return self

    def __exit__(self):
        """
        Function that is called at the very end when using the 'with' statement.
        The 'with' statement help the users to write better code

        :return:
        """
        if self.is_open:
            self.close()

    def test(self):
        '''
        Loops through all angles and curvatures with a 1 second delay
        between them. This can be used to verify that the PhotoBoard
        is operational
        
        :return:
        '''
        curvature = 89
        while curvature > -20:
            rotation = 358
            while rotation > 0:
                self.move(curvature, rotation)
                rotation -= 40
                print("Curve: " + str(curvature) + " Rotatation: " + str(rotation))
                sleep(1)
            curvature -= 10


    def open(self, port=2):
        """
        Open a connection to the PhotoBoard hardware

        :param port: The port to use for the connection
        :return:
        """
        if not self.is_open:
            self._photoboard_dll.PTOpen(port)
            self.is_open = True
            self.reference_point()
        else:
            raise PhotoBoard.NoConnectionError("Connection already established to the PhotoBoard hardware i.e.\
                                                Call to photoboard.open() already made")

    def reference_point(self):
        """
        Will reset the Reference Point of the PhotoBoard hardware.

        On startup the reference point is the current position,
        therefore all move commands will be based around the current position
        instead of what we might percive as the standard.

        Failure to call this function will result in all angels and curvatures
        becoming offset.

        :return:
        """
        if self.is_open:
            self._photoboard_dll.PTRefPoint()
            self.rotation = 0
            self.curvature = 0
        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")

    def close(self):
        """
        Close the connection to the PhotoBoard hardware.

        Failure to call this function when shutting of the system
        might make the hardware unresponsive until it is restarted

        :return:
        """
        if self.is_open:
            self._photoboard_dll.PTClose()
            self.is_open = False
        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")

    def set_rotation(self, rotation):
        """
        Will move the PhotoBoard to the desired position

        :param rotation: The rotation for the PhotoBoard to assume,
            the valid range is 359 >= rotation >= 0
        :return:
        """
        self.rotation = rotation 
        if self.is_open:
            if 90 >= self.curvature >= -20 and 359 >= self.rotation >= 0:
                self._photoboard_dll.PTMove(self.curvature, self.rotation)
            else:
                raise ValueError("The valid range for the curvature is between 90 and -20,\
                                the valid range for the rotation is from 359 to 0")

        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")


    def get_rotation(self):
        """
        Get the rotation of the PhotoBoard

        :return: The rotation of the PhotoBoard,
            the valid range is 90 >= curvature >= -20
        """
        if self.is_open:        
            return self.rotation
        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")
        
    def get_curvature(self):
        """
        Get the curvature of the PhotoBoard

        :return: The curvature of the PhotoBoard,
            the valid range is 90 >= curvature >= -20
        """
        
        if self.is_open:        
            return self.curvature
        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")
        
    def set_curvature(self, curvature):
        """
        Will move the PhotoBoard to the desired position

        :param curvature: The curvature for the PhotoBoard to assume,
            the valid range is 90 >= curvature >= -20
        :return:
        """
        self.curvature = curvature 
        if self.is_open:
            if 90 >= self.curvature >= -20 and 359 >= self.rotation >= 0:
                self._photoboard_dll.PTMove(self.curvature, self.rotation)
            else:
                raise ValueError("The valid range for the curvature is between 90 and -20,\
                                the valid range for the rotation is from 359 to 0")
        else:
            raise PhotoBoard.NoConnectionError("No connection is established to the PhotoBoard hardware i.e.\
                                                Failure to call photoboard.open()")

    @staticmethod
    class NoConnectionError(RuntimeError):
        """
        Raise this when someone is trying to use a function without first connecting to the PhotoBoard Hardware
        """
    
    @staticmethod
    class ConnectionEstablishedError(RuntimeError):
        """
        Raise this when someone is trying to open a connection to the PhotoBoard Hardware when one is already established
        """
        
if __name__ == '__main__':
    with PhotoBoard('c:\\Fotobord\\Fotobord\\FotoBordDLL\\Debug\\', 'FotoBordDLL') as p:
        p.test()
