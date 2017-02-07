"""
File: sickicp_hardware_wrapper.py
Authors: Johan Jansson, Niklas Hätty

File containing the hardware wrapper for a SICK IVP Ruler E600(E2111).

This wrapper contains the Ruler class used for control of the
camera Ruler E. The wrapper is an abraction of the RulerNet API
using the iCon 4.0 library. Please note that this wrapper does not make
use of the iCon 4.0 API.

For reference documentation, see:
    - Ruler_E_Reference_Manual.pdf
    - ruler_c_api.chm 

"""
import os

from ctypes import *
from enum import Enum

_author_ = 'Johan Jansson, Niklas Hätty'

# Error code handling and probable causes of them
error_codes = {
    0: 'IVP_E_ALL_OK',
    1: 'IVP_E_CAMERA_NOT_INITED: The camera has not been properly '
    'initialized.',
    2: 'IVP_E_IN VALID_PRM_PATH: Make sure the .prm path is correct',
    3: 'IVP_E_CAMERA_NOT_RESPONDING: The camera did not respond.',
    4: 'IVP_E_INVALID_CAMERA_REPLY',
    5: 'IVP_E_INVALID_PRM_FILE: The .prm file was invalid.',
    6: 'IVP_E_FILE_ERROR: Could not open the .prm file',
    7: 'IVP_E_PARAMETER_NOT_FOUND',
    8: 'IVP_E_ALREADY_INITED: The operation cannot be done when the camera is '
    'already initialized.',
    9: 'IVP_E_PARAMETER_ERROR',
    10: 'IVP_E_MEASURE_ALREADY_RUNNING: There is already a measurement '
    'running.',
    11: 'IVP_E_MEASURE_NOT_RUNNING: There is no measurement running',
    12: 'IVP_E_LOOP_GENERATION_FAILED',
    13: 'IVP_E_CAMERA_PARAMETER_ERROR',
    14: 'IVP_E_CAMERA_STATUS_NOT_OK',
    15: 'IVP_E_INVALID_CONFIGURATION',
    16: 'IVP_E_ARRAY_SIZE',
    17: 'IVP_E_TIMEOUT: Function timed out while trying to read buffers. '
        'This typically means the buffer was empty.',
    18: 'IVP_E_PARTIAL_DATA',
    19: 'IVP_E_UNSUPPORTED_CAMERA_TYPE',
    20: 'IVP_E_MODE_ERROR: The mode must be either "Measurement" or "Image"',
    21: 'IVP_E_FIFO_FULL: Buffer was full. Profiles were lost because the '
        'buffer was full'
}

# Camera related errors. Integers corresponding to error_codes
#  dictionary
camera_error = [1, 3, 4, 8, 10, 11, 12, 13, 14, 15, 17, 18, 19]

# Input related errors. Integers corresponding to error_codes
#  dictionary
input_error = [2, 5, 6, 7, 9]

# Miscellaneous errors. Integers corresponding to error_codes
#  dictionary
misc_error = [20, 21]


class RulerState(Enum):
    """
    The API can be in three different states:
        - NOTINITIALIZED: The camera has not been intialized
        - STOPPED: Initialized but not measuring
        - STARTED: Initialized and measuring.
    """
    
    NOTINITIALIZED = 0
    STOPPED = 1
    STARTED = 2


class CameraState(Enum):
    """
    The camera can be in different states. These should correspond to state
    of the API with some extra states.
    """
    PING_NOTINITIALIZED = 0
    PING_STOPPED = 1
    PING_WAITINGFORENABLE = 2
    PING_RUNNING = 3
    PING_HALTED = 4
    PING_ERROR = 5
    PING_UNKNOWN = 6


class CameraError(Exception):
    """
    Errors that relate to the camera.
    """
    pass


class InputError(Exception):
    """
    Errors that relate to faulty inputs.
    """

    pass


def configure_ruler():
    """
    Help function to make initialization easier.
    :return: Ruler object.
    """
    prof_per_buf = 256
    timeout = 500
    buffer_size = 50

    ruler = Ruler('C:\\treeD\\', 'icon_c_40')
    ruler.initialize_ruler('192.168.0.12', 'C:\\treeD\\settings.prm',
                           timeout, prof_per_buf, buffer_size)

    return ruler


class Ruler:
    """
    Main class for a ruler object
    """

    def __init__(self, dll_path, dll_name):
        self._ruler_dll = windll.LoadLibrary(os.path.join(dll_path,
                                                          dll_name))
        self.handle = self._ruler_dll.Ruler_createRuler(bytes('ruler_handle',
                                                              'utf8'))

    def __enter__(self):
        return self

    def __exit__(self):
        return self

    def __check_for_error(self, res, operation):
        """
        :param res: Error code returned from API calls.
        :param operation: String message to clarify what operation was tried.
        :return: None
        """
        if res != 0:
            if res in camera_error:
                raise CameraError(operation + ' failed \n' + error_codes[res])
            elif res in input_error:
                raise InputError(operation + ' failed \n' + error_codes[res])
            elif res in misc_error:
                raise Exception(operation + ' failed \n' + error_codes[res])

        elif res == 0:
            print(operation, 'successful:', error_codes[res])

        else:
            raise Exception(operation + ' failed \n Unknown error code: ' + res)

    def initialize_ruler(self, ruler_ip, parameter_file, timeout,
                         profiles_per_buffer, buffer_size):
        """
        Initialize a ruler with specified settings.
        If successful, state: NOTINITIALIZED -> STOPPED

        :param ruler_ip: ip address of camera.
        :param parameter_file: .prm file for configuration.
        :param timeout: time until initializing will time out.
        :param profiles_per_buffer: number of profiles will contain
        :param buffer_size: size of buffer [MB]
        :return: None
        """

        # Configure settings not specified in the parameter file
        # Since the api expects bytes and not strings as inputs,
        # they are converted.
        log_path = c_char_p(bytes('log.log', 'utf-8'))
        self._ruler_dll.Ruler_openLogFile(log_path)
        
        try:
            ip = c_char_p(bytes(ruler_ip, 'utf-8'))
        except TypeError as e:
            raise TypeError('Invalid type for IP-address (should be '
                            'string).').with_traceback(e.__traceback__)
        
        res = self._ruler_dll.Ruler_setCameraAddress(self.handle, ip)
        self.__check_for_error(res, 'Set camera address')
        
        try:
            res = self._ruler_dll.Ruler_setParameterFile(self.handle,
                                                         bytes(parameter_file,
                                                               'utf-8'))
        except TypeError as e:
            raise TypeError('Invalid type for parameter file (should be '
                            'string).').with_traceback(e.__traceback__)
        
        self.__check_for_error(res, 'Set parameter file')

        if isinstance(timeout, int) and timeout > -2:
            res = self._ruler_dll.Ruler_setTimeout(self.handle, timeout)
        else:
            raise Exception(
                    "Invalid timeout, has to be integer: -1 (unlimited),"
                    " 0 (no timeout)"
                    " or positive value (wait for set amount of time)")

        self.__check_for_error(res, 'Set timeout')

        if isinstance(profiles_per_buffer, int) and 0 < profiles_per_buffer \
                < 65537:
            res = self._ruler_dll.Ruler_setProfilesPerBuffer(self.handle,
                                                             profiles_per_buffer)
        else:
            raise Exception(
                "Invalid profiles per buffer. Make sure it is an"
                " integer between "
                "1 and 65536")

        self.__check_for_error(res, 'Set profiles per buffer')

        if isinstance(buffer_size, int) and buffer_size > 0:
            res = self._ruler_dll.Ruler_setMemoryBufferSize(self.handle,
                                                            buffer_size)
        else:
            raise Exception("The buffer size has to be an integer "
                            "and bigger than 0")

        self.__check_for_error(res, 'Set buffer size')
        res = self._ruler_dll.Ruler_setRulerMode(self.handle,
                                                 bytes('Measurement', 'utf-8'))
        self.__check_for_error(res, 'Set ruler mode')
        res = self._ruler_dll.Ruler_init(self.handle)
        self.__check_for_error(res, 'Initialization')

    def close(self):
        """
        Close an already initialized ruler
        If successful, state: STOPPED -> NOTINITIALIZED

        :return: None
        """

        res = self._ruler_dll.Ruler_close(self.handle)
        self.__check_for_error(res, 'Ruler close')

    def start(self):
        """
         Start an already initialized ruler.
         If successful, state: STOPPED -> STARTED

        :return: None
        """

        res = self._ruler_dll.Ruler_start(self.handle)
        self.__check_for_error(res, 'Ruler start')

    def stop(self):
        """
        Stop a running ruler.
        If successful, state: STARTED -> STOPPED

        :return: None
        """

        res = self._ruler_dll.Ruler_stop(self.handle)
        self.__check_for_error(res, 'Ruler stop')

    def delete_ruler(self):
        """
        Destructor of a ruler object.
        If successful, this will destroy the handle.
        :return: None
        """

        res = self._ruler_dll.Ruler_deleteRuler(self.handle)
        self.__check_for_error(res, 'Ruler delete')

    def get_available_buffers(self):
        """
        Returns the amount of available buffers that the camera has gotten so
        far.
        :return: Returns the current amount of buffers available for retrieval
        """
        avail_buf_ptr = POINTER(c_int)(c_int())
        res = self._ruler_dll.Ruler_getAvailableBuffers(self.handle,
                                                        avail_buf_ptr)
        self.__check_for_error(res, 'Get available buffers')
        available_buffers = avail_buf_ptr[0]

        return available_buffers

    def read_buffer(self):
        """
        Requests data from a STARTED ruler. It will gather data
        from one buffer
        :return: A list containing that data from one buffer.
        """
        # Get ProfilesPerBuffer and width
        # Since the out parameter is an int pointer in C, we have to create
        # a buffer to store the output.
        profiles = create_string_buffer(4)
        res = self._ruler_dll.Ruler_getProfilesPerBuffer(self.handle,
                                                         profiles)
        self.__check_for_error(res, 'Get profiles per buffer')
        profiles_per_buf = int.from_bytes(profiles.raw, byteorder='little')
        width_ptr = POINTER(c_int)(c_int())
        res = self._ruler_dll.Ruler_getProfileWidth(self.handle, width_ptr)
        self.__check_for_error(res, 'Get profile width')
        profile_width = width_ptr[0]
        print('prof width', profile_width)

        # We will have to create C arrays of sizes specified in the
        # ruler_c_api.
        # These are used as out parameters when requesting data.
        IntArr = c_int * profiles_per_buf
        status_arr = IntArr()
        mark_arr = IntArr()
        id_arr = IntArr()

        FloatArr = c_float * (profiles_per_buf * profile_width)
        range_arr = FloatArr()
        x_arr = FloatArr()

        CharArr = c_ubyte * (profiles_per_buf * profile_width)
        intensity_arr = CharArr()

        res = self._ruler_dll.Ruler_requestDataSeparate(self.handle,
                                                        profiles_per_buf,
                                                        profiles_per_buf *
                                                        profile_width,
                                                        id_arr,
                                                        status_arr,
                                                        mark_arr,
                                                        x_arr,
                                                        range_arr,
                                                        intensity_arr,
                                                        None)
        data = [id_arr, status_arr, mark_arr, x_arr, range_arr,
                intensity_arr]

        self.__check_for_error(res, 'Request data from buffer')

        def conv_arr_to_lst(func, data):
            lst = []
            for item in range(len(data)):
                lst.append(data[item])
            return lst

        res_lst = []
        for item in range(len(data)):
            res_lst.append(conv_arr_to_lst(lambda x: (x), data[item]))

        return res_lst

    def read_all_buffers(self):
        """
        Requests data from a STARTED ruler. It will gather all data
        that is stored in the camera buffer.
        :return: List of data output list. Each element is data from
                 a buffer.
        """

        # Get ProfilesPerBuffer and width
        # Since the out parameter is an int pointer in C, we have to create
        # a buffer to store the output.
        profiles = create_string_buffer(4)
        res = self._ruler_dll.Ruler_getProfilesPerBuffer(self.handle,
                                                         profiles)
        self.__check_for_error(res, 'Get profiles per buffer')
        profiles_per_buf = int.from_bytes(profiles.raw, byteorder='little')
        width_ptr = POINTER(c_int)(c_int())
        res = self._ruler_dll.Ruler_getProfileWidth(self.handle, width_ptr)
        self.__check_for_error(res, 'Get profile width')
        profile_width = width_ptr[0]

        avail_buf_ptr = POINTER(c_int)(c_int())
        res = self._ruler_dll.Ruler_getAvailableBuffers(self.handle,
                                                        avail_buf_ptr)
        self.__check_for_error(res, 'Get available buffers')
        available_buffers = avail_buf_ptr[0]
        print("Available buffers: ", available_buffers)

        data_buffers = []

        # Gather data stored in all available buffers. Store them in
        # data_buffers list
        for i in range(available_buffers):
            # We will have to create C arrays of sizes specified in the
            # ruler_c_api.
            # These are used as out parameters when requesting data.
            IntArr = c_int * profiles_per_buf
            status_arr = IntArr()
            mark_arr = IntArr()
            id_arr = IntArr()

            FloatArr = c_float * (profiles_per_buf * profile_width)
            range_arr = FloatArr()
            x_arr = FloatArr()

            CharArr = c_ubyte * (profiles_per_buf * profile_width)
            intensity_arr = CharArr()

            res = self._ruler_dll.Ruler_requestDataSeparate(self.handle,
                                                            profiles_per_buf,
                                                            profiles_per_buf *
                                                            profile_width,
                                                            id_arr,
                                                            status_arr,
                                                            mark_arr,
                                                            x_arr,
                                                            range_arr,
                                                            intensity_arr,
                                                            None)
            data_temp = [id_arr, status_arr, mark_arr, x_arr, range_arr,
                         intensity_arr]

            self.__check_for_error(res, 'Request data from buffer')

            data_buffers.append(data_temp)

        def conv_arr_to_lst(func, data):
            lst = []
            for item in range(len(data)):
                lst.append(data[item])
            return lst

        res_lst = []
        for item in range(len(data_buffers)):
            res_lsts = []
            for lists in range(len(data_buffers[item])):
                res_lsts.append(conv_arr_to_lst(lambda x: float(x),
                                                data_buffers[item][lists]))
            res_lst.append(res_lsts)

        return res_lst

    def empty_camera_buffers(self):
        """
        Empty all buffers that store the ruler data.
        :return: None
        """
        avail_buf_ptr = POINTER(c_int)(c_int())
        res = self._ruler_dll.Ruler_getAvailableBuffers(self.handle,
                                                        avail_buf_ptr)
        self.__check_for_error(res, 'Get available buffers')
        available_buffers = avail_buf_ptr[0]
        print("Available buffers: ", available_buffers)
        while available_buffers > 0:
            res = self._ruler_dll.Ruler_requestDataSeparate(self.handle,
                                                            None,
                                                            None,
                                                            None,
                                                            None,
                                                            None,
                                                            None,
                                                            None,
                                                            None,
                                                            None)
            self.__check_for_error(res, 'Request data from buffer')
            print("Available buffers: ", available_buffers)

        print("Available buffers: ", available_buffers)

    def ping(self):
        """
        Ping CameraState of the camera and prints it.
        :return: None:
        """

        state_buffer = create_string_buffer(4)
        self._ruler_dll.Ruler_ping(self.handle, state_buffer)
        state = int.from_bytes(state_buffer.raw, byteorder='little')
        print('Camera ping status: ', CameraState(state).name)

    def print_api_state(self):
        """
        Print the status of the API ruler_c_api
        :return: None
        """

        state = create_string_buffer(4)
        self._ruler_dll.Ruler_getState(self.handle, state)
        state = int.from_bytes(state.raw, byteorder='little')
        print('API status: ', RulerState(state).name)


def create_point_cloud(data, ruler_object):
    """
    Help function to create a point cloud from data buffers.
    :param data: data object from read_all_buffers() that contains all
    data buffers from a scan.
    :param ruler_object: Ruler object.
    :return: None 
    """
    prof_per_buf = len(data[0][0])
    available_buffers = len(data)
    INDEX_OF_X = 3
    INDEX_OF_RANGE = 4

    width_ptr = POINTER(c_int)(c_int())
    res = ruler_object._ruler_dll.Ruler_getProfileWidth(ruler_object.handle,
                                                        width_ptr)
    prof_width = width_ptr[0]

    file = open('output.pcd', 'w')

    outputstr = []
    outputstr.append("VERSION .7\n")
    outputstr.append("FIELDS x y z\n")
    outputstr.append("SIZE 4 4 4\n")
    outputstr.append("TYPE F F F\n")
    outputstr.append("WIDTH ")
    outputstr.append(str(prof_per_buf * available_buffers))
    outputstr.append("\n")
    outputstr.append("HEIGHT ")
    outputstr.append(str(prof_width))
    outputstr.append("\n")
    outputstr.append("VIEWPOINT 0 0 0 1 0 0 0\n")
    outputstr.append("POINTS ")
    outputstr.append(str(prof_per_buf * prof_width * available_buffers))
    outputstr.append("\n")
    outputstr.append("DATA ascii\n")

    for h in range(available_buffers):
        print("Writing buffer nr: ", h)
        for i in range(len(data[0][0])):
            for j in range(prof_width):
                outputstr.append(
                    str(data[h][INDEX_OF_X][j + prof_per_buf * i]))
                outputstr.append(" ")
                outputstr.append(
                    str(data[h][INDEX_OF_RANGE][j + prof_per_buf * i]))
                outputstr.append(" ")
                outputstr.append(str(i + prof_per_buf * h))
                outputstr.append("\n")

        file.write(''.join(outputstr))
        outputstr = []
