#!/usr/bin/env python3.4

"""
File: treeD_control_node.py
Authors: davpo041, vikri500, sarsv839, felha990
"""
import rospy
import asyncio

from networkcommunication.master import *
import treeD_control_pkg.srv as srvs
from treeD_control_pkg.msg import *
from diagnostic_msgs.msg import DiagnosticStatus
#import treeD_control_pkg.key_handler as kh

def load_settings(path):
    """
    Loads and parses a json file from disk.

    :param path: the path to this file, either relative or absolute (unix/
        linux).
    :return: The loaded configuration as a dictionary.
    """
    try:
        with open(path) as data_file:
            conf = json.load(data_file)
    except:
        raise
        #print("ERROR: JSON not valid, defaulting")
    return conf

class TreeDControlNode:
    """
    This class is a class defining a ROS-node. It is part of the
    treeD_control_pkg ROS-package. It is the control node that
    all the clients in the system connect to and which sends
    all the commands to the hardware units. This node should
    be unique in the system, meaning only one such node should
    be running at any given time. The node is also essential to
    be able to run the system, since all the commands and data
    go through this node.
    """
    def __init__(self, config):
        """
        Contructor for the control node.

        :param config: configuration settings used when initializing
            the object.
        """
        self.__register_devices(config)
        self.options = config['options']
        self.__reset_slices()
        self.board_reset = False
        self.chains = []
        self.__error_occured = False

    def __register_devices(self, config):
        """
        Private function used to register the devices that this
        node is supposed to control and add them att attributes
        to the object.

        :param config: configuration settings with information
            about the devices to register.
        :raises Exception: if the configuration was not loaded properly.
        """
        try:
            devices = config['devices']
            for network_name, val in devices.items():
                conf = load_settings(val['conf_path'])
                setattr(self, network_name, Network(conf, self.logging,
                                                    self.error,
                                                    name=network_name))
                self.__register_valid_functions(val['set'], network_name, 'set')
                self.__register_valid_functions(val['get'], network_name, 'get')
                self.__register_valid_functions(val['reset'], network_name, 'reset')
        except:
            print("Could not load configuration correctly.")
            raise

    def __register_valid_functions(self, mapping, device, valid_type):
        """
        Private function used to register the functions that can be
        called on the different devices that this ndoe is controlling.

        :param mapping: mapping from function names called in this object
        to the actual function names in the devices themselves.
        :param device: the device on for which the functions are registered.
        :param valid_type: the type of functions that are registered.
        :raises Exception: if provided an invalid device that was not in the
            configuraion.
        """

        name = "__" + valid_type
        try:
            valid_funcs = getattr(self, name)
        except:
            valid_funcs = {}

        for key, val in mapping.items():
            try:
                network_obj = getattr(self, device)
            except:
                raise Exception("Invalid network object provided, " +
                    "check configuration")
            setting = str(Network.EXECUTABLE_PREFIX) + str(val)
            valid_funcs[key] = getattr(network_obj, setting)
        setattr(self, name, valid_funcs)  # Registers __"VALID_NAME" as a {}

    def get_func(self, func_type, name):
        """
        Function to get a function of the given type and with the
        given name.

        :param func_type: the type of the function to get.
        :param name: the name of the function to get.
        :raises Exception: if there is no function with the given name
            or type.
        :return: a function that has the given type and name.
        """
        try:
            funcs = getattr(self, '__' + func_type)
        except:
            print("Could not get the function list:", func_type)
            raise
        try:
            return funcs[name]
        except:
            print("Could not get the function:", name, "from", funcs)
            raise

    def __cancel_current(self):
        """
        Function to cancel all the current and queued operations.
        """
        # here we should cancel the current execution as some error occured
        #self.__error_occured = True
        for chain in self.chains:
            chain.cancel()

    def error(self, msg, args, network):
        """
        Method used to publish error data on the debug topic. When
        this happens it also cancels any ongoing or queued
        activity.

        :param msg: the message being sent.
        :param args: any arguments to be passed with the message.
        :param network: the network device in which the error
            occured.
        """
        msg = DiagnosticStatus(
            name=network.name,
            message=msg,
            level=DiagnosticStatus.ERROR,
            values=args if args else []
        )
        self.debug_channel.publish(msg)
        self.__cancel_current()

    def logging(self, msg, args, network):
        """
        Function used for logging data to the debug topic.

        :param msg: the message to be sent on the channel
        :param args: optional arguments to be passed with the message
        :param network: the network device about which the sent
            information is about.
        """
        msg = DiagnosticStatus(
            name=network.name,
            message=msg,
            level=DiagnosticStatus.OK,
            values=args if args else []
        )
        self.debug_channel.publish(msg)

    @asyncio.coroutine
    def running(self, network):
        """
        En empty function used when starting netwrok objects.

        :param network: the network device being started.
        """
        pass

    def __acquire_callback(self, req):
        """
        Callback function used to repsond to the acquire service.

        :param req: the input to the service.
        :return:
        """
        return srvs.acquireResponse(3, '','ross')#kh.get_key(), "ross")

    def __release_callback(self, req):
        """
        Callback function used to respond to the realease service.

        :param req: the input to the service.
        :return: a response object as defined in the service definition
        """
        rospy.loginfo("release_callback")
        return srvs.releaseResponse(3, '','ross')


    def __convert_buffer(self, buff):
        """
        Function to convert a received buffer of points to an object
        of type PointSlice containing objects of type PointMsg.

        :param buff: an array of points data.
        :raises Exception: if the conversion failes, probably because of
            index out of bound error.
        :return: a PointSlice object.
        """
        points = []
        try:
            #TODO create constants
            width = len(buff[0])
            height = int(len(buff[3]) / width)

            print("Width",width)
            print("Height",height)
            print(len(buff),len(buff[0]), len(buff[3]), len(buff[4]), len(buff[5]))

            for i in range(width):
                point_id = buff[0][i]
                for j in range(height):
                    point = PointMsg(
                        x=float(buff[3][i*height + j]),
                        y=float(buff[4][i*height + j]),
                        z=-float(point_id),
                        intensity=float(buff[5][i*height + j])
                    )
                    points.append(point)
        except Exception as e:
            print(e)
            raise


        self.id_offset += width
        self.current_slice_id += 1
        return PointSlice(
            points=points,
            sliceId=self.current_slice_id-1,
            width=width,
            height=height
        )

    def __reset_slices(self):
        """
        Function to reset the indeces that are keeping track of
        the received slices.
        """
        self.id_offset = 0
        self.current_slice_id = 0

    def __scan_callback(self, req):
        """
        Callback function that gets called when the scan service gets
        called.

        :param req: the input to the service.
        :return: a response with a message saying wether starting the scan
            succeded or failed.
        """
        if self.__error_occured:
            return srvs.scanResponse("error", "Due to an earlier error, " +
                               "the system needs to be reset.")
        finished = False
        scanned_object = []

        self.__reset_slices()

        def start_ruler(res, network):
            """
            Function to initialize and start the camera. After that the
            function checks for data from the camera.

            :param res: ammount of time to sleep before initializing the camera.
            :param network: the network device of the camera.
            """
            opts = self.options['ruler']
            chain_ruler = self.ruler.sleep(res)\
                .run_init(opts['ip'],
                          opts['prm_file'],
                          opts['timeout'],
                          opts['profiles_per_buffer'],
                          opts['buffer_size'])\
                .run_start(callback=start_moving)\
                .sleep(0.1)\
                .run_peek(callback=grabbing)


        def build_object(res, network):
            """
            Funciton to get data from the camera and build an object
            that contains all the data.

            :param res: the received data.
            :param network: the network device object of the camera.
            """
            nonlocal scanned_object
            scanned_object.append(res)
            points = self.__convert_buffer(res)
            print("has converted")
            self.points_channel.publish(points)
            network.sleep(self.options['ruler']['poll_rate'])\
                .run_peek(callback=grabbing)

        def finish(network):
            """
            Function to signal that the scan is finished and the data has been
            received.

            :param network: the network device object of the camera.
            """
            nonlocal finished
            finished = True

        def grabbing(res, network):
            """
            Utility function used to read data from the camera and then signal
            when all the data has been read.

            :param res:
            :param network: the network device object of the camera.
            """
            nonlocal finished
            nonlocal scanned_object
            if res > 0:
                network.run_read(callback=build_object)
                return
            if finished:
                self.done_channel.publish(DoneMsg(lastSliceId=self.current_slice_id))
                self.linear.run_set_position(self.options['start_position'])
                self.ruler.run_close()
                return
            print("No more data polling soon,", res)
            network.sleep(self.options['ruler']['poll_rate'])\
                .run_peek(callback=grabbing)

        def start_moving(network):
            """
            Function to start moving the camera and then wait for it to
            finish.

            :param network: the network device object of the camera.
            """
            def wait(res, network):
                """
                Function to wait for the camera to finish moving.

                :param res: time to wait.
                :param network: the network device object of the camera.
                """
                network.sleep(res + 1, callback=finish)
            self.linear.run_set_position(self.options['finish_position'],
                                         callback=wait)

        chain_linear = self.linear.run_set_position(self.options['start_position'],
                                                    callback=start_ruler)
        chain_linear.wait_for()  # unused currently

        return srvs.scanResponse("success", "")

    def __set_callback(self, req):
        """
        Callback function called when the set service gets called.

        :param req: a message saying what setting to set and the value to set it
            to.
        :return: a service respoonse saying wether setting the given setting
            succeded or failed.
        """
        setting = req.setting
        if self.__error_occured:
            return srvs.setResponse("error", "Due to an earlier error, " +
                               "the system needs to be reset.")

        # Convert the setting(s) to expandable, if multiple split them
        vals = list(map(lambda el: int(el), req.value.split(" ")))\
            if hasattr(req.value, '__iter__') else [int(req.value)]

        try:
            fn = self.get_func('set', setting)
            chain = fn(*vals)
            chain.wait_for()
            return srvs.setResponse("success", " ")
        except Exception as e:
            return srvs.setResponse("error", "Not a valid setting")


    def __get_callback(self, req):
        """
        Callback function that gets called when the get service gets called.

        :param req: message saying which setting to get the value of.
        :return: returns a service response with the value of the setting or
            an error if it failed getting the value.
        """
        result = 0
        setting = req.setting
        if self.__error_occured:
            return srvs.getResponse("error", "Due to an earlier error, " +
                               "the system needs to be reset.")

        def callback(res, network):
            """
            Callback that gets called when getting a response from the device.

            :param res: the response from the device.
            :param network: the network device from which to get the setting
                value.
            """
            nonlocal result
            result = res
        try:
            self.get_func('get', setting)(callback=callback).wait_for()
            return srvs.getResponse("success", str(result))
        except:
            return srvs.getResponse("error", "Not a valid setting")

    def __reset_callback(self, req):
        """
        Callback function that gets called when the reset service gets called.

        :param req: message containing which device to reset.
        :return: a service response message saying wether resetting failed or
            succeded.
        """
        device = req.device

        try:
            fn = self.get_func('reset', device)
            chain = fn()
            chain.wait_for()
            return srvs.resetResponse("success", " ")
        except Exception as e:
            return srvs.resetResponse("error", "Not a valid setting")


    @asyncio.coroutine
    def __reset_board(self):
        """
        Function used to reset to rotaional board.
        """
        def is_reset():
            """
            Function used to signal that to board is reset.
            """
            self.board_reset = True
        self.board.sleep(0.1).run_open(self.options['board']['port'],
                            callback=is_reset)

    def init(self):
        """
        Initialization function. Sets up all the topics that the node is writing
        to and all the services that the node is providing. Also starts all the
        network devices.
        """
        self.debug_channel = rospy.Publisher("debug", DiagnosticStatus, queue_size=10)
        self.points_channel = rospy.Publisher("points", PointSlice, queue_size=10)
        self.started_channel = rospy.Publisher("started", StartedMsg, queue_size=10)
        self.done_channel = rospy.Publisher("done", DoneMsg, queue_size=10)
        self.settingchanged_channel = rospy.Publisher("settingchanged", SettingChangedMsg, queue_size=10)

        rospy.Service('acquire', srvs.acquire, self.__acquire_callback)
        rospy.Service('release', srvs.release, self.__release_callback)
        rospy.Service('scan', srvs.scan, self.__scan_callback)
        rospy.Service('set', srvs.set, self.__set_callback)
        rospy.Service('get', srvs.get, self.__get_callback)
        rospy.Service('reset', srvs.reset, self.__reset_callback)

        rospy.init_node("treeD_control_node", anonymous=False)

        self.linear.start(self.running)
        self.ruler.start(self.running)
        self.board.start(self.__reset_board)
        self.linear.run_forever()
        print("Closing ROS.")
        self.linear.close()

if __name__ == '__main__':
    try:
        ros = TreeDControlNode(load_settings('/etc/treed/ros_config.json'))
        ros.init()
    except rospy.ROSInterruptException:
        raise
