import rospy

import treeD_client_pkg.srv as srvs
from treeD_client_pkg.msg import *
from diagnostic_msgs.msg import DiagnosticStatus
from treeD_client_pkg.pcd import create_point_cloud

REALLY_BIG_NUMBER = 0xFFFFFFFF

class TreeDClientNode:
    """
    This is a class of the ROS-node that acts as a client. It is a part
    of the treeD_client_pkg ROS-package. The client connects to the ROS
    control node to control the system. There can be multiple clients
    active at once. The node sends commands by calling services on the
    control node and receives data by listening to topics.
    """
    def __init__(self, options):
        """
        The constructor function of the class.

        :param options: the global options from the CLI
        """
        self.wait_for_slice = REALLY_BIG_NUMBER
        self.points = []
        self.last_slice_id = -1

        self.options = options
        self.started = True
        self.done = False

        rospy.init_node("treeD_client_node", anonymous=True)

    def __check_results(self):
        """
        Function that checks if the last data slice has been received and then
        create a pointcloud from the data.
        """
        if self.last_slice_id >= self.wait_for_slice:
            if self.options['verbose']:
                print("Saving pcd...")
            create_point_cloud(self.points, self.options['file_path'],
                               self.width, self.height, self.wait_for_slice + 1)
            print("File saved to " + self.options['file_path'])
            rospy.signal_shutdown("Finished scan.")

    def __add_buff(self, buff):
        """
        Function used to add received data to the rest of the data.

        :param buff: buffer of data to add to the rest.
        """
        self.points += buff

    def __started_callback(self, data):
        """
        Callback function called when data is received on the started topic.

        :param data: the data received on the topic.
        """
        rospy.loginfo(data.scanId)

    def __debug_callback(self, data):
        """
        Callback function called when receiving data on the debug topic. Only
        prints the data if it's an error or if the verbose flag has been
        given.

        :param data: received debug data.
        """
        if data.level == DiagnosticStatus.ERROR:
            print(data.name, 'error:', data.message)
            rospy.signal_shutdown("Error in system, terminating")
            return

        if 'verbose' in self.options and self.options['verbose']:
            print(data.name, 'debug:', data.message)

    def __points_callback(self, data):
        """
        Callback function used to receive pointcloud data.

        :param data: points received on the point topic.
        """
        self.__add_buff(data.points)
        self.last_slice_id = max(self.last_slice_id, data.sliceId)
        self.width = data.width
        self.height = data.height
        self.__check_results()

    def __done_callback(self, data):
        """
        Callback function used to signal that a scan is done. It responds to
        data that comes on the done topic.

        :param data: the done message being received on the topic.
        """
        self.wait_for_slice = data.lastSliceId - 1
        self.__check_results()
        self.done = True


    def __settingchanged_callback(self, data):
        """
        Callback function used to receive notifications when a setting gets
        changed. This function gets called when a message is received on then
        settingchanged topic.

        :param data: the message begin received on the topic.
        """
        rospy.loginfo("settingchanged_callback")


    def __reset_device(self, device):
        """
        Function used to reset a hardware device. Calls the reset service
        on the control node.

        :param device: the hardware device to reset.
        """
        rospy.wait_for_service("reset")
        rospy.Subscriber("debug", DiagnosticStatus , self.__debug_callback)

        try:
            reset_proxy = rospy.ServiceProxy("reset", srvs.reset)
            resp = reset_proxy("", device)
        except:
            raise

    def reset_lineardrive(self):
        """
        Funciton called by the cli to reset the lineardrive.
        """
        self.__reset_device("lineardrive")

    def reset_board(self):
        """
        Function called by the cli to reset the rotational board.
        """
        self.__reset_device("board")


    def __set_setting(self, setting, value):
        """
        Function used to set a setting to a value on a hardware device.
        Calls the set service on the control node.

        :param setting: which setting to set.
        :param value: the value to set the setting to.
        """
        rospy.wait_for_service("set")
        rospy.Subscriber("debug", DiagnosticStatus , self.__debug_callback)
        try:
            set_proxy = rospy.ServiceProxy("set", srvs.set)
            resp = set_proxy("", setting, str(int(value)))

        except:
            print("Set failed")
            raise

    def set_cart_position(self, position):
        """
        Function called by the cli to set the position of the lineardrive.

        :param position: the position to set the lineardrive to.
        """
        self.__set_setting("position", position)

    def set_cart_relative_position(self, position):
        """
        Function called by the cli to set the relative position
        of the lineardrive.

        :param position: the relative position of the lineardrive.
        """
        self.__set_setting("relative", position)

    def set_cart_speed(self, speed):
        """
        Function called by the cli to set the speed of the lineardrive.

        :param speed: the speed to set the lineardrive to.
        """
        self.__set_setting("speed", speed)

    def set_table_rotation(self, rotation):
        """
        Function called by the cli to set the rotation of the board.

        :param rotation: the rotation to set the board to.
        """
        self.__set_setting("board_rotation", rotation)

    def set_table_curve(self, curve):
        """
        Function called by the cli to set the curve of the board.

        :param curve: the curve to set the board to.
        """
        self.__set_setting("board_curvature", curve)

    def __get_value_of_setting(self, setting):
        """
        Function to get the value of a setting on a hardware device. Calls
        the get service on the control node.

        :param setting: the setting to get the value of.
        :return: the service response containing the requested value.
        """
        rospy.wait_for_service("get")
        rospy.Subscriber("debug", DiagnosticStatus , self.__debug_callback)

        try:
            get_proxy = rospy.ServiceProxy("get", srvs.get)
            resp = get_proxy(setting)
            return resp.message
        except:
            raise

    def get_cart_position(self):
        """
        Function called by the cli to get the position of the lineardrive.

        :return: the position of the lineardrive.
        """
        return self.__get_value_of_setting("position")

    def get_cart_speed(self):
        """
        Function called by the cli to get the speed of the lineardrive.

        :return: the speed of the lineardrive.
        """
        return self.__get_value_of_setting("speed")

    def get_table_rotation(self):
        """
        Function called by the cli to get the rotation of the board.

        :return: the rotation of the board.
        """
        return self.__get_value_of_setting("board_rotation")

    def get_table_curve(self):
        """
        Function called by the cli to get the curve of the table.

        :return: the curve of the board.
        """
        return self.__get_value_of_setting("board_curvature")

    def scan(self):
        """
        Function called by the cli to start a scan. Calls the scan service
        on the control node.
        """
        rospy.Subscriber("started", StartedMsg, self.__started_callback)
        rospy.Subscriber("done", DoneMsg, self.__done_callback)
        rospy.Subscriber("points", PointSlice, self.__points_callback)
        rospy.Subscriber("debug", DiagnosticStatus , self.__debug_callback)

        rospy.wait_for_service("scan")

        try:
            print("Starting scan")
            scan_proxy = rospy.ServiceProxy("scan", srvs.scan)
            resp = scan_proxy(1)
        except:
            raise

        rospy.spin()
