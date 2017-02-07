"""
File: messages.py
Authors: vikri500
"""
from enum import Enum
import pickle


class Message:
    """
    The message class used to describe those messages that are sent between the
    slave and the master.

    A message always consist of the command attribute, other than that what it
    contains is fully up to the message in question. All attributes on a class
    instance (thus appearing in __dict__) will be packed together as a dict
    and sent upon transmission.

    Receiving is done by unpacking, that is reversing the packing and again
    register the attributes in the received dict back on a class instance.

    Special case is DebugMessage, which inherits this class, which is there to
    provide some for these messages specific things, such as printing etc.
    """
    EXEC_CONF_KEY = 'executables'

    def __init__(self, command="", args=None):
        """
        The Message class constructor. Note it's not normally used when a
        message is received over network as "unpack" is used instead.

        :param command: the command of the message, it should be the same as
            one in the Network config to point at a function that will be
            executed when this message is received.
        :param args: Arguments (a dict) attached to this Message.
        :return: None.
        """
        self.command = command
        if isinstance(args, dict) and len(args) > 0:
            for key in args:
                setattr(self, key, args[key])

    @staticmethod
    def unpack(received, config):
        """
        Unpacks a message received using config from network.

        :param received: the message that was received and will be parsed as a
            message.
        :param config: the configuration that dictates what commands are valid.
        :return: A DebugMessage if the type of received data is a for Debug, a
            Message if the received data is a normal valid message, otherwise
            None.

        :todo: EDIT NETWORK TO OTHER TEXT, SINCE IT DOESNT TAKE NW ANY LONGER
        """
        loaded = None
        try:
            loaded = pickle.loads(received)
        except:
            return None
        if(not isinstance(loaded, dict)):
            # SIGNAL ERROR
            print("Not dict")
            return None
        if('command' not in loaded or
                loaded['command'] not in config[Message.EXEC_CONF_KEY]):
            # SIGNAL INVALID
            return None
        return Message.__generate(loaded)

    @staticmethod
    def __generate(values):
        """
        A function returning a message assembled from a received "dictionary".
        """
        if('command' in values and values['command'] == 'debug'):
            return DebugMessage(values['message'], values['severity'],
                                values['code'], values['args'])

        msg = Message()
        for key in values:
            setattr(msg, key, values[key])
        return msg

    def pack(self):
        """
        Packs a message as a pickle version of it containing just the data
        members.

        :return: A pickled version of self, as bytes.
        """
        return pickle.dumps(self.__dict__)

    def get_args(self):
        """
        Retrieves all the data members present in "args".

        :return: A list with arguments.
        """
        return self.args

    def get_command(self):
        """
        Returns the command string.

        :return: string
        """
        return self.command

    def get_module(self, config):
        """
        Returns the name of the module (or class) this message executable
        function points at by looking up in config.

        :param config: The dictionary with configurations.
        :return: The module or class instance referenced by this message.
        """
        cmd = config[self.EXEC_CONF_KEY][self.command]
        if 'module' in cmd:
            return cmd['module']
        return cmd['class']

    def get_func(self, config):
        """
        Retrieves the function that will be executed by this message.

        :todo: make sure it doesn't raise exeption on invalid.

        :param config: The dictionary with configurations.
        :return: The function.
        """
        return config[self.EXEC_CONF_KEY][self.command]['func']

# The debug level enum, at this level as opposed to in the DebugClass to be
# able to pickle it.
DEBUG_LEVEL = Enum('DEBUG_LEVEL', 'NOTICE WARNING ERROR')


class DebugMessage(Message):

    def __init__(self, message, severity=DEBUG_LEVEL.NOTICE,
                 code=-1, args=None):
        """
        An inherited version of Message with some defaults already set to make
        handling of them easier, such as printing.

        :param message: the message string associated with this debug message.
        :param severity: the severity enum DEBUG_LEVEL used to identify the
            severity of the message, NOTICE, WARNING, or ERROR.
        :param code: an optional error code used to identify this debug
            message.
        :param values: an optional parameter used to send with additional
            information related to the debug message.

        :return: None.
        """
        self.severity = severity
        self.message = message
        self.code = code
        self.args = args
        self.command = 'debug'

    def __str__(self):
        """Returns a prettified version of the debug message."""
        return DEBUG_LEVEL(self.severity).name + ': ' + str(self.message)
        + (' (Code: ' + str(self.code) + ')' if self.code != -1 else '')
