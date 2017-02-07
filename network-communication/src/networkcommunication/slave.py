"""
File: slave.py
Authors: vikri500, vicse557

A file containing the Network class, the class that handles all communication
from a slave (ie. hardware node) to it's master (ie. the ROS-node).

The Network class takes in a dictionary containing configurations. It contains:
    - slave_ip, [string] the ip of the slave
    - port, [integer] the port listened to
    - executables, [dictionary] a dictionary containing the methods that can
        potentially be ran by any message received. Every element in this
        dictionary contains:
            - module OR class, [string] the module or class that contains the
                executable that will be ran once a message of this type is
                received.
            - func: The function that will be ran from the module or class.
    - files, [dictionary] a dictionary containing the files/class info used by
        the elements in "executables" to load these at runtime. Each element
        in this dictionary contains:
            - path, [string] the path to the file, relative from current
                directory or absolute.
            - module, [string] the name of the module, ie the name of the file
                minus ".py".
            - class_name, [string, optional] if provided instead of loading the
                base module a class will be loaded and instanciated with this
                name.
             - args, [dict, optional] arguments that will be passed on to the
                class constructor when a version of it is instanciated.

"""
import asyncio
import importlib
import importlib.machinery
import os
import sys
import inspect

from networkcommunication.messages import *


class Network(asyncio.Protocol):
    """
    This class is responsible for intepreting the messages sent from the master
    and launch the functions those commands reference.

    When initialized, a config dictionary is traversed to load all the modules
    that these functions reside in, and if it's a class, also instansiate a
    version of it.

    Any valid messages then received will execute a function and when returning
    send the result back with a "success" message. Parameter-verification isn't
    done anywhere outside the functions, and the functions themselves thus need
    to take care of this so that no errenous results are given. This is done
    by raising an exception when something goes awry.

    Any exception raised will then be caught by the executor and an error
    message is generated to indicate this for the master. Logging however is
    done by simply printing, these will be sent to the master as debug-info
    messages.
    """
    _settings = {}  # TODO defaults here

    def __init__(self, config):
        """
        The Network class constructor.

        :param config: a dict with configuration parameters. Refer to the top of
            this file for a thorough description of what this dictionary
            contains.

        :return: None.
        """
        self.config = config
        self.loop = asyncio.get_event_loop()
        self.__init_modules()

    def __import_module(path, module, name=None):
        """Helper function for dynamically importing a python file."""
        try:
            if path:
                full_path = path + module + '.py'
                loader = importlib.machinery.SourceFileLoader(module, full_path)
                mod = loader.load_module()
            else:
                mod = importlib.import_module(module)
            return mod
        except:
            raise ImportError("Loading module failed.")

    def __custom_print(self, *kargs, **kwargs):
        """A print method that also generates INFO debug messages."""
        print(*kargs, **kwargs)
        s = " ".join(map(str, kargs))
        self.generate_debug_message(s, DEBUG_LEVEL.NOTICE)

    def __init_modules(self):
        """Imports all files listed in config['files']."""
        self.modules = {}
        files = self.config['files']
        for key, f in files.items():
            try:
                path = f['path'] if 'path' in f else ''
                mod = Network.__import_module(path, f['module'])
                # We hijack print in the modules to send debug info on prints
                mod.print = self.__custom_print
                if 'class_name' in f:
                    args = f['args'] if 'args' in f else {}
                    # We load the class with args passed to the constructor
                    attr = getattr(mod, f['class_name'])(**args)
                    self.modules[key] = attr
                else:
                    self.modules[key] = mod
            except:
                print("Could not load modules")
                raise

    def connection_made(self, transport):
        """
        The connection_made method extended from asyncio.Protocol overriden to
        do the handling of new connections.

        :param transport: the asyncio transport object associated with this
            connection, see asyncio docs for more information.
        :return: None.
        """
        peername = transport.get_extra_info('peername')
        print('Connection from {}'.format(peername))
        self._transport = transport

    @asyncio.coroutine
    def execute(self, future, message):
        """
        The coroutine launching the function a certain message is pointing at
        and :return: the result once it has finished executing.

        :param future: the asyncio future used to store the eventual result of
            the message's function call.
        :param message: the message that points at a function to run.
        :return: None, but the future will eventually resolve and contain the
            function call's result.
        """
        module = message.get_module(self.config)
        fn_name = message.get_func(self.config)
        kargs = message.get_args()
        try:
            fn = getattr(self.modules[module], fn_name)
            # We get a description of the function to be ran to validate if a
            # network parameter is given, if so we pass along ourself to the
            # function. This is done to facilitate chaining.
            arg_desc = inspect.getargspec(fn)
            kwargs = {}
            if arg_desc.args and 'network' in arg_desc.args:
                kwargs['network'] = self
            # Attempt to run the function with provided arguments
            result = fn(*kargs, **kwargs)
        except Exception as e:
            result = e  # ERROR OCCURRED

        future.set_result({
            'msg': message,
            'result': result
        })

    def feedback(self, future):
        """
        The callback function ran once a future resolves.

        :param future: the asyncio future that this function was called by.
        :return: None.
        """
        r = future.result()
        causing_msg = r['msg']
        result = r['result']
        if isinstance(result, Exception):
            msg = result.args[0] if len(result.args) > 0 else ""
            args = result.args[1:] if len(result.args) > 1 else ()
            self.generate_error_message(causing_msg, msg, args)
        else:
            self.generate_success_message(causing_msg, result)

    def data_received(self, data):
        """
        The asyncio method from asyncio.Protocol overriden to load a message
        once data is received. If loading is successful the function the
        message is pointing to will be executed asynchroically, with the result
        of the execution being sent back to the sender once it resolves. Since
        the received data might contain multiple messages, these will be
        parsed and executed sequentially.

        :param data: the data that was received.
        :return: None.
        """
        d = bytes(data)
        while True:
            try:
                loaded_data = pickle.loads(d)
            except:
                return
            length = len(pickle.dumps(loaded_data))
            msg = Message.unpack(d, self.config)
            d = d[length:]
            self._exec_message(msg)

    def _exec_message(self, msg):
        """
        Executes a message if it's a valid one, otherwise returning.

        :param msg: the message that should be executed.
        :return: None.
        """
        if(msg is None):
            self.generate_debug_message("Couldn't unpack message.", DEBUG_LEVEL.WARNING)
            return

        if(isinstance(msg, DebugMessage)):
            print(str(msg))
            return
        future = asyncio.Future()
        task = asyncio.async(self.execute(future, msg), loop=self.loop)
        future.add_done_callback(self.feedback)

    def get_conf(self, which):
        """
        Returns the value of a certain config.

        :param which: what configuration that is to be looked up.

        :return: The value of which in the config, if it exists otherwise None.
        """
        if which not in self.config:
            return None
        return self.config[which]

    def start(self, callback):
        """
        The start method from asyncio.Protocol overriden to launch the listener
        server using config to set ip and port.

        :param callback: what function to be called once the server launches.
            This function receives the network object as parameter.

        :return: None.
        """
        self.coro = self.loop.create_server(lambda: self,
                                            self.config['slave_ip'],
                                            self.config['port'])
        self.server = self.loop.run_until_complete(self.coro)
        callback(self)

    def close(self):
        """
        The close method from asyncio.Protocol overriden to close all the
        connections made upon termination.

        :return: None.
        """
        self.server.close()
        self.loop.run_until_complete(self.server.wait_closed())
        self.loop.close()

    def generate_debug_message(self, message, severity, code=-1, args=()):
        """
        Convenience method to generate and send a debug message back to the
        clients connected to this server.

        :param message: the message associated with the debug message.
        :param severity: the severity enum (or integer) associated with the
            message.
        :param code: the optional code used to identify this message.
        :param args: an optional extra field for passing along extra info about
            the debug message.

        :return: None.
        """
        if not self._transport:
            return
        self._transport.write(DebugMessage(message, severity, code, args)
                              .pack())

    def generate_success_message(self, msg, result=None):
        """
        Convenience method to generate and send a success message back to
        the clients connected to this server.

        :param msg: the Message that was successfully handled.
        :param result: an optional parameter that can be passed to include
            response values back to the clients.
        :return: None.
        """
        if not self._transport:
            return
        self._transport.write(Message("success", args={
                                  'args': result,
                                  'acknowledge_id': msg.transaction_id
                              }).pack())

    def generate_message(self, command, data):
        """
        Generate a generic message with data. These still have to be defined
        in the config (like "error" and "success") but with no module/funciton
        associated with them. So, for example with a integer value of 0.

        :param command: the command string that is attached to the message.
        :param data: the data that is passed along with the message and thus
            then also to the callback function registered to this command type
            on the master.
        :return: None.
        """
        if not self._transport:
            return
        self._transport.write(Message(command, args={
                                  'args': data
                              }).pack())

    def generate_error_message(self, causing_msg, msg, error_args):
        """
        Convenience method to generate and send a success message back to
        the clients connected to this server.

        :param causing_msg: the Message that failed.
        :param msg: An error text that describes the error that occured.
        :param error_args: an optional parameter that can be passed to include
            extra data regarding the error.
        :return: None.
        """
        if not self._transport:
            return
        self._transport.write(Message("error", args={
                                  'message': msg,
                                  'args': error_args,
                                  'acknowledge_id': causing_msg.transaction_id
                              }).pack())
