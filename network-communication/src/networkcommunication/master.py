"""
File: master.py
Authors: vikri500, vicse557
"""

import asyncio
import pickle
import datetime
import json
import types
import time
import threading
import inspect

from networkcommunication.messages import *


class Chainer:
    """
    This class is a convenience class that aims to make it easier to chain
    together multiple network calls that should be done sequentially.

    The functionality is achieved by overriding the two magic methods __call__
    and __getattr__provided by python. __getattr__ fakes an attribute existing
    and fetches it from "network" instead (which should be a function), while
    call then is used to fake a call on the function, instead adding it to the
    execution queue maintained.

    Note that this class should not be instanciated manually, but rather is the
    returned result of any function ran on network.

    :Example:

    >>> network.run_something(callback=hi).run_different("hey", callback=hi_2)

    .. note:: since execution should not continue when an error occurs these
        halts any continued execution. Instead, as normally, the network's
        error function will be ran.
    .. todo:: make sure that when an exception is raised in a function, the
        complete chain terminates, to not leave trailing tasks.
    """

    def __init__(self, future, network, disabled=False):
        """
        Constructor for the chainer.

        :param future: the future that this chainer will wait for it to be
            finished before firing of it's chained functions.
        :param network: the network that is responsible for communication.
        """
        self.future = future
        self.network = network
        self.queue = []
        if isinstance(future, asyncio.Future) and not disabled:
            future.add_done_callback(self.__start_queue)
        self.current = None
        self.lock = asyncio.Lock(loop=network.loop)
        self.busy_lock = threading.Lock()
        self.busy_lock.acquire()
        self.__is_locked = False
        self.disabled = disabled

    def __call__(self, *kargs, **kwargs):
        """
        The magic function call that will add the current function to the list
        of those to execute.

        :param callback: the callback function that will be ran upon finished
            execution. None if none is needed (ie. likely when multiple more
            chainings will be done).
        :param **kwargs: any arguments that should be passed on to the function
            being ran, not the callback but the one marked with __get__.
        :return: self, for further chaining.
        """
        if self.disabled:
            return self

        callback = kwargs['callback'] if 'callback' in kwargs else None
        if self.current:
            self.queue.insert(0, (self.current, callback, kargs))
            self.current = None
        return self

    def __start_queue(self, future):
        """
        The method ran upon the original future terminating.

        :param future: the future that just terminated.
        :return: None.
        """
        if future.cancelled() or isinstance(future.result(), Exception):
            self.cancel()
            return
        else:
            asyncio.async(self.executer(), loop=self.network.loop)

    @asyncio.coroutine
    def executer(self):
        """
        The coroutine responsible for working through all the chained together
        functions one by one.

        :return: None
        """
        def unlock_and_run(callback, *k):
            if isinstance(callback, types.FunctionType):
                callback(*k)
            self.lock.release()
        while len(self.queue) > 0:
            yield from self.lock.acquire()
            if len(self.queue) == 0:
                break  # if an error occured, the queue has been emptied
            t = self.queue.pop()
            func = t[0]
            callback = t[1]
            args = t[2]
            # Run the function, and unlock itself once result occurs.
            func(*args, callback=lambda *k: unlock_and_run(callback, *k))
        self.busy_lock.release()

    def __getattr__(self, name):
        """
        Overriden magic member that fetches any "run_" (or rather
        Network.EXECUTABLE_PREFIX) from the network and marks it as the current
        method handled. This makes it so that when __call__ is executed it can
        be correctly added to the execution queue.

        :param name: the name of the attribute that was attempted to be
            retrieved.
        :raises NameError: if the attribute was not one starting with
            Network.EXECUTABLE_PREFIX
        :return: Self, so that __call__ will be ran when paranthesis are added
            after the attribute fetching, simulating a normal function call.
        """
        if name[:len(Network.EXECUTABLE_PREFIX)] == \
                Network.EXECUTABLE_PREFIX or name == "sleep":
            self.current = getattr(self.network, name)
            return self
        else:
            raise NameError("Not a valid method in network: " + str(name))

    def cancel(self):
        """
        Cancels the current chain and unlocks anything halted by this chain.

        :return: None.
        """
        self.queue = []
        if self.__is_locked:
            self.lock.release()  # the busy lock is released by executer

    def wait_for(self):
        """
        Suspends the current thread until the chain has finished. Note this has
        to be done on another thread than the thread running the event loop.

        :return: None.
        """
        self.__is_locked = True
        self.busy_lock.acquire()
        self.__is_locked = False


class Network(asyncio.Protocol):
    """
    This class is responsible of maintaining the connection with the slave,
    acting as a client to it.

    It inherits from the asyncio.Protocol class, refer to the asyncio
    documentation for more thorough information on the generic base class.

    Upon instantiation of this object a config (descbribed in slave.py) will
    be traversed, generating functions on this class prefexied with "run_" (or
    rather Network.EXECUTABLE_PREFIX). When called, these will do a network
    request for the slave to run the corresponding commands, then at once
    finished sending a message back with the (potential) result of the call.
    All calls are done asynchroically, if sequential result is desired the
    functions has to be chained together, either in the callback function or
    by chaining them with the Chainer class, an example of this can be found in
    that class.

    The "run_" functions all take an optional callback function that will be
    executed once the result from the slave is received. If none is required it
    can be omitted, or given the value None. Before this, any parameters that
    the function called on the slave takes should be passed as a *karg list (ie
    just parameters). The parameters passed needs to be pickle:able.

    The callback functions can work in one of two ways, depending on the result
    back from the function ran on the slave. If the function is one that does
    not return a value the callback function will only take one parameter, the
    network object to allow further network calls. If the returned value from
    the function is anything but None a result parameter will prior to the
    network parameter is given too.

    Errors occuring in the slave are considered fatal, any such occuring will
    call an error function given. Any subsequent calls that was chained up
    will not be done.

    Logging might arrive from the slave, these call a certain logging function
    that should present the logging data back to the user. Refer to the
    constructor for more details.

    Generic messages sent from the slave can be bound to a result function by
    calling the "register_callback" function, when that type of message is then
    received, this callback will be executed.

    Internally, the functionality is done asynchroically, thus registering and
    executing network commands will not pause normal execution.

    :Example:

    >>> def print_result(result, network):
    >>>     print("We got a result:", result)
    >>> ...
    >>> network.run_something("Hello", 4, callback=print_result)

    """
    RECONNECT_PERIOD = 2
    EXECUTABLE_PREFIX = "run_"
    loop = None

    def __init__(self, config, logging_func, error_func, name=""):
        """
        The master's network constructor.

        :param config: the configuration dictionary, refer to the description
            at the top in slave.py for a description of it.
        :param logging_func: a function to call when log info is received. It
            should take two parameters, a describing info text and a args
            parameter that is whatever extra data is sent along to describe the
            info message, usually a dict with values.
        :param error_func: a function that is called when an error is received
            this like the parameter logging_func takes the message string and
            extra arguments describing the error, usually the raised Exception.
        :param name: a name that describes this network, useful when mutliple
            network instances are ran in parallell.
        :return: None.
        """
        self.config = config
        self.loop = Network.get_loop()
        self.__generate_executables(config['executables'])
        self.transaction_counter = 0
        self.futures = {}
        self.logging_func = logging_func
        self.error_func = error_func
        self.callbacks = {}
        self.rest_data = bytes()
        self.connected = False
        self.name = name
        self.__error_occured = False

    @staticmethod
    def run_forever():
        """
        A static method that runs the event loop forever, suspending normal
        execution and yields it to the event loops. This is exited once a
        KeyboardInterrupt exception occurs, ie on Ctrl+C.

        :return: None.
        """
        try:
            Network.get_loop().run_forever()
        except KeyboardInterrupt:
            print("Exiting")
        except:
            raise

    @staticmethod
    def get_loop():
        """
        Returns the globally used event loop for all Network instances. If none
        is set, one is created.

        :return: asyncio.BaseEventLoop
        """
        if Network.loop is None:
            Network.loop = asyncio.get_event_loop()
        return Network.loop

    def __generate_executables(self, executables):
        """
        A method that registers the functions that can be run from "executables"
        as "run_<FUNC>" where <FUNC> is whatever tags are present "executables".

        :param executables: a dictionary/list with the names of the functions
            that should be registered.
        """
        for e in executables:
            def func(*args, callback=None, fn=e):
                return self.generate_message(fn, *args, callback=callback)
            setattr(self, Network.EXECUTABLE_PREFIX + e, func)

    def register_callback(self, command, callback):
        """
        Registers a callback that occurs when a generic message is sent from
        the slave.

        :param command: string, the generic message's command, used to identify
            when this callback should run.
        :param callback: the function to run. This function will receive any
            passed along with the message sent from the slave as the first arg.
        :return: None.
        """
        self.callbacks[command] = callback

    def connection_made(self, transport):
        """
        The overriden method from asyncio.Protocol that is executed once a
        connection is made. This will save the transport for later use.

        :param transport: the asyncio Transport generated for the connection.
        :return: None.
        """
        print("Connection established.")
        self._transport = transport
        self.connected = True

    def data_received(self, data):
        """
        The overriden method from asyncio.Protocol that is executed when any
        complete data message is retrieved. This method attempts to unpack
        and intepret the message. Since the received data might contain
        multiple messages, these will be parsed and executed sequentially.

        :param data: the received data.
        :return: None.
        """
        d = self.rest_data + bytes(data)
        self.rest_data = bytes()
        while True:
            try:
                loaded_data = pickle.loads(d)
            except:
                # TODO: A faulty message needs to be handled seperately to
                # flush rest_data.
                if len(d) != 0:
                    self.rest_data = d
                return
            length = len(pickle.dumps(loaded_data))
            msg = Message.unpack(d, self.config)
            d = d[length:]
            self.__interpret_message(msg)

    def __interpret_message(self, msg):
        """
        A method intepreting and doing whatever is required by the message,
        depending on the type of message received.

        If it's debug a log function is ran, if it's a "success" or "error"
        the corresponding

        :param msg: the Message that will be intepreted and handled.
        :return: None.
        """
        if not msg:
            self.logging_func("Received faulty message.", None)
            return
        elif isinstance(msg, DebugMessage):
            self.logging_func(msg.message, msg.args, self)
        elif msg.get_command() == 'success':
            # other message received, likely send a success or busy back to cli
            # and run the callback
            self.futures[msg.acknowledge_id].set_result(msg.args)
        elif msg.get_command() == 'error':
            self.futures[msg.acknowledge_id].cancel()
            self.error_func(msg.message, msg.args, self)
            self.__error_occured = True
        elif msg.get_command() in self.callbacks:
            self.callbacks[msg.get_command()](msg.args)

    def connection_lost(self, exc):
        """
        The overriden connection_lost method from asyncio.Protocol that is ran
        once a connection has been lost. This will attempt to reconnect the
        master to the slave after a wait of "Network.RECONNECT_PERIOD".

        :param exc: the reason for the connection loss.
        :return: None.
        """
        print('Lost connection to server, reconnecting in',
              Network.RECONNECT_PERIOD, 'seconds.')
        self.connected = False
        if exc:
            print(exc)
        #self.loop.stop()
        self.connect()

    @asyncio.coroutine
    def do_connect(self):
        """
        A coroutine that attempts to connect the master to the slave, if it
        fails it will sleep for Network.RECONNECT_PERIOD seconds then be ran
        again from connect.

        :return: None.
        """
        try:
            coro = yield from\
                self.loop.create_connection(lambda: self,
                                            self.config['slave_ip'],
                                            self.config['port'])
            self.connect_future.set_result(True)
            asyncio.async(self.main(), loop=self.loop)
            self.__error_occured = False
        except Exception as e:
            print("Can't connect, retrying in", Network.RECONNECT_PERIOD,
                  "seconds.")
            yield from asyncio.sleep(Network.RECONNECT_PERIOD)
            self.connect_future.set_result(e)

    def connect(self):
        """
        Attempts to connect the master to the slave, retrying the connection if
        it fails each Network.RECONNECT_PERIOD.

        :return: None.
        """
        def check_connection(future):
            res = future.result()
            if isinstance(res, Exception):
                self.connect()
            else:
                asyncio.async(self.main(self), loop=self.loop)
        self.connect_future = asyncio.Future(loop=self.loop)
        self.connect_future.add_done_callback(check_connection)
        task = asyncio.async(self.do_connect(), loop=self.loop)

    def start(self, callback):
        """
        The function that starts up a connection and executes a callback to a
        coroutine object once the connection is made.

        :param callback: the coroutine that will be ran once connection is
            made.
        """
        self.main = callback
        self.connect()

    def close(self):
        """
        A function that closes down the event loop. Beware that this needs to
        be done once all Network instances has finished using it.

        :return: None.
        """
        if Network.loop is not None:
            print("Stopping the event loop.")
            Network.loop.close()

    def generate_message(self, command, *kargs, callback=None):
        """
        A method to generate and send a message to the slave. This is done
        asynchroically.

        :param command: the command to execute on the slave. This needs to be a
            valid command from the configuration file.
        :param callback: a function that will be executed once the result of
            this execution is known. Any data contained in the result message
            from the client will be passed along to this function, and as well
            as the network instance to facilitate further requests.
        :param *kargs: any parameters that should be sent along to the
            function executed on the slave.
        :return: None.
        """
        def do_callback(future):
            # TODO add handling in case future is cancelled/timed out.
            if future.cancelled() or not isinstance(callback,
                                                    types.FunctionType):
                return
            result = future.result()
            if result is not None:
                callback(result, self)
            else:
                callback(self)
        if self.connected and not self.__error_occured:
            args = {
                'transaction_id': self.transaction_counter,
                'args': kargs
            }
            self._transport.write(Message(command, args).pack())
            future = asyncio.Future(loop=self.loop)
            future.add_done_callback(do_callback)
            self.futures[self.transaction_counter] = future
            self.transaction_counter += 1
            return Chainer(future, self)
        else:
            return Chainer(None, self, disabled=True)

    def sleep(self, time, callback=None):
        """
        Asynchronically pauses execution.

        :param time: seconds to sleep.
        :param callback: an optional function that can be executed when sleep
            is over.
        :return: a Chainer object to facilitate chaining.
        """
        @asyncio.coroutine
        def sleep(future, seconds):
            yield from asyncio.sleep(seconds)
            future.set_result(True)

        def do_callback(future):
            if future.cancelled() or not isinstance(callback,
                                                    types.FunctionType):
                return
            callback(self)
        future = asyncio.Future(loop=self.loop)
        future.add_done_callback(do_callback)
        asyncio.async(sleep(future, time), loop=self.loop)
        return Chainer(future, self)
