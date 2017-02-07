import asyncio
import json
from master import *


def say_something(res, network):
    print("Something:", res)


def say_something_else(res, network):
    print("Saying something else:", res)


def say_hi(network):
    print("hi")


def say_something_different(res):
    print("We got a procedural message:", res)


def error(msg, args):
    print("We got an error:", msg)


def logging(msg, args):
    print("We got a debug message:", msg)


@asyncio.coroutine
def running(network):
    try:
        pass
        #network.run_do_thing2(say_something, 10).sleep(2, callback=say_hi)\
        #    .run_do_thing(say_something_else)
        #network.run_do_error(say_something)
        #yield from asyncio.sleep(10)
    except KeyboardInterrupt:
        pass


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
    return conf


def main():
    """The main function that will start the master and connect to the slave"""
    print("Starting client")
    network = Network(load_settings('./conf.json'), logging, error)
    network2 = Network(load_settings('./conf2.json'), logging, error)

    network.start(running)
    network2.start(running)
    network.sleep(3).run_do_thing2(0, callback=say_something).sleep(3)\
        .run_do_thing2(0, callback=say_something)
    network2.sleep(1).run_do_thing2(1, callback=say_something).sleep(3, callback=say_hi)\
        .run_do_thing2(1, callback=say_something)
    network.run_forever()
    network.close()


if(__name__ == '__main__'):
    main()
