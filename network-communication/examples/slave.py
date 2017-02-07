"""
File: slave.py
Authors: vikri500
"""
import asyncio
import sys
import json

import networkcommunication.slave as nw


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


def running(network):
    """The function handling keyboard interrupt, otherwise indefinitely runs"""
    try:
        network.loop.run_forever()
    except KeyboardInterrupt:
        pass


def main():
    """The main function launching the listener server"""
    print("Starting server")
    conf = load_settings('./conf.json')
    network = nw.Network(conf)
    network.start(running)
    network.close()


if(__name__ == '__main__'):
    main()
