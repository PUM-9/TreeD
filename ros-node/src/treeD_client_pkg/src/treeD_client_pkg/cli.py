import os
import time
import math
import subprocess

import click
import configparser as cp

from treeD_client_pkg.client_node import TreeDClientNode

DEFAULT_CONFIG_PATH = os.path.join(os.path.expanduser("~"), '.treed.rc')

CONFIG_FILES = [
    os.path.join(os.path.abspath(os.sep), 'etc', 'treed', 'config.rc'),
    DEFAULT_CONFIG_PATH,
    os.path.join(os.path.expanduser("~"), '.treed.rc')
]

default_options = {
    'address' : 'localhost',
    'port' : 1337,
    'lockfile' : '~/.config/treed/treed.lock',
    'configfile' : '~/.config/treed/treed.ini',
    'verbose' : False
}

options = {}

def set_option(key):
    """
    A callback function that gets called when a global flag is input.
    """
    def callback(ctx, param, value):
        """
        A callback function used to set the value of the global flag to
        a global variable.
        """
        options[key] = value
        return value
    return callback

global_options = [
#    click.option('--address', '-a'  , help="Address to the target ROS-node", expose_value=False,callback=set_option('address')),
#    click.option('--lockfile'   , help="File in which to store the lock", type=click.File(),expose_value=False, callback=set_option('lockfile')),
    click.option('--verbose'    , '-v', is_flag=True, help="Increase verbosity", expose_value=False,callback=set_option('verbose')),
#    click.option('--quiet'      , '-q', is_flag=True, help="Be more quiet", expose_value=False,callback=set_option('quiet')),
#    click.option('--procedural' , '-p', is_flag=True, help="Output results continuously", expose_value=False,callback=set_option('procedural')),
#    click.option('--progress'   , is_flag=True, help="Show progress", expose_value=False,callback=set_option('progress'))
]

def common_options(f):
    """
    Function that sets up all the common options.
    """
    for option in global_options:
        f = option(f)
    return f

@click.group()
def treed():
    """
    Function that gets called for the main command.
    """
    options = get_config()


@treed.command()
@common_options
@click.option('--output', '-o', type=click.Path(), help="Output file")
@click.option('--view', '-w', is_flag=True, help="View the scan when completed")
def scan(output, view):
    """
    Function that gets called when the scan flag i used. This command starts a
    scan.

    :param output: flag specifying whether to view the pointcloud when it has
        been received.
    :param view: flag specifying wheter to print status information to the
        terminal.
    """
    global options
    if not output:
        output = ''.join(["/tmp/scan_",str(math.floor(time.time())), ".pcd"])
    options['file_path'] = output
    client = TreeDClientNode(options)
    client.scan()

    if view:
        subprocess.call(["pcl_viewer", options['file_path']])

@treed.command()
@common_options
@click.option('--cart-position', type=click.IntRange(0, 744), help="The position of the cart")
@click.option('--cart-relative-position', type=click.IntRange(-744, 744), help="The relative position of the cart")
@click.option('--cart-speed', type=click.IntRange(1,600), help="The speed of the cart")
@click.option('--table-curve', type=click.IntRange(-20, 90), help="The curvature of the table")
@click.option('--table-rotation', type=click.IntRange(0, 359), help="The rotation of the table")
def set(cart_position, cart_relative_position, cart_speed, table_curve, table_rotation):
    """
    Function that gets called when the set command is input. This command is
    used to set a setting to a certain value.

    :param cart-position: flag specifying the absolute position of the
        lineardrive.
    :param cart-relative-position: flag used to specify the relative position
        of the lineardrive.
    :param cart-speed: flag used to specify the speed of the lineardrive.
    :param table-curve: flag used to set the curve of the rotational board.
    :param table-rotation: flag used to set the rotation of the rotational
        board.
    """
    client = TreeDClientNode(options)

    if not cart_position is None:
        client.set_cart_position(cart_position)

    if not cart_relative_position is None:
        client.set_cart_relative_position(cart_relative_position)

    if not cart_speed is None:
        client.set_cart_speed(cart_speed)

    if not table_curve is None:
        client.set_table_curve(table_curve)

    if not table_rotation is None:
        client.set_table_rotation(table_rotation)


@treed.command()
@common_options
@click.option('--cart', is_flag=True, help="Reset the linear cart")
@click.option('--table', is_flag=True, help="Reset the board")
def reset(cart, table):
    """
    Function called when using the reset command. This command is used to reset
    a device to default position.

    :param cart: option used to reset the lineardrive.
    :param table: option used to reset the rotational board.
    """
    client = TreeDClientNode(options)

    if cart:
        client.reset_lineardrive()

    if table:
        client.reset_board()

@treed.command()
@common_options
@click.option('--cart-position', is_flag=True, help="Key of the setting to change")
@click.option('--cart-speed', is_flag=True, help="The speed of the cart")
@click.option('--table-curve', is_flag=True, help="The curvature of the table")
@click.option('--table-rotation', is_flag=True, help="The rotation of the table")
def get(cart_position, cart_speed, table_curve, table_rotation):
    """
    Function that gets called when the get command is input. The get command is
    used to get the value of a setting.

    :param cart-position: option to get the position of the lineardrive.
    :param cart-speed: option used to get the current speed of the lineardrive.
    :param table-curve: option used to get the curve of the rotational board.
    :param table-rotation: option used to get the current rotation of the board.
    """
    client = TreeDClientNode(options)

    if cart_position:
        res = client.get_cart_position()
        print("Cart position is:",res)

    if cart_speed:
        res = client.get_cart_speed()
        print("Cart speed is:",res)

    if table_curve:
        res = client.get_table_curve()
        print("Table curve is:",res)

    if table_rotation:
        res = client.get_table_rotation()
        print("Table rotation is:",res)


def write_default_config():
    """
    Write the default options to file.
    """
    config = cp.ConfigParser()
    config['DEFAULT'] = default_options
    with open(DEFAULT_CONFIG_PATH, 'w') as configfile:
        config.write(configfile)

def get_config():
    """
    Function used to read configuration files.
    """
    global options
    config = cp.ConfigParser()
    result = []
    for conf_path in CONFIG_FILES:
        result = config.read(conf_path)
        if result:
            break

    if not result:
        write_default_config()
        result = config.read(DEFAULT_CONFIG_PATH)
        options = result
    return options
