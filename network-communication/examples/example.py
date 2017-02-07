
def do_thing():
    return("Hello world!")


def do_thing2(arg):
    return "I received " + str(arg)


def reuse_network(network):
    network.generate_message('something', 'with data')


def do_error():
    raise Exception("This is an error", 10)
