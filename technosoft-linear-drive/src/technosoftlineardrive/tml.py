STANDARD_AXIS = 0x0ff0

def create_operand(opcode, name, number_of_operands, axis=STANDARD_AXIS):
    """
    Create a assembly operand from machine code generated from tml code.
    
    :param opcode: machine code for operation code in tml.
    :param name: a human readable name of operand, used for debugging.
    :param number_of_operands: number of parameters tml operation takes.
    :param axis: to use in the linear drive.
    :return create_instruction: function to create the operand.
    """
    assert number_of_operands <= 4
    assert number_of_operands >= 0
    assert opcode <= 2**16
    assert opcode >= 0
    assert axis <= 2**16
    assert axis >= 0

    def create_instruction(*args):
        assert len(args) == number_of_operands
        for i in args:
            assert i <= 2**16
            assert i >= 0

        return Instruction(opcode, list(args), human_readable=name+str(args))

    return create_instruction


class Instruction:
    """
    A small abstraction layer over raw data dumps. This class allows for usage of
    abstract tml instructions in a somewhat safe way.

    :Example:

    >>> inst = Instruction(0x24a2)
    >>> print(inst.assemble())

    """
    
    def __init__(self, opcode, data=[], axis=STANDARD_AXIS, human_readable=""):
        """
        Constructor of Instruction class.

        :param opcode: for the instruction.
        :param data: list of data to send with instruction.
        :param axis: to use on linear drive.
        :param human_readable: text used for debugging.
        """
        assert len(data) <= 4
        assert axis <= 2**16
        assert axis >= 0
        assert opcode <= 2**16
        assert opcode >= 0
        assert isinstance(data, list)
        assert isinstance(human_readable, str)
        for i in data:
            assert i <= 2**16
            assert i >= 0

        self.opcode = opcode
        self.data = data
        self.axis = axis
        self.human_readable = human_readable

    def assemble(self):
        """
        Create a list of numbers representing the instruction. Each number is in
        [0,255].

        :return message8: of instruction to send to linear drive.
        """
        message_length = 4 + 2*len(self.data)
        message16 = [self.axis, self.opcode] + self.data

        # Need to split up into high byte and low byte
        message8 = []
        for word16 in message16:
            message8 += [(word16 // 2**8)]
            message8 += [(word16  % 2**8)]

        # Checksum is sum of all bytes module 2**8
        message8 = [message_length] + message8 + [(sum(message8) + message_length) % 2**8]
        return message8

    def string(self):
        """
        Return a human-readable string representation of the instruction.
        
        :return human_readable: for easy debugging of instructions.
        """
        if self.human_readable == "":
            return str(hex(self.assemble()))
        return self.human_readable


def assemble_program(program):
    """
    Assemble a list of instructions into a list of list of integers in [0,255]
    
    :param program: as a list of instructions to turn into a assembled program.
    :return complete_program: assembled to send to the technosoft linear drive. 
    """
    complete_program = []
    for instruction in program:
        complete_program.append(instruction.assemble())
    return complete_program

# Instructions generated from EasyMotion Studio in TML.
CACC = create_operand(0x24a2, "cacc", 2) # Set acceleration
CSPD = create_operand(0x24a0, "cspd", 2) # Set speed
CPOS = create_operand(0x249e, "cpos", 2) # Move to position
CSET = create_operand(0x5909, "cset", 2) # Set position stuff
UPD = create_operand(0x0108, "upd", 0)   # Execute immediate / Update
NOTMC = create_operand(0x700F, "!mc", 0) # ?
WAIT = create_operand(0x0408, "wait", 0) # Wait!
MASTERID = create_operand(0x2327, "masterid", 1) # Set host address
SRH_MASK = create_operand(0x2363, "srh_mask", 1)
SRL_MASK = create_operand(0x2362, "srl_mask", 1)

