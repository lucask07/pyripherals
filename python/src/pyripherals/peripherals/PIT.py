from ..core import Endpoint, Register
import copy
import numpy as np

Endpoint.pit=Endpoint.get_chip_endpoints('PIT')

class PIT:
    """Class for PIT timer on the FPGA with a WishBone interface.

    Attributes
    ----------
    WB_SET_ADDRESS : int
        Set address command for the Wishbone.
    WB_WRITE : int
        Write command for the Wishbone.
    WB_READ : int
        Read command for the Wishbone.
    WB_CLK_FREQ : int
        Frequency of the clock the Wishbone runs on.
    fpga : FPGA
        FPGA instance this controller uses to communicate.
    endpoints : dict
        Endpoints on the FPGA this controller uses to communicate.
    master_config : int
        Value of the CTRL register in the Wishbone.
    registers : dict
        Name-Register pairs for the internal registers of the Wishbone.
    """

    WB_CLK_FREQ = 200e6  # clk_sys = 200 MHz in the top_level_module.v comments
    COUNT_DEPTH = 2**16-1 # 16 bits
    # NOTE: resulting timing frequency is 200/2 
    # NOTE: the NO_PRESCALE is a synthesis option 
    registers = Register.get_chip_registers('PIT')

    def __init__(self, fpga, endpoints, master_config=registers.get('CTRL').default):
        self.fpga = fpga
        self.master_config = master_config
        self.endpoints = endpoints

    @classmethod
    def create_chips(cls, fpga, number_of_chips, endpoints=Endpoint.pit, master_config=None):
        """Instantiate a number of new chips.

        The number must be an integer greater than zero. The endpoints between
        each instance will be incremented. If the endpoints argument is left
        as None, then we will use copies of the endpoints_from_defines
        dictionary for the endpoints for each instance, and update that
        original dictionary when we increment the endpoints. This way, the
        endpoints there are ready for another instantiation if needed.
        """

        if type(number_of_chips) is not int or number_of_chips <= 0:
            print('number_of_chips must be an integer greater than 0')
            return False

        chips = []
        if master_config is None:
            # Use class default for master_config
            for i in range(number_of_chips):
                chips.append(cls(fpga=fpga, endpoints=endpoints))
                # Use deepcopy to keep the endpoints for different instances separate
                endpoints = copy.deepcopy(chips[-1].endpoints)
                Endpoint.advance_endpoints(endpoints)
        else:
            for i in range(number_of_chips):
                chips.append(cls(fpga=fpga, endpoints=copy.deepcopy(
                    endpoints), master_config=master_config))
                # Use deepcopy to keep the endpoints for different instances separate
                endpoints = copy.deepcopy(chips[-1].endpoints)
                Endpoint.advance_endpoints(endpoints)

        if endpoints is None:
            # Increment shared endpoints dictionary
            # We need to get the shared endpoints in endpoints_from_defines to
            # increment and we only have the endpoints given to us in the
            # argument.
            shared_full_eps = Endpoint.endpoints_from_defines
            shared_chip_eps = shared_full_eps[
                list(shared_full_eps.keys())[
                    list(shared_full_eps.values()).index(chips[0].endpoints)
                ]
            ]
            Endpoint.advance_endpoints(shared_chip_eps, number_of_chips)
        else:
            # Increment custom dictionary
            Endpoint.advance_endpoints(endpoints, number_of_chips)

        return chips

    def wb_write(self, reg, data):
        # reg is name of PIT register (CTRL - or sub registers in CTRL, CNT, MOD)
        adr = self.registers[reg].address << self.endpoints['ADDR'].bit_index_low

        data = (data & (2**16-1)) << self.registers[reg].bit_index_low 
        mask = (2**self.registers[reg].bit_width - 1) << self.registers[reg].bit_index_low
        mask = mask ^ 0xffff
        if self.registers[reg].address == self.registers['CTRL'].address: 
            current_ctrl = self.wb_read('CTRL')
            print(f'Current ctrl registers {hex(current_ctrl)}')
            print(f'Mask {hex(mask)}')
            data = data | (current_ctrl & mask)

        we = 1 << self.endpoints['WE'].bit_index_low
        cycle = 1 << self.endpoints['CYCLE'].bit_index_low
        strobe = 1 << self.endpoints['STROBE'].bit_index_low
        reset = 1 << self.endpoints['RESET'].bit_index_low

        command = (adr|data|we|cycle|strobe|reset)
        print(f'WB write: PIT register {hex(adr)}, value {hex(command)}')
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, command , 0xffffffff)

        # moved strobe to a wire bit because single cycle might not be possible
        #self.fpga.send_trig(self.endpoints['STROBE'])

        # deassert cycle and we 
        mask = we | cycle | strobe
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, 0, mask)

    def wb_read(self, reg):
        """ Select a register and read data selected register."""
        # reg is name of PIT register (CTRL - or sub registers in CTRL, CNT, MOD)
        adr = self.registers[reg].address << self.endpoints['ADDR'].bit_index_low
        cycle = 1 << self.endpoints['CYCLE'].bit_index_low
        strobe = 1 << self.endpoints['STROBE'].bit_index_low
        reset = 1 << self.endpoints['RESET'].bit_index_low

        command = (adr|cycle|strobe|reset) # we is 0 since reading! 
        print(f'WB read: {hex(adr)}, {hex(command)}')
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, command , 0xffffffff)

        #self.fpga.send_trig(self.endpoints['STROBE'])
        data = self.fpga.read_wire(self.endpoints['WIRE_OUT'].address)
        print(f'PIT wishbone raw reading {hex(data)}')
        data = data >> self.registers[reg].bit_index_low
        data = data & (2**self.registers[reg].bit_width-1)

        # deassert cycle and strobe
        mask = cycle|strobe
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, 0 , mask)

        return data 

    def wb_reset(self):
        self.fpga.send_trig(self.endpoints['WB_RESET'])

    def reset(self):
        # active low 
        command = 0 << self.endpoints['RESET'].bit_index_low 
        mask = 1 << self.endpoints['RESET'].bit_index_low 
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, command, mask)


    def reset_clear(self):
        # reset is active low
        command = 1 << self.endpoints['RESET'].bit_index_low 
        mask = 1 << self.endpoints['RESET'].bit_index_low 
        self.fpga.set_wire(
            self.endpoints['WIRE_IN'].address, command, mask)

    def set_period(self, T):
        # target period in seconds 
        # use prescalar power of 2 mode (not decade)
        # requires that wb.read('DECADE_EN') returns 0 

        # find prescalar value 
        cnt = T/(1/PIT.WB_CLK_FREQ) # total cnt, might overflow and require prescaler 
        
        if cnt < (PIT.COUNT_DEPTH):  # COUNT_DEPTH = 65535
            prescaler = 0
            cnt = int(cnt)
        else: # count overflowed and need to use the prescaler 
            prescaler = int(np.ceil(np.log2(cnt/(PIT.COUNT_DEPTH))))
            cnt = int(cnt/2**prescaler)
        if prescaler > 15:
            print('ERROR - prescaler is too large. Period must be below 10.73 seconds')

        self.stop()
        print(f'Stopping PIT timer')
        print(f'Set period with prescaler {prescaler} and count {cnt}')
        self.wb_write('MOD', cnt)
        self.wb_write('PRE_SCALR', prescaler)
        print(f'Starting PIT timer')
        self.start()

    def stop(self):
        self.wb_write('CNT_EN', 0)

    def start(self):
        self.wb_write('CNT_EN', 1)