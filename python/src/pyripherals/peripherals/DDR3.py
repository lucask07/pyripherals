from ..core import Endpoint
from ..utils import test_bit, gen_mask, custom_signed_to_int
import numpy as np
import time
import os
import h5py


class DDR3():
    """
    The DDR is divided into 2 buffers. Each buffer has an incoming and outgoing FIFO.

    **1st buffer:**

    * Function generator like data that provides a data-stream to the DACs
    * Write from the host when DAC_WRITE_ENABLE is set (clear dac read and clear adc read and write)
    * Organized into groups of 16 bits to 8 channels
    * Read to the DACs with set_dac_read
    * 6 AD5453 (14 bits each)
    * 2 DAC80508 (16 bits each) 
    * DAC80508 channel data in the MSBs of the AD5453 stream
    * FIFO: OpalKelly in -> out to DDR [32bits x 1024words = 4096 bytes]
    * FIFO: DDR in -> DAC data out [out: 256bits x 256 = 8192 bytes]

    **2nd buffer:**

    * Buffering for ADC data (AD7961; ADS8686), DAC outgoing data, timestamps.
    * ADC data writes to the DDR set_adc_write()
    * ADC data in DDR can be read to the host with set_adc_read()
    * The DDR througput is sufficient for 1st buffer reading while reading and writing to the 2nd buffer
    * FIFO: OpalKelly in -> out to DDR [in: 32bits x 1024words = 4096 bytes]
    * FIFO: DDR in -> OpalKelly out [256bits x 128 = 4096 bytes] OpalKelly block size is half this.

    **Expected sequence of operations:**

    1) At startup write pattern for DACs using write_channels() [set_dac_write(), clear_dac_write() are called within write_channels()]
    2) set_dac_read() # starts DAC data output to DACs via SPI and then set_adc_write() ADC data captured into DDR
    3) set_adc_read() # allows host to read PipeOut as PipeOut is continuously filled if emptied

    **ADC data ordering and saving:**

    * deswizzle()
    * save_data()

    **DDR configuration bits:**

    * DAC_WRITE_ENABLE
    * DAC_READ_ENABLE
    * ADC_WRITE_ENABLE
    * ADC_TRANSFER_ENABLE
    
    """

    BLOCK_SIZE = 2048  # 1/2 the incoming FIFO depth in bytes (size of the BlockPipeIn)
    # number of channels that the DDR is striped between (for DACs)
    NUM_CHANNELS = 8
    UPDATE_PERIOD = 400e-9  # 2.5 MHz -- requires SCLK ~ 50 MHZ
    PORT1_INDEX = 0x3_7f_ff_f8
    NUM_ADC_CHANNELS = 8  # number of 2 byte chunks in DDR
    ADC_PERIOD = 200e-9

    # the index is the DDR address that the circular buffer stops at.
    # need to write all the way up to this stoping point otherwise the SPI output will glitch
    SAMPLE_SIZE = int((PORT1_INDEX + 8)/4)

    def __init__(self, fpga, endpoints=None, data_version='TIMESTAMPS'):
        if endpoints is None:
            endpoints = Endpoint.get_chip_endpoints('DDR3')
        self.fpga = fpga
        self.endpoints = endpoints
        self.data_version = data_version  # sets deswizzling mode

        self.data_arrays = []
        for i in range(DDR3.NUM_CHANNELS):
            self.data_arrays.append(np.zeros(
                DDR3.SAMPLE_SIZE).astype(np.uint16))

        self.clear_adc_debug()

    def set_adc_debug(self):
        """Set the ADC debug bit.

        That bit multiplexes a counter to ADC channel 0 and bits 47:0 of the
        DAC data to ADC channels 1,2,3.

        Not supported in all versions of the FPGA design.
        """

        self.fpga.set_wire_bit(self.endpoints['ADC_DEBUG'].address,
                               self.endpoints['ADC_DEBUG'].bit_index_low)

    def clear_adc_debug(self):
        """Clear the ADC debug bit.

        DDR ADC data will be from the ADC.
        """
        self.fpga.clear_wire_bit(self.endpoints['ADC_DEBUG'].address,
                                 self.endpoints['ADC_DEBUG'].bit_index_low)

    @staticmethod
    def make_flat_voltage(amplitude):
        """Return a constant unit16 array of value amplitude.

        Array length based on the sample_size parameter. The conversion from
        float or int voltage to int digital (binary) code should take place
        BEFORE this function.

        Parameters
        ----------
        amplitude: int
            Digital (binary) value of the flat voltage

        Returns
        -------
            amplitude : numpy.ndarray 
                to be assigned to DDR data array
        """

        amplitude = np.ones(DDR3.SAMPLE_SIZE)*amplitude
        amplitude = amplitude.astype(np.uint16)
        return amplitude

    @staticmethod
    def closest_frequency(freq, sample_size=None):
        """Determine closest frequency so the waveform evenly divides into the length of the DDR3

        Parameters
        ----------
        freq : float
            Desired frequency
        sample_size : int 
            length of the array to populate 
            almost always use None so that sample_size = DDR3.SAMPLE_SIZE
            
        Returns
        -------
        new_frequency : float 
            The closest possible frequency
        """

        if sample_size is None:
            sample_size = DDR3.SAMPLE_SIZE

        samples_per_period = (1/freq) / DDR3.UPDATE_PERIOD

        if samples_per_period <= 2:
            print('Frequency is too high for the DDR update rate')
            return None
        total_periods = sample_size/samples_per_period
        # round and recalculate frequency
        round_total_periods = np.round(total_periods)
        round_samples_per_period = sample_size / \
            round_total_periods
        new_frequency = 1 / \
            (DDR3.UPDATE_PERIOD * round_samples_per_period)

        return new_frequency

    @staticmethod
    def make_sine_wave(amplitude, frequency,
                       offset=0x2000, actual_frequency=True):
        """Return a sine-wave array for writing to DDR.

        The conversion from float or int voltage to int digital (binary) code
        should take place BEFORE this function.

        Parameters
        ----------
        amplitude : int
            Digital (binary) value of the sine wave.
        frequency : float
            Desired frequency in Hz.
        offset : int
            Digital (binary) value offset.
        actual_frequency : bool
            Decide whether closest frequency that fits an integer number of periods is used.

        Returns
        -------
        ddr_seq : numpy.ndarray 
            to be assigned to DDR data array  

        frequency : float
            actual frequency after closest_frequency
        """

        if (amplitude) > offset:
            print('Error: amplitude in sine-wave is too large')
            return -1
        if actual_frequency:
            frequency = DDR3.closest_frequency(frequency)

        t = np.arange(0, DDR3.UPDATE_PERIOD*DDR3.SAMPLE_SIZE,
                      DDR3.UPDATE_PERIOD)
        # print('length of time axis after creation ', len(t))
        ddr_seq = (amplitude)*np.sin(t*frequency*2*np.pi) + offset
        if any(ddr_seq < 0) or any(ddr_seq > (2**16-1)):
            print('Error: Uint16 overflow in make sine wave')
            return -1
        ddr_seq = ddr_seq.astype(np.uint16)
        return ddr_seq, frequency

    @staticmethod
    def make_chirp(amplitude, frequencies, periods,
                       offset=0x2000, actual_frequency=True):
        """Return a chirp multiple sine-wave frequencies array for writing to DDR.

        The conversion from float or int voltage to int digital (binary) code
        should take place BEFORE this function.

        Parameters
        ----------
        amplitude : int
            Digital (binary) value of the sine wave.
        frequencies : np.array (floats)
            Desired frequencies in Hz.
        periods : int or np.array (of ints)
            Number of periods for all frequencies. or one for each
        offset : int
            Digital (binary) value offset.
        actual_frequency : bool
            Decide whether closest frequency that fits an integer number of periods is used.

        Returns
        -------
        ddr_seq : numpy.ndarray 
            to be assigned to DDR data array  

        frequencies : np.array (floats)
            actual frequencies after closest_frequency
        """
        ddr_seq = np.zeros(DDR3.SAMPLE_SIZE)

        if (amplitude) > offset:
            print('Error: amplitude in sine-wave is too large')
            return -1
        
        if type(periods) == int:
            periods = [periods]*len(frequencies)
        if len(periods) != len(frequencies):
            print('Error length of periods must match frequencies')

        idx_left = 0
        frequency_out = []
        indices = []

        for frequency, period in zip(frequencies, periods):
            if frequency != frequencies[-1]:
                chirp_length = int(period*(1/frequency)/DDR3.UPDATE_PERIOD)
            else:
                chirp_length = DDR3.SAMPLE_SIZE - idx_left

            if actual_frequency:
                frequency = DDR3.closest_frequency(frequency, chirp_length)
            

            t = np.arange(0, DDR3.UPDATE_PERIOD*chirp_length,
                        DDR3.UPDATE_PERIOD)
            # print('length of time axis after creation ', len(t))
            ddr_seq_tmp = (amplitude)*np.sin(t*frequency*2*np.pi) + offset
            if any(ddr_seq_tmp < 0) or any(ddr_seq_tmp > (2**16-1)):
                print('Error: Uint16 overflow in make sine wave')
                return -1
            ddr_seq[idx_left:(idx_left + len(t))] = ddr_seq_tmp.astype(np.uint16)
            frequency_out.append(frequency)
            indices.append((idx_left, idx_left + len(t)))

            idx_left = idx_left + len(t)
        return ddr_seq, frequency_out, indices


    @staticmethod
    def make_ramp(start, stop, step, actual_length=True):
        """Create a ramp signal to write to the DDR.

        The conversion from float or int voltage to int digital (binary) code
        should take place BEFORE this function.

        Parameters
        ----------
        start : int
            Digital (binary) value to start the ramp at
        stop : int
            Digital (binary) value to stop the ramp at
        step : int
            Digital (binary) code to step by

        Returns
        -------
            ddr_seq : numpy.ndarray 
                to be assigned to DDR data array        
        """

        # change the stop value for integer number of cycles
        ramp_seq = np.arange(start, stop, step)
        len_ramp_seq = len(ramp_seq)
        if actual_length:
            length = int(
                DDR3.SAMPLE_SIZE/np.round(DDR3.SAMPLE_SIZE/len_ramp_seq))
            stop = start + length*step
            ramp_seq = np.arange(start, stop, step)
        num_tiles = DDR3.SAMPLE_SIZE//len(ramp_seq)
        extras = DDR3.SAMPLE_SIZE % len(ramp_seq)
        ddr_seq = np.tile(ramp_seq, num_tiles)
        ddr_seq = np.hstack((ddr_seq, ramp_seq[0:extras]))
        ddr_seq = ddr_seq.astype(np.uint16)
        return ddr_seq

    @staticmethod
    def make_step(low, high, length, actual_length=True, duty=50):
        """Return a step signal (square wave) to write to the DDR.

        The conversion from float or int voltage to int digital (binary) code
        should take place BEFORE this function.

        Parameters
        ----------
        low : int
            Digital (binary) code for the low value of the step.
        high : int
            Digital (binary) code for the high value of the step.
        length : TODO add type for length
            TODO add description for length
        actual_length : bool
            TODO add description for actual_length
        duty : int or float
            Duty cycle percentage. Enter as a percentage [0.0, 100.0].

        Returns
        -------
        ddr_seq : numpy.ndarray 
            to be assigned to DDR data array  
        """

        if actual_length:
            length = int(
                DDR3.SAMPLE_SIZE/np.round(DDR3.SAMPLE_SIZE/length))
        l_first = int(length/100*duty)
        l_end = int(length/100*(100-duty))
        ramp_seq = np.concatenate((np.ones(l_first)*low, np.ones(l_end)*high))
        num_tiles = DDR3.SAMPLE_SIZE//len(ramp_seq)
        extras = DDR3.SAMPLE_SIZE % len(ramp_seq)
        ddr_seq = np.tile(ramp_seq, num_tiles)
        ddr_seq = np.hstack((ddr_seq, ramp_seq[0:extras]))
        ddr_seq = ddr_seq.astype(np.uint16)
        return ddr_seq

    def write_channels(self, set_ddr_read=True):
        """Write the channels as striped data to the DDR."""

        data = np.zeros(
            int(len(self.data_arrays[0])*DDR3.NUM_CHANNELS))
        data = data.astype(np.uint16)

        for i in range(DDR3.NUM_CHANNELS):
            if i % 2 == 0:  # extra order swap on the 32 bit wide pipe
                data[(7-i - 1)::8] = self.data_arrays[i]
            else:
                data[(7-i + 1)::8] = self.data_arrays[i]

        print('Length of data DDR data [2 byte words] = {}'.format(len(data)))
        return self.write_buf(bytearray(data), set_ddr_read=set_ddr_read)

    def write_buf(self, buf, set_ddr_read=True):
        """Write a bytearray to the DDR3.

        Parameters
        ----------
        buf : bytearray
            bytearray to write to the DDR

        Returns
        -------
        block_pipe_return : int
            length of the buffer written to the DDR (or error code if unsuccessful)
        speed_MBs : float
            speed of the write in MB/s
        """

        print('Length of buffer being written to DDR [bytes]: ', len(buf))
        self.clear_dac_read()
        self.reset_fifo(name='DAC_IN')
        self.set_dac_write()

        print('Writing to DDR...')
        time1 = time.time()
        block_pipe_return = self.fpga.xem.WriteToBlockPipeIn(epAddr=self.endpoints['BLOCK_PIPE_IN'].address,
                                                             blockSize=DDR3.BLOCK_SIZE,
                                                             data=buf)
        print(f'The length of the DDR write was {block_pipe_return}')

        time2 = time.time()
        time3 = (time2-time1)
        speed_MBs = (int)(block_pipe_return/1024/1024/time3)
        print(f'The speed of the write was {speed_MBs} MB/s')

        # below prepares the HDL into read mode
        self.clear_dac_write()
        if set_ddr_read:
            self.set_dac_read()
            self.set_adc_read()
        return block_pipe_return, speed_MBs

    def reset_mig_interface(self):
        """Reset user interface to the MIG (memory interface generator). Resets
            the DDR address pointers for read/write of both buffers 
            (does not reset the MIG controller).
        """
        self.fpga.send_trig(self.endpoints['UI_RESET'])

    def fifo_status(self):
        """Check the empty, full, and count status of the DDR interfacing FIFOs

        Returns
        -------
        fifo_status : dict 
            dictionary of fifo status ('EMPTY', 'FULL', 'ADC_DATA_COUNT') for 'IN', 'OUT', and channels 1, 2
        """
        wire_status = self.fpga.read_wire(
            self.endpoints['INIT_CALIB_COMPLETE'].address)
        fifo_status = {}
        for fe in ['EMPTY', 'FULL']:
            for num in [1, 2]:
                for inout in ['IN', 'OUT']:
                    ep_name = '{}{}_{}'.format(inout, num, fe)
                    bit_pos = self.endpoints[ep_name].bit_index_low
                    val = test_bit(wire_status, bit_pos)
                    fifo_status[ep_name] = val
                    print('{} = {}'.format(ep_name, val))

        ep_name = 'ADC_DATA_COUNT'
        cnt_bit_low = self.endpoints[ep_name].bit_index_low
        cnt_bit_width = self.endpoints[ep_name].bit_width
        cnt_msk = gen_mask(np.arange(cnt_bit_low, cnt_bit_width))
        fifo_status[ep_name] = (wire_status & cnt_msk) >> cnt_bit_low
        print('{} = {}'.format(ep_name, val))

        # self.print_fifo_status(fifo_status)
        return fifo_status

    def print_fifo_status(self, fifo_status):
        """Print the FIFO status dictionary.

        Parameters
        ----------
        fifo_status : dict
            Dictionary of fifo status.
        """

        for k in fifo_status:
            print('{} = {}'.format(k, fifo_status[k]))

    def set_dac_read(self):
        """Set DDR / FIFOs read enable. Enables DDR data going to the 
        DACs and ADC data into DDR
        """

        self.fpga.set_wire_bit(self.endpoints['DAC_READ_ENABLE'].address,
                               self.endpoints['DAC_READ_ENABLE'].bit_index_low)

    def clear_dac_read(self):
        """Clear DDR / FIFOs read enable. Stops DDR data from going to 
        the DACs and ADC data into DDR
        """

        self.fpga.clear_wire_bit(self.endpoints['DAC_READ_ENABLE'].address,
                                 self.endpoints['DAC_READ_ENABLE'].bit_index_low)

    def set_dac_write(self):
        """Set DDR / FIFOs write enable into DDR via Pipe.
        """

        self.fpga.set_wire_bit(self.endpoints['DAC_WRITE_ENABLE'].address,
                               self.endpoints['DAC_WRITE_ENABLE'].bit_index_low)

    def clear_dac_write(self):
        """Clear DDR / FIFOs write enable into DDR via Pipe.
        """

        self.fpga.clear_wire_bit(self.endpoints['DAC_WRITE_ENABLE'].address,
                                 self.endpoints['DAC_WRITE_ENABLE'].bit_index_low)

    def set_adc_write(self):
        """Set DDR / FIFOs write enable ADC data into DDR.
        """

        self.fpga.set_wire_bit(self.endpoints['ADC_WRITE_ENABLE'].address,
                               self.endpoints['ADC_WRITE_ENABLE'].bit_index_low)

    def set_adc_dac_simultaneous(self):
        """
        Set DDR / FIFOs read enable. Enables DDR data going to the 
        DACs and ADC data into DDR
        """
        bit_addr = [self.endpoints['ADC_WRITE_ENABLE'].bit_index_low,
                    self.endpoints['DAC_READ_ENABLE'].bit_index_low,
                    self.endpoints['ADC_TRANSFER_ENABLE'].bit_index_low]
        bit_vals = [1, 1, 1]

        self.fpga.set_ep_simultaneous(self.endpoints['ADC_WRITE_ENABLE'].address,
                                      bit_addr, bit_vals)

    def clear_adc_write(self):
        """Clear DDR / FIFOs write enable ADC data into DDR.
        """

        self.fpga.clear_wire_bit(self.endpoints['ADC_WRITE_ENABLE'].address,
                                 self.endpoints['ADC_WRITE_ENABLE'].bit_index_low)

    def set_adc_read(self):
        """Set DDR / FIFO read enable DDR data from the ADCs out via a PipeOut.
        """
        self.fpga.set_wire_bit(self.endpoints['ADC_TRANSFER_ENABLE'].address,
                               self.endpoints['ADC_TRANSFER_ENABLE'].bit_index_low)

    def clear_adc_read(self):
        """Clear DDR / FIFO read enable DDR data from the ADCs out via a PipeOut.
        """

        self.fpga.clear_wire_bit(self.endpoints['ADC_TRANSFER_ENABLE'].address,
                                 self.endpoints['ADC_TRANSFER_ENABLE'].bit_index_low)

    def set_adcs_connected(self):
        """Set that AD7961s are connected to the FPGA so that DDR3 write enable comes from 
            the AD7961.v module.
        """
        self.fpga.set_wire_bit(self.endpoints['USE_ADC_READY'].address,
                               self.endpoints['USE_ADC_READY'].bit_index_low)

    def clear_adcs_connected(self):
        """ AD7961s are not connected to the FPGA so the DDR3 write enable signal is emulated
            by the global_timing module.
        """
        self.fpga.clear_wire_bit(self.endpoints['USE_ADC_READY'].address,
                                 self.endpoints['USE_ADC_READY'].bit_index_low)

    def reset_fifo(self, name):
        """Reset FIFO interfaces to DDR .
        """
        fifo_reset_names = ['DAC_IN', 'DAC_READ', 'ADC_IN', 'ADC_TRANSFER']
        if (name not in fifo_reset_names) and (not name == 'ALL'):
            print(
                f'DDR3 FIFO reset name of {name} is not in list of {fifo_reset_names}')

        else:
            if name == 'ALL':
                for name_tmp in fifo_reset_names:
                    ep_name = 'FIFO_' + name_tmp + '_RST'
                    self.fpga.set_wire_bit(self.endpoints[ep_name].address,
                                           self.endpoints[ep_name].bit_index_low)
                    self.fpga.clear_wire_bit(self.endpoints[ep_name].address,
                                             self.endpoints[ep_name].bit_index_low)
            else:
                ep_name = 'FIFO_' + name + '_RST'
                self.fpga.set_wire_bit(self.endpoints[ep_name].address,
                                       self.endpoints[ep_name].bit_index_low)
                self.fpga.clear_wire_bit(self.endpoints[ep_name].address,
                                         self.endpoints[ep_name].bit_index_low)

    def adc_single(self):
        """Set ADC read address to the ADC write address.

        Emulates an immediate "trigger" of an oscilloscope.
        """

        self.fpga.set_wire_bit(self.endpoints['ADC_ADDR_SET'].address,
                               self.endpoints['ADC_ADDR_SET'].bit_index_low)
        self.fpga.send_trig(self.endpoints['ADC_ADDR_RESET'])

    def read_adc_block(self, sample_size=None, source='ADC', DEBUG_PRINT=False):
        """Read ADC (and other) DDR data. 
        Block size must be a power of two from 16 to 16384
        will automatically perform multiple transfers to complete the full LENGTH.
        The length must be an integer multiple of 16 for USB3.0
        and the length must be an Integer multiple of Block Size.
        see https://docs.opalkelly.com/fpsdk/frontpanel-api/ section 3.3.1

        Parameters
        ----------
        sample_size : int
            Length of read in bytes. If none uses DDR parameter 'sample_size.'

        source : str
            FIFO output buffer to read. Either 'ADC' or 'FG'. 'FG' just reads
            back what is written for DACs (as function generator) so not so
            useful.

        Returns
        -------
        data_buf : byearray
            adc data read as a bytearray
        read_cnt : int
            The count (or error code) read from the OpalKelly interface
        """

        if sample_size is None:
            data = np.zeros((DDR3.SAMPLE_SIZE,), dtype=int)
            data_buf = bytearray(data)
        else:
            data_buf = bytearray(sample_size)

        block_size = DDR3.BLOCK_SIZE
        # check block size
        if block_size % 16 != 0:
            print('Error in read adc. Block size is not a multiple of 16')
            return -1, -1
        if block_size > 16384:
            print('Error in read adc. Block size is greater than 16384')
            return -2, -2

        if source == 'ADC':
            read_cnt = self.fpga.xem.ReadFromBlockPipeOut(epAddr=self.endpoints['BLOCK_PIPE_OUT'].address,
                                                          blockSize=block_size,
                                                          data=data_buf)

        elif source == 'FG':  # TODO: test, don't expect this to work
            read_cnt = self.fpga.xem.ReadFromBlockPipeOut(epAddr=self.endpoints['BLOCK_PIPE_OUT_FG'].address,
                                                          blockSize=block_size,
                                                          data=data_buf)
        else:
            print('Incorrect source in read_adc')
            return -11, -11

        if DEBUG_PRINT:
            print(
                f'The length [num of bytes] of the BlockPipeOut read is: {read_cnt}')
        return data_buf, read_cnt

    def deswizzle(self, d, convert_twos=True):
        """Reorder DDR data to match the ADC channels. Shift MSBytes up by 8 
        and combine with LSBytes. Swap channels to match ADC channel numbering.

        Parameters
        ----------
        d : array
            array of bytes. 
        convert_twos : Boolean
            if true converts data to signed

        Returns
        -------
        chan_data : dict
            dictionary of data arrays (keys are channel numbers)

        """
        bits = 16
        chan_data_swz = {}  # this data is swizzled

        # first version of ADC data before DACs + timestamps are stored
        if self.data_version == 'ADC_NO_TIMESTAMPS':
            for i in range(4):
                chan_data_swz[i] = (d[(0 + i * 2):: 8] << 0) + \
                    (d[(1 + i * 2):: 8] << 8)

            chan_data = {}
            chan_data[0] = chan_data_swz[2]
            chan_data[1] = chan_data_swz[3]
            chan_data[2] = chan_data_swz[0]
            chan_data[3] = chan_data_swz[1]
            if convert_twos:
                for i in range(4):
                    chan_data[i] = custom_signed_to_int(chan_data[i], bits)

        # first version of ADC data before DACs + timestamps are stored
        if self.data_version == 'TIMESTAMPS':
            for i in range(8):
                chan_data_swz[i] = (d[(0 + i * 2):: 16] << 0) + \
                    (d[(1 + i * 2):: 16] << 8)

            chan_data = {}
            chan_data[0] = chan_data_swz[6]
            chan_data[1] = chan_data_swz[7]
            chan_data[2] = chan_data_swz[5]
            chan_data[3] = chan_data_swz[4]

            chan_data[4] = chan_data_swz[2]
            chan_data[5] = chan_data_swz[3]
            chan_data[6] = chan_data_swz[0]
            chan_data[7] = chan_data_swz[1]

        return chan_data

    def data_to_names(self, chan_data, bitfile_version=None):
        """
        Put deswizzled data into dictionaries with names that match with the data sources. 
        Complete twos complement conversion where necessary. Check timestamps for skips.
        Check the constant values for errors.

        This supports 2 versions of the FPGA code:
        'ADC_NO_TIMESTAMPS': DDR data is only the fast ADC. AD7961
        'TIMESTAMPS': DDR data is numerous. AD7961, AD5453 out, ADS8686, timestamps, readcheck


        Parameters
        ----------
        chan_data : dict of np.arrays
            data from reading DDR (minimally processed into 2 byte containers)
        bitfile_version : int
            The bitfile version that the .h5 file which held chan_data was created with. Defaults to the bitfile version of the FPGA
        
        Returns
        -------
        adc_data : dict 
            fast adc data (double format @ 5 MSPS)
        timestamp : np.array  
            timestamps
        dac_data : dict 
            DAC output data
        ads : dict 
            ADS8686 ADC data (double format @ 1 MSPS)

        ads_seq_cnt : dict
            two keys 0, 1. Each value is an array of sequence counts (counts from 0 to 23)
            the first key will have 0,2,4,6, ... , 22 (or similar)
            the second key will have 1,3,5, ..., 23 (or similar)

        error : boolean 
            if True the constant read values were wrong 
               or the timestamp steps are not all the same
        """

        if bitfile_version is None:
            # Old code, hasn't been updated to pass bitfile_version from .h5 file header
            # Default to FPGA bitfile version, or version 1 if the FPGA bitfile version is None
            if self.fpga.bitfile_version is None:
                bitfile_version = 1
            else:
                bitfile_version = self.fpga.bitfile_version

        # first version of ADC data before DACs + timestamps are stored
        if self.data_version == 'ADC_NO_TIMESTAMPS':
            adc_data = chan_data
            timestamp = np.nan
            read_check = np.nan
            dac_data = np.nan
            ads = np.nan

        # first version of ADC data before DACs + timestamps are stored
        if self.data_version == 'TIMESTAMPS':

            adc_data = {}
            for i in range(4):
                # adc_data[i] = custom_signed_to_int(chan_data[i], 16)
                adc_data[i] = chan_data[i]

            if bitfile_version < 2: # 00.00.02 -> 2
                lsb = chan_data[6][0::5].astype(np.uint64)
            else:
                lsb = chan_data[7][1::5].astype(np.uint64)
            mid_b = ((chan_data[6][1::5].astype(np.uint64)) << 16)
            msb = ((chan_data[7][2::5].astype(np.uint64)) << 32)
            t_len = np.size(msb)
            timestamp = (lsb[0:(t_len-1)] +
                         mid_b[0:(t_len-1)] + msb[0:(t_len-1)])

            read_check = {}
            read_check[0] = chan_data[7][3::10]
            read_check[1] = chan_data[7][4::10]
            read_check[2] = chan_data[7][8::10]
            read_check[3] = chan_data[7][9::10]

            ads_seq_cnt = {}
            ads_seq_cnt[0] = (chan_data[7][4::10] & 0x001f)
            ads_seq_cnt[1] = (chan_data[7][9::10] & 0x001f)

            dac_data = {}
            dac_data[0] = chan_data[4][0::2]
            dac_data[1] = chan_data[4][1::2]
            dac_data[2] = chan_data[5][0::2]
            dac_data[3] = chan_data[5][1::2]
            # dac channels 4,5 are available but not every sample. skip for now. TODO: add channels 4,5
            dac_data[4] = chan_data[6][2::5] # observer at 1 MSPS 
            dac_data[5] = chan_data[6][3::5] # observer at 1 MSPS - shifted by 200 ns 
            dac_data[6] = chan_data[6][4::5] # observer sampled at 1 MSPS - shifted by 200 ns twice 
            ads = {}
            ads['A'] = custom_signed_to_int(chan_data[7][0::5], 16)
            if bitfile_version < 2: # 00.00.02 -> 2
                ads['B'] = custom_signed_to_int(chan_data[7][1::5], 16)
            else:
                ads['B'] = custom_signed_to_int(chan_data[6][0::5], 16) # cycle cnt 9 and 4
            error = False
            # check that the constant values are constant
            constant_values = {0: 0xaa55, 1: (0x28b<<5), 2: 0x77bb, 3: (0x28c<<5)}
            constant_value_mask = {0: 0xffff, 1: 0xffe0, 2: 0xffff, 3: 0xffe0}
            for i in range(4):
                if not np.all( (read_check[i] & constant_value_mask[i]) == constant_values[i]):
                    print(f'Error in constant value: {constant_values[i]} ')
                    print(
                        f'Number of errors: {np.sum(read_check[i] != constant_values[i])}')
                    error = True

            # check the timestamps for a skip
            unq_time_intervals = np.unique(np.diff(timestamp))
            if np.size(unq_time_intervals) > 1:
                print('Warning: Multiple time intervals')
                error = True

            return adc_data, timestamp, dac_data, ads, ads_seq_cnt, error

    def save_data(self, data_dir, file_name, num_repeats=4, blk_multiples=40, append=False):
        """
        read and save DDR data to an hdf file 

        Parameters
        ----------
        data_dir : string
            directory for data
        file_name : string
            filename to save data (does not append extension).
        num_repeats : int
            total data read is 2048 bytes * num_repeats * blk_multiples
        blk_multiples : int 
            number of blocks read by adc_read. adc_read is a single OpalKelly API call 

        Returns
        -------
        new_data : np.ndarray
            The new data saved in the h5 file.
        """

        # If the file doesn't already exist, write a new one
        full_data_name = os.path.join(data_dir, file_name)
        if append:
            if not os.path.exists(full_data_name):
                print(f'No existing file found at {full_data_name}, creating new file')
                append = False
        else:
            try:
                os.remove(full_data_name)
            except OSError:
                pass

        if append:
            file_mode = 'a'
        else:
            file_mode = 'w'

        chunk_size = int(DDR3.BLOCK_SIZE * blk_multiples / (
            DDR3.NUM_ADC_CHANNELS*2))  # readings per ADC
        repeat = 0
        adc_readings = chunk_size*num_repeats
        # print(f'Anticipated chunk size (readings per channel) {chunk_size}')
        print(
            f'Reading {adc_readings*2/1024} kB per ADC channel for a total of {adc_readings*DDR3.ADC_PERIOD*1000} ms of data')

        self.set_adc_read()  # enable data into the ADC reading FIFO
        #time.sleep(adc_readings*DDR3.ADC_PERIOD)

        # Save ADC DDR data to a file
        with h5py.File(full_data_name, file_mode) as file:
            if append:
                data_set = file['adc']
                new_data_index = data_set.shape[1]
                if data_set.attrs['bitfile_version'] != self.fpga.bitfile_version:
                    raise Exception(f"File {os.path.join(data_dir, file_name)} bitfile version {data_set.attrs['bitfile_version']} does not match FPGA bitfile version {self.fpga.bitfile_version}")
                # Make space for first round of new data. Not needed when not appending because the data set is created with chunk_size space
                data_set.resize(data_set.shape[1] + chunk_size, axis=1)
            else:
                data_set = file.create_dataset("adc", (DDR3.NUM_ADC_CHANNELS, chunk_size), maxshape=(
                    DDR3.NUM_ADC_CHANNELS, None))
                data_set.attrs['bitfile_version'] = self.fpga.bitfile_version
                new_data_index = 0

            while repeat < num_repeats:
                d, bytes_read_error = self.read_adc(blk_multiples)
                if self.data_version == 'ADC_NO_TIMESTAMPS':
                    chan_data = self.deswizzle(d)

                elif self.data_version == 'TIMESTAMPS':
                    chan_data = self.deswizzle(d)

                if DDR3.NUM_ADC_CHANNELS == 4:
                    chan_stack = np.vstack(
                        (chan_data[0], chan_data[1], chan_data[2], chan_data[3]))

                if DDR3.NUM_ADC_CHANNELS == 8:
                    chan_stack = np.vstack((chan_data[0], chan_data[1], chan_data[2], chan_data[3],
                                            chan_data[4], chan_data[5], chan_data[6], chan_data[7]))

                repeat += 1
                if repeat == 0:
                    print(f'Chunk size by chan data size {chunk_size}')
                    data_set[:] = chan_stack
                else:
                    data_set[:, -chunk_size:] = chan_stack
                if repeat < num_repeats:
                    data_set.resize(data_set.shape[1] + chunk_size, axis=1)
            new_data = data_set[:, new_data_index:]

        print(f'Done with DDR reading: saved as {full_data_name}')
        return new_data

    def read_adc(self, blk_multiples=2048):
        """ 
        read DDR data into a numpy buffer of bytes

        Parameters
        ----------
        blk_multiples : int
            total size of the read is blk_multiples * block_size  

        Returns
        -------
        d : bytearray
            data as uint32    
        bytes_read_error : int 
            bytes read or error code          
        """

        t, bytes_read_error = self.read_adc_block(  # just reads from the block pipe out
            sample_size=DDR3.BLOCK_SIZE * blk_multiples
        )
        d = np.frombuffer(t, dtype=np.uint8).astype(np.uint32)
        # print(f'Bytes read: {bytes_read_error}')
        return d, bytes_read_error

    def set_index(self, factor, factor2=None):
        """
        No longer used. Index (the DDR address that wraps-around to 0).
        This is now fixed to improve timing performance.
        """
        print('Set index is no longer used. Index is fixed to: {}'.format(
            DDR3.PORT1_INDEX))

    def write_setup(self, data_driven_clock=True):
        """Set up DDR for writing."""

        if data_driven_clock:
            self.set_adcs_connected()
        else:
            self.clear_adcs_connected()
        self.clear_dac_read()
        self.clear_adc_write()
        self.clear_adc_read()    # Stop putting data in outgoing FIFO for Pipe read
        self.reset_fifo(name='ALL')
        self.reset_mig_interface()

    def repeat_setup(self):
        """Setup for reading new data without writing to the DDR again."""

        # stop access to the FIFOs so that after reset of the FIFO(s) no new data is added/extracted
        self.clear_adc_read()
        self.clear_adc_write()
        self.clear_dac_read()
        self.reset_fifo(name='ALL')
        # self.fpga.send_trig(self.endpoints['UI_RESET'])
        self.reset_mig_interface()
        # note that the MIG interface addresses are driven by the FIFOs so will idle
        # until the FIFOs are reenable with write_finish()
        self.write_finish()
        time.sleep(0.01)

    def write_finish(self):
        # reenable both DACs
        self.set_adc_dac_simultaneous()  # enable DAC playback and ADC writing to DDR
