from digitalio import DigitalInOut
import board
import busio
import time
import math
from adafruit_bus_device.spi_device import SPIDevice

WRITE_SINGLE_BYTE = 0x00
WRITE_BURST = 0x40
READ_SINGLE_BYTE = 0x80
READ_BURST = 0xC0

IOCFG2 = 0x00  # GDO2 Output Pin Configuration
IOCFG1 = 0x01  # GDO1 Output Pin Configuration
IOCFG0 = 0x02  # self.GDO0 Output Pin Configuration
FIFOTHR = 0x03  # RX FIFO and TX FIFO Thresholds
SYNC1 = 0x04  # Sync Word, High Byte
SYNC0 = 0x05  # Sync Word, Low Byte
PKTLEN = 0x06  # Packet Length
PKTCTRL1 = 0x07  # Packet Automation Control
PKTCTRL0 = 0x08  # Packet Automation Control
ADDR = 0x09  # Device Address
CHANNR = 0x0A  # Channel Number
FSCTRL1 = 0x0B  # Frequency Synthesizer Control
FSCTRL0 = 0x0C  # Frequency Synthesizer Control
FREQ2 = 0x0D  # Frequency Control Word, High Byte
FREQ1 = 0x0E  # Frequency Control Word, Middle Byte
FREQ0 = 0x0F  # Frequency Control Word, Low Byte
MDMCFG4 = 0x10  # Modem Configuration
MDMCFG3 = 0x11  # Modem Configuration
MDMCFG2 = 0x12  # Modem Configuration
MDMCFG1 = 0x13  # Modem Configuration
MDMCFG0 = 0x14  # Modem Configuration
DEVIATN = 0x15  # Modem Deviation Setting
MCSM2 = 0x16  # Main Radio Control State Machine Configuration
MCSM1 = 0x17  # Main Radio Control State Machine Configuration
MCSM0 = 0x18  # Main Radio Control State Machine Configuration
FOCCFG = 0x19  # Frequency Offset Compensation Configuration
BSCFG = 0x1A  # Bit Synchronization Configuration
AGCCTRL2 = 0x1B  # AGC Control
AGCCTRL1 = 0x1C  # AGC Control
AGCCTRL0 = 0x1D  # AGC Control
WOREVT1 = 0x1E  # High Byte Event0 Timeout
WOREVT0 = 0x1F  # Low Byte Event0 Timeout
WORCTRL = 0x20  # Wake On Radio Control
FREND1 = 0x21  # Front End RX Configuration
FREND0 = 0x22  # Front End TX Configuration
FSCAL3 = 0x23  # Frequency Synthesizer Calibration
FSCAL2 = 0x24  # Frequency Synthesizer Calibration
FSCAL1 = 0x25  # Frequency Synthesizer Calibration
FSCAL0 = 0x26  # Frequency Synthesizer Calibration
RCCTRL1 = 0x27  # RC Oscillator Configuration
RCCTRL0 = 0x28  # RC Oscillator Configuration

# Configuration Register Details - Registers that Loose Programming in SLEEP State

FSTEST = 0x29  # Frequency Synthesizer Calibration Control
PTEST = 0x2A  # Production Test
AGCTEST = 0x2B  # AGC Test
TEST2 = 0x2C  # Various Test Settings
TEST1 = 0x2D  # Various Test Settings
TEST0 = 0x2E  # Various Test Settings

# Command Strobe Registers

SRES = 0x30  # Reset chip
SFSTXON = 0x31  # Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
# If in RX (with CCA): Go to a wait state where only the synthesizer
# is running (for quick RX / TX turnaround).

SXOFF = 0x32  # Turn off crystal oscillator.
SCAL = 0x33  # Calibrate frequency synthesizer and turn it off.
# SCAL can be strobed from IDLE mode without setting manual calibration mode.

SRX = 0x34  # Enable RX. Perform calibration first if coming from IDLE and MCSM0.FS_AUTOCAL=1.
STX = 0x35  # In IDLE state: Enable TX. Perform calibration first
# if MCSM0.FS_AUTOCAL=1.
# If in RX state and CCA is enabled: Only go to TX if channel is clear.

SIDLE = 0x36  # Exit RX / TX, turn off frequency synthesizer and exit Wake-On-Radio mode if applicable.
SWOR = 0x38  # Start automatic RX polling sequence (Wake-on-Radio)
# as described in Section 19.5 if WORCTRL.RC_PD=0.

SPWD = 0x39  # Enter power down mode when CSn goes high.
SFRX = 0x3A  # Flush the RX FIFO buffer. Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
SFTX = 0x3B  # Flush the TX FIFO buffer. Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
SWORRST = 0x3C  # Reset real time clock to Event1 value.
SNOP = 0x3D  # No operation. May be used to get access to the chip status byte.

PATABLE = 0x3E  # PATABLE
TXFIFO = 0x3F  # TXFIFO
RXFIFO = 0x3F  # RXFIFO

# Status Register Details

PARTNUM = 0xF0  # Chip ID
VERSION = 0xF1  # Chip ID
FREQEST = 0xF2  # Frequency Offset Estimate from Demodulator
LQI = 0xF3  # Demodulator Estimate for Link Quality
RSSI = 0xF4  # Received Signal Strength Indication
MARCSTATE = 0xF5  # Main Radio Control State Machine State
WORTIME1 = 0xF6  # High Byte of WOR Time
WORTIME0 = 0xF7  # Low Byte of WOR Time
PKTSTATUS = 0xF8  # Current GDOx Status and Packet Status
VCO_VC_DAC = 0xF9  # Current Setting from PLL Calibration Module
TXBYTES = 0xFA  # Underflow and Number of Bytes
RXBYTES = 0xFB  # Overflow and Number of Bytes
RCCTRL1_STATUS = 0xFC  # Last RC Oscillator Calibration Result
RCCTRL0_STATUS = 0xFD  # Last RC Oscillator Calibration Result

#PA_TABLE = [0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
#PA_TABLE = [0xC0, 0x00]
PA_TABLE = [0xFF]



class CC1101:
    def __init__(self, spi, cs, gdo0, baudrate, frequency, syncword, offset=0): #optional frequency offset in Hz
        self.gdo0 = gdo0
        self.device = SPIDevice(spi, cs, baudrate=50000, polarity=0, phase=0)
        self.strobe(SRES) # reset
        self.baudrate = baudrate
        self.setFrequency(frequency, offset)
        self.setBaudRate(baudrate)

        assert len(syncword) == 4
        self.writeSingleByte(SYNC1, int(syncword[:2], 16))
        self.writeSingleByte(SYNC0, int(syncword[2:], 16))

        self.writeBurst(PATABLE, PA_TABLE)      
        self.strobe(SFTX) # flush TX FIFO
        self.strobe(SFRX) # flush RX FIFO


    def compute_cc1101_drate(self, target_baud, f_xtal=26_000_000):
        """
        Return (DRATE_E, DRATE_M, actual_baud)
        """
        target = target_baud * (1 << 28) / f_xtal
        best_err = float('inf')
        best = None
        for e in range(16):
            m = target / (2**e) - 256
            if m < 0 or m > 255:
                continue
            m_int = int(round(m))
            actual = (256 + m_int) * (2**e) * f_xtal / (1 << 28)
            err = abs(actual - target_baud)
            if err < best_err:
                best_err = err
                best = (e, m_int, actual)
        return best

    def compute_gap_bytes(self, baudrate, gap_time_s=0.007):
        """
        Compute number of bytes of 0x00 needed to represent gap_time_s at `baudrate`.
        One byte = 8 bits -> time per byte = 8 / baudrate seconds.
        Returns integer number of bytes (>=1).
        """
        bytes_per_sec = baudrate / 8.0
        gap_bytes = int(round(gap_time_s * bytes_per_sec))
        return max(1, gap_bytes)


    def setFrequency(self, frequency, offset):
        frequency_fp = int((frequency<<16) / 26000000)


        byte2 = (frequency_fp >> 16) & 0xff;
        byte1 = (frequency_fp >>  8) & 0xff;
        byte0 = frequency_fp & 0xff;

        self.writeSingleByte(FREQ2, byte2)
        self.writeSingleByte(FREQ1, byte1)
        self.writeSingleByte(FREQ0, byte0)  
        print(f"Freq, {byte0:02x}, {byte1:02x}, {byte2:02x}")    

    def setBaudRate(self, baudrate):
        drate_e, drate_m, actual_baud = self.compute_cc1101_drate(baudrate)
        self.gap_bytes = self.compute_gap_bytes(actual_baud, gap_time_s=0.007)

        print(f"wanted {baudrate} got {actual_baud}  drate_e {drate_e:02x} drate_m {drate_m:02x}")
        channel_bandwidth = 0b01 # Narrow bandwidth

        modemcfg4 = (channel_bandwidth << 6) | drate_e
        modemcfg3 = drate_m
        modemcfg2 = 0x30  # OOK
        modemcfg1 = 0x0
        modemcfg0 = 0x0

        print(f"mdmcfg3 {modemcfg3:02x} mdmcfg4 {modemcfg4:02x}")
        self.writeSingleByte(MDMCFG0, modemcfg0)
        self.writeSingleByte(MDMCFG1, modemcfg1)
        self.writeSingleByte(MDMCFG2, modemcfg2)  
        self.writeSingleByte(MDMCFG3, modemcfg3)  
        self.writeSingleByte(MDMCFG4, modemcfg4)  



    def getSampleRate(self, freq_xosc = 26000000):
        drate_mantissa = self.readSingleByte(MDMCFG3)
        drate_exponent = self.readSingleByte(MDMCFG4) & 0xF
        sample_rate = (256 + drate_mantissa) * \
            pow(2, drate_exponent - 28) * freq_xosc
        return sample_rate

    def setSampleRate_4000(self):
        self.writeSingleByte(MDMCFG3, 0x43)

   # TODO: Implement set sample rate function
    def setSampleRate(self):
        pass
    
    def setupRX(self):
        self.writeSingleByte(IOCFG2, 0x29)    
        self.writeSingleByte(IOCFG1, 0x2E)    
        self.writeSingleByte(IOCFG0, 0x06)    
        self.writeSingleByte(FIFOTHR, 0x47)   
        self.writeSingleByte(PKTCTRL1, 0x00)  
        self.writeSingleByte(PKTCTRL0, 0x00)  
        self.writeSingleByte(ADDR, 0x00)
        self.writeSingleByte(CHANNR, 0x00)
        self.writeSingleByte(FSCTRL1, 0x08)   
        self.writeSingleByte(FSCTRL0, 0x00)           

        self.writeSingleByte(MDMCFG4, 0xF7)    
        self.writeSingleByte(MDMCFG3, 0x10)    
        self.writeSingleByte(MDMCFG2, 0x32)   
        self.writeSingleByte(MDMCFG1, 0x22)   
        self.writeSingleByte(MDMCFG0, 0xF8)

        self.writeSingleByte(DEVIATN, 0x00)   
        self.writeSingleByte(MCSM2, 0x07)
        self.writeSingleByte(MCSM1, 0x30)     
        self.writeSingleByte(MCSM0, 0x18)
        self.writeSingleByte(FOCCFG, 0x16)
        self.writeSingleByte(BSCFG, 0x6C)
        self.writeSingleByte(AGCCTRL2, 0x06)  
        self.writeSingleByte(AGCCTRL1, 0x00)  
        self.writeSingleByte(AGCCTRL0, 0x95)
        self.writeSingleByte(WOREVT1, 0x87)
        self.writeSingleByte(WOREVT0, 0x6B)
        self.writeSingleByte(WORCTRL, 0xFB)
        self.writeSingleByte(FREND1, 0xB6)    
        self.writeSingleByte(FREND0, 0x11)    
        self.writeSingleByte(FSCAL3, 0xE9)
        self.writeSingleByte(FSCAL2, 0x2A)
        self.writeSingleByte(FSCAL1, 0x00)
        self.writeSingleByte(FSCAL0, 0x1F)
        self.writeSingleByte(RCCTRL1, 0x41)
        self.writeSingleByte(RCCTRL0, 0x00)
        self.writeSingleByte(FSTEST, 0x59)
        self.writeSingleByte(PTEST, 0x7F)   
        self.writeSingleByte(AGCTEST, 0x3F)
        self.writeSingleByte(TEST2, 0x81)     
        self.writeSingleByte(TEST1, 0x35)     
        self.writeSingleByte(TEST0, 0x09)

    def setupTXDEPREECATED(self):
        self.writeSingleByte(IOCFG2, 0x29)    
        self.writeSingleByte(IOCFG1, 0x2E)    
        self.writeSingleByte(IOCFG0, 0x06)    
        self.writeSingleByte(FIFOTHR, 0x47)   
        self.writeSingleByte(PKTCTRL1, 0x00)  
        self.writeSingleByte(PKTCTRL0, 0x00)  
        self.writeSingleByte(ADDR, 0x00)
        self.writeSingleByte(CHANNR, 0x00)
        self.writeSingleByte(FSCTRL1, 0x06)   
        self.writeSingleByte(FSCTRL0, 0x00)   
        self.writeSingleByte(MDMCFG4, 0xE7)    
        self.writeSingleByte(MDMCFG3, 0x10)    
        self.writeSingleByte(MDMCFG2, 0x30)    #. 32 would be 16/16 sync word bits .#
        self.writeSingleByte(MDMCFG1, 0x22)   
        self.writeSingleByte(MDMCFG0, 0xF8)
        self.writeSingleByte(DEVIATN, 0x15)   
        self.writeSingleByte(MCSM2, 0x07)
        self.writeSingleByte(MCSM1, 0x20)     
        self.writeSingleByte(MCSM0, 0x18)
        self.writeSingleByte(FOCCFG, 0x14)
        self.writeSingleByte(BSCFG, 0x6C)
        self.writeSingleByte(AGCCTRL2, 0x03)  
        self.writeSingleByte(AGCCTRL1, 0x00)  
        self.writeSingleByte(AGCCTRL0, 0x92)
        self.writeSingleByte(WOREVT1, 0x87)
        self.writeSingleByte(WOREVT0, 0x6B)
        self.writeSingleByte(WORCTRL, 0xFB)
        self.writeSingleByte(FREND1, 0x56)    
        self.writeSingleByte(FREND0, 0x11)    
        self.writeSingleByte(FSCAL3, 0xE9)
        self.writeSingleByte(FSCAL2, 0x2A)
        self.writeSingleByte(FSCAL1, 0x00)
        self.writeSingleByte(FSCAL0, 0x1F)
        self.writeSingleByte(RCCTRL1, 0x41)
        self.writeSingleByte(RCCTRL0, 0x00)
        self.writeSingleByte(FSTEST, 0x59)
        self.writeSingleByte(PTEST, 0x7F)   
        self.writeSingleByte(AGCTEST, 0x3F)
        self.writeSingleByte(TEST2, 0x81)     
        self.writeSingleByte(TEST1, 0x35)     
        self.writeSingleByte(TEST0, 0x0B)

    def setupTX(self):
        print("setupTx")
        self.writeSingleByte(IOCFG2, 0x29)    
        self.writeSingleByte(IOCFG1, 0x2E)    
        self.writeSingleByte(IOCFG0, 0x06)    
        self.writeSingleByte(FIFOTHR, 0x47)
        #self.writeSingleByte(PKTCTRL0, 0x00)
        #self.writeSingleByte(PKTCTRL1, 0x00)  

        self.writeSingleByte(PKTCTRL0, 0x00)
        self.writeSingleByte(PKTCTRL1, 0x00)  

        self.writeSingleByte(ADDR, 0x00)
        self.writeSingleByte(CHANNR, 0x00)
        #self.writeSingleByte(FSCTRL1, 0x06)   
        #self.writeSingleByte(FSCTRL0, 0x00)   
        #self.writeSingleByte(MDMCFG4, 0xE7)    
        #self.writeSingleByte(MDMCFG3, 0x10)    
        #self.writeSingleByte(MDMCFG1, 0x22)   
        #self.writeSingleByte(MDMCFG2, 0x30)    #. 32 would be 16/16 sync word bits .#
        #self.writeSingleByte(MDMCFG0, 0xF8)
        #self.writeSingleByte(DEVIATN, 0x15)
        self.writeSingleByte(DEVIATN, 0x00)   

        self.writeSingleByte(MCSM2, 0x07)
        self.writeSingleByte(MCSM1, 0x20)     
        self.writeSingleByte(MCSM0, 0x18)
        #self.writeSingleByte(FOCCFG, 0x14)
        #self.writeSingleByte(FOCCFG, 0)

        self.writeSingleByte(BSCFG, 0x6C)
        self.writeSingleByte(AGCCTRL2, 0x03)  
        self.writeSingleByte(AGCCTRL1, 0x00)  
        self.writeSingleByte(AGCCTRL0, 0x92)
        self.writeSingleByte(WOREVT1, 0x87)
        self.writeSingleByte(WOREVT0, 0x6B)
        self.writeSingleByte(WORCTRL, 0xFB)

        self.writeSingleByte(FREND1, 0x56)    
        self.writeSingleByte(FREND0, 0x11)

        #self.writeSingleByte(FSCAL3, 0xE9)
        #self.writeSingleByte(FSCAL2, 0x2A)
        #self.writeSingleByte(FSCAL1, 0x00)
        #self.writeSingleByte(FSCAL0, 0x1F)
        #self.writeSingleByte(RCCTRL1, 0x41)
        #self.writeSingleByte(RCCTRL0, 0x00)
        #self.writeSingleByte(FSTEST, 0x59)
        #self.writeSingleByte(PTEST, 0x7F)   
        #self.writeSingleByte(AGCTEST, 0x3F)
        #self.writeSingleByte(TEST2, 0x81)     
        #self.writeSingleByte(TEST1, 0x35)     
        #self.writeSingleByte(TEST0, 0x0B)

    def writeSingleByte(self, address, byte_data):
        databuffer = bytearray([WRITE_SINGLE_BYTE | address, byte_data])
        with self.device as d:
            d.write(databuffer)

    def readSingleByte(self, address):
        databuffer = bytearray([READ_SINGLE_BYTE | address, 0x00])
        
        with self.device as d:
            d.write(databuffer, end=1)
            d.readinto(databuffer, end=2)
        return databuffer[0]

    def readBurst(self, start_address, length):        
        databuffer = []
        ret = bytearray(length+1)

        for x in range(length + 1):
            addr = (start_address + (x*8)) | READ_BURST
            databuffer.append(addr)

        with self.device as d:
            d.write_readinto(bytearray(databuffer), ret)
        return ret

    def writeBurst(self, address, data):
        temp = list(data)
        temp.insert(0, (WRITE_BURST | address))
        with self.device as d:
            d.write(bytearray(temp))

    def strobe(self, address):
        databuffer = bytearray([address, 0x00])
        with self.device as d:
            d.write(databuffer, end=1)
            d.readinto(databuffer, end=2)
        return databuffer

    def setupCheck(self):
        self.strobe(SFRX)
        self.strobe(SRX)
        print("ready to detect data")

    def receiveData(self, length):
        self.writeSingleByte(PKTLEN, length)
        self.strobe(SRX)
        print("waiting for data")

        while self.gdo0.value == False:
            pass 
        #detected rising edge

        while self.gdo0.value == True:
            pass
        #detected falling edge

        data_len = length#+2 # add 2 status bytes
        data = self.readBurst(RXFIFO, data_len)
        dataStr = ''.join(list(map(lambda x: "{0:0>8}".format(x[2:]), list(map(bin, data)))))
        newStr = dataStr[8:]
        print("Data: ", newStr)
        self.strobe(SIDLE)
        while (self.readSingleByte(MARCSTATE) != 0x01):
            pass
        self.strobe(SFRX)
        return newStr


    def sendData(self, frame_bytes: bytes):
        """
        Send a prepared frame (already in bytes).
        """
        self.writeSingleByte(PKTLEN, len(frame_bytes))

        # Enter IDLE and flush FIFO
        self.strobe(SIDLE)
        while (self.readSingleByte(MARCSTATE) & 0x1F) != 0x01:
            pass
        self.strobe(SFTX)
        time.sleep(0.001)

        # Load frame and transmit
        self.writeBurst(TXFIFO, frame_bytes)
        self.strobe(STX)

        # Wait for TXFIFO to empty
        remaining_bytes = self.readSingleByte(TXBYTES) & 0x7F
        while remaining_bytes != 0:
            time.sleep(0.001)
            remaining_bytes = self.readSingleByte(TXBYTES) & 0x7F

        # Cleanup
        self.strobe(SFTX)
        self.strobe(SFRX)
        time.sleep(0.001)

        return (self.readSingleByte(TXBYTES) & 0x7F) == 0


    def sendElroFrame(self, frame_bytes: bytearray, repeat=14, gap_byte=0x00):
        """
        Send a full repeated Elro-DB286A frame via CC1101.

        cc1101        : instance of your CC1101 driver
        frame_bytes   : packed single frame (bytearray, e.g., 5 bytes)
        repeat        : number of repetitions (usually 14)
        gap_byte      : byte inserted between frames
        """

        # Build the repeated packet in Python only once
        packet = bytearray()
        for _ in range(repeat):
            packet += frame_bytes
            packet.append(gap_byte)

        # Write packet length to CC1101
        self.writeSingleByte(PKTLEN, len(packet))

        # Ensure CC1101 is in idle and TX FIFO is clear
        self.strobe(SIDLE)
        while (self.readSingleByte(MARCSTATE) & 0x1F) != 0x01:
            pass
        self.strobe(SFTX)

        # Write the full packet to TX FIFO in one burst
        self.writeBurst(TXFIFO, packet)

        # Start transmission
        self.strobe(STX)

        # Wait until all bytes are transmitted
        remaining = self.readSingleByte(TXBYTES) & 0x7F
        while remaining != 0:
            time.sleep(0.01)
            remaining = self.readSingleByte(TXBYTES) & 0x7F

        # Clean up
        self.strobe(SFTX)
        self.strobe(SFRX)







SYMBOL_MAP_SIMPLE = {
    '0': 0x88,
    '1': 0xEE
}

def elro_id_to_bits(id_hex: str) -> str:
    """Convert hex ID to 33-bit string (32 bits + trailing '1')"""
    id_bits = f"{int(id_hex,16):032b}" + "1"
    print("[elro_id_to_bits] 33-bit string:", id_bits)
    return id_bits

def bits_to_chip_stream(bits: str) -> list[int]:
    """Map each bit to a single chip byte"""
    stream = [SYMBOL_MAP_SIMPLE[b] for b in bits]
    print("[bits_to_chip_stream] Chip stream (hex):", [hex(c) for c in stream])
    print("[bits_to_chip_stream] Length:", len(stream))
    return stream

def pack_chip_stream(chip_stream: list[int]) -> bytearray:
    """Pack 33 chip bytes into 5 bytes (MSB first), padding the last byte"""
    packed = bytearray()
    val = 0
    for i, chip in enumerate(chip_stream):
        bit = 1 if chip == 0xEE else 0
        val = (val << 1) | bit
        if (i+1) % 8 == 0:
            packed.append(val)
            val = 0
    remaining_bits = len(chip_stream) % 8
    if remaining_bits:
        val = val << (8 - remaining_bits)
        packed.append(val)
    print("[pack_chip_stream] Single frame length (bytes):", len(packed))
    print("[pack_chip_stream] Single frame hex dump:", packed.hex(" "))
    return packed

def repeat_frame(frame: bytearray, reps=14, gap=0x00) -> bytearray:
    """Repeat frame multiple times, inserting gap byte between frames"""
    full = bytearray()
    for _ in range(reps):
        full += frame
        full.append(gap)
    print("[repeat_frame] Full repeated packet length:", len(full))
    print("[repeat_frame] Full repeated packet hex dump:", full.hex(" "))
    return full


SYMBOL = {'0': '1000', '1': '1110'}  # 2.0 kbps mapping

def elro_bits_with_marker(id_hex: str) -> str:
    return f"{int(id_hex,16):032b}" + "1"


def expand_pwm(bits: str) -> str:
    return ''.join(SYMBOL[b] for b in bits)  # 33 -> 132 base bits

def pack_bits_to_bytes(bits: str) -> bytearray:
    out = bytearray()
    for i in range(0, len(bits), 8):
        chunk = bits[i:i+8]
        if len(chunk) < 8:
            chunk = chunk + '0'*(8 - len(chunk))  # pad with 0s
        val = 0
        for j, b in enumerate(chunk):
            if b == '1':
                val |= 1 << (7 - j)  # MSB first
        out.append(val)
    return out

def build_elro_frame_bytes(id_hex: str) -> bytearray:
    bits33 = elro_bits_with_marker(id_hex)
    base_bits = expand_pwm(bits33)          # 132 bits
    frame = pack_bits_to_bytes(base_bits)   # 17 bytes
    return frame


def pack_bits_to_bytes(bits: str) -> bytearray:
    out = bytearray()
    for i in range(0, len(bits), 8):
        chunk = bits[i:i+8].ljust(8, '0')
        val = 0
        for j, b in enumerate(chunk):
            if b == '1':
                val |= 1 << (7 - j)
        out.append(val)
    return out

def build_repeated_packet(frame: bytearray, reps: int, gap_bits: int) -> bytearray:
    gap_bytes = pack_bits_to_bytes('0' * gap_bits)
    pkt = bytearray()
    for _ in range(reps):
        pkt += frame
        pkt += gap_bytes
    print(f"Frame bytes: {len(frame)}, gap bytes: {len(gap_bytes)}, reps: {reps}")
    print(f"Total packet length: {len(pkt)} bytes")
    return pkt

# Examples:



if __name__ == "__main__":
    id_hex = "6EDD2A6C"
    
    # Step 1: ID → 33-bit string
    bits = elro_id_to_bits(id_hex)
    
    # Step 2: 33-bit string → chip bytes
    chip_stream = bits_to_chip_stream(bits)
    
    # Step 3: Pack into frame bytes
    single_frame = pack_chip_stream(chip_stream)
    
    # Step 4: Repeat frame 14 times with gap
    full_packet = repeat_frame(single_frame, reps=14, gap=0x00)




    # Option 1: 13 reps, accurate gap (~7ms -> 14 bits -> 2 bytes)
    frame = build_elro_frame_bytes(id_hex)          # 17 bytes
    partial_packet = build_repeated_packet(frame, reps=13, gap_bits=14 )  # 17*13 + 2*14 = 238 bytes
    partial_packet = build_repeated_packet(frame, reps=13, gap_bits=14)
    
    #partial_packet = build_repeated_packet(frame, reps=13, gap_bits=14)

    # -X 'n=BELL,m=OOK_PWM,s=468,l=1460,r=7000,g=1540,t=397,y=0'
    #hex_B = "6edd2a6c"            # your Code B
    #hex_A = "2e1e7a2c"
    #frameB = elro_id_to_frame(hex_B) #make_single_frame_bytes(hex_B)   # 322 bytes by default





    myspi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
    cs = DigitalInOut(board.C0)
    gdo0 = DigitalInOut(board.C1)
    rx = CC1101(myspi, cs, gdo0, 1999, 433_920_000, "666A")
    # SPI object, Chip Select Pin, baudrate, frequency in Hz, Syncword

    rx.setupTX()
    rx.sendData(partial_packet)
    #rx.sendElroFrame(frame)
#for i in range(14):
# rx.sendData(packet)
#for _ in range(14):
#    rx.sendData(frameB)

