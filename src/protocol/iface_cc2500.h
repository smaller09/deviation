#ifndef _IFACE_CC2500_H_
#define _IFACE_CC2500_H_

enum {
    CC2500_00_IOCFG2           = 0x00,        // GDO2 output pin configuration
    CC2500_01_IOCFG1           = 0x01,        // GDO1 output pin configuration
    CC2500_02_IOCFG0           = 0x02,        // GDO0 output pin configuration
    CC2500_03_FIFOTHR          = 0x03,        // RX FIFO and TX FIFO thresholds
    CC2500_04_SYNC1            = 0x04,        // Sync word, high byte
    CC2500_05_SYNC0            = 0x05,        // Sync word, low byte
    CC2500_06_PKTLEN           = 0x06,        // Packet length
    CC2500_07_PKTCTRL1         = 0x07,        // Packet automation control
    CC2500_08_PKTCTRL0         = 0x08,        // Packet automation control
    CC2500_09_ADDR             = 0x09,        // Device address
    CC2500_0A_CHANNR           = 0x0A,        // Channel number
    CC2500_0B_FSCTRL1          = 0x0B,        // Frequency synthesizer control
    CC2500_0C_FSCTRL0          = 0x0C,        // Frequency synthesizer control
    CC2500_0D_FREQ2            = 0x0D,        // Frequency control word, high byte
    CC2500_0E_FREQ1            = 0x0E,        // Frequency control word, middle byte
    CC2500_0F_FREQ0            = 0x0F,        // Frequency control word, low byte
    CC2500_10_MDMCFG4          = 0x10,        // Modem configuration
    CC2500_11_MDMCFG3          = 0x11,        // Modem configuration
    CC2500_12_MDMCFG2          = 0x12,        // Modem configuration
    CC2500_13_MDMCFG1          = 0x13,        // Modem configuration
    CC2500_14_MDMCFG0          = 0x14,        // Modem configuration
    CC2500_15_DEVIATN          = 0x15,        // Modem deviation setting
    CC2500_16_MCSM2            = 0x16,        // Main Radio Cntrl State Machine config
    CC2500_17_MCSM1            = 0x17,        // Main Radio Cntrl State Machine config
    CC2500_18_MCSM0            = 0x18,        // Main Radio Cntrl State Machine config
    CC2500_19_FOCCFG           = 0x19,        // Frequency Offset Compensation config
    CC2500_1A_BSCFG            = 0x1A,        // Bit Synchronization configuration
    CC2500_1B_AGCCTRL2         = 0x1B,        // AGC control
    CC2500_1C_AGCCTRL1         = 0x1C,        // AGC control
    CC2500_1D_AGCCTRL0         = 0x1D,        // AGC control
    CC2500_1E_WOREVT1          = 0x1E,        // High byte Event 0 timeout
    CC2500_1F_WOREVT0          = 0x1F,        // Low byte Event 0 timeout
    CC2500_20_WORCTRL          = 0x20,        // Wake On Radio control
    CC2500_21_FREND1           = 0x21,        // Front end RX configuration
    CC2500_22_FREND0           = 0x22,        // Front end TX configuration
    CC2500_23_FSCAL3           = 0x23,        // Frequency synthesizer calibration
    CC2500_24_FSCAL2           = 0x24,        // Frequency synthesizer calibration
    CC2500_25_FSCAL1           = 0x25,        // Frequency synthesizer calibration
    CC2500_26_FSCAL0           = 0x26,        // Frequency synthesizer calibration
    CC2500_27_RCCTRL1          = 0x27,        // RC oscillator configuration
    CC2500_28_RCCTRL0          = 0x28,        // RC oscillator configuration
    CC2500_29_FSTEST           = 0x29,        // Frequency synthesizer cal control
    CC2500_2A_PTEST            = 0x2A,        // Production test
    CC2500_2B_AGCTEST          = 0x2B,        // AGC test
    CC2500_2C_TEST2            = 0x2C,        // Various test settings
    CC2500_2D_TEST1            = 0x2D,        // Various test settings
    CC2500_2E_TEST0            = 0x2E,        // Various test settings

// Status registers
    CC2500_30_PARTNUM          = 0x70,        // Part number
    CC2500_31_VERSION          = 0x71,        // Current version number
    CC2500_32_FREQEST          = 0x72,        // Frequency offset estimate
    CC2500_33_LQI              = 0x73,        // Demodulator estimate for link quality
    CC2500_34_RSSI             = 0x74,        // Received signal strength indication
    CC2500_35_MARCSTATE        = 0x75,        // Control state machine state
    CC2500_36_WORTIME1         = 0x76,        // High byte of WOR timer
    CC2500_37_WORTIME0         = 0x77,        // Low byte of WOR timer
    CC2500_38_PKTSTATUS        = 0x78,        // Current GDOx status and packet status
    CC2500_39_VCO_VC_DAC       = 0x79,        // Current setting from PLL cal module
    CC2500_3A_TXBYTES          = 0x7A,        // Underflow and # of bytes in TXFIFO
    CC2500_3B_RXBYTES          = 0x7B,        // Overflow and # of bytes in RXFIFO

// Multi byte memory locations
    CC2500_3E_PATABLE          = 0x3E,
    CC2500_3F_TXFIFO           = 0x3F,
    CC2500_3F_RXFIFO           = 0x3F,
    CC2500_3F_FIFO             = 0x3F,
};

// Definitions for burst/single access to registers
#define CC2500_WRITE_SINGLE     0x00
#define CC2500_WRITE_BURST      0x40
#define CC2500_READ_SINGLE      0x80
#define CC2500_READ_BURST       0xC0

// Strobe commands
#define CC2500_SRES             0x30        // Reset chip.
#define CC2500_SFSTXON          0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                            // If in RX/TX: Go to a wait state where only the synthesizer is
                                            // running (for quick RX / TX turnaround).
#define CC2500_SXOFF            0x32        // Turn off crystal oscillator.
#define CC2500_SCAL             0x33        // Calibrate frequency synthesizer and turn it off
                                            // (enables quick start).
#define CC2500_SRX              0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                            // MCSM0.FS_AUTOCAL=1.
#define CC2500_STX              0x35        // In IDLE state: Enable TX. Perform calibration first if
                                            // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                            // Only go to TX if channel is clear.
#define CC2500_SIDLE            0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                            // Wake-On-Radio mode if applicable.
#define CC2500_SAFC             0x37        // Perform AFC adjustment of the frequency synthesizer
#define CC2500_SWOR             0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CC2500_SPWD             0x39        // Enter power down mode when CSn goes high.
#define CC2500_SFRX             0x3A        // Flush the RX FIFO buffer.
#define CC2500_SFTX             0x3B        // Flush the TX FIFO buffer.
#define CC2500_SWORRST          0x3C        // Reset real time clock.
#define CC2500_SNOP             0x3D        // No operation. May be used to pad strobe commands to two
                                            // bytes for simpler software.
//----------------------------------------------------------------------------------
// Chip Status Byte
//----------------------------------------------------------------------------------

// Bit fields in the chip status byte
#define CC2500_STATUS_CHIP_RDYn_BM             0x80
#define CC2500_STATUS_STATE_BM                 0x70
#define CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM  0x0F

// Chip states
#define CC2500_STATE_IDLE                      0x00
#define CC2500_STATE_RX                        0x10
#define CC2500_STATE_TX                        0x20
#define CC2500_STATE_FSTXON                    0x30
#define CC2500_STATE_CALIBRATE                 0x40
#define CC2500_STATE_SETTLING                  0x50
#define CC2500_STATE_RX_OVERFLOW               0x60
#define CC2500_STATE_TX_UNDERFLOW              0x70

//----------------------------------------------------------------------------------
// Other register bit fields
//----------------------------------------------------------------------------------
#define CC2500_LQI_CRC_OK_BM                   0x80
#define CC2500_LQI_EST_BM                      0x7F

void CC2500_WriteReg(u8 addr, u8 data);
u8 CC2500_ReadReg(u8 addr);
int CC2500_Reset();
void CC2500_Strobe(u8 cmd);
void CC2500_WriteRegisterMulti(u8 address, const u8 data[], u8 length);
void CC2500_ReadRegisterMulti(u8 address, u8 data[], u8 length);
void CC2500_WriteData(u8 *packet, u8 length);
void CC2500_ReadData(u8 *dpbuffer, int len);
void CC2500_SetTxRxMode(enum TXRX_State);
void CC2500_SetPower(int power);

void XN297L_Configure(u8 scramble_en, u8 crc_en, u8 cc2500_packet_len);
void XN297L_SetTXAddr(const u8* addr, u8 len);
void XN297L_WritePayload(u8* msg, u8 len);
void XN297L_SetChannel(u8 ch);
void XN297L_WriteEnhancedPayload(u8* msg, u8 len, u8 noack);
#define XN297L_NOCRC 0
#define XN297L_CRC 1

u16 crc16_update(u16 crc, u8 a, u8 bits);
u8 bit_reverse(u8 b_in);

#endif
