/*
 This project is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 Deviation is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 You should have received a copy of the GNU General Public License
 along with Deviation.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "common.h"
#include "interface.h"
#include "mixer.h"
#include "config/model.h"
#include "config/tx.h"  // for Transmitter

#ifdef PROTO_HAS_CC2500

#ifdef EMULATOR
#define USE_FIXED_MFGID
#define OMP_BIND_COUNT      20
#define OMP_PACKET_PERIOD   100
#define dbgprintf printf
#else
#define OMP_BIND_COUNT      100
#define OMP_PACKET_PERIOD   5000  // Timeout for callback in uSec
// printf inside an interrupt handler is really dangerous
// this shouldn't be enabled even in debug builds without explicitly
// turning it on
#define dbgprintf if (0) printf
#endif

#define OMP_PACKET_SIZE           16
#define OMP_RF_BIND_CHANNEL       35
#define OMP_NUM_RF_CHANNELS       8
#define OMP_ADDR_LEN              5


static const char * const omp_opts[] = {
  _tr_noop("Freq-Fine"),  "-127", "127", NULL,
  _tr_noop("Telemetry"), _tr_noop("On"), _tr_noop("Off"), NULL,
  NULL
};

enum {
    PROTOOPTS_FREQFINE = 0,
    PROTOOPTS_TELEMETRY,
    LAST_PROTO_OPT,
};

ctassert(LAST_PROTO_OPT <= NUM_PROTO_OPTS, too_many_protocol_opts);

#define TELEM_ON  0
#define TELEM_OFF 1


static u8 tx_power;
static u8 packet[32];
static u8 hopping_frequency_no = 0;
static u8 rx_tx_addr[5];
static u8 hopping_frequency[OMP_NUM_RF_CHANNELS];
static u16 bind_counter;
static u8 phase;
static u8 calibration[OMP_NUM_RF_CHANNELS];
static u8 calibration_fscal2;
static u8 calibration_fscal3;
static s8 fine;
static u8 telm_req = 0;
static u16 tx_wait = 0;

enum{
    OMP_BIND,
    OMP_DATA,
    OMP_PACKET_SEND,
    OMP_TELE
};

enum {
    CHANNEL1 = 0,
    CHANNEL2,
    CHANNEL3,
    CHANNEL4,
    CHANNEL5,
    CHANNEL6,
    CHANNEL7,
};

// Bit vector from bit position
#define BV(bit) (1 << bit)

#define CHAN_RANGE (CHAN_MAX_VALUE - CHAN_MIN_VALUE)
static u16 scale_channel(u8 ch, u16 destMin, u16 destMax)
{
    s32 chanval = Channels[ch];
    s32 range = (s32) destMax - (s32) destMin;

    if (chanval < CHAN_MIN_VALUE)
        chanval = CHAN_MIN_VALUE;
    else if (chanval > CHAN_MAX_VALUE)
        chanval = CHAN_MAX_VALUE;
    return (range * (chanval - CHAN_MIN_VALUE)) / CHAN_RANGE + destMin;
}

#define GET_FLAG(ch, mask) (Channels[ch] > 0 ? mask : 0)


// calibrate used RF channels for faster hopping
static void calibrate_rf_chans()
{
    for (int c = 0; c < OMP_NUM_RF_CHANNELS; c++) {
        CLOCK_ResetWatchdog();
        CC2500_Strobe(CC2500_SIDLE);
        XN297L_SetChannel(hopping_frequency[c]);
        CC2500_Strobe(CC2500_SCAL);
        usleep(900);
        calibration[c] = CC2500_ReadReg(CC2500_25_FSCAL1);
    }
    calibration_fscal3 = CC2500_ReadReg(CC2500_23_FSCAL3);  // only needs to be done once
    calibration_fscal2 = CC2500_ReadReg(CC2500_24_FSCAL2);  // only needs to be done once
    CC2500_Strobe(CC2500_SIDLE);
}

static void OMP_init()
{
    // setup cc2500 for xn297L@250kbps emulation, scrambled, crc enabled
    XN297L_Configure(XN297L_SCRAMBLED, XN297L_CRC, OMP_PACKET_SIZE+10);  // packet_size + 5byte address + 2 byte pcf + 2byte crc + 1byte preamble
    calibrate_rf_chans();
    CC2500_SetPower(tx_power);
}

static void OMP_initialize_txid()
{
    u32 lfsr = 0xb2c54a2ful;
    u8 i, j;

    if (Model.fixed_id) {
       for (i = 0, j = 0; i < sizeof(Model.fixed_id); ++i, j += 8)
           rand32_r(&lfsr, (Model.fixed_id >> j) & 0xff);
    }
    // Pump zero bytes for LFSR to diverge more
    for (i = 0; i < sizeof(lfsr); ++i) rand32_r(&lfsr, 0);

    for (i=0, j=0; i < 4; i++, j+=8)
        rx_tx_addr[i] = (lfsr >> j) & 0xff;

    rand32_r(&lfsr, 0);
    rx_tx_addr[4] = lfsr & 0xff;
    u8 tmp = (rx_tx_addr[4] & 0x07) +1;
    // channels
    for (i=0; i < OMP_NUM_RF_CHANNELS; i++)
        hopping_frequency[i] = (i+3+tmp)*5+tmp;
}

static void omp_update_telemetry()
{
// packet_in = 01 00 98 2C 03 19 19 F0 49 02 00 00 00 00 00 00
// all bytes are fixed and unknown except 2 and 3 which represent the battery voltage: packet_in[3]*256+packet_in[2]=lipo voltage*100 in V
    const u8 *update = NULL;
    static const u8 omp_telem[] = { TELEM_DEVO_VOLT1, 0 };
    // raw receive data    first byte should be 0x55 (last xn297l preamble word) and then 5 byte address
    u8 telem_pkt[26];  // unscramble data
    u8 pid;
    u8 ack;
    u8 pkt_len;
    u8 i;
    u8 rx_fifo = CC2500_ReadReg(CC2500_3B_RXBYTES | CC2500_READ_BURST) & 0x7F;
    if (rx_fifo == 26)
        {
            CC2500_ReadData(packet, rx_fifo);

            for ( i = 0; i < OMP_ADDR_LEN; ++i )   // xn297l addr(5)
                telem_pkt[OMP_ADDR_LEN-1-i] = packet[i+1] ^ xn297_scramble[i];

            for ( i = OMP_ADDR_LEN; i< 23; i++ )  // unscramble pcf(2) payload(16)
                telem_pkt[i] = packet[i+1] ^ xn297_scramble[i];

            pkt_len = telem_pkt[OMP_ADDR_LEN] >> 1;  // payload len, pid and ack
            pid = ((telem_pkt[OMP_ADDR_LEN] & 0x01) << 1)|(telem_pkt[OMP_ADDR_LEN+1] >> 7);
            ack = (telem_pkt[OMP_ADDR_LEN+1] >> 6)&0x01;

            telem_pkt[OMP_ADDR_LEN] = pkt_len;

            for (u8 i = OMP_ADDR_LEN+1; i <= OMP_ADDR_LEN+16; i++ )   // regroup payload
                telem_pkt[i] = bit_reverse(telem_pkt[i] << 2 | telem_pkt[i+1] >> 6);

            telem_pkt[22] = pid | (ack << 4);

            u16 crc = 0xb5d2;
            for ( i = 1; i < 26; ++i)
                crc = crc16_update(crc, packet[i], 8);
            u8 crcxored = crc16_update(crc, packet[26] & 0xc0, 2);

            crc = (packet[23] << 10) | (packet[24] << 2) | (packet[25] >> 6);

            telem_pkt[23] = (crc >> 8)&0xff;
            telem_pkt[24] = crc&0xff;

            if (crc == crcxored)
                telem_pkt[25] = 1;
            else
                telem_pkt[25] = 0;

            if ((pkt_len == 16) && (crc == crcxored))
                {
                    Telemetry.value[TELEM_DEVO_VOLT1] = ((telem_pkt[OMP_ADDR_LEN+3] << 8) + telem_pkt[OMP_ADDR_LEN+2])/100;
                    update = omp_telem;
                }
        }

    CC2500_Strobe(CC2500_SIDLE);
    CC2500_Strobe(CC2500_SFRX);
    CC2500_SetTxRxMode(TXRX_OFF);

    if (update) {
        while (*update) {
            TELEMETRY_SetUpdated(*update++);
        }
    }
}

static void OMP_send_packet(u8 bind)
{
    CC2500_SetTxRxMode(TX_EN);
    CLOCK_ResetWatchdog();
    CLOCK_RunMixer();
    if (!bind)
    {
    memset(packet, 0x00, OMP_PACKET_SIZE);
    packet[0] = hopping_frequency_no;
    telm_req = hopping_frequency_no;
    if (telm_req == 0)
        packet[0] |= 0x40;
    CC2500_WriteReg(CC2500_23_FSCAL3, calibration_fscal3);
    CC2500_WriteReg(CC2500_24_FSCAL2, calibration_fscal2);
    CC2500_WriteReg(CC2500_25_FSCAL1, calibration[hopping_frequency_no]);
    XN297L_SetChannel(hopping_frequency[hopping_frequency_no]);
    hopping_frequency_no++;
    hopping_frequency_no &= OMP_NUM_RF_CHANNELS-1;

        packet[1] = 0x08                                // unknown
                    | GET_FLAG(CHANNEL5, 0x20);         // HOLD
        packet[2] = 0x40;                               // unknown
        u16 ch = scale_channel(CHANNEL6, 0, 0x7FF);     // throttle
        if (ch > 0x600)
            packet[2] |= 0x20;                          // IDLE2
        else if (ch > 0x200)
            packet[1] |= 0x40;                          // IDLE1
        ch = scale_channel(CHANNEL7, 0, 0x7FF);         // throttle
        if (ch > 0x600)
            packet[2] |= 0x08;                          // 3D
        else if (ch > 0x200)
            packet[2] |= 0x04;                          // ATTITUDE
        ch = scale_channel(CHANNEL3, 0, 0x7FF);         // throttle
        packet[7] = ch;
        packet[8] = ch >> 8;
        ch = scale_channel(CHANNEL1, 0, 0x7FF);         // aileron
        packet[8] |= ch << 3;
        packet[9] = ch >> 5;
        ch = scale_channel(CHANNEL2, 0, 0x7FF);         // elevator
        packet[9] |= ch << 6;
        packet[10] = ch >> 2;
        packet[11] = ch >> 10;
        ch = scale_channel(CHANNEL4, 0, 0x7FF);         // rudder
        packet[11] |= ch << 1;
        packet[12] = ch >> 7;
        packet[15] = 0x04;
    }

    XN297L_WriteEnhancedPayload(packet, OMP_PACKET_SIZE, telm_req != 0);   // ack/8packet

    if (tx_power != Model.tx_power)  // Keep transmit power updated
    {
        tx_power = Model.tx_power;
        CC2500_SetPower(tx_power);
    }
}


static u16 OMP_callback()
{
    u16 timeout = OMP_PACKET_PERIOD;
    if (fine != (s8)Model.proto_opts[PROTOOPTS_FREQFINE])
    {
        fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
        CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    }
    switch (phase) {
        case OMP_BIND:
            if (bind_counter == 0)
                {
                    PROTOCOL_SetBindState(0);
                    XN297L_SetTXAddr(rx_tx_addr, 5);
                    phase = OMP_DATA;
                }
            else
                {
                    OMP_send_packet(1);
                    bind_counter--;
                }
            break;

        case OMP_DATA:
            OMP_send_packet(0);
                    phase = OMP_PACKET_SEND;
                    timeout = 1250;
                    tx_wait = 0;
            break;

        case OMP_PACKET_SEND:
            if (CC2500_ReadReg(CC2500_35_MARCSTATE | CC2500_READ_BURST) == 0x13)
                {
                    timeout = 50;
                    tx_wait += 50;
                    if (tx_wait > 1000)
                        {
                            phase = OMP_DATA;
                            timeout = OMP_PACKET_PERIOD-2250;
                        }
                    break;
                }
                if ((telm_req == 0) && (Model.proto_opts[PROTOOPTS_TELEMETRY] != TELEM_OFF))
                    {
                        CC2500_Strobe(CC2500_SIDLE);
                        CC2500_SetTxRxMode(RX_EN);
                        CC2500_Strobe(CC2500_SFRX);
                        CC2500_Strobe(CC2500_SRX);
                        phase = OMP_TELE;
                        timeout = 1550;
                    }
                else
                    {
                        CC2500_Strobe(CC2500_SIDLE);
                        CC2500_SetTxRxMode(TXRX_OFF);
                        timeout = OMP_PACKET_PERIOD-1250-tx_wait;
                        phase = OMP_DATA;
                    }
            break;

        case OMP_TELE:
            timeout = OMP_PACKET_PERIOD-2800-tx_wait;
            phase = OMP_DATA;
            omp_update_telemetry();
            break;
    }

    return timeout;
}

static void initialize(u8 bind)
{
    CLOCK_StopTimer();
    OMP_initialize_txid();
    tx_power = Model.tx_power;
    OMP_init();
    fine = (s8)Model.proto_opts[PROTOOPTS_FREQFINE];
    CC2500_WriteReg(CC2500_0C_FSCTRL0, fine);
    if (bind)
        {
            bind_counter = OMP_BIND_COUNT;
            PROTOCOL_SetBindState(OMP_BIND_COUNT * OMP_PACKET_PERIOD / 1000);
            phase = OMP_BIND;
            XN297L_SetTXAddr((u8*)"FLPBD", 5);
            XN297L_SetChannel(OMP_RF_BIND_CHANNEL);
            CC2500_Strobe(CC2500_SCAL);
            usleep(900);
            CC2500_Strobe(CC2500_SIDLE);
            memset(packet, 0x00, OMP_PACKET_SIZE);
            memcpy(packet, "BND", 3);
            memcpy(&packet[3], rx_tx_addr, 5);
            memcpy(&packet[8], hopping_frequency, 8);
        }
    else
        {
            XN297L_SetTXAddr(rx_tx_addr, 5);
            phase = OMP_DATA;
        }

    CLOCK_StartTimer(OMP_PACKET_PERIOD, OMP_callback);
}

uintptr_t OMP_Cmds(enum ProtoCmds cmd)
{
    switch (cmd) {
        case PROTOCMD_INIT:  initialize(0); return 0;
        case PROTOCMD_DEINIT:
        case PROTOCMD_RESET:
            CLOCK_StopTimer();
            return (CC2500_Reset() ? 1 : -1);
        case PROTOCMD_CHECK_AUTOBIND: return 0;
        case PROTOCMD_BIND:  initialize(1); return 0;
        case PROTOCMD_NUMCHAN: return 7;
        case PROTOCMD_DEFAULT_NUMCHAN: return 7;
        case PROTOCMD_CURRENT_ID: return Model.fixed_id;
        case PROTOCMD_GETOPTIONS: return (uintptr_t)omp_opts;
        case PROTOCMD_TELEMETRYSTATE:
            return (Model.proto_opts[PROTOOPTS_TELEMETRY] != TELEM_OFF ? PROTO_TELEM_ON : PROTO_TELEM_OFF);
        case PROTOCMD_TELEMETRYTYPE:
            return TELEM_DEVO;
        case PROTOCMD_CHANNELMAP: return AETRG;
        default: break;
    }
    return 0;
}

#endif
