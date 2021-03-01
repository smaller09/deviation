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
#include "config/tx.h"
#include "protocol/interface.h"
#include "protospi.h"

#ifdef PROTO_HAS_NRF24L01

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
//#define NOP           0xFF	// already defined as NRF24L01_FF_NOP in iface_nrf24l01.h

// Bit vector from bit position
#define BV(bit) (1 << bit)

//GPIOA.14

static u8 rf_setup;

static void CS_HI() {
    PROTO_CS_HI(NRF24L01);
}

static void CS_LO() {
    PROTO_CS_LO(NRF24L01);
}
void NRF24L01_Initialize()
{
    rf_setup = 0x0F;
}

u8 NRF24L01_WriteReg(u8 reg, u8 data)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_REGISTER | (REGISTER_MASK & reg));
    PROTOSPI_xfer(data);
    CS_HI();
    return res;
}

u8 NRF24L01_WriteRegisterMulti(u8 reg, const u8 data[], u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_REGISTER | ( REGISTER_MASK & reg));
    for (u8 i = 0; i < length; i++)
    {
        PROTOSPI_xfer(data[i]);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_WritePayload(u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(W_TX_PAYLOAD);
    for (u8 i = 0; i < length; i++)
    {
        PROTOSPI_xfer(data[i]);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_ReadReg(u8 reg)
{
    CS_LO();
    PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    u8 data = PROTOSPI_xfer(0xFF);
    CS_HI();
    return data;
}

u8 NRF24L01_ReadRegisterMulti(u8 reg, u8 data[], u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(R_REGISTER | (REGISTER_MASK & reg));
    for(u8 i = 0; i < length; i++)
    {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

u8 NRF24L01_ReadPayload(u8 *data, u8 length)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(R_RX_PAYLOAD);
    for(u8 i = 0; i < length; i++)
    {
        data[i] = PROTOSPI_xfer(0xFF);
    }
    CS_HI();
    return res;
}

static u8 Strobe(u8 state)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(state);
    CS_HI();
    return res;
}

u8 NRF24L01_FlushTx()
{
    return Strobe(FLUSH_TX);
}

u8 NRF24L01_FlushRx()
{
    return Strobe(FLUSH_RX);
}

u8 NRF24L01_GetStatus(void)
{
    return Strobe(NRF24L01_FF_NOP);
}

u8 NRF24L01_GetDynamicPayloadSize(void)
{
    CS_LO();
    PROTOSPI_xfer(R_RX_PL_WID);
    const u8 res = PROTOSPI_xfer(NRF24L01_FF_NOP);
    CS_HI();
    return res;
}

u8 NRF24L01_Activate(u8 code)
{
    CS_LO();
    u8 res = PROTOSPI_xfer(ACTIVATE);
    PROTOSPI_xfer(code);
    CS_HI();
    return res;
}

u8 NRF24L01_SetBitrate(u8 bitrate)
{
    // Note that bitrate 250kbps (and bit RF_DR_LOW) is valid only
    // for nRF24L01+. There is no way to programmatically tell it from
    // older version, nRF24L01, but the older is practically phased out
    // by Nordic, so we assume that we deal with with modern version.

    // Bit 0 goes to RF_DR_HIGH, bit 1 - to RF_DR_LOW
    rf_setup = (rf_setup & 0xD7) | ((bitrate & 0x02) << 4) | ((bitrate & 0x01) << 3);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}

// Power setting is 0..3 for nRF24L01
// Claimed power amp for nRF24L01 from eBay is 20dBm.
//      Raw            w 20dBm PA
// 0 : -18dBm  (16uW)   2dBm (1.6mW)
// 1 : -12dBm  (60uW)   8dBm   (6mW)
// 2 :  -6dBm (250uW)  14dBm  (25mW)
// 3 :   0dBm   (1mW)  20dBm (100mW)
// So it maps to Deviation as follows
/*
TXPOWER_100uW  = -10dBm
TXPOWER_300uW  = -5dBm
TXPOWER_1mW    = 0dBm
TXPOWER_3mW    = 5dBm
TXPOWER_10mW   = 10dBm
TXPOWER_30mW   = 15dBm
TXPOWER_100mW  = 20dBm
TXPOWER_150mW  = 22dBm
*/
u8 NRF24L01_SetPower(u8 power)
{
    u8 nrf_power = 0;
    switch(power) {
        case TXPOWER_100uW: nrf_power = 0; break;
        case TXPOWER_300uW: nrf_power = 1; break;
        case TXPOWER_1mW:   nrf_power = 2; break;
        case TXPOWER_3mW:   nrf_power = 3; break;
        default:            nrf_power = 0; break;
    }
    // Power is in range 0..3 for nRF24L01
    rf_setup = (rf_setup & 0xF9) | ((nrf_power & 0x03) << 1);
    return NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, rf_setup);
}
static void CE_lo()
{
#if HAS_MULTIMOD_SUPPORT
    PROTOCOL_SetSwitch(NRF24L01);
#endif
}
static void CE_hi()
{
#if HAS_MULTIMOD_SUPPORT
    u8 en = SPI_ProtoGetPinConfig(NRF24L01, ENABLED_PIN);
    u8 csn = SPI_ProtoGetPinConfig(NRF24L01, CSN_PIN);
    SPI_ConfigSwitch(en | 0x0f, en | (0x0f ^ csn));
#endif
}

void NRF24L01_SetTxRxMode(enum TXRX_State mode)
{
    if(mode == TX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP));
        usleep(130);
        CE_hi();
    } else if (mode == RX_EN) {
        CE_lo();
        NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
        NRF24L01_WriteReg(NRF24L01_07_STATUS, (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                                            | (1 << NRF24L01_07_TX_DS)
                                            | (1 << NRF24L01_07_MAX_RT));
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                                            | (1 << NRF24L01_00_CRCO)
                                            | (1 << NRF24L01_00_PWR_UP)
                                            | (1 << NRF24L01_00_PRIM_RX));
        usleep(130);
        CE_hi();
    } else {
        NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); //PowerDown
        CE_lo();
    }
}

int NRF24L01_Reset()
{
    NRF24L01_FlushTx();
    NRF24L01_FlushRx();
    u8 status1 = Strobe(NRF24L01_FF_NOP);
    u8 status2 = NRF24L01_ReadReg(NRF24L01_07_STATUS);
    NRF24L01_SetTxRxMode(TXRX_OFF);
#ifdef EMULATOR
    return 1;
#endif
    return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

//
// HS6200 emulation layer
///////////////////////////
static u8 hs6200_crc;
static u16 hs6200_crc_init;
static u8 hs6200_tx_addr[5];
static u8 hs6200_address_length;

static const u8 hs6200_scramble[] = {
    0x80,0xf5,0x3b,0x0d,0x6d,0x2a,0xf9,0xbc,
    0x51,0x8e,0x4c,0xfd,0xc1,0x65,0xd0}; // todo: find all 32 bytes ...

void HS6200_SetTXAddr(const u8* addr, u8 len)
{
    if(len < 4)
        len = 4;
    else if(len > 5)
        len = 5;
    
    // use nrf24 address field as a longer preamble
    if(addr[len-1] & 0x80)
        NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8*)"\x55\x55\x55\x55\x55", 5);
    else
        NRF24L01_WriteRegisterMulti(NRF24L01_10_TX_ADDR, (u8*)"\xaa\xaa\xaa\xaa\xaa", 5);
    
    // precompute address crc
    hs6200_crc_init = 0xffff;
    for(int i=0; i<len; i++)
        hs6200_crc_init = crc16_update(hs6200_crc_init, addr[len-1-i], 8);
    memcpy(hs6200_tx_addr, addr, len);
    hs6200_address_length = len;
}

static u16 hs6200_calc_crc(u8* msg, u8 len)
{
    u8 pos;
    u16 crc = hs6200_crc_init;
    
    // pcf + payload
    for(pos=0; pos < len-1; pos++) { 
        crc = crc16_update(crc, msg[pos], 8);
    }
    // last byte (1 bit only)
    if(len > 0) {
        crc = crc16_update(crc, msg[pos+1], 1);
    }
    
    return crc;
}

void HS6200_Configure(u8 flags)
{
    hs6200_crc = !!(flags & BV(NRF24L01_00_EN_CRC));
    flags &= ~(BV(NRF24L01_00_EN_CRC) | BV(NRF24L01_00_CRCO));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, flags & 0xff);      
}

u8 HS6200_WritePayload(u8* msg, u8 len)
{
    u8 payload[32];
    const u8 no_ack = 1; // never ask for an ack
    static u8 pid;
    u8 pos = 0;
    
    if(len > sizeof(hs6200_scramble))
        len = sizeof(hs6200_scramble);
    
    // address
    for(int i=hs6200_address_length-1; i>=0; i--) {
        payload[pos++] = hs6200_tx_addr[i];
    }
    
    // guard bytes
    payload[pos++] = hs6200_tx_addr[0];
    payload[pos++] = hs6200_tx_addr[0];
    
    // packet control field
    payload[pos++] = ((len & 0x3f) << 2) | (pid & 0x03);
    payload[pos] = (no_ack & 0x01) << 7;
    pid++;
    
    // scrambled payload
    if(len > 0) {
        payload[pos++] |= (msg[0] ^ hs6200_scramble[0]) >> 1; 
        for(u8 i=1; i<len; i++)
            payload[pos++] = ((msg[i-1] ^ hs6200_scramble[i-1]) << 7) | ((msg[i] ^ hs6200_scramble[i]) >> 1);
        payload[pos] = (msg[len-1] ^ hs6200_scramble[len-1]) << 7; 
    }
    
    // crc
    if(hs6200_crc) {
        u16 crc = hs6200_calc_crc(&payload[hs6200_address_length+2], len+2);
        uint8_t hcrc = crc >> 8;
        uint8_t lcrc = crc & 0xff;
        payload[pos++] |= (hcrc >> 1);
        payload[pos++] = (hcrc << 7) | (lcrc >> 1);
        payload[pos++] = lcrc << 7;
    }
    
    return NRF24L01_WritePayload(payload, pos);
}
//
// End of HS6200 emulation
////////////////////////////

#endif // defined(PROTO_HAS_NRF24L01)
