// XN297 emulation layer

#define     XN297_UNSCRAMBLED    0
#define     XN297_SCRAMBLED      1
#define     XN297L_NOCRC    0
#define     XN297L_CRC      1

u8 bit_reverse(u8 b_in);
u16 crc16_update(u16 crc, u8 a, u8 bits);

void XN297_SetTXAddr(const u8* addr, u8 len);
void XN297_SetRXAddr(const u8* addr, u8 len);
void XN297_Configure(u8 flags);
void XN297_SetScrambledMode(const u8 mode);
void XN297_WritePayload(u8* msg, u8 len);
void XN297_WriteEnhancedPayload(u8* msg, u8 len, u8 noack);
u8 XN297_ReadPayload(u8* msg, u8 len);
u8 XN297_ReadEnhancedPayload(u8* msg, u8 len);

void XN297L_Configure(u8 scramble_en, u8 crc_en, u8 cc2500_packet_len);
void XN297L_SetTXAddr(const u8* addr, u8 len);
void XN297L_WritePayload(u8* msg, u8 len);
void XN297L_SetChannel(u8 ch);
void XN297L_WriteEnhancedPayload(u8* msg, u8 len, u8 noack);

extern const u8 xn297_scramble[];
extern const u16 xn297_crc_xorout_scrambled[];
extern const u16 xn297_crc_xorout[];
extern const u16 xn297_crc_xorout_scrambled_enhanced[];
extern const u16 xn297_crc_xorout_enhanced[];
