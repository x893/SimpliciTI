/* Deviation = 9.521484 */
/* Base frequency = 433.074799 */
/* Carrier frequency = 433.074799 */
/* Channel number = 0 */
/* Carrier frequency = 433.074799 */
/* Modulated = true */
/* Modulation format = GFSK */
/* Manchester enable = false */
/* Sync word qualifier mode = 30/32 sync word bits detected */
/* Preamble count = 4 */
/* Channel spacing = 25.390625 */
/* Carrier frequency = 433.074799 */
/* Data rate = 99.9756 */
/* RX filter BW = 270.833333 */
/*  = Normal mode */
/* Length config = Variable packet length mode. Packet length configured by the first byte after sync word */
/* CRC enable = true */
/* Packet length = 255 */
/* Device address = 0 */
/* Address config = No address check */
/*  = false */
/* PA ramping = false */
/* TX power = 0 */
/***************************************************************
 *  SmartRF Studio(tm) Export
 *
 *  Radio register settings specifed with C-code
 *  compatible #define statements.
 *
 ***************************************************************/

#ifndef SMARTRF_CC1110_H
#define SMARTRF_CC1110_H

#define SMARTRF_RADIO_CC1110

#define SMARTRF_SETTING_FSCTRL1     0x06
#define SMARTRF_SETTING_FSCTRL0     0x00
#define SMARTRF_SETTING_FREQ2       0x10
#define SMARTRF_SETTING_FREQ1       0xA8
#define SMARTRF_SETTING_FREQ0       0x1F
#define SMARTRF_SETTING_MDMCFG4     0x6B
#define SMARTRF_SETTING_MDMCFG3     0xF8
#define SMARTRF_SETTING_MDMCFG2     0x13
#define SMARTRF_SETTING_MDMCFG1     0x20
#define SMARTRF_SETTING_MDMCFG0     0x00
#define SMARTRF_SETTING_CHANNR      0x00
#define SMARTRF_SETTING_DEVIATN     0x24
#define SMARTRF_SETTING_FREND1      0x56
#define SMARTRF_SETTING_FREND0      0x10
#define SMARTRF_SETTING_MCSM0       0x18
#define SMARTRF_SETTING_FOCCFG      0x16
#define SMARTRF_SETTING_BSCFG       0x6C
#define SMARTRF_SETTING_AGCCTRL2    0x43
#define SMARTRF_SETTING_AGCCTRL1    0x40
#define SMARTRF_SETTING_AGCCTRL0    0x91
#define SMARTRF_SETTING_FSCAL3      0xEA
#define SMARTRF_SETTING_FSCAL2      0x2A
#define SMARTRF_SETTING_FSCAL1      0x00
#define SMARTRF_SETTING_FSCAL0      0x1F
#define SMARTRF_SETTING_TEST2       0x81
#define SMARTRF_SETTING_TEST1       0x35
#define SMARTRF_SETTING_TEST0       0x09
#define SMARTRF_SETTING_PA_TABLE0   0x60
#define SMARTRF_SETTING_PKTCTRL1    0x04
#define SMARTRF_SETTING_PKTCTRL0    0x05
#define SMARTRF_SETTING_ADDR        0x00
#define SMARTRF_SETTING_PKTLEN      0xFF

#endif
