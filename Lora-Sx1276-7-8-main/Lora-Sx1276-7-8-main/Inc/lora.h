/* This Driver is suitable for SX1276/7/8 Lora module
 * Author: Vinod Kumar from Vinod Embedded
 * Goto: vinodembedded.wordpress.com for detailed explanation of the 
 * lora driver
 */
 
#ifndef __LORA_H__
#define __LORA_H__

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include <stdint.h>
#include <stdio.h>
#include "..\Src\definitions.h"

#define LORA_SS_PIN    GPIO_PIN_3
#define LORA_RESET_PIN GPIO_PIN_0
//#define LORA_DIO0_PIN  GPIO_PIN_10

#define LORA_SS_PORT    GPIOA
#define LORA_RESET_PORT GPIOB
//#define LORA_DIO0_PORT  GPIOB

typedef struct {
	int pin;
	void * port;
} lora_gpio_t;

typedef struct {
	lora_gpio_t reset;
	lora_gpio_t dio0;
	lora_gpio_t nss;
	void * spi;
} lora_pins_t;

typedef struct {
	lora_pins_t * pin;
	uint8_t frequency;
} lora_t;


#define REG_FIFO                 0x00
#define REG_OP_MODE              0x01
#define REG_FRF_MSB              0x06
#define REG_FRF_MID              0x07
#define REG_FRF_LSB              0x08
#define REG_PA_CONFIG            0x09
#define REG_LNA                  0x0c
#define REG_FIFO_ADDR_PTR        0x0d
#define REG_FIFO_TX_BASE_ADDR    0x0e
#define REG_FIFO_RX_BASE_ADDR    0x0f
#define REG_FIFO_RX_CURRENT_ADDR 0x10
#define REG_IRQ_FLAGS            0x12
#define REG_RX_NB_BYTES          0x13
#define REG_PKT_SNR_VALUE        0x19
#define REG_PKT_RSSI_VALUE       0x1a
#define REG_MODEM_CONFIG_1       0x1d
#define REG_MODEM_CONFIG_2       0x1e
#define REG_PREAMBLE_MSB         0x20
#define REG_PREAMBLE_LSB         0x21
#define REG_PAYLOAD_LENGTH       0x22
#define REG_MODEM_CONFIG_3       0x26
#define REG_RSSI_WIDEBAND        0x2c
#define REG_DETECTION_OPTIMIZE   0x31
#define REG_DETECTION_THRESHOLD  0x37
#define REG_SYNC_WORD            0x39
#define REG_DIO_MAPPING_1        0x40
#define REG_VERSION              0x42

// modes
#define MODE_LONG_RANGE_MODE     0x80
#define MODE_SLEEP               0x00
#define MODE_STDBY               0x01
#define MODE_TX                  0x03
#define MODE_RX_CONTINUOUS       0x05
#define MODE_RX_SINGLE           0x06

// PA config
#define PA_BOOST                 0x80

// IRQ masks
#define IRQ_TX_DONE_MASK           0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK 0x20
#define IRQ_RX_DONE_MASK           0x40

#define MAX_PKT_LENGTH           255


int lora_prasePacket(lora_t * module);
uint8_t lora_available(lora_t * module);
uint8_t lora_read(lora_t * module);
uint8_t lora_begin_packet(lora_t * module);
void lora_tx(lora_t * module, uint8_t * buf, uint8_t size);
uint8_t lora_end_packet(lora_t * module);

#if LORA_LIB_VINOD == 1







#define FREQ_433MHZ			0
#define FREQ_865MHZ			1
#define FREQ_866MHZ			2
#define FREQ_867MHZ			3

static const uint64_t FREQUENCY[4] = { 433E6, 865E6, 866E6, 867E6}; 

uint8_t lora_read_reg(lora_t * module, uint8_t addr);
void lora_write_reg(lora_t * module, uint8_t addr, uint8_t cmd);
uint8_t lora_init(lora_t * module);
void lora_set_frequency(lora_t * module, uint64_t freq);

#endif

/*   End of File
 * Author  : Vinod Kumar
 * Website : vinodembedded.wordpress.com
 */
 
 #endif
 
 #if LORA_LIB_VINOD == 0
 #include <stdbool.h>
 
 /*
 typedef   unsigned char uint8_t;
typedef   unsigned int  uint16_t;
typedef   unsigned long uint32_t;
*/

// XOSC frequency,in lora1276/1278, F(XOSC)=32M
#define LORA_XOSC           32000000
// RF frequency band.Low frequency band is up to 525M
// lora1278 only support low frequency band
#define LORA_FREQUENCY_BAND     LR_LowFreqModeOn_868M
// the first bit in SPI address byte is a wnr bit
// 1 for write access , 0 for read access
#define LORA_SPI_WNR           0x80
// Waiting time for txdone irq
// this time is depended on BW ,SF,CR ,length of packet
// you must change this value according to actual parameter and length
// see more detail on page 27 of datasheet
#define LORA_TX_TIMEOUT         1000
// called by setAntSwitch
#define LORA_MODE_RX          1
#define LORA_MODE_TX          2
#define LORA_MODE_STBY          0

//LORA Internal registers Addres
#define LR_RegFifo                       0x00
#define LR_RegOpMode                     0x01
#define LR_RegFrMsb                      0x06
#define LR_RegFrMid                      0x07
#define LR_RegFrLsb                      0x08
#define LR_RegPaConfig                   0x09
#define LR_RegPaRamp                     0x0A
#define LR_RegOcp                        0x0B
#define LR_RegLna                        0x0C
#define LR_RegFifoAddrPtr                0x0D
#define LR_RegFifoTxBaseAddr             0x0E
#define LR_RegFifoRxBaseAddr             0x0F
#define LR_RegFifoRxCurrentaddr          0x10
#define LR_RegIrqFlagsMask               0x11
#define LR_RegIrqFlags                   0x12
#define LR_RegRxNbBytes                  0x13
#define LR_RegRxHeaderCntValueMsb        0x14
#define LR_RegRxHeaderCntValueLsb        0x15
#define LR_RegRxPacketCntValueMsb        0x16
#define LR_RegRxPacketCntValueLsb        0x17
#define LR_RegModemStat                  0x18
#define LR_RegPktSnrValue                0x19
#define LR_RegPktRssiValue               0x1A
#define LR_RegRssiValue                  0x1B
#define LR_RegHopChannel                 0x1C
#define LR_RegModemConfig1               0x1D
#define LR_RegModemConfig2               0x1E
#define LR_RegSymbTimeoutLsb             0x1F
#define LR_RegPreambleMsb                0x20
#define LR_RegPreambleLsb                0x21
#define LR_RegPayloadLength              0x22
#define LR_RegMaxPayloadLength           0x23
#define LR_RegHopPeriod                  0x24
#define LR_RegFifoRxByteAddr             0x25
#define LR_RegModemConfig3               0x26
#define LR_RegDIOMAPPING1                0x40
#define LR_RegDIOMAPPING2                0x41
#define LR_RegVERSION                    0x42
#define LR_RegPLLHOP                     0x44
#define LR_RegTCXO                       0x4B
#define LR_RegPADAC                      0x4D
#define LR_RegFORMERTEMP                 0x5B
#define LR_RegAGCREF                     0x61
#define LR_RegAGCTHRESH1                 0x62
#define LR_RegAGCTHRESH2                 0x63
#define LR_RegAGCTHRESH3                 0x64

//#define LR_RegOpMode                     0x01
#define LR_LongRangeMode_FSK               0x00
#define LR_LongRangeMode_LORA              0x80
#define LR_ModulationType_FSK              0x00
#define LR_ModulationType_OOK              0x20
#define LR_LowFreqModeOn_868M              0x00
#define LR_LowFreqModeOn_433M              0x08
#define LR_Mode_SLEEP                    0x00
#define LR_Mode_STBY                     0x01
#define LR_Mode_FSTX                     0x02
#define LR_Mode_TX                         0x03
#define LR_Mode_FSRX                     0x04
#define LR_Mode_RXCONTINUOUS               0x05
#define LR_Mode_RXSINGLE                   0x06
#define LR_Mode_CAD                      0x07

//#define LR_RegPaConfig                   0x09
#define LR_PASELECT_RFO                    0x00
#define LR_PASELECT_PA_POOST               0x80

//#define LR_RegOcp                        0x0B
#define LR_OCPON_ON                        0x20
#define LR_OCPON_OFF                       0x00

//#define LR_RegLna                        0x0C
#define LR_LNAGAIN_G1                    0x20
#define LR_LNAGAIN_G2                    0x40
#define LR_LNAGAIN_G3                    0x60
#define LR_LNAGAIN_G4                    0x80
#define LR_LNAGAIN_G5                    0xA0
#define LR_LNAGAIN_G6                    0xC0
#define LR_LNABOOSTHF_0                    0x00
#define LR_LNABOOSTHF_1                    0x03

//#define LR_RegIrqFlagsMask               0x11
#define LR_RXTIMEOUT_MASK                  0x80
#define LR_RXDONE_MASK                   0x40
#define LR_RXPCRCERROR_MASK                0x20
#define LR_VALIDHEADER_MASK                0x10
#define LR_TXDONE_MASK                   0x08
#define LR_CADDONE_MASK                    0x04
#define LR_FHSSCHANGECH_MASK               0x02
#define LR_CADDETECTED_MASK                0x01

//#define LR_RegModemConfig1               0x1D
#define LR_BW_7p8k                       0x00
#define LR_BW_10p4k                        0x10
#define LR_BW_15p6k                        0x20
#define LR_BW_20p8k                        0x30
#define LR_BW_31p25k                       0x40
#define LR_BW_41p7k                        0x50
#define LR_BW_62p5k                        0x60
#define LR_BW_125k                         0x70
#define LR_BW_250k                         0x80
#define LR_BW_500k                         0x90
#define LR_CODINGRATE_1p25                 0x02   // 4/5
#define LR_CODINGRATE_1p5                  0x04   // 4/6
#define LR_CODINGRATE_1p75               0x06   // 4/7
#define LR_CODINGRATE_2                    0x08   // 4/8
#define LR_IMPLICIT_HEADER_MODE            0x01
#define LR_EXPLICIT_HEADER_MODE            0x00

//#define LR_RegModemConfig2               0x1E
#define LR_SPREADING_FACTOR_6              0x60
#define LR_SPREADING_FACTOR_7              0x70
#define LR_SPREADING_FACTOR_8              0x80
#define LR_SPREADING_FACTOR_9              0x90
#define LR_SPREADING_FACTOR_10             0xa0
#define LR_SPREADING_FACTOR_11             0xb0
#define LR_SPREADING_FACTOR_12             0xc0
#define LR_TX_CONTINUOUS_MODE            0x08
#define LR_TX_NORMAL_MODE              0x00
#define LR_PAYLOAD_CRC_ON              0x04
#define LR_PAYLOAD_CRC_OFF               0x00

//#define LR_RegModemConfig3               0x26
#define LR_LOWDATARATEOPTIMIZE_DISABLED    0x00
#define LR_LOWDATARATEOPTIMIZE_ENABLED     0x08
#define LR_AGC_AUTO_ON                 0x04

//P41
//#define LR_RegDIOMAPPING1               0x40
#define LR_DIO0_RXDONE              0x00
#define LR_DIO0_TXDONE              0x40
#define LR_DIO0_CADDONE                 0x80
#define LR_DIO1_RXTIMEOUT             0x00
#define LR_DIO1_FHSSCHANGECH            0x10
#define LR_DIO1_CADDETECTED             0x20
#define LR_DIO2_FHSSCHANGECH            0x00
#define LR_DIO3_CADDONE               0x00
#define LR_DIO3_VALIDHEADER             0x01
#define LR_DIO3_CRCERROR              0x02

//#define LR_RegDIOMAPPING2               0x41
#define LR_DIO4_CADDETECTED             0x00
#define LR_DIO4_PLLLOCK                 0x40
#define LR_DIO5_MODEREADY               0x00
#define LR_DIO5_CLKOUT                0x10

//#define LR_RegTCXO                      0x4B
#define LR_EXT_CRYSTAL                0x00
#define LR_TCXO_INPUT_ON                0x10
#define LR_REGTCXO_RESERVED             0x09

//#define LR_RegPADAC                     0x4D
#define LR_REGPADAC_RESERVED              0x80
#define LR_20DB_OUTPUT_ON                 0x07
#define LR_20DB_OUTPUT_OFF                0x04


bool config(lora_t * module);
bool setFrequency(lora_t * module, uint32_t freq);
bool setRFpara(lora_t * module, uint8_t BW,uint8_t CR,uint8_t SF,uint8_t payloadCRC);
bool setPreambleLen(lora_t * module, uint16_t length);
bool setHeaderMode(lora_t * module, uint8_t mode);
bool setPayloadLength(lora_t * module, uint8_t len);
bool setTxPower(lora_t * module, uint8_t power);
bool setRxTimeOut(lora_t * module, uint16_t symbTimeOut);
uint8_t readRSSI(lora_t * module, uint8_t mode);
void powerOnReset();
bool lora_init_not_vinod(lora_t * module);
 
 #endif
 
