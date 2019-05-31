#include "stm32f0xx.h"        // Device header

//PA0 - IRQ (SX1276_Dio0)	, PA1 - LCD_RES	, PA2 - RX\TX mode
//PA3 - SX1276_Res				,	PA4 - NSS			,	PA5 - SCK	
//PA6 - MISO							,	PA7 - MOSI
//PF0 - SDA	,	PF1 - SCL

#define T191_Addr					0x78
#define LCD_ON()					GPIOA->BSRR |=1<<1
#define LCD_RES()					GPIOA->BRR  |=1<<1

#define CS_LO()						GPIOA->BRR 	|=1<<4
#define CS_HI()						GPIOA->BSRR |=1<<4
#define MISO							1<<6				//Pa6

#define SX1276_RES()				GPIOA->BRR 	|=1<<3
#define SX1276_REL()				GPIOA->BSRR |=1<<3

#define RX_TX_MODE()			(GPIOA->IDR & 1<<2) //Pa2 - 0 - Rx mode/ 1 - Tx mode

#if 1			//SX-1276 LoRa Registers

#define REG_LR_FIFO                                 0x00
// Common settings
#define REG_LR_OPMODE                               0x01
#define REG_LR_FRFMSB                               0x06
#define REG_LR_FRFMID                               0x07
#define REG_LR_FRFLSB                               0x08
// Tx settings
#define REG_LR_PACONFIG                             0x09
#define REG_LR_PARAMP                               0x0A
#define REG_LR_OCP                                  0x0B
// Rx settings
#define REG_LR_LNA                                  0x0C
// LoRa registers
#define REG_LR_FIFOADDRPTR                          0x0D
#define REG_LR_FIFOTXBASEADDR                       0x0E
#define REG_LR_FIFORXBASEADDR                       0x0F
#define REG_LR_FIFORXCURRENTADDR                    0x10
#define REG_LR_IRQFLAGSMASK                         0x11
#define REG_LR_IRQFLAGS                             0x12
#define REG_LR_RXNBBYTES                            0x13
#define REG_LR_RXHEADERCNTVALUEMSB                  0x14
#define REG_LR_RXHEADERCNTVALUELSB                  0x15
#define REG_LR_RXPACKETCNTVALUEMSB                  0x16
#define REG_LR_RXPACKETCNTVALUELSB                  0x17
#define REG_LR_MODEMSTAT                            0x18
#define REG_LR_PKTSNRVALUE                          0x19
#define REG_LR_PKTRSSIVALUE                         0x1A
#define REG_LR_RSSIVALUE                            0x1B
#define REG_LR_HOPCHANNEL                           0x1C
#define REG_LR_MODEMCONFIG1                         0x1D
#define REG_LR_MODEMCONFIG2                         0x1E
#define REG_LR_SYMBTIMEOUTLSB                       0x1F
#define REG_LR_PREAMBLEMSB                          0x20
#define REG_LR_PREAMBLELSB                          0x21
#define REG_LR_PAYLOADLENGTH                        0x22
#define REG_LR_PAYLOADMAXLENGTH                     0x23
#define REG_LR_HOPPERIOD                            0x24
#define REG_LR_FIFORXBYTEADDR                       0x25
#define REG_LR_MODEMCONFIG3                         0x26
#define REG_LR_FEIMSB                               0x28
#define REG_LR_FEIMID                               0x29
#define REG_LR_FEILSB                               0x2A
#define REG_LR_RSSIWIDEBAND                         0x2C
#define REG_LR_IFFREQ1                              0x2F
#define REG_LR_IFFREQ2                              0x30
#define REG_LR_DETECTOPTIMIZE                       0x31
#define REG_LR_INVERTIQ                             0x33
#define REG_LR_HIGHBWOPTIMIZE1                      0x36
#define REG_LR_DETECTIONTHRESHOLD                   0x37
#define REG_LR_SYNCWORD                             0x39
#define REG_LR_HIGHBWOPTIMIZE2                      0x3A
#define REG_LR_INVERTIQ2                            0x3B

// end of documented register in datasheet
// I/O settings
#define REG_LR_DIOMAPPING1                          0x40
#define REG_LR_DIOMAPPING2                          0x41
// Version
#define REG_LR_VERSION                              0x42
// Additional settings
#define REG_LR_PLLHOP                               0x44
#define REG_LR_TCXO                                 0x4B
#define REG_LR_PADAC                                0x4D
#define REG_LR_FORMERTEMP                           0x5B
#define REG_LR_BITRATEFRAC                          0x5D
#define REG_LR_AGCREF                               0x61
#define REG_LR_AGCTHRESH1                           0x62
#define REG_LR_AGCTHRESH2                           0x63
#define REG_LR_AGCTHRESH3                           0x64
#define REG_LR_PLL                                  0x70
#endif

#if 1			//SX-1276 LoRa bits control definition

//-------------OpMode---------------
#define RFLR_OPMODE_LONGRANGEMODE_MASK              0x7F
#define RFLR_OPMODE_LONGRANGEMODE_OFF               0x00 // Default
#define RFLR_OPMODE_LONGRANGEMODE_ON                0x80

#define RFLR_OPMODE_ACCESSSHAREDREG_MASK            0xBF
#define RFLR_OPMODE_ACCESSSHAREDREG_ENABLE          0x40
#define RFLR_OPMODE_ACCESSSHAREDREG_DISABLE         0x00 // Default

#define RFLR_OPMODE_FREQMODE_ACCESS_MASK            0xF7
#define RFLR_OPMODE_FREQMODE_ACCESS_LF              0x08 // Default
#define RFLR_OPMODE_FREQMODE_ACCESS_HF              0x00

#define RFLR_OPMODE_MASK                            0xF8
#define RFLR_OPMODE_SLEEP                           0x00
#define RFLR_OPMODE_STANDBY                         0x01 // Default
#define RFLR_OPMODE_SYNTHESIZER_TX                  0x02
#define RFLR_OPMODE_TRANSMITTER                     0x03
#define RFLR_OPMODE_SYNTHESIZER_RX                  0x04
#define RFLR_OPMODE_RECEIVER                        0x05
// LoRa specific modes
#define RFLR_OPMODE_RECEIVER_SINGLE                 0x06
#define RFLR_OPMODE_CAD                             0x07

//---------Frequency------------61.035*fRm 
#define RFLR_FRFMSB_868                         		0xD9 // Default
#define RFLR_FRFMID_868                         		0x20 // Default
#define RFLR_FRFLSB_868                         		0x04 // Default

#define RFLR_PACONFIG_PASELECT_MASK                 0x7F
#define RFLR_PACONFIG_PASELECT_PABOOST              0x80
#define RFLR_PACONFIG_PASELECT_RFO                  0x00 // Default

#define RFLR_PACONFIG_MAX_POWER_MASK                0x8F

#define RFLR_PACONFIG_OUTPUTPOWER_MASK              0xF0


//---------RegPaRamp----------------

#define RFLR_PARAMP_TXBANDFORCE_MASK                0xEF
#define RFLR_PARAMP_TXBANDFORCE_BAND_SEL            0x10
#define RFLR_PARAMP_TXBANDFORCE_AUTO                0x00 // Default

#define RFLR_PARAMP_MASK                            0xF0
#define RFLR_PARAMP_3400_US                         0x00
#define RFLR_PARAMP_2000_US                         0x01
#define RFLR_PARAMP_1000_US                         0x02
#define RFLR_PARAMP_0500_US                         0x03
#define RFLR_PARAMP_0250_US                         0x04
#define RFLR_PARAMP_0125_US                         0x05
#define RFLR_PARAMP_0100_US                         0x06
#define RFLR_PARAMP_0062_US                         0x07
#define RFLR_PARAMP_0050_US                         0x08
#define RFLR_PARAMP_0040_US                         0x09 // Default
#define RFLR_PARAMP_0031_US                         0x0A
#define RFLR_PARAMP_0025_US                         0x0B
#define RFLR_PARAMP_0020_US                         0x0C
#define RFLR_PARAMP_0015_US                         0x0D
#define RFLR_PARAMP_0012_US                         0x0E
#define RFLR_PARAMP_0010_US                         0x0F

//-------------RegOcp----------------
#define RFLR_OCP_MASK                               0xDF
#define RFLR_OCP_ON                                 0x20 // Default
#define RFLR_OCP_OFF                                0x00

#define RFLR_OCP_TRIM_MASK                          0xE0
#define RFLR_OCP_TRIM_045_MA                        0x00
#define RFLR_OCP_TRIM_050_MA                        0x01
#define RFLR_OCP_TRIM_055_MA                        0x02
#define RFLR_OCP_TRIM_060_MA                        0x03
#define RFLR_OCP_TRIM_065_MA                        0x04
#define RFLR_OCP_TRIM_070_MA                        0x05
#define RFLR_OCP_TRIM_075_MA                        0x06
#define RFLR_OCP_TRIM_080_MA                        0x07
#define RFLR_OCP_TRIM_085_MA                        0x08
#define RFLR_OCP_TRIM_090_MA                        0x09
#define RFLR_OCP_TRIM_095_MA                        0x0A
#define RFLR_OCP_TRIM_100_MA                        0x0B  // Default
#define RFLR_OCP_TRIM_105_MA                        0x0C
#define RFLR_OCP_TRIM_110_MA                        0x0D
#define RFLR_OCP_TRIM_115_MA                        0x0E
#define RFLR_OCP_TRIM_120_MA                        0x0F
#define RFLR_OCP_TRIM_130_MA                        0x10
#define RFLR_OCP_TRIM_140_MA                        0x11
#define RFLR_OCP_TRIM_150_MA                        0x12
#define RFLR_OCP_TRIM_160_MA                        0x13
#define RFLR_OCP_TRIM_170_MA                        0x14
#define RFLR_OCP_TRIM_180_MA                        0x15
#define RFLR_OCP_TRIM_190_MA                        0x16
#define RFLR_OCP_TRIM_200_MA                        0x17
#define RFLR_OCP_TRIM_210_MA                        0x18
#define RFLR_OCP_TRIM_220_MA                        0x19
#define RFLR_OCP_TRIM_230_MA                        0x1A
#define RFLR_OCP_TRIM_240_MA                        0x1B

//---------Reg_LNA-----------
#define RFLR_LNA_GAIN_MASK                          0x1F
#define RFLR_LNA_GAIN_G1                            0x20 // Default
#define RFLR_LNA_GAIN_G2                            0x40
#define RFLR_LNA_GAIN_G3                            0x60
#define RFLR_LNA_GAIN_G4                            0x80
#define RFLR_LNA_GAIN_G5                            0xA0
#define RFLR_LNA_GAIN_G6                            0xC0

#define RFLR_LNA_BOOST_LF_MASK                      0xE7
#define RFLR_LNA_BOOST_LF_DEFAULT                   0x00 // Default

#define RFLR_LNA_BOOST_HF_MASK                      0xFC
#define RFLR_LNA_BOOST_HF_OFF                       0x00 // Default
#define RFLR_LNA_BOOST_HF_ON                        0x03

//---------FiFo
#define RFLR_FIFOADDRPTR                            0x00 // Default
#define RFLR_FIFOTXBASEADDR                         0x80 // Default
#define RFLR_FIFORXBASEADDR                         0x00 // Default

//---------IRQ---------------
#define RFLR_IRQFLAGS_RXTIMEOUT_MASK                0x80
#define RFLR_IRQFLAGS_RXDONE_MASK                   0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR_MASK          0x20
#define RFLR_IRQFLAGS_VALIDHEADER_MASK              0x10
#define RFLR_IRQFLAGS_TXDONE_MASK                   0x08
#define RFLR_IRQFLAGS_CADDONE_MASK                  0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL_MASK       0x02
#define RFLR_IRQFLAGS_CADDETECTED_MASK              0x01

#define RFLR_IRQFLAGS_RXTIMEOUT                     0x80
#define RFLR_IRQFLAGS_RXDONE                        0x40
#define RFLR_IRQFLAGS_PAYLOADCRCERROR               0x20
#define RFLR_IRQFLAGS_VALIDHEADER                   0x10
#define RFLR_IRQFLAGS_TXDONE                        0x08
#define RFLR_IRQFLAGS_CADDONE                       0x04
#define RFLR_IRQFLAGS_FHSSCHANGEDCHANNEL            0x02
#define RFLR_IRQFLAGS_CADDETECTED                   0x01

//------------ModemStat-----------------
#define RFLR_MODEMSTAT_RX_CR_MASK                   0x1F
#define RFLR_MODEMSTAT_MODEM_STATUS_MASK            0xE0
//-----------HopeChannel
#define RFLR_HOPCHANNEL_PLL_LOCK_TIMEOUT_MASK       0x7F
#define RFLR_HOPCHANNEL_PLL_LOCK_FAIL               0x80
#define RFLR_HOPCHANNEL_PLL_LOCK_SUCCEED            0x00 // Default

#define RFLR_HOPCHANNEL_CRCONPAYLOAD_MASK           0xBF
#define RFLR_HOPCHANNEL_CRCONPAYLOAD_ON             0x40
#define RFLR_HOPCHANNEL_CRCONPAYLOAD_OFF            0x00 // Default

#define RFLR_HOPCHANNEL_CHANNEL_MASK                0x3F
//------------ModemConfig1--------------
#define RFLR_MODEMCONFIG1_BW_MASK                   0x0F
#define RFLR_MODEMCONFIG1_BW_7_81_KHZ               0x00
#define RFLR_MODEMCONFIG1_BW_10_41_KHZ              0x10
#define RFLR_MODEMCONFIG1_BW_15_62_KHZ              0x20
#define RFLR_MODEMCONFIG1_BW_20_83_KHZ              0x30
#define RFLR_MODEMCONFIG1_BW_31_25_KHZ              0x40
#define RFLR_MODEMCONFIG1_BW_41_66_KHZ              0x50
#define RFLR_MODEMCONFIG1_BW_62_50_KHZ              0x60
#define RFLR_MODEMCONFIG1_BW_125_KHZ                0x70 // Default
#define RFLR_MODEMCONFIG1_BW_250_KHZ                0x80
#define RFLR_MODEMCONFIG1_BW_500_KHZ                0x90

#define RFLR_MODEMCONFIG1_CODINGRATE_MASK           0xF1
#define RFLR_MODEMCONFIG1_CODINGRATE_4_5            0x02
#define RFLR_MODEMCONFIG1_CODINGRATE_4_6            0x04 // Default
#define RFLR_MODEMCONFIG1_CODINGRATE_4_7            0x06
#define RFLR_MODEMCONFIG1_CODINGRATE_4_8            0x08

#define RFLR_MODEMCONFIG1_IMPLICITHEADER_MASK       0xFE
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_ON         0x01
#define RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF        0x00 // Default

//------------ModemConfig2--------------
#define RFLR_MODEMCONFIG2_SF_MASK                   0x0F
#define RFLR_MODEMCONFIG2_SF_6                      0x60
#define RFLR_MODEMCONFIG2_SF_7                      0x70 // Default
#define RFLR_MODEMCONFIG2_SF_8                      0x80
#define RFLR_MODEMCONFIG2_SF_9                      0x90
#define RFLR_MODEMCONFIG2_SF_10                     0xA0
#define RFLR_MODEMCONFIG2_SF_11                     0xB0
#define RFLR_MODEMCONFIG2_SF_12                     0xC0

#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_MASK     0xF7
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_ON       0x08
#define RFLR_MODEMCONFIG2_TXCONTINUOUSMODE_OFF      0x00

#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_MASK         0xFB
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON           0x04
#define RFLR_MODEMCONFIG2_RXPAYLOADCRC_OFF          0x00 // Default

#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB_MASK       0xFC
#define RFLR_MODEMCONFIG2_SYMBTIMEOUTMSB            0x00 // Default

//----------SubTimeOut-----------------
#define RFLR_SYMBTIMEOUTLSB_SYMBTIMEOUT             0x64 // Default

//----------Preamble-------------------
#define RFLR_PREAMBLELENGTHMSB                      0x00 // Default
#define RFLR_PREAMBLELENGTHLSB                      0x08 // Default
//----------Payload--------------------
#define RFLR_PAYLOADLENGTH                          0x0E // Default
#define RFLR_PAYLOADMAXLENGTH                       0xFF // Default
//----------HopePeriod
#define RFLR_HOPPERIOD_FREQFOPPINGPERIOD            0x00 // Default
//------------ModemConfig3--------------
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_MASK  0xF7
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_ON    0x08
#define RFLR_MODEMCONFIG3_LOWDATARATEOPTIMIZE_OFF   0x00 // Default

#define RFLR_MODEMCONFIG3_AGCAUTO_MASK              0xFB
#define RFLR_MODEMCONFIG3_AGCAUTO_ON                0x04 // Default
#define RFLR_MODEMCONFIG3_AGCAUTO_OFF               0x00
//-----------DetectOptimize-------------
#define RFLR_DETECTIONOPTIMIZE_MASK                 0xF8
#define RFLR_DETECTIONOPTIMIZE_SF7_TO_SF12          0x03 // Default
#define RFLR_DETECTIONOPTIMIZE_SF6                  0x05
//-----------InvertIQ---------------
#define RFLR_INVERTIQ_RX_MASK                       0xBF
#define RFLR_INVERTIQ_RX_OFF                        0x00
#define RFLR_INVERTIQ_RX_ON                         0x40
#define RFLR_INVERTIQ_TX_MASK                       0xFE
#define RFLR_INVERTIQ_TX_OFF                        0x01
#define RFLR_INVERTIQ_TX_ON                         0x00
//-----------DetectionThresh------------
#define RFLR_DETECTIONTHRESH_SF7_TO_SF12            0x0A // Default
#define RFLR_DETECTIONTHRESH_SF6                    0x0C

#define RFLR_INVERTIQ2_ON                           0x19
#define RFLR_INVERTIQ2_OFF                          0x1D

#define RFLR_DIOMAPPING1_DIO0_MASK                  0x3F
#define RFLR_DIOMAPPING1_DIO0_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO0_01                    0x40
#define RFLR_DIOMAPPING1_DIO0_10                    0x80
#define RFLR_DIOMAPPING1_DIO0_11                    0xC0

#define RFLR_DIOMAPPING1_DIO1_MASK                  0xCF
#define RFLR_DIOMAPPING1_DIO1_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO1_01                    0x10
#define RFLR_DIOMAPPING1_DIO1_10                    0x20
#define RFLR_DIOMAPPING1_DIO1_11                    0x30

#define RFLR_DIOMAPPING1_DIO2_MASK                  0xF3
#define RFLR_DIOMAPPING1_DIO2_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO2_01                    0x04
#define RFLR_DIOMAPPING1_DIO2_10                    0x08
#define RFLR_DIOMAPPING1_DIO2_11                    0x0C

#define RFLR_DIOMAPPING1_DIO3_MASK                  0xFC
#define RFLR_DIOMAPPING1_DIO3_00                    0x00  // Default
#define RFLR_DIOMAPPING1_DIO3_01                    0x01
#define RFLR_DIOMAPPING1_DIO3_10                    0x02
#define RFLR_DIOMAPPING1_DIO3_11                    0x03

#define RFLR_DIOMAPPING2_DIO4_MASK                  0x3F
#define RFLR_DIOMAPPING2_DIO4_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO4_01                    0x40
#define RFLR_DIOMAPPING2_DIO4_10                    0x80
#define RFLR_DIOMAPPING2_DIO4_11                    0xC0

#define RFLR_DIOMAPPING2_DIO5_MASK                  0xCF
#define RFLR_DIOMAPPING2_DIO5_00                    0x00  // Default
#define RFLR_DIOMAPPING2_DIO5_01                    0x10
#define RFLR_DIOMAPPING2_DIO5_10                    0x20
#define RFLR_DIOMAPPING2_DIO5_11                    0x30

#define RFLR_DIOMAPPING2_MAP_MASK                   0xFE
#define RFLR_DIOMAPPING2_MAP_PREAMBLEDETECT         0x01
#define RFLR_DIOMAPPING2_MAP_RSSI                   0x00  // Default

#define RFLR_PLLHOP_FASTHOP_MASK                    0x7F
#define RFLR_PLLHOP_FASTHOP_ON                      0x80
#define RFLR_PLLHOP_FASTHOP_OFF                     0x00 // Default

#define RFLR_TCXO_TCXOINPUT_MASK                    0xEF
#define RFLR_TCXO_TCXOINPUT_ON                      0x10
#define RFLR_TCXO_TCXOINPUT_OFF                     0x00  // Default

#define RFLR_PADAC_20DBM_MASK                       0xF8
#define RFLR_PADAC_20DBM_ON                         0x07
#define RFLR_PADAC_20DBM_OFF                        0x04  // Default

#define RF_BITRATEFRAC_MASK                         0xF0

#define RF_PLL_BANDWIDTH_MASK                       0x3F
#define RF_PLL_BANDWIDTH_75                         0x00
#define RF_PLL_BANDWIDTH_150                        0x40
#define RF_PLL_BANDWIDTH_225                        0x80
#define RF_PLL_BANDWIDTH_300                        0xC0  // Default

#endif

//--------------------- Defines for SPI ----------------------------
#define 	WRITE_SINGLE     				0x80
#define 	READ_SINGLE     				0x00

#define SPI1_DR_8bit 			*((__IO uint8_t *)&SPI1->DR)		// Limit for spi bus 8 bit

struct GPS_RX {
	uint8_t	time_mm;
	uint8_t time_ss;
	uint8_t time_hh;
	uint32_t 	lat;
	uint8_t		lat_sign; // N=0,S=1
	uint32_t 	lon;
	uint8_t		lon_sign;	//E=0,W=1
	uint16_t 	speed;
	uint16_t	course;
} GPS_RMC;


uint8_t i,Flag=0,rx_bytes,counter;
uint8_t rssi,fr_rssi,snr,act_RFM=6;
uint8_t flag_update_lcd_rfm=0;
char RX_BUF[16];
//----------------- for tx module--------------------------
char TX_BUF[16] = {'T','e','s','t'};

//---------------Timer settings-------------
uint16_t TimingDelay,led_count,ms1000;
uint8_t ms200=0,msec200=0,sec_tic=0;

void TimingDelayDec(void) 																													{
 if (TimingDelay			!=0x00) TimingDelay--;
 if (!led_count) {led_count=500; GPIOB->ODR ^=1;}
 if (!ms1000) {ms1000=1000;sec_tic=1;}
 if (!ms200) {ms200=200;msec200=1;}
 led_count--;ms1000--;ms200--;
 }

void TIM17_IRQHandler(void)																													{
		if (TIM17->SR & TIM_SR_UIF) {
					TimingDelayDec();
  				TIM17->SR &=(~TIM_SR_UIF);
		}
}	
void delay_ms (uint16_t DelTime) 																										{
    TimingDelay=DelTime;
  while(TimingDelay!= 0x00);
}

//--------------------------------LORA CONFIGURATION------------------------
const unsigned char LoRa_config[]={			// 868MHz, SF12, 125kHz, 300bps, MaxPower, OcpOn, 9Byte info 
	
	REG_LR_OPMODE,					0x80, 								//Lora mode, HF, Sleep
	REG_LR_FRFMSB, 					0xD9,									//Freq = 868MHz
	REG_LR_FRFMID, 					0x00,									//Freq = 868MHz
	REG_LR_FRFLSB, 					0x04,									//Freq = 868MHz
	REG_LR_PACONFIG, 				0b11111111,						//Max power
	REG_LR_OCP,							0x1F,									//OCP-on, Current 130 mA
	REG_LR_MODEMCONFIG1,		0b01110010,						//125kHz,4/5 Coding Rate/ Explicit
	REG_LR_MODEMCONFIG2, 		0xC2,									//
	REG_LR_PAYLOADLENGTH,		0x10,									//16 bytes // Standart -1

    
	// -------------------Standart parameters	-----------------------	
/*
	REG_LR_IRQFLAGSMASK, 		0x48,									//Tx_Complete IRQ, RX_Complete IRQ	
	REG_LR_PARAMP,					0x09,									//Standart 40us
	REG_LR_FIFOADDRPTR,			0x00,									//Standart
	REG_LR_FIFOTXBASEADDR, 	0x80,									//Standart
	REG_LR_FIFORXBASEADDR, 	0x00,									//Standart
	REG_LR_LNA,							0b00100000,						//Standart
	REG_LR_SYMBTIMEOUTLSB,	0x64, 								//Standart
	REG_LR_PREAMBLEMSB,			0x00,									//Standart
	REG_LR_PREAMBLELSB,			0x08,									//Standart
	REG_LR_PAYLOADMAXLENGTH,0xFF,									//Standart
	REG_LR_HOPPERIOD,				0x00,									//Standart  
*/	
};

uint8_t SX1276_WriteSingle(uint8_t command,uint8_t value) 													{//WriteSingle

uint8_t temp;
CS_LO();
//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready

while (!(SPI1->SR & SPI_SR_TXE)){};  
SPI1_DR_8bit = (WRITE_SINGLE | command);
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;
	
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = value;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
temp=SPI1_DR_8bit;	

CS_HI();
return temp;	
}

uint8_t SX1276_ReadSingle(uint8_t command) 																					{//ReadSingle

	uint8_t temp;
CS_LO();
while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready
	
while (!(SPI1->SR & SPI_SR_TXE)){}; 
SPI1_DR_8bit = (command | READ_SINGLE);
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
command = SPI1_DR_8bit;
while (!(SPI1->SR & SPI_SR_TXE)){};	 
SPI1_DR_8bit = 0x00;
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){};
temp = SPI1_DR_8bit;

CS_HI();	
return temp;	
}
void 		SX1276_WriteBurst( uint8_t addr, char *buff, uint8_t size )							{//WriteBurst

	uint8_t j_;
CS_LO();
//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready
while (!(SPI1->SR & SPI_SR_TXE)){};	
SPI1_DR_8bit = (addr | WRITE_SINGLE);
while (SPI1->SR & SPI_SR_BSY){};		
while (!(SPI1->SR & SPI_SR_RXNE)){}; 
SPI1_DR_8bit;	
for( j_ = 0; j_ < size; j_ ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	
	SPI1_DR_8bit = buff[j_]; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1_DR_8bit;	
 }
CS_HI();
}
void 		SX1276_ReadBurst( uint8_t cmd, char *buff, uint8_t size )								{//ReadBurst
 
	uint8_t j_;
CS_LO();
//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready
while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = (cmd | READ_SINGLE);
while (SPI1->SR & SPI_SR_BSY){};
while (!(SPI1->SR & SPI_SR_RXNE)){}; SPI1_DR_8bit;	
for( j_ = 0; j_ < size; j_ ++ )
 {
  while (!(SPI1->SR & SPI_SR_TXE)){};	SPI1_DR_8bit = 0x00; 
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){}; buff[j_] = SPI1_DR_8bit;		
 }
 CS_HI();
}
void SX1276_Init(void)																															{//CC1101_Init
uint8_t qnt,i_temp=0;
qnt=sizeof (LoRa_config);
	
SX1276_RES();
delay_ms(10);
SX1276_REL();	
delay_ms(10);
	
while (i_temp < qnt)
{
	SX1276_WriteSingle(LoRa_config[i_temp],LoRa_config[i_temp+1]);
	i_temp+=2;	
}
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
}



//------------------------------------T191 LCD ------------------------------
const unsigned char T191_font[]= 																										{// Font 8*5 
	0x00, 0x00, 0x00, 0x00, 0x00 ,   // sp  32
     0x00, 0x00, 0x2f, 0x00, 0x00 ,   // !   33
     0x00, 0x07, 0x00, 0x07, 0x00 ,   // "   34
     0x14, 0x7f, 0x14, 0x7f, 0x14 ,   // #   35
     0x24, 0x2a, 0x7f, 0x2a, 0x12 ,   // $   36
     0xc4, 0xc8, 0x10, 0x26, 0x46 ,   // %   37
     0x36, 0x49, 0x55, 0x22, 0x50 ,   // &   38
     0x00, 0x05, 0x03, 0x00, 0x00 ,   // '   39
     0x00, 0x1c, 0x22, 0x41, 0x00 ,   // (   40
     0x00, 0x41, 0x22, 0x1c, 0x00 ,   // )   41
     0x14, 0x08, 0x3E, 0x08, 0x14 ,   // *   42
     0x08, 0x08, 0x3E, 0x08, 0x08 ,   // +   43
     0x00, 0x00, 0x50, 0x30, 0x00 ,   // ,   44
     0x10, 0x10, 0x10, 0x10, 0x10 ,   // -   45
     0x00, 0x60, 0x60, 0x00, 0x00 ,   // .   46
     0x20, 0x10, 0x08, 0x04, 0x02 ,   // /   47
     0x3E, 0x51, 0x49, 0x45, 0x3E ,   // 0   48
     0x00, 0x42, 0x7F, 0x40, 0x00 ,   // 1   49
     0x42, 0x61, 0x51, 0x49, 0x46 ,   // 2   50
     0x21, 0x41, 0x45, 0x4B, 0x31 ,   // 3   51
     0x18, 0x14, 0x12, 0x7F, 0x10 ,   // 4   52
     0x27, 0x45, 0x45, 0x45, 0x39 ,   // 5   53
     0x3C, 0x4A, 0x49, 0x49, 0x30 ,   // 6   54
     0x01, 0x71, 0x09, 0x05, 0x03 ,   // 7   55
     0x36, 0x49, 0x49, 0x49, 0x36 ,   // 8   56
     0x06, 0x49, 0x49, 0x29, 0x1E ,   // 9   57
     0x00, 0x36, 0x36, 0x00, 0x00 ,   // :   58
     0x00, 0x56, 0x36, 0x00, 0x00 ,   // ;   59
     0x08, 0x14, 0x22, 0x41, 0x00 ,   // <   60
     0x14, 0x14, 0x14, 0x14, 0x14 ,   // =   61
     0x00, 0x41, 0x22, 0x14, 0x08 ,   // >   62
     0x02, 0x01, 0x51, 0x09, 0x06 ,   // ?   63
     0x32, 0x49, 0x59, 0x51, 0x3E ,   // @   64
     0x7E, 0x11, 0x11, 0x11, 0x7E ,   // A   65
     0x7F, 0x49, 0x49, 0x49, 0x36 ,   // B   66
     0x3E, 0x41, 0x41, 0x41, 0x22 ,   // C   67
     0x7F, 0x41, 0x41, 0x22, 0x1C ,   // D   68
     0x7F, 0x49, 0x49, 0x49, 0x41 ,   // E   69
     0x7F, 0x09, 0x09, 0x09, 0x01 ,   // F   70
     0x3E, 0x41, 0x49, 0x49, 0x7A ,   // G   71
     0x7F, 0x08, 0x08, 0x08, 0x7F ,   // H   72
     0x00, 0x41, 0x7F, 0x41, 0x00 ,   // I   73
     0x20, 0x40, 0x41, 0x3F, 0x01 ,   // J   74
     0x7F, 0x08, 0x14, 0x22, 0x41 ,   // K   75
     0x7F, 0x40, 0x40, 0x40, 0x40 ,   // L   76
     0x7F, 0x02, 0x0C, 0x02, 0x7F ,   // M   77
     0x7F, 0x04, 0x08, 0x10, 0x7F ,   // N   78
     0x3E, 0x41, 0x41, 0x41, 0x3E ,   // O   79
     0x7F, 0x09, 0x09, 0x09, 0x06 ,   // P   80
     0x3E, 0x41, 0x51, 0x21, 0x5E ,   // Q   81
     0x7F, 0x09, 0x19, 0x29, 0x46 ,   // R   82
     0x46, 0x49, 0x49, 0x49, 0x31 ,   // S   83
     0x01, 0x01, 0x7F, 0x01, 0x01 ,   // T   84
     0x3F, 0x40, 0x40, 0x40, 0x3F ,   // U   85
     0x1F, 0x20, 0x40, 0x20, 0x1F ,   // V   86
     0x3F, 0x40, 0x38, 0x40, 0x3F ,   // W   87
     0x63, 0x14, 0x08, 0x14, 0x63 ,   // X   88
     0x07, 0x08, 0x70, 0x08, 0x07 ,   // Y   89
     0x61, 0x51, 0x49, 0x45, 0x43 ,   // Z   90
     0x00, 0x7F, 0x41, 0x41, 0x00 ,   // [   91
     0x55, 0x2A, 0x55, 0x2A, 0x55 ,   // 55  92
     0x00, 0x41, 0x41, 0x7F, 0x00 ,   // ]   93
     0x04, 0x02, 0x01, 0x02, 0x04 ,   // ^   94
     0x40, 0x40, 0x40, 0x40, 0x40 ,   // _   95
     0x00, 0x01, 0x02, 0x04, 0x00 ,   // '   96
     0x20, 0x54, 0x54, 0x54, 0x78 ,   // a   97
     0x7F, 0x48, 0x44, 0x44, 0x38 ,   // b   98
     0x38, 0x44, 0x44, 0x44, 0x20 ,   // c   99
     0x38, 0x44, 0x44, 0x48, 0x7F ,   // d   100
     0x38, 0x54, 0x54, 0x54, 0x18 ,   // e   101
     0x08, 0x7E, 0x09, 0x01, 0x02 ,   // f   102
     0x0C, 0x52, 0x52, 0x52, 0x3E ,   // g   103
     0x7F, 0x08, 0x04, 0x04, 0x78 ,   // h   104
     0x00, 0x44, 0x7D, 0x40, 0x00 ,   // i   105
     0x20, 0x40, 0x44, 0x3D, 0x00 ,   // j   106
     0x7F, 0x10, 0x28, 0x44, 0x00 ,   // k   107
     0x00, 0x41, 0x7F, 0x40, 0x00 ,   // l   108
     0x7C, 0x04, 0x18, 0x04, 0x78 ,   // m   109
     0x7C, 0x08, 0x04, 0x04, 0x78 ,   // n   110
     0x38, 0x44, 0x44, 0x44, 0x38 ,   // o   111
     0x7C, 0x14, 0x14, 0x14, 0x08 ,   // p   112
     0x08, 0x14, 0x14, 0x18, 0x7C ,   // q   113
     0x7C, 0x08, 0x04, 0x04, 0x08 ,   // r   114
     0x48, 0x54, 0x54, 0x54, 0x20 ,   // s   115
     0x04, 0x3F, 0x44, 0x40, 0x20 ,   // t   116
     0x3C, 0x40, 0x40, 0x20, 0x7C ,   // u   117
     0x1C, 0x20, 0x40, 0x20, 0x1C ,   // v   118
     0x3C, 0x40, 0x30, 0x40, 0x3C ,   // w   119
     0x44, 0x28, 0x10, 0x28, 0x44 ,   // x   120
     0x0C, 0x50, 0x50, 0x50, 0x3C ,   // y   121
     0x44, 0x64, 0x54, 0x4C, 0x44 ,   // z   122
    //
     0x00, 0x08, 0x36, 0x41, 0x00 ,   //7B- {
     0x00, 0x00, 0x7f, 0x00, 0x00 ,   //7C- |
     0x00, 0x41, 0x36, 0x08, 0x00 ,   //7D- }
     0x04, 0x02, 0x04, 0x08, 0x04 ,   //7E- ~
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //7F- 
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //80- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //81- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //82- ‚
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //83- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //84- „
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //85- …
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //86- †
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //87- ‡
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //88- €
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //89- ‰
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8A- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8B- ‹
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8C- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8D- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8E- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //8F- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //90- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //91- ‘
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //92- ’
     0x00, 0x00, 0x00, 0x00, 0x00 ,   // 93- “
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //94- ”
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //95- •
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //96- –
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //97- —
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //98- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //99- ™ 
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9A- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9B- ›
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9C- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9D- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9E- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //9F- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A0-  
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A1- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A2- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A3- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A4- ¤
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A5- ?
     0x00, 0x00, 0x36, 0x00, 0x00 ,   //A6- ¦
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A7- §
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A8- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //A9- ©
     0x3E, 0x49, 0x49, 0x49, 0x22 ,   //AA- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AB- «
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AC- ¬
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AD- ­
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //AE- ®
     0x44, 0x45, 0x7C, 0x45, 0x44 ,   //AF- ?
     0x06, 0x09, 0x09, 0x06, 0x00 ,   //B0- ° - Degree Symbol
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B1- ±
     0x00, 0x41, 0x7F, 0x41, 0x00 ,   //B2- ?
     0x00, 0x44, 0x7D, 0x40, 0x00 ,   //B3- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B4- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B5- µ
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B6- ¶
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B7- ·
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B8- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //B9- ?
     0x38, 0x54, 0x44, 0x28, 0x00 ,   //BA- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BB- »
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BC- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BD- ?
     0x00, 0x00, 0x00, 0x00, 0x00 ,   //BE- ?
     0x4A, 0x48, 0x7A, 0x40, 0x40 ,   //BF- ?                             
    //???????
 0x7E, 0x11, 0x11, 0x11, 0x7E , // A        /*192*/
 0x7F, 0x49, 0x49, 0x49, 0x31 , // ?
 0x7F, 0x49, 0x49, 0x49, 0x36 , // B
 0x7F, 0x01, 0x01, 0x01, 0x03 , // ?
 0x60, 0x3E, 0x21, 0x21, 0x7F , // ?
 0x7F, 0x49, 0x49, 0x49, 0x41 , // E
 0x63, 0x14, 0x7F, 0x14, 0x63 , // ?
 0x22, 0x49, 0x49, 0x49, 0x36 , // ?
 0x7F, 0x10, 0x08, 0x04, 0x7F , // ?
 0x7F, 0x10, 0x09, 0x04, 0x7F , // ?
 0x7F, 0x08, 0x14, 0x22, 0x41 , // K
 0x7C, 0x02, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x02, 0x0C, 0x02, 0x7F , // M
 0x7F, 0x08, 0x08, 0x08, 0x7F , // H
 0x3E, 0x41, 0x41, 0x41, 0x3E , // O
 0x7F, 0x01, 0x01, 0x01, 0x7F , // ?
 0x7F, 0x09, 0x09, 0x09, 0x06 , // P
 0x3E, 0x41, 0x41, 0x41, 0x22 , // C
 0x01, 0x01, 0x7F, 0x01, 0x01 , // T
 0x07, 0x48, 0x48, 0x48, 0x3F , // ?
 0x1C, 0x22, 0x7F, 0x22, 0x1C , // ?
 0x63, 0x14, 0x08, 0x14, 0x63 , // X
 0x3F, 0x20, 0x20, 0x3F, 0x60 , // ?
 0x07, 0x08, 0x08, 0x08, 0x7F , // ?
 0x7F, 0x40, 0x7F, 0x40, 0x7F , // ?
 0x3F, 0x20, 0x3F, 0x20, 0x7F , // ?
 0x01, 0x7F, 0x48, 0x48, 0x30 , // ?
 0x7F, 0x48, 0x48, 0x30, 0x7F , // ?
 0x7F, 0x48, 0x48, 0x48, 0x30 , // ?
 0x22, 0x49, 0x49, 0x49, 0x3E , // ?
 0x7F, 0x08, 0x3E, 0x41, 0x3E , // ?
 0x46, 0x29, 0x19, 0x09, 0x7F , // ?
 0x20, 0x54, 0x54, 0x54, 0x78 , // a
 0x78, 0x54, 0x54, 0x54, 0x20 , // b
 0x7C, 0x54, 0x54, 0x54, 0x28 , // ?
 0x7C, 0x04, 0x04, 0x04, 0x00 , // ?
 0x60, 0x38, 0x24, 0x38, 0x60 , // ?
 0x38, 0x54, 0x54, 0x54, 0x18 , // e
 0x6C, 0x10, 0x7C, 0x10, 0x6C , // ?
 0x28, 0x44, 0x54, 0x54, 0x28 , // ?
 0x3C, 0x40, 0x40, 0x20, 0x7C , // ?
 0x3C, 0x40, 0x42, 0x20, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x28, 0x44 , // ?
 0x60, 0x10, 0x08, 0x04, 0x7C , // ?
 0x7C, 0x08, 0x10, 0x08, 0x7C , // ?
 0x7C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x38, 0x44, 0x44, 0x44, 0x38 , // o
 0x7C, 0x04, 0x04, 0x04, 0x7C , // ?
 0x7C, 0x14, 0x14, 0x14, 0x08 , // p
 0x38, 0x44, 0x44, 0x44, 0x20 , // c
 0x04, 0x04, 0x7C, 0x04, 0x04 , // ?
 0x0C, 0x50, 0x50, 0x50, 0x3C , // y
 0x18, 0x24, 0x7C, 0x24, 0x18 , // ?
 0x44, 0x28, 0x10, 0x28, 0x44 , // x
 0x3C, 0x20, 0x20, 0x3C, 0x60 , // ?
 0x0C, 0x10, 0x10, 0x10, 0x7C , // ?
 0x7C, 0x40, 0x7C, 0x40, 0x7C , // ?
 0x3C, 0x20, 0x3C, 0x20, 0x7C , // ?
 0x04, 0x7C, 0x48, 0x48, 0x30 , // ?
 0x7C, 0x48, 0x30, 0x00, 0x7C , // ?
 0x7C, 0x48, 0x48, 0x48, 0x30 , // ?
 0x28, 0x44, 0x54, 0x54, 0x38 , // ?
 0x7C, 0x10, 0x38, 0x44, 0x38 , // ?
 0x58, 0x24, 0x24, 0x24, 0x7C  //  ?
 };


void I2C_write(unsigned char byte_send)																			    		{//WRITE
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; 
	 I2C1->TXDR = byte_send;
	}
void I2C_Start(void)    																														{//Start
	 while(I2C1->ISR & I2C_ISR_BUSY){}; //I2C_ISR_BUSY
	 I2C1->CR2 |= (T191_Addr << I2C_CR2_SADD_Pos) | (0xff << I2C_CR2_NBYTES_Pos) | I2C_CR2_START;
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	}
void I2C_Stop(void) 																																{//Stop
	 while(!(I2C1->ISR & I2C_ISR_TXE)){}; // TX buf Empty
	 I2C1->CR2 |= I2C_CR2_STOP;
	 while(!(I2C1->ISR & I2C_ISR_STOPF)){};
	 I2C1->ICR |= I2C_ICR_STOPCF;		
 }
void LCD_Init(void) 																																{// INIT LCD T191
    
     LCD_RES();delay_ms(10);
  	 LCD_ON();delay_ms(10);

     I2C_Start();
     I2C_write(0x00);I2C_write(0x31);
     I2C_write(0x14);I2C_write(0x06); 																			// Initial Settings
     I2C_write(0x30);I2C_write(0x11);
     I2C_write(0x05);I2C_write(0x31);
     I2C_write(0x9A); 																											// Contrast
     I2C_write(0x0C); 																											//0x0C -Normal mode, 00 - Mirror mode
     I2C_write(0x30);I2C_write(0x0C); 																			// ?????????? ?????
     I2C_write(0x40);I2C_write(0x80); 																			// GoTo x=0, y=0
     I2C_Stop();
    }
void LCD_Gotoxy ( uint8_t x, uint8_t y )														    						{//78.00.30.80|x.40|y
       I2C_Start();
       I2C_write(0x00);
       I2C_write(0x30);
       I2C_write(0x80 | 1+x*6);  																//(0...15 Colls
       I2C_write(0x40 | y);  																		//(0...7 - Rows)
       I2C_Stop();
    }

void LCD_Write_data (unsigned char dat_byte)																	      {//78.40.XX
       I2C_Start();
       I2C_write(0x40);
       I2C_write(dat_byte);
       I2C_Stop();
      }

void LCD_mode(uint8_t mode)																													{//78.00.30.0C - Normal; 78.00.30.0D - Inverse 
      
       I2C_Start();
       if(mode > 0)
       {I2C_write(0x00);I2C_write(0x30);I2C_write(0x0C);}
       else
       {I2C_write(0x00);I2C_write(0x30);I2C_write(0x0D);}
       I2C_Stop();
      }
void LCD_Char(char flash_o, uint8_t mode)															        			{// Print Symbol
      	 unsigned char i,ch; 
         I2C_Start();
         I2C_write(0x40);
          for(i=0; i<5; i++)
      	  {
      	    ch = T191_font[(flash_o - 32)*5+i];
      	     if(mode) { I2C_write(~ch); }
      			 else  	 { I2C_write(ch);}
      		 if(i == 4)
      			  {
      			     if(mode) { I2C_write(0xff); }
      				  else   {	 I2C_write(0x00); }
      			  }
      	   }
         I2C_Stop();
          }
void LCD_PrintStr(char *s, uint8_t mode)																	    			{// Print String
          while (*s)
        {
          LCD_Char(*s, mode);
          s++;
        }
      }
void LCD_Clear(void)      																													{// Clear LCD
    	 unsigned char i;
    	for(i = 0; i < 10; i++) LCD_PrintStr("                ",0);
      }
       
void LCD_ClearStr(uint8_t y,uint8_t qn)      																				{// Clear String
 uint8_t temp_i; 	 
	for (temp_i=0;temp_i<qn;temp_i++)
	 {
		 LCD_Gotoxy(0,y+temp_i);    	
		 LCD_PrintStr("                ",0);
   }
}
void LCD_PrintDec(long value,uint8_t mode) 																					{// Print Dec
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=10)  {
				d = value % 10; 																				// ??????? ?? ???????
				text[i] = d + '0'; 																			// ????? ASCII ?????? ? ???????? ? ?????
				value /= 10; 																						// "?????????? ????? ??????" -- ??????? ?? 10
			}
		else 
			{	text[i] = value + '0';
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}
			
void LCD_PrintHex(long value,uint8_t mode) 																					{// Print Hex
	
	char i=1,d=0;
	unsigned char text[10];
	do 
  { 
    if (value >=0x10)  {
				d = value % 0x10; 																				
				if(d<0x0A) text[i] = d + '0'; 																			
				else 			 text[i] = d + 0x37;
				value /= 0x10; 																						
			}
		else 
			{	
				if(value < 0x0A)	text[i] = value + '0';			//0..9
				else 							text[i] = value + 0x37;			//A...F
				value=0;
			}
 		i++;
  }
	while(value); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}

void LCD_PrintBin(uint8_t value,uint8_t mode) 																			{// Print Bin
	
	uint8_t i=1,d=0;
	uint8_t text[8];
	do 
  { 
    if (value >=2)  {
				d = value % 2; 																				
				text[i] = d + '0'; 																			
				value /= 0x2; 																						
			}
		else 
			{	
				text[i] = value + '0';			//0..9
				value=0;
			}
 		i++;
  }
	while(i<9); 
	i--;			
	do
	{	LCD_Char(text[i], mode);
		i--;
	}
	while(i);
}
//---------------------------------End of LCD INIT----------------------------
void EXTI0_1_IRQHandler(void)																				{
  if(EXTI->PR & EXTI_PR_PIF0)
	{
		EXTI->PR |= EXTI_PR_PIF0;
		Flag=1;
	}
}


void initial (void)		{
//---------------TIM17------------------
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;    																			//HSI 8 MHz - 1 msek
  TIM17->PSC = 8000-1;
  TIM17->ARR = 1;
  TIM17->CR1 |= TIM_CR1_ARPE | TIM_CR1_DIR | TIM_CR1_CEN; 											// 
	TIM17->DIER |=TIM_DIER_UIE;
	NVIC_EnableIRQ (TIM17_IRQn);
	NVIC_SetPriority(TIM17_IRQn,0x05);	

//-------------------GPIOB-Blinking Led		
	RCC->AHBENR  |= RCC_AHBENR_GPIOBEN; 																					//
	GPIOB->MODER |= GPIO_MODER_MODER0_0;																					//Pb0-Out 
//-------------------GPIOA-CONFIG RX MODE----------
	RCC->AHBENR  |= RCC_AHBENR_GPIOAEN; 																					//
	GPIOA->MODER &=(~GPIO_MODER_MODER2);																					//switch	
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR2_0;																						//Pull-Up
//------------I2C1---------------------	
	RCC->AHBENR 		|=RCC_AHBENR_GPIOFEN;
	GPIOF->MODER 		|=GPIO_MODER_MODER0_1 		| GPIO_MODER_MODER1_1; 							//Alt -mode /Pf0 - SDA, Pf1- SCL
	GPIOF->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR0 	| GPIO_OSPEEDER_OSPEEDR1;						//50 MHz max speed
	GPIOF->AFR[0] 	|=(1<<GPIO_AFRL_AFRL0_Pos) |(1<<GPIO_AFRL_AFRL1_Pos);  				//I2C - Alternative

	GPIOA->MODER |= GPIO_MODER_MODER1_0;																					//LCD_Res_pin

	RCC->APB1ENR |=RCC_APB1ENR_I2C1EN;
	I2C1->TIMINGR |=(0x5<< I2C_TIMINGR_PRESC_Pos);						// If HSI = 8MHz,  Presc =0x05 - 380kHz Freq
	I2C1->CR2 &=(~I2C_CR2_HEAD10R) & (~I2C_CR2_ADD10);
	I2C1->CR1 |=I2C_CR1_PE;
	
//------------------------SPI-----------------------------------
	RCC->AHBENR 		|=RCC_AHBENR_GPIOAEN;
	GPIOA->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR5 | GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7;
	GPIOA->MODER 		|=GPIO_MODER_MODER5_1 | GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1; 	//Pa5..7 - Alt_mode 
	GPIOA->AFR[0] 	|=(0<<GPIO_AFRL_AFRL7_Pos) |(0<<GPIO_AFRL_AFRL6_Pos) | (0<<GPIO_AFRL_AFRL5_Pos);  // SPI - Alternative
	
	GPIOA->OSPEEDR 	|=GPIO_OSPEEDER_OSPEEDR3 | GPIO_OSPEEDER_OSPEEDR4;							//pa3,Pa4 - High speed
	GPIOA->MODER 		|=GPIO_MODER_MODER3_0 |GPIO_MODER_MODER4_0;											//pa3 - Res_SX1276, Pa4 - Nss - out,
	
	RCC->APB2ENR |=RCC_APB2ENR_SPI1EN;
	SPI1->CR1 |=SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_MSTR | (2<<SPI_CR1_BR_Pos);  // if HSI8 - SpiSpeed (BR=2) - 1MHz
	SPI1->CR2 |=SPI_CR2_FRXTH;
	SPI1->CR1 |=SPI_CR1_SPE;
	CS_HI();
//----------------------EXTI-----------------------------------
	RCC->APB2ENR |=RCC_APB2ENR_SYSCFGEN;
	GPIOA->MODER &=(~GPIO_MODER_MODER0);																			// PA0- IRQ from 	SX1276 (Rx- Completed, Tx - completed) 		
	GPIOA->PUPDR |=GPIO_PUPDR_PUPDR0_1;																				//Pull_down
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR1_EXTI0_PA;
	//EXTI->FTSR |= EXTI_FTSR_TR0; //Falling
	EXTI->RTSR |= EXTI_RTSR_RT0; //rising
	
  NVIC_SetPriority(EXTI0_1_IRQn, 2); 
  NVIC_EnableIRQ(EXTI0_1_IRQn); 
	__enable_irq ();	
} 


int main(void) 									{	//main
initial();
EXTI->IMR |= EXTI_IMR_MR0;
SX1276_Init();
//------------------Initial parameters-----------------------------------
if(!RX_TX_MODE()) 							{//RX-MODE
LCD_Init();
LCD_Clear();
LCD_Gotoxy (0,0);LCD_PrintStr(" Test RFM95W ",1);
SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00);
SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);	
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);	
}
else 														{// TX-MODE
	
}





//------------------Main Loop--------------------------------------------
while (1)  											{// Main loop
	
if (!RX_TX_MODE())  {//RX-Mode
	
	if(Flag )					{//Has got received Data from RFM
	
	rx_bytes= SX1276_ReadSingle(REG_LR_RXNBBYTES);
	rssi=SX1276_ReadSingle(REG_LR_PKTRSSIVALUE);
	snr= SX1276_ReadSingle(REG_LR_PKTSNRVALUE);
	SX1276_ReadBurst( REG_LR_FIFO, RX_BUF, rx_bytes);
	
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);
	
	Flag=0;act_RFM=0;				
	flag_update_lcd_rfm=1;
	}
	else 							{//Got no information in buffer
		if (sec_tic) { act_RFM++;}
}

	if (msec200) 			{//Print new data on LCD every 200 msek
		if(flag_update_lcd_rfm)							{//Has data for update rfm 
					
			LCD_Gotoxy (0,2);		LCD_PrintStr("RSSI -",0); LCD_PrintDec((157-rssi),0);LCD_PrintStr("  ",0);
			LCD_Gotoxy (10,2);	LCD_PrintStr("SNR ",0); LCD_PrintDec((snr/4),0);LCD_PrintStr("  ",0);
			LCD_ClearStr(4,1);
			LCD_Gotoxy (0,4);		LCD_PrintStr(RX_BUF,0);LCD_Gotoxy (10,4);LCD_PrintStr(" ",0);LCD_PrintDec(RX_BUF[13],0);LCD_PrintStr("   ",0);
			flag_update_lcd_rfm=0;
		}
		if (act_RFM>5)											{// 5sec - Clean LCD if no new data from RFM
			LCD_ClearStr(1,7);
			LCD_Gotoxy (0,4);LCD_PrintStr("Not Connected",0);
			fr_rssi=SX1276_ReadSingle(REG_LR_RSSIVALUE);
			LCD_Gotoxy (0,2);	LCD_PrintStr("RSSI -",0); LCD_PrintDec((157-fr_rssi),0);LCD_PrintStr("   ",0);
			act_RFM=0;
		}
	msec200=0;
	}
	sec_tic=0;	
}

else								{// TX-MODE Send 1 time in 3 seconds

i++;
TX_BUF[13]=i;	
SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_01);
SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);	
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x80);
SX1276_WriteBurst( REG_LR_FIFO, TX_BUF, 16);
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_TRANSMITTER|0x80); 

while (!Flag) {};	
SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);	
Flag=0;
delay_ms(3000);

sec_tic=0;
}	


} // end - main loop 
} // end - Main  
	
#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{ while (1)  {  } }
#endif
