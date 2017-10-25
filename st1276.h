/**
 * st1276.h - 
 * C - Driver Interface for TTN gateway using HopeRf95W
 * 
 * @author - Nazmul Alam (fadedreamz) - naz@atlantsembedded.com
 * @date - July, 2017
 * 
 * Copyright - atlantsembedded 2017, Feel free to hack the shit out of it :-)
 **/

#ifndef _H__
#define _H__

typedef struct gpio_pins_s {
	int CS;
	int DIO0;
	int RST;
}gpio_pins_t;

int do_run();
void do_tx(unsigned char * data, int len);

#define RegValueByMask(_reg, _mask) ((_reg) & (_mask))

#define RegFifo			0x00

#define RegOpMode		0x01

#define LongRangeMode_FSK	0x00
#define LongRangeMode_LoRa	0x80

#define MASK_LongRangeMode	0x80

#define OpMode_SLEEP		0x00
#define OpMode_STDBY		0x01
#define OpMode_FSTX		0x02
#define	OpMode_TX		0x03
#define OpMode_FSRX		0x04
#define OpMode_RXCONTINUOUS	0x05
#define OpMode_RXSINGLE		0x06
#define	OpMode_CAD		0x07

#define MASK_OpMode		0x07

#define LowFrequencyModeOn	0x08
#define LowFrequencyModeOff	0x00

#define MASK_LowFrequencyMode	0x80

#define RegFrMsb		0x06
#define RegFrMid		0x07
#define RegFrLsb		0x08

#define RegFifoTxBaseAddr	0x0E
#define RegFifoRxBaseAddr	0x0F
#define RegFifoAddrPtr		0x0D

#define RegIrqFlags 		0x12

#define RxTimeout		0x80
#define RxDone			0x40
#define	PayloadCrcError		0x20
#define ValidHeader		0x10
#define	TxDone			0x08
#define	CadDone			0x04
#define	FhssChangeChannel	0x02
#define	CadDetected		0x01

#define MASK_RxTimeout		0x80
#define MASK_RxDone		0x40
#define	MASK_PayloadCrcError	0x20
#define MASK_ValidHeader	0x10
#define	MASK_TxDone		0x08
#define	MASK_CadDone		0x04
#define	MASK_FhssChangeChannel	0x02
#define	MASK_CadDetected	0x01

#define RegModemConfig1		0x1D
#define RegPayloadLength 	0x22
#define RegModemConfig2		0x1E

#define SF_6			0x60
#define SF_7 			0x70
#define SF_8			0x80
#define SF_9			0x90
#define SF_10			0xA0
#define SF_11			0xB0
#define SF_12			0xC0

#define RxPayloadCrcOn		0x04
#define RxPayloadCrcOff		0x00

#define RegModemConfig3		0x26



#define BW_7_8		0x00
#define BW_10_4		0x10
#define BW_15_6		0x20
#define BW_20_8		0x30
#define BW_31_25	0x40
#define BW_41_7		0x50
#define BW_62_5		0x60
#define BW_125		0x70
#define BW_250		0x80
#define BW_500		0x90

#define CR_4_5		0x2
#define CR_4_6		0x4
#define CR_4_7		0x6
#define CR_4_8		0x8

#define ExplicitHeader	0x0
#define	ImplicitHeader	0x1

#define RegDioMapping1		0x40

#define Dio0Mapping_TxDone	0x40
#define Dio0Mapping_RxDone	0x00


#define RegRxNbBytes 		0x13
#define RegFifoRxCurrentAddr	0x10

#define RegPaRamp		0x0A

#define PA_RAMP_50us		0x08


#endif
