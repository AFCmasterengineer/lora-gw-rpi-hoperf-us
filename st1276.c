/**
 * st1276.c - 
 * C - Driver Interface for TTN gateway using HopeRf95W
 * 
 * @author - Nazmul Alam (fadedreamz) - naz@atlantsembedded.com
 * @date - July, 2017
 * 
 * Copyright - atlantsembedded 2017, Feel free to hack the shit out of it :-)
 **/

#include "st1276.h"
#include <stdio.h>
#include <stdlib.h>
#ifdef _USING_WIRINGPI
#include <wiringPi.h>
#include <wiringPiSPI.h>
#endif
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#ifndef _USING_WIRINGPI
#define pinMode(a,b)
#define digitalWrite(a,b)
#define wiringPiISR(a,b,c)
#endif

#define ASSERT(cond, fmt, ...) do {if (!cond) {printf(fmt, ##__VA_ARGS__);exit(1);}}while(0)

unsigned char spi_single(unsigned char address, unsigned char data);

#define delay_us(_span) usleep(_span)
#define delay_ms(_span) usleep(_span * 1000)

typedef unsigned char u8_t;

unsigned char buff1[2];

#define US_923_3 	0xe6d333
#define US_915		0xe4c000
#define US_902_3	0xe19333


gpio_pins_t pins = { 6, 7, 0 };

int spi_if;
u8_t spi_buffer(unsigned char address, u8_t * dt, u8_t len);

extern void onLoraRx(u8_t * dt, int len);

void do_rx_cont();

u8_t txCounter;
u8_t txBuffer[256];
u8_t txBufferLength;

void chip_select() {
	digitalWrite(pins.CS, LOW);
}

void chip_deselect() {
	digitalWrite(pins.CS, HIGH);
}

void chip_reset() {
	digitalWrite(pins.RST, LOW);
	delay_us(150);
	digitalWrite(pins.RST, HIGH);
	delay_ms(10);
}

int spi_write_single(unsigned char address, unsigned char dt) {
	struct spi_ioc_transfer wxfer[1];
	unsigned char data[2];
	int rc;
	memset(&wxfer[0], 0, sizeof(struct spi_ioc_transfer));
	data[0] = address | 80;
	data[1] = dt;
	wxfer[0].tx_buf = data;
	wxfer[0].len = 2;
	chip_select();
	rc = ioctl(spi_if, SPI_IOC_MESSAGE(1), wxfer); //write(spi_if, data, 2);
	printf("ioctl returned : %02x\n", rc);
	chip_deselect();
	return rc; 
}

static void print_channel_info() {
    u8_t mc1 = spi_single(RegModemConfig1, 0x0);
    u8_t mc2 = spi_single(RegModemConfig2, 0x0);
    u8_t frf0 = spi_single(RegFrMsb, 0x0);
    u8_t frf1 = spi_single(RegFrMid, 0x0);
    u8_t frf2 = spi_single(RegFrLsb, 0x0);

    unsigned long long int freq = (frf0 << 16) | (frf1 << 8) | frf2;
    freq = freq * 32000000 / (1 << 19);
    printf("FREQ: %llu, ", freq);

    printf("SF : %d, ", (mc2 & 0xF0) >> 4);
    switch(mc1 & 0xF0) {
        case BW_125:
                printf("BW 125, ");
                break;
        case BW_250:
                printf("BW 250, ");
                break;
        case BW_500:
                printf("BW 500, ");
        default:
                printf("BW Unknown, ");
                break;
    }
    switch(mc1 & 0x0E) {
        case CR_4_5:
                printf("CR : 4/5");
                break;
        case CR_4_6:
                printf("CR : 4/6");
                break;
        case CR_4_7:
                printf("CR : 4/7");
                break;
        case CR_4_8:
                printf("CR : 4/8");
                break;
        default:
                printf("CR : UNKNOWN");
    }
    printf("\n");
}

void do_tx(unsigned char * data, int len) {
	if (txBuffer != data) {
		u8_t i = 0;
		memcpy(txBuffer, data, len);
		txBufferLength = len;
		for(i = 0; i<len; i++) {
			printf("%02x", data[i]);
		}
		printf("\n");
	}
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_STDBY);
	u8_t txBase = spi_single(RegFifoTxBaseAddr, 0x0);
        spi_single(RegFifoAddrPtr | 0x80, txBase);
        spi_single(RegModemConfig1 | 0x80, BW_500 | CR_4_5 | ImplicitHeader);
	spi_single(RegPayloadLength | 0x80, (u8_t)len);
	spi_single(RegModemConfig1 | 0x80, BW_500 | CR_4_5 | ExplicitHeader);
	spi_single(RegModemConfig2 | 0x80, SF_12 | RxPayloadCrcOn);
	
	set_freq(US_923_3);
	spi_single(RegDioMapping1 | 0x80, Dio0Mapping_TxDone);
	spi_single(RegDioMapping1, 0x0);	
	spi_single(0x39 | 0x80,  0x34);
	printf("RegFifoRx = %02x\n", spi_single(RegFifoRxBaseAddr, 0x0));
        printf("RegFifoTx = %02x\n", spi_single(RegFifoTxBaseAddr, 0x0));
        if (len != spi_buffer(RegFifo | 0x80, data, len)) {
		printf("spi tx write length mismatch\n");
	}
	printf("RxSymbol : %02x\n", spi_single(0x1F, 0x0));
        printf("current pointer address : %02x\n", spi_single(RegFifoAddrPtr, 0x0));
        spi_single(RegDioMapping1 | 0x80, Dio0Mapping_TxDone);
        spi_single(RegDioMapping1, 0x0);
	print_channel_info();
	usleep(1000000);
        spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_TX);
}



u8_t spi_buffer(unsigned char address, u8_t * dt, u8_t len) {
	struct spi_ioc_transfer xfer;
	u8_t res;
 	memset(&xfer, 0, sizeof(xfer));
  	unsigned char dataBuffer[len+1];
  	dataBuffer[0] = address;
  	memcpy(&dataBuffer[1], dt, len);
	xfer.tx_buf = (unsigned long)dataBuffer;
  	xfer.rx_buf = (unsigned long)dataBuffer;
  	xfer.len = len+1;
  	xfer.speed_hz = 100000;
  	xfer.cs_change = 0;
	xfer.delay_usecs = 0;
  	xfer.bits_per_word = 8;
  	res = ioctl(spi_if, SPI_IOC_MESSAGE(1), &xfer);
	if (res > 0)
		memcpy(dt, &dataBuffer[1], len);
	return res - 1;
}

unsigned char spi_single(unsigned char address, unsigned char dt) {
	struct spi_ioc_transfer xfer;
	int res;
 	memset(&xfer, 0, sizeof(xfer));
  	unsigned char dataBuffer[2];
  	dataBuffer[0] = address;
	dataBuffer[1] = dt;
  	xfer.tx_buf = (unsigned long)dataBuffer;
  	xfer.rx_buf = (unsigned long)dataBuffer;
  	xfer.len = 2;
  	xfer.speed_hz = 100000;
  	xfer.cs_change = 0;
	xfer.delay_usecs = 0;
  	xfer.bits_per_word = 8;
	printf("[%02x %02x] --> ", dataBuffer[0], dataBuffer[1]);
  	res = ioctl(spi_if, SPI_IOC_MESSAGE(1), &xfer);
//  	printf("SPI result: %d\n", res);
	printf("[%02x %02x]\n", dataBuffer[0], dataBuffer[1]);
	return dataBuffer[1];
}

unsigned char spi_wr_single(unsigned char address, unsigned char dt) {
	spi_write_single(address, dt);
	//return spi_read_single(address, dt);
	return 0;
}

void newRx() {
	u8_t irqFlags = spi_single(RegIrqFlags, 0x0);
	int i =0;
	if (RegValueByMask(irqFlags, MASK_TxDone) == TxDone) {	
		if (txCounter++ < 3) {
			do_tx(txBuffer, txBufferLength);
			printf("Tx Completed : %d\n");
		} else {
			txCounter = 0;
			do_rx_cont();
		}
	} else if (RegValueByMask(irqFlags, MASK_RxDone) == RxDone) {
		u8_t buff[255];
		u8_t buffLen;
		buffLen = spi_single(RegRxNbBytes, 0x0);
		spi_single(RegFifoAddrPtr | 0x80, spi_single(RegFifoRxCurrentAddr, 0x0));
		spi_buffer(RegFifo, buff, buffLen);		
		printf("Rx Completed\n");
		onLoraRx(buff, buffLen);
	} else
		printf("Unknown flag (not tx or rx)\n");
	spi_single(RegIrqFlags | 0x80, 0xFF);
}

int gpio() {
	pinMode(pins.CS, OUTPUT);
	pinMode(pins.DIO0, INPUT);
	pinMode(pins.RST, OUTPUT);
	digitalWrite(pins.RST, HIGH);

	wiringPiISR(pins.DIO0, INT_EDGE_RISING, newRx); 
}


int set_freq(unsigned long long int freq) {
	spi_single(RegFrMsb | 0x80, (u8_t)(freq >> 16));
	spi_single(RegFrMid | 0x80, (u8_t)(freq >> 8));
	spi_single(RegFrLsb | 0x80, (u8_t)(freq));
	return 0;
}

int cur_freq() {
	u8_t frf0 = spi_single(RegFrMsb, 0x0);
	u8_t frf1 = spi_single(RegFrMid, 0x0);
	u8_t frf2 = spi_single(RegFrLsb, 0x0);

	unsigned long long int freq = (frf0 << 16) | (frf1 << 8) | frf2;
	freq = freq * 32000000 / (1 << 19);
	printf("frequency: %llu\n", freq);
	return 0;
}

int radio() {
	// get chip version to verify
	u8_t chipVersion = spi_single(0x42, 0x0);
	ASSERT(chipVersion == 0x12, "unable to find correct module");
	// enter sleep mode
	spi_single(0x01 | 0x80, 0x0);
	ASSERT(RegValueByMask(spi_single(RegOpMode, OpMode_SLEEP), MASK_OpMode) == OpMode_SLEEP, "unable to put into sleep mode");
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa);
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_STDBY);
	printf("Current sync word : %02x\n", spi_single(0x39, 0x0));
	spi_single(0x39 | 0x80, 0x34);
	ASSERT(RegValueByMask(spi_single(RegOpMode, 0x0), MASK_LongRangeMode) == LongRangeMode_LoRa, "unable to put into lora mode");
	/*spi_single(RegModemConfig1 | 0x80, BW_125 | CR_4_5 | ImplicitHeader);
	//spi_single(RegPayloadLength | 0x80, 0x05);
	spi_single(RegModemConfig1 | 0x80, BW_125 | CR_4_5 | ExplicitHeader);
	set_freq(US_902_3);
	cur_freq();
	spi_single(RegDioMapping1 | 0x80, Dio0Mapping_RxDone);
	spi_single(RegDioMapping1, 0x0);	
	u8_t txBase = spi_single(RegFifoTxBaseAddr, 0x0);
	spi_single(RegFifoAddrPtr | 0x80, txBase);
	printf("RegFifoRx = %02x\n", spi_single(RegFifoRxBaseAddr, 0x0));
	printf("RegFifoTx = %02x\n", spi_single(RegFifoTxBaseAddr, 0x0));
	char payload[5];
	strcpy(payload, "hello");
	if (5 != spi_buffer(RegFifo | 0x80, payload, 5));
	printf("current pointer address : %02x\n", spi_single(RegFifoAddrPtr, 0x0));
	spi_single(RegDioMapping1 | 0x80, Dio0Mapping_TxDone);
	spi_single(RegDioMapping1, 0x0);
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_TX);
	spi_single(RegModemConfig2 | 0x80, SF_7 | RxPayloadCrcOff);
	spi_single(RegPaRamp | 0x80, spi_single(RegPaRamp, 0x0) & 0xF0 | PA_RAMP_50us);
	spi_single(RegPaRamp, 0x0);
	spi_single(0x39, 0x34);
	printf("ModemConfig1 = %02x\n", spi_single(RegModemConfig1, 0x0));
	printf("ModemConfig2 = %02x\n", spi_single(RegModemConfig2, 0x0));
	printf("ModemConfig3 = %02x\n", spi_single(RegModemConfig3, 0x0));
	printf("Sync Word = %02x\n", spi_single(0x39, 0x0));
	print_channel_info();
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_RXCONTINUOUS);
	u8_t curMode = spi_single(0x01, 0x0);
	if (0x80 != (curMode & 0x80)) {
		printf("unable to enter lora mode : %02x\n", spi_single(0x01, 0x0));
	}*/
	do_rx_cont();
	return 0;	
}


void do_rx_cont() {
	spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_STDBY);
	set_freq(US_902_3);
	
	spi_single(RegDioMapping1 | 0x80, Dio0Mapping_RxDone);
	spi_single(RegDioMapping1, 0x0);	

	spi_single(RegModemConfig1 | 0x80, BW_125 | CR_4_5 | ExplicitHeader);	
	spi_single(RegModemConfig2 | 0x80, SF_7 | RxPayloadCrcOff);
        spi_single(RegPaRamp | 0x80, spi_single(RegPaRamp, 0x0) & 0xF0 | PA_RAMP_50us);
        spi_single(RegPaRamp, 0x0);
        spi_single(0x39, 0x34);
        
	print_channel_info();
        spi_single(RegOpMode | 0x80, LongRangeMode_LoRa | OpMode_RXCONTINUOUS);
        u8_t curMode = spi_single(0x01, 0x0);
}


int do_run() {
	//int n;
	int speed = 300000;
	int max_speed;
	int mode;
	unsigned char lstfirst;
	wiringPiSetup();
	spi_if = open("/dev/spidev0.0",O_RDWR);
	int retCode = ioctl(spi_if, SPI_IOC_RD_MODE, &mode);
	printf("IOCTL ret = %d\n", retCode);
	printf("SPI mode = %d\n", mode);
	ioctl(spi_if, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	ioctl(spi_if, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed);
	ioctl(spi_if, SPI_IOC_RD_LSB_FIRST, &lstfirst);
	printf("SPI LSB FIRST : %02x\n", lstfirst);
	printf("SPI max speed : %d\n", max_speed);
	int bits_per_word = 0;
	ioctl(spi_if, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word);
	printf("SPI bits/word = %d\n", bits_per_word);
//	ioctl(spi_if, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word);
//	wiringPiSPISetup(0, 100000);
	gpio();
	radio();
	printf("RF operating mode = %02x\n", spi_single(0x01, 0x00));
	//scanf("%d", &n);
}
