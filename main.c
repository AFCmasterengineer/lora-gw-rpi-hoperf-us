/**
 * main.c - 
 * Standalone test applicatoin for LoRa HopeRf95W
 * 
 * @author - Nazmul Alam (fadedreamz) - naz@atlantsembedded.com
 * @date - July, 2017
 * 
 * Copyright - atlantsembedded 2017, Feel free to hack the shit out of it :-)
 **/

#include "st1276.h"
#include <stdio.h>


void onLoraRx(unsigned char * dt, int len) {
	int i = 0;
	for(i = 0; i < len; i++) {
		printf("%02x ", dt[i]);
	}
	printf("\n");
}

int main() {
	int input;
	do_run();
	scanf("%d", &input);
}
