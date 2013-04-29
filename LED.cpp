//
//  LED.cpp
//  
//
//  Created by Rayce Stipanovich on 4/25/13.
//
//

#include <Arduino.h>
#include <Scheduler.h>
#include <SPI.h>
#include "LED.h"

//data packet defiinitions
#define DEVICE_ID "MLD_ISP_01"
#define B_ACK 0xFF
#define B_GET_ID 0xF7
#define B_SET_INPUT 252
#define B_END_INPUT 253
#define B_INPUT_ACK 254
#define B_INPUT_NAK 238
#define B_SET_PROG_RATE 220
#define B_SET_SHOW_INPUT 241
#define B_SET_BLANK 242
#define B_SET_UNBLANK 243

//led mosfets
#define FET_PIN_1 52
#define FET_PIN_2 50
#define FET_PIN_3 48
#define FET_PIN_4 46
#define FET_PIN_5 44
#define FET_PIN_6 42
#define FET_PIN_7 40
#define FET_PIN_8 38
#define FET_PIN_9 36
#define FET_PIN_10 34
#define FET_PIN_11 32
#define FET_PIN_12 30
#define FET_PIN_13 45
#define FET_PIN_14 43
#define FET_PIN_15 41
#define FET_PIN_16 39

//status pins
#define STAT_LED1_PIN 8

//shift registers
#define SCK_PIN 6
#define DATA_PIN 5
#define BLANK_PIN 12
#define DCPRG_PIN 13
#define VPRG_PIN 26
#define GSCLK_PIN 7 //LED pwm clock
#define LATCH_PIN_1 11 // G
#define LATCH_PIN_2 11 //Blue
#define LATCH_PIN_3 11 //

//extra serial pins







//other deffinitions
#define POV_PERIOD 90 //progressive frame refresh rate in ms
#define STAT_FAIL 0
#define STAT_OK 1
#define STAT_BUSY 2
#define STAT_RUN 3
#define BLINK_MILLIS 300

int inByte = 0;         // incoming serial byte

unsigned int scanrate = 150*POV_PERIOD / 16;

byte pixels[256][3];
byte pixels_old[256][3];

byte row = 0;
byte fetMap[16];
bool selfTest = false;

bool disableBufferSerial = false;
bool overrideOUTPUT = false;
bool showInput = false;
bool blank = false;
bool blinkEnable = true;
bool _blinkEnableBlink = true;

bool l = false;
uint32_t enableGSCLK = 0;

short int _status = STAT_FAIL;

bool status_led_stat = true;

void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
	PIOC->PIO_SODR = g_APinDescription[GSCLK_PIN].ulPin;
	PIOC->PIO_CODR = g_APinDescription[GSCLK_PIN].ulPin;
}

void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency) {
	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
	TC_Configure(tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK4);
	uint32_t rc = VARIANT_MCK/128/frequency; //128 because we selected TIMER_CLOCK4 above
	TC_SetRA(tc, channel, rc/2); //50% high, 50% low
	TC_SetRC(tc, channel, rc);
	TC_Start(tc, channel);
	tc->TC_CHANNEL[channel].TC_IER=TC_IER_CPCS;
	tc->TC_CHANNEL[channel].TC_IDR=~TC_IER_CPCS;
	NVIC_EnableIRQ(irq);
}

void setStat(short int stat) {
	_status = stat;
}

void updateOutput() {
	if (!overrideOUTPUT) {

		short tr = 0;
		for (tr = 0; tr < 16; tr++) {
			digitalWrite(fetMap[tr], HIGH);
		}
		if (row == 16){
			row = 0;
		}

		//let's load out bytes

		int i = 0;
		int color = 0;

		byte tempSPI[72];
		uint16_t curbyte = 0;
		//red

		tempSPI[curbyte++] = pixels[16*row + 0][color];
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 2][color];
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 4][color];
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 6][color];
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 8][color];
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 10][color];
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 12][color];
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 14][color];
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] << 4) & 0xF0);

		//blue
		color = 2;
		tempSPI[curbyte++] = pixels[16*row + 0][color];
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 2][color];
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 4][color];
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 6][color];
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 8][color];
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 10][color];
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 12][color];
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 14][color];
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] << 4) & 0xF0);

		//green
		color = 1;
		tempSPI[curbyte++] = pixels[16*row + 0][color];
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 1][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 2][color];
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 3][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 4][color];
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 5][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 6][color];
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 7][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 8][color];
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 9][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 10][color];
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 11][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 12][color];
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 13][color] << 4) & 0xF0);

		tempSPI[curbyte++] = pixels[16*row + 14][color];
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] >> 4) & 0x0F);
		tempSPI[curbyte++] = ((pixels[16*row + 15][color] << 4) & 0xF0);

		//send it all out
		int cell;
		for (cell=0; cell < 24; cell++) {
			SPI.transfer(4, tempSPI[cell], SPI_CONTINUE);
			//SPI.transfer(4, 0, SPI_CONTINUE);
		}
		for (cell=24; cell < 48; cell++) {
			SPI.transfer(4, tempSPI[cell], SPI_CONTINUE);
			//SPI.transfer(4, 0, SPI_CONTINUE);
		}
		
		for (cell=48; cell < 71; cell++) {
			SPI.transfer(4, tempSPI[cell], SPI_CONTINUE);
			//SPI.transfer(4, 255, SPI_CONTINUE);
		}
		//stop on the last one
		SPI.transfer(4, tempSPI[71]);

		digitalWrite(BLANK_PIN, HIGH);
		digitalWrite(LATCH_PIN_1, HIGH);
		delayMicroseconds(20);
		digitalWrite(LATCH_PIN_1, LOW);
		digitalWrite(BLANK_PIN, LOW);

		
		if (!blank) {
			if (blinkEnable) {
				if (_blinkEnableBlink) digitalWrite(fetMap[row], LOW);
			} else {
				digitalWrite(fetMap[row], LOW);
			}
		}

		delayMicroseconds(scanrate);

		//set our POV delay
		row++;
	}
	yield();
}

void getFrame() {
	disableBufferSerial = true;

	if (!showInput)  {
		digitalWrite(FET_PIN_1, HIGH);
		digitalWrite(FET_PIN_2, HIGH);
		digitalWrite(FET_PIN_3, HIGH);
		digitalWrite(FET_PIN_4, HIGH);
		digitalWrite(FET_PIN_5, HIGH);
		digitalWrite(FET_PIN_6, HIGH);
		digitalWrite(FET_PIN_7, HIGH);
		digitalWrite(FET_PIN_8, HIGH);
		digitalWrite(FET_PIN_9, HIGH);
		digitalWrite(FET_PIN_10, HIGH);
		digitalWrite(FET_PIN_11, HIGH);
		digitalWrite(FET_PIN_12, HIGH);
		digitalWrite(FET_PIN_13, HIGH);
		digitalWrite(FET_PIN_14, HIGH);
		digitalWrite(FET_PIN_15, HIGH);
		digitalWrite(FET_PIN_16, HIGH);
	}

	unsigned int byteCount = 0;
	short int pixBytes = 0;
	bool loopa = true;
	while (loopa && byteCount < 257) {
		if (Serial.available() > 0) {
			inByte = Serial.read();
			byteCount++;
			if (inByte == B_END_INPUT) {
				if (byteCount >= 256) {
					Serial.write(B_INPUT_ACK);
				} else {
					Serial.write(B_INPUT_NAK);
				}
				loopa = false;
			} else if (inByte == B_SET_INPUT) {
				getFrame(); //retrigger capture
				loopa = false;
			} else if (inByte >= 128) {
				loopa = false;
				Serial.write(B_INPUT_NAK);
			} else {
				//color input
				if (pixBytes >= 255) pixBytes = 255;

				//DO PROCESSING
				pixels[pixBytes][0] = (inByte & 0x70) << 1;
				pixels[pixBytes][1] = (inByte & 0x0C) << 4;
				pixels[pixBytes][2] = (inByte & 0x02) << 6;
				pixBytes++;
			}
		}
		if(showInput) yield();
	}
	disableBufferSerial = false;
	yield();
}

void bufferSerial() {
	if(!disableBufferSerial)
		if(Serial.available() > 0) {
			// get incoming byte:
			inByte = Serial.read();
			// read first analog input, divide by 4 to make the range 0-255:
			if (inByte==B_GET_ID) {
				Serial.print(DEVICE_ID);
				blinkEnable = false;
				selfTestProgram();
			} else if (inByte==B_SET_INPUT) {
				getFrame();
			} else if (inByte==B_ACK) {
				Serial.write(B_ACK);
			} else if (inByte==B_SET_PROG_RATE) {
				while (Serial.available() == 0);
				scanrate = 150*Serial.read()/16;
				Serial.write(B_ACK);
			} else if (inByte==B_SET_SHOW_INPUT) {
				memcpy(pixels_old, pixels, sizeof(pixels));
				setBusy();
				while (Serial.available() == 0);
				showInput = (Serial.read() > 64);
				Serial.write(B_ACK);
				delay(500);
				memcpy(pixels, pixels_old, sizeof(pixels));
			} else if (inByte==B_SET_BLANK) {
				blank = true;
				Serial.write(B_ACK);
			} else if (inByte==B_SET_UNBLANK) {
				blank = false;
				Serial.write(B_ACK);
			}
		}
	yield();
}

void setBusy() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[i][c] = 0;
		}
	}


	int x;
	int y;
	for (x=6; x<10; x++ ) {
		for (y=0; y< 10; y++) {
			pixels[16*y + x][0] = 255;
			pixels[16*y + x][1] = 255;
		}
	}

	for (x=6; x<10; x++ ) {
		for (y=12; y< 15; y++) {
			pixels[16*y + x][0] = 255;
			pixels[16*y + x][1] = 255;
		}
	}
}

void setNotOK() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[i][c] = 0;
		}
	}

	for (i=0; i< 16; i++ ) {
		pixels[16*i + i][0] = 255;
		pixels[16*i + (15-i)][0] = 255;
	}

	for (i=0; i< 14; i++ ) {
		pixels[16*i + i+1][0] = 255;
		pixels[16*i + (14-i)][0] = 255;
	}

	for (i=1; i< 15; i++ ) {
		pixels[16*i + i-1][0] = 255;
		pixels[16*i + (16-i)][0] = 255;
	}

	pixels[16*14 + 0][0] = 255;
	pixels[16*14 + 15][0] = 255;
	pixels[16*15 + 1][0] = 255;
	pixels[16*15 + 14][0] = 255;
}

void setOK() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[i][c] = 0;
		}
	}

	int x;
	for (x=0; x<8; x++) {
		pixels[16*(15-x) + (7-x)][1] =255;
		pixels[16*(14-x) + (7-x)][1] =255;
	}
	for (x=0; x<15; x++) {
		pixels[16*(15-x) + x/2+7][1] =255;
		pixels[16*(14-x) + x/2+7][1] =255;
	}
}

void handleBlinking() {
	if (blinkEnable) {
		_blinkEnableBlink = !_blinkEnableBlink;
		delay(BLINK_MILLIS);
	} else {
		yield();
	}
}

void handleStatusLEDS() {
	digitalWrite(STAT_LED1_PIN , status_led_stat);
	status_led_stat = !status_led_stat;
	delay(500);
}

void setup(){
	fetMap[15] = FET_PIN_16;
	fetMap[0] = FET_PIN_1;
	fetMap[1] = FET_PIN_2;
	fetMap[2] = FET_PIN_3;
	fetMap[3] = FET_PIN_4;
	fetMap[4] = FET_PIN_5;
	fetMap[5] = FET_PIN_6;
	fetMap[6] = FET_PIN_7;
	fetMap[7] = FET_PIN_8;
	fetMap[8] = FET_PIN_9;
	fetMap[9] = FET_PIN_10;
	fetMap[10] = FET_PIN_11;
	fetMap[11] = FET_PIN_12;
	fetMap[12] = FET_PIN_13;
	fetMap[13] = FET_PIN_14;
	fetMap[14] = FET_PIN_15;

	pinMode(FET_PIN_1, OUTPUT);
	pinMode(FET_PIN_2, OUTPUT);
	pinMode(FET_PIN_3, OUTPUT);
	pinMode(FET_PIN_4, OUTPUT);
	pinMode(FET_PIN_5, OUTPUT);
	pinMode(FET_PIN_6, OUTPUT);
	pinMode(FET_PIN_7, OUTPUT);
	pinMode(FET_PIN_8, OUTPUT);
	pinMode(FET_PIN_9, OUTPUT);
	pinMode(FET_PIN_10, OUTPUT);
	pinMode(FET_PIN_11, OUTPUT);
	pinMode(FET_PIN_12, OUTPUT);
	pinMode(FET_PIN_13, OUTPUT);
	pinMode(FET_PIN_14, OUTPUT);
	pinMode(FET_PIN_15, OUTPUT);
	pinMode(FET_PIN_16, OUTPUT);

	pinMode(DATA_PIN, OUTPUT);
	pinMode(SCK_PIN, OUTPUT);
	pinMode(LATCH_PIN_1, OUTPUT);
	pinMode(LATCH_PIN_2, OUTPUT);
	pinMode(LATCH_PIN_3, OUTPUT);

	pinMode(GSCLK_PIN, OUTPUT);
	pinMode(BLANK_PIN, OUTPUT);
	pinMode(STAT_LED1_PIN, OUTPUT);
	pinMode(DCPRG_PIN, OUTPUT);
	digitalWrite(DCPRG_PIN, LOW);
	pinMode(VPRG_PIN, OUTPUT);
	digitalWrite(VPRG_PIN, LOW);

	digitalWrite(BLANK_PIN, LOW);

	startTimer(TC1, 0, TC3_IRQn, 300000);

	SPI.begin(4);

	digitalWrite(FET_PIN_1, HIGH);
	digitalWrite(FET_PIN_2, HIGH);
	digitalWrite(FET_PIN_3, HIGH);
	digitalWrite(FET_PIN_4, HIGH);
	digitalWrite(FET_PIN_5, HIGH);
	digitalWrite(FET_PIN_6, HIGH);
	digitalWrite(FET_PIN_7, HIGH);
	digitalWrite(FET_PIN_8, HIGH);
	digitalWrite(FET_PIN_9, HIGH);
	digitalWrite(FET_PIN_10, HIGH);
	digitalWrite(FET_PIN_11, HIGH);
	digitalWrite(FET_PIN_12, HIGH);
	digitalWrite(FET_PIN_13, HIGH);
	digitalWrite(FET_PIN_14, HIGH);
	digitalWrite(FET_PIN_15, HIGH);
	digitalWrite(FET_PIN_16, HIGH);

	// start serial port at 9600 bps:
	Serial.begin(115200);
	while (!Serial) {
		;
	}
	Serial.write(B_ACK);

	Scheduler.startLoop(updateOutput);
	Scheduler.startLoop(bufferSerial);
	Scheduler.startLoop(handleBlinking);
	Scheduler.startLoop(handleStatusLEDS);

	setNotOK();
}

void selfTestProgram() {

	//snag the memory
	if (selfTest) memcpy(pixels_old, pixels, sizeof(pixels));

	setBusy();
	delay(700);

	overrideOUTPUT = true;

	int cell;
	for (cell=0; cell < 24; cell++) {
		SPI.transfer(4, 255, SPI_CONTINUE);
	}
	for (cell=24; cell < 48; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}

	for (cell=48; cell < 72; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}

	digitalWrite(BLANK_PIN, HIGH);
	digitalWrite(LATCH_PIN_1, HIGH);
	delayMicroseconds(20);
	digitalWrite(LATCH_PIN_1, LOW);
	digitalWrite(BLANK_PIN, LOW);

	digitalWrite(FET_PIN_1, LOW);
	digitalWrite(FET_PIN_2, LOW);
	digitalWrite(FET_PIN_3, LOW);
	digitalWrite(FET_PIN_4, LOW);
	digitalWrite(FET_PIN_5, LOW);
	digitalWrite(FET_PIN_6, LOW);
	digitalWrite(FET_PIN_7, LOW);
	digitalWrite(FET_PIN_8, LOW);
	digitalWrite(FET_PIN_9, LOW);
	digitalWrite(FET_PIN_10, LOW);
	digitalWrite(FET_PIN_11, LOW);
	digitalWrite(FET_PIN_12, LOW);
	digitalWrite(FET_PIN_13, LOW);
	digitalWrite(FET_PIN_14, LOW);
	digitalWrite(FET_PIN_15, LOW);
	digitalWrite(FET_PIN_16, LOW);

	delay(1000);

	for (cell=0; cell < 24; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}
	for (cell=24; cell < 48; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}

	for (cell=48; cell < 72; cell++) {
		SPI.transfer(4, 255, SPI_CONTINUE);
	}

	digitalWrite(BLANK_PIN, HIGH);
	digitalWrite(LATCH_PIN_1, HIGH);
	delayMicroseconds(20);
	digitalWrite(LATCH_PIN_1, LOW);
	digitalWrite(BLANK_PIN, LOW);

	digitalWrite(FET_PIN_1, LOW);
	digitalWrite(FET_PIN_2, LOW);
	digitalWrite(FET_PIN_3, LOW);
	digitalWrite(FET_PIN_4, LOW);
	digitalWrite(FET_PIN_5, LOW);
	digitalWrite(FET_PIN_6, LOW);
	digitalWrite(FET_PIN_7, LOW);
	digitalWrite(FET_PIN_8, LOW);
	digitalWrite(FET_PIN_9, LOW);
	digitalWrite(FET_PIN_10, LOW);
	digitalWrite(FET_PIN_11, LOW);
	digitalWrite(FET_PIN_12, LOW);
	digitalWrite(FET_PIN_13, LOW);
	digitalWrite(FET_PIN_14, LOW);
	digitalWrite(FET_PIN_15, LOW);
	digitalWrite(FET_PIN_16, LOW);

	delay(1000);

	for (cell=0; cell < 24; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}
	for (cell=24; cell < 48; cell++) {
		SPI.transfer(4, 255, SPI_CONTINUE);
	}

	for (cell=48; cell < 72; cell++) {
		SPI.transfer(4, 0, SPI_CONTINUE);
	}

	digitalWrite(BLANK_PIN, HIGH);
	digitalWrite(LATCH_PIN_1, HIGH);
	delayMicroseconds(20);
	digitalWrite(LATCH_PIN_1, LOW);
	digitalWrite(BLANK_PIN, LOW);

	digitalWrite(FET_PIN_1, LOW);
	digitalWrite(FET_PIN_2, LOW);
	digitalWrite(FET_PIN_3, LOW);
	digitalWrite(FET_PIN_4, LOW);
	digitalWrite(FET_PIN_5, LOW);
	digitalWrite(FET_PIN_6, LOW);
	digitalWrite(FET_PIN_7, LOW);
	digitalWrite(FET_PIN_8, LOW);
	digitalWrite(FET_PIN_9, LOW);
	digitalWrite(FET_PIN_10, LOW);
	digitalWrite(FET_PIN_11, LOW);
	digitalWrite(FET_PIN_12, LOW);
	digitalWrite(FET_PIN_13, LOW);
	digitalWrite(FET_PIN_14, LOW);
	digitalWrite(FET_PIN_15, LOW);
	digitalWrite(FET_PIN_16, LOW);

	delay(1000);

	digitalWrite(FET_PIN_1, HIGH);
	digitalWrite(FET_PIN_2, HIGH);
	digitalWrite(FET_PIN_3, HIGH);
	digitalWrite(FET_PIN_4, HIGH);
	digitalWrite(FET_PIN_5, HIGH);
	digitalWrite(FET_PIN_6, HIGH);
	digitalWrite(FET_PIN_7, HIGH);
	digitalWrite(FET_PIN_8, HIGH);
	digitalWrite(FET_PIN_9, HIGH);
	digitalWrite(FET_PIN_10, HIGH);
	digitalWrite(FET_PIN_11, HIGH);
	digitalWrite(FET_PIN_12, HIGH);
	digitalWrite(FET_PIN_13, HIGH);
	digitalWrite(FET_PIN_14, HIGH);
	digitalWrite(FET_PIN_15, HIGH);
	digitalWrite(FET_PIN_16, HIGH);

	overrideOUTPUT = false;

	if (!selfTest) setOK();
	else {
		setOK();
		delay(1000);
		memcpy(pixels, pixels_old, sizeof(pixels));
	}

	selfTest = true;
	yield();
}

void loop()
{
	bufferSerial();
	yield();
}

