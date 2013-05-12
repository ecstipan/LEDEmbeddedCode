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
#define B_DUMP 65

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

#define RED_MIN 0
#define GREEN_MIN 0
#define BLUE_MIN 0
#define RED_MAX 220
#define GREEN_MAX 220
#define BLUE_MAX 220





//other deffinitions
#define POV_PERIOD 125 //progressive frame refresh rate in ms
#define STAT_FAIL 0
#define STAT_OK 1
#define STAT_BUSY 2
#define STAT_RUN 3
#define BLINK_MILLIS 300

int inByte = 0;         // incoming serial byte

unsigned int scanrate = 80*POV_PERIOD / 16;

byte pixels[8][256][3];
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
volatile uint32_t enableGSCLK = 0;

short int _status = STAT_FAIL;

bool status_led_stat = true;

byte tempSPI[72];

unsigned short int frameNumber = 0;
unsigned short int frameNumberOld = 0;


unsigned int map_color_red[128];
unsigned int map_color_green[128];
unsigned int map_color_blue[128];

void populateTestGradient(short int color) {
	blinkEnable = false;
	int row;
	int fill = 1;

	for (row=0; row < 16; row++) {
		int col;
		for (col=0; col<16; col++) {
			pixels[0][16*row + col][0] = 0;
			pixels[0][16*row + col][1] = 0;
			pixels[0][16*row + col][2] = 0;
		}
	}

	for (row=0; row < 16; row++) {
		int col;
		fill = 0;
		for (col=0; col<16; col++) {
			pixels[0][16*row + col][color] = constrain(fill - 1, 0, 255);
			fill+=16;
		}
	}
	
}

void setupColorMap() {
	map_color_red[0] = 0;
	map_color_red[1] = 2;
	map_color_red[2] = 4;
	map_color_red[3] = 9;
	map_color_red[4] = 12;
	map_color_red[5] = 14;
	map_color_red[6] = 15;
	map_color_red[7] = 16;//
	map_color_red[8] = 16;
	map_color_red[9] = 17;
	map_color_red[10] = 17;
	map_color_red[11] = 18;
	map_color_red[12] = 18;
	map_color_red[13] = 19;
	map_color_red[14] = 19;
	map_color_red[15] = 20;//
	map_color_red[16] = 20;
	map_color_red[17] = 21;
	map_color_red[18] = 21;
	map_color_red[19] = 22;
	map_color_red[20] = 22;
	map_color_red[21] = 23;
	map_color_red[22] = 23;
	map_color_red[23] = 24;//
	map_color_red[24] = 24;
	map_color_red[25] = 24;
	map_color_red[26] = 25;
	map_color_red[27] = 25;
	map_color_red[28] = 26;
	map_color_red[29] = 26;
	map_color_red[30] = 27;
	map_color_red[31] = 28;//
	map_color_red[32] = 28;
	map_color_red[33] = 29;
	map_color_red[34] = 30;
	map_color_red[35] = 31;
	map_color_red[36] = 32;
	map_color_red[37] = 32;
	map_color_red[38] = 33;
	map_color_red[39] = 34;//
	map_color_red[40] = 34;
	map_color_red[41] = 35;
	map_color_red[42] = 35;
	map_color_red[43] = 36;
	map_color_red[44] = 36;
	map_color_red[45] = 37;
	map_color_red[46] = 37;
	map_color_red[47] = 38;//
	map_color_red[48] = 38;
	map_color_red[49] = 39;
	map_color_red[50] = 39;
	map_color_red[51] = 40;
	map_color_red[52] = 40;
	map_color_red[53] = 41;
	map_color_red[54] = 41;
	map_color_red[55] = 42;//
	map_color_red[56] = 42;
	map_color_red[57] = 43;
	map_color_red[58] = 44;
	map_color_red[59] = 45;
	map_color_red[60] = 45;
	map_color_red[61] = 45;
	map_color_red[62] = 44;
	map_color_red[63] = 45;//
	map_color_red[64] = 47;
	map_color_red[65] = 49;
	map_color_red[66] = 51;
	map_color_red[67] = 53;
	map_color_red[68] = 55;
	map_color_red[69] = 57;
	map_color_red[70] = 60;
	map_color_red[71] = 62;
	map_color_red[72] = 64;
	map_color_red[73] = 66;
	map_color_red[74] = 68;
	map_color_red[75] = 70;
	map_color_red[76] = 72;
	map_color_red[77] = 73;
	map_color_red[78] = 75;
	map_color_red[79] = 76;
	map_color_red[80] = 78;
	map_color_red[81] = 79;
	map_color_red[82] = 80;
	map_color_red[83] = 81;
	map_color_red[84] = 83;
	map_color_red[85] = 84;
	map_color_red[86] = 85;
	map_color_red[87] = 87;
	map_color_red[88] = 88;
	map_color_red[89] = 90;
	map_color_red[90] = 92;
	map_color_red[91] = 94;
	map_color_red[92] = 95;
	map_color_red[93] = 96;
	map_color_red[94] = 98;
	map_color_red[95] = 100;
	map_color_red[96] = 110;
	map_color_red[97] = 125;
	map_color_red[98] = 140;
	map_color_red[99] = 150;
	map_color_red[100] = 150;
	map_color_red[101] = 150;
	map_color_red[102] = 150;
	map_color_red[103] = 150;
	map_color_red[104] = 170;
	map_color_red[105] = 180;
	map_color_red[106] = 200;
	map_color_red[107] = 230;
	map_color_red[108] = 260;
	map_color_red[109] = 300;
	map_color_red[110] = 300;
	map_color_red[111] = 300;
	map_color_red[112] = 400;
	map_color_red[113] = 500;
	map_color_red[114] = 600;
	map_color_red[115] = 700;
	map_color_red[116] = 850;
	map_color_red[117] = 1000;
	map_color_red[118] = 1500;
	map_color_red[119] = 2000;
	map_color_red[120] = 2262;
	map_color_red[121] = 2524;
	map_color_red[122] = 2786;
	map_color_red[123] = 3048;
	map_color_red[124] = 3310;
	map_color_red[125] = 3572;
	map_color_red[126] = 3834;
	map_color_red[127] = 4095;
}

int rMap(byte input) {
	return map_color_red[(input & 0xFE) >> 1];
}

int gMap(byte input) {
	return map_color_red[(input & 0xFE) >> 1];
}

int bMap(byte input) {
	return map_color_red[(input & 0xFE) >> 1];
}

void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
	if (enableGSCLK < 4094) {
		PIOC->PIO_SODR = g_APinDescription[GSCLK_PIN].ulPin;
		PIOC->PIO_CODR = g_APinDescription[GSCLK_PIN].ulPin;
		enableGSCLK++;
	}
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

void SPI_Transfer(int a, int b) {
	SPI.transfer(4, byte((a>>4)&0xFF), SPI_CONTINUE);
	SPI.transfer(4, byte((a)&0xF0) | (byte((b >> 8) & 0x0F)), SPI_CONTINUE);
	SPI.transfer(4, byte(b&0xF0));
}

byte test;

void updateOutput() {
  if (!overrideOUTPUT) {
    row++;
    if (row == 16){
      row = 0;
    }
    
    unsigned short int nextrow;
    if (row < 15) nextrow = row + 1;
    else nextrow = 0;
    
    //disable output MOSFETS
    short tr = 0;
    for (tr = 0; tr < 16; tr++) {
		digitalWrite(fetMap[tr], HIGH);
    }
    

    //latch our current line
    
    digitalWrite(LATCH_PIN_1, HIGH);
    delayMicroseconds(15);
    digitalWrite(LATCH_PIN_1, LOW);
    
    digitalWrite(BLANK_PIN, HIGH);
    digitalWrite(BLANK_PIN, LOW);

	  enableGSCLK = 0;
    
    //latch our current output MOSFETS
    if (!blank) {
      if (blinkEnable) {
        if (_blinkEnableBlink) digitalWrite(fetMap[row], LOW);
      } else {
        digitalWrite(fetMap[row], LOW);
      }
    }
	  digitalWrite(fetMap[row], LOW);

    //shift in our next line
	int color = 0;

	uint16_t curbyte = 0;
		//red

	  //tempSPI[curbyte++] = pixels[16*nextrow + 0][color];

	  SPI_Transfer(rMap(pixels[0][16*nextrow + 0][0]),
				   rMap(pixels[0][16*nextrow + 1][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 2][0]),
				   rMap(pixels[0][16*nextrow + 3][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 4][0]),
				   rMap(pixels[0][16*nextrow + 5][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 6][0]),
				   rMap(pixels[0][16*nextrow + 7][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 8][0]),
				   rMap(pixels[0][16*nextrow + 9][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 10][0]),
				   rMap(pixels[0][16*nextrow + 11][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 12][0]),
				   rMap(pixels[0][16*nextrow + 13][0]));
	  SPI_Transfer(rMap(pixels[0][16*nextrow + 14][0]),
				   rMap(pixels[0][16*nextrow + 15][0]));

	  SPI_Transfer(bMap(pixels[0][16*nextrow + 0][2]),
				   bMap(pixels[0][16*nextrow + 1][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 2][2]),
				   bMap(pixels[0][16*nextrow + 3][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 4][2]),
				   bMap(pixels[0][16*nextrow + 5][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 6][2]),
				   bMap(pixels[0][16*nextrow + 7][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 8][2]),
				   bMap(pixels[0][16*nextrow + 9][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 10][2]),
				   bMap(pixels[0][16*nextrow + 11][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 12][2]),
				   bMap(pixels[0][16*nextrow + 13][2]));
	  SPI_Transfer(bMap(pixels[0][16*nextrow + 14][2]),
				   bMap(pixels[0][16*nextrow + 15][2]));

	  SPI_Transfer(gMap(pixels[0][16*nextrow + 0][1]),
				   gMap(pixels[0][16*nextrow + 1][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 2][1]),
				   gMap(pixels[0][16*nextrow + 3][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 4][1]),
				   gMap(pixels[0][16*nextrow + 5][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 6][1]),
				   gMap(pixels[0][16*nextrow + 7][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 8][1]),
				   gMap(pixels[0][16*nextrow + 9][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 10][1]),
				   gMap(pixels[0][16*nextrow + 11][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 12][1]),
				   gMap(pixels[0][16*nextrow + 13][1]));
	  SPI_Transfer(gMap(pixels[0][16*nextrow + 14][1]),
				   gMap(pixels[0][16*nextrow + 15][1]));



	  //if (test==255) test = 0;
	  //test++;

	
    //delay our time
    delayMicroseconds(scanrate);
  }
  yield();
}


void dumpMemory() {
  int i;
  for (i=0; i < 256; i++ ) {
    Serial.print("Pixel #");
    Serial.print(i);
    Serial.print(" (");
    Serial.print(pixels[0][i][0], DEC);
    Serial.print(", ");
    Serial.print(pixels[0][i][1], DEC);
    Serial.print(", ");
    Serial.print(pixels[0][i][2], DEC);
    Serial.print(")");
  }
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
	short int color = 0;
	while (loopa && byteCount < 767) {
		if (Serial.available() > 0) {
			inByte = Serial.read();
			byteCount++;
			if (inByte == B_END_INPUT) {
				if (byteCount >= 768) {
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
				pixels[0][pixBytes][color++] = inByte << 1;

				if (color==3) {
					color = 0;
					pixBytes++;
				}
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
				scanrate = 80*Serial.read()/16;
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
			} else if (inByte==B_DUMP) {
                          dumpMemory();
                        }
		}
	yield();
}

void setBusy() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[0][i][c] = 0;
		}
	}


	int x;
	int y;
	for (x=6; x<10; x++ ) {
		for (y=0; y< 10; y++) {
			pixels[0][16*y + x][0] = 255;
			pixels[0][16*y + x][1] = 255;
		}
	}

	for (x=6; x<10; x++ ) {
		for (y=12; y< 15; y++) {
			pixels[0][16*y + x][0] = 255;
			pixels[0][16*y + x][1] = 255;
		}
	}
}

void setNotOK() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[0][i][c] = 0;
		}
	}

	for (i=0; i< 16; i++ ) {
		pixels[0][16*i + i][0] = 255;
		pixels[0][16*i + (15-i)][0] = 255;
	}

	for (i=0; i< 14; i++ ) {
		pixels[0][16*i + i+1][0] = 255;
		pixels[0][16*i + (14-i)][0] = 255;
	}

	for (i=1; i< 15; i++ ) {
		pixels[0][16*i + i-1][0] = 255;
		pixels[0][16*i + (16-i)][0] = 255;
	}

	pixels[0][16*14 + 0][0] = 255;
	pixels[0][16*14 + 15][0] = 255;
	pixels[0][16*15 + 1][0] = 255;
	pixels[0][16*15 + 14][0] = 255;
}

void setOK() {
	int i;
	int c;
	for (i=0; i < 256; i++){
		for (c=0; c<3; c++){
			pixels[0][i][c] = 0;
		}
	}

	int x;
	for (x=0; x<8; x++) {
		pixels[0][16*(15-x) + (7-x)][1] =255;
		pixels[0][16*(14-x) + (7-x)][1] =255;
	}
	for (x=0; x<15; x++) {
		pixels[0][16*(15-x) + x/2+7][1] =255;
		pixels[0][16*(14-x) + x/2+7][1] =255;
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

	setupColorMap();

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

	//delay(1000);
	//populateTestGradient(2);
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
        enableGSCLK = 0;

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
        enableGSCLK = 0;

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
        enableGSCLK = 0;

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

