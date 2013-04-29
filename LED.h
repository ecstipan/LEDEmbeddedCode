//
//  LED.h
//  
//
//  Created by Rayce Stipanovich on 4/25/13.
//
//

#ifndef ____LED__
#define ____LED__

#include <Arduino.h>
#include <Scheduler.h>

void setStat(short int stat);
void updateOutput();
void getFrame();
void setBusy();
void setNotOK();
void setOK();
void setup();
void selfTestProgram();
void loop();


#endif /* defined(____LED__) */
