#ifndef PARSE_H
#define PARSE_H

#include <Arduino.h>
#include "GcodeToken.h"

#define G 0
#define M 0
#define X 1
#define Y 2
#define F 3

/* global variables */
extern String inputBuffer;
extern GcodeToken TokenArray[MAX_TOKENS];  // index 0 = G/M, 1 = X, 2 = Y, 3 = F


/* Functions */
void initialiseTokenArray(void);    // to be called in setup() in .ino
void readLine(void);
void tokenise(String Line);
void resetArray(void);
String returnToken(String line, char Letter);
String tidyString(String line);





#endif