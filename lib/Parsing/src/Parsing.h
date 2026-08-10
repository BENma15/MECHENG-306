#ifndef PARSE_H
#define PARSE_H

#include <Arduino.h>
#include <GcodeToken.h>

#define G 0
#define M 0
#define X 1
#define Y 2
#define F 3 //fix: #define F 3 collides with Arduino's F() macro used for storing strings

/* Functions */
void initialiseTokenArray(void);    // to be called in setup() in .ino
int readLine(void); // returns 0 if function succesfully reads a valid token/s, returns 1 if not
int tokenise(String Line);  // return 1 if token invalid. error messages printed by function.
String returnToken(String line, char Letter);
String tidyString(String line);
GcodeToken Parsing_getToken(int index);
void resetTokenArray();

#endif