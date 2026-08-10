/* Tmporary Headers for while we are testing */
/* To be removed in the end */
#include <HelperFunctions.h>
#include <HomingDiagonal.h>
#include <LimitSwitch.h>    
#include <PID.cpp>
// #include <LimitSwitchDebounce.h> should only be used by LimitSwitch.h so is not included
#include <Motion.h>
#include <Parsing.h>
/* #include <GcodeToken.h> should NOT be used in main.cpp so in not included. 
(can be inlcuded in FSM.h or other modules that need to read/edit token) */
#include <Arduino.h>

/* vvvv DO NOT REMOVE vvvv */
#include <FSM.h>
/* ^^^^ DO NOT REMOVE ^^^^ */


void setup() {
    Serial.begin(115200);   // no other file should contain this line 

    /* insert initialisation functions below  */
    Encoder_Init();
    PID_Init();
    
    // ^^^^^ (not to be kept in final version)


}


void loop() {
    testlLoop();
}