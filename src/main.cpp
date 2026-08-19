/* Tmporary Headers for while we are testing */
/* To be removed in the end */
#include <HelperFunctions.h>
// #include <HomingDiagonal.h>
// #include <LimitSwitch.h>    
// #include <PID.cpp>
// #include <MotorMove.h>
#include <LimitSwitch.h>
#include <Motor.h>
#include <Parsing.h>
#include <Encoder.h>
#include <PID_FSM.h>
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
    FSM_init();
    setupLimitSwitches();
    Motor_Init();
    initialiseTokenArray();
    Serial.println(countA);
    Serial.println(countB);
    

    // MotorMoveConfig config = {4, 5, 6, 7, 255, 1000};
    // motorMoveBegin(config);

    // ^^^^^ (not to be kept in final version)


}


void loop() {
    FSM_update();
    //setLeftMotor(1, 255);
    //setRightMotor(1, 255);
}

//1 is cw, 0 is stop, -1 is ccw