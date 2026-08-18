#include "FSM.h"
#include <Encoder.h>
#include <HelperFunctions.h>
#include <Homing.h>
#include <Parsing.h>
#include <GcodeToken.h>
#include <LimitSwitch.h>
#include <PID_FSM.h>
#include <Motor.h>
static SystemState currentState = STATE_IDLE;   // System begins in IDLE state
// static MotionSubstate motionState = MOTION_ACCEL;   // Motion_state begins in Acceloration

static SystemState lastPrintedState = (SystemState)-1;


void FSM_init() {

    currentState = STATE_IDLE;

}


static void handleIdle() {  // Idle --> Parsing. Event: Serial buffer recieves input

    if (Serial.available() > 0) {
        currentState = STATE_PARSING;
    }

}


static void handleParsing() {
    int err = readLine();
    if (err == 2) {
        // stay in Parsing
        return;
    }

    // check if command was invalid
    if (err == 1) {
        resetTokenArray();
        currentState = STATE_IDLE;
        return;
    }

    int value = (Parsing_getToken(G)).GetValue();   // returns value of G token

    if (value == 28) {
        currentState = STATE_HOMING;
        homingStart();
        return;
    }

    //bounds checking needs to be implemented after homing and pid are finished

    // check if G01 command will hit a limit switch
    /* ASSUMING WE CHANGE 0,0 TO BOTTOM LEFT!!! */ // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<**************
    long left = Encoder_getLeftEncoderCount() ;
    long right = Encoder_getRightEncoderCount();
    long Xpos = (left + right) / 2;
    long Ypos = (left - right) / 2;

    Xpos += distanceToCounts(Parsing_getToken(X).GetValue());
    Ypos += distanceToCounts(Parsing_getToken(Y).GetValue());

    // if (Xpos >= X_MAX || Xpos <= X_MIN || Ypos >= Y_MAX || Ypos <= Y_MIN) {
    //     resetTokenArray();
    //     currentState = STATE_IDLE;  // command would take 
    //     return;
    // } else {
    //     currentState = STATE_MOTION;
    //     return;
    // }
    currentState = STATE_MOTION;
    return;

}


static void handleMotion() {
    long x = Parsing_getToken(X).GetValue();
    long y = Parsing_getToken(Y).GetValue();
    long vf = Parsing_getToken(F_token).GetValue();

    move_FSM(x, y, vf);

    if(moveActive == false) {
        currentState = STATE_IDLE;
        moveStarted = false;
        resetTokenArray();
        return;
    }
}


static void handleHoming() {
    HomingResult result = homingUpdate();

    if (result == HOMING_FAULT)
    {
        currentState = STATE_FAULT;
        return;
    }

    if (result == HOMING_COMPLETE)
    {
        currentState = STATE_IDLE;
        moveActive = false;  // Reset moveActive to false after homing is complete
        moveStarted = false; // Reset moveStarted to false after homing is complete
        resetTokenArray();
        return;
    }
}


static void handleFault() {
    setLeftMotor(0,0);
    setRightMotor(0,0);
    
    if (Serial.available() > 0) {

        int err = readLine();

        if (err) {
            return;
        }

        GcodeToken token = Parsing_getToken(M);
        if (token.GetLetter() == 'M' && token.GetValue() == 999) {
            resetTokenArray();
            currentState = STATE_IDLE;
            return;
        }
    }
}

static void printStateIfChanged()
{
    if (currentState == lastPrintedState)
    {
        return;
    }


    lastPrintedState = currentState;


    switch (currentState)
    {
        case STATE_IDLE:
            Serial.println("IDLE");
            break;

        case STATE_PARSING:
            Serial.println("PARSING");
            break;

        case STATE_MOTION:
            Serial.println("MOTION");
            break;

        case STATE_HOMING:
            Serial.println("HOMING");
            break;

        case STATE_FAULT:
            Serial.println("FAULT");
            break;
    }
}

void FSM_update() {
    // Call the handler for the current state (non-blocking)
    switch(currentState) {
        case STATE_IDLE:     handleIdle();         break;
        case STATE_PARSING:  handleParsing();      break;
        case STATE_MOTION:   handleMotion();       break;
        case STATE_HOMING:   handleHoming();       break;
        case STATE_FAULT:    handleFault();        break;
    }

    printStateIfChanged();

}

SystemState FSM_getCurrentState() {
    return currentState;
}

void FSM_triggerFault() {
    currentState = STATE_FAULT;
}
