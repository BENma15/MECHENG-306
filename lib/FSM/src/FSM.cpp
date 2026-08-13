#include "FSM.h"
#include <Encoder.h>
#include <HelperFunctions.h>
#include <Homing.h>
#include <Parsing.h>
#include <GcodeToken.h>
#include <LimitSwitchDebounce.h>

static SystemState currentState = STATE_IDLE;   // System begins in IDLE state
// static MotionSubstate motionState = MOTION_ACCEL;   // Motion_state begins in Acceloration


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

    // check if command was invalid
    if (err) {
        currentState = STATE_IDLE;
        return;
    }

    int value = (Parsing_getToken(G)).GetValue();   // returns value of G token

    if (value == 28) {
        currentState = STATE_HOMING;
        homingStart();
        return;
    }

    // check if G01 command will hit a limit switch
    /* ASSUMING WE CHANGE 0,0 TO BOTTOM LEFT!!! */ // <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<**************
    long left = Encoder_getLeftEncoderCount() ;
    long right = Encoder_getRightEncoderCount();
    long Xpos = (left + right) / 2;
    long Ypos = (left - right) / 2;

    Xpos += distanceToCounts(Parsing_getToken(X).GetValue());
    Ypos += distanceToCounts(Parsing_getToken(Y).GetValue());

    if (Xpos >= X_MAX || Xpos <= X_MIN || Ypos >= Y_MAX || Ypos <= Y_MIN) {
        resetTokenArray();
        currentState = STATE_IDLE;  // command would take 
        return;
    } else {
        currentState = STATE_MOTION;
        return;
    }

}


static void handleMotion() {

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
        return;
    }
}


static void handleFault() {
    
    if (Serial.available() > 0) {

        int err = readLine();

        if (err) {
            return;
        }

        GcodeToken token = Parsing_getToken(M);
        if (token.GetLetter() == 'M' && token.GetValue() == 999) {
            currentState = STATE_IDLE;
        }
    }
}


void FSM_update() {

    switch(currentState) {
        case STATE_IDLE:     handleIdle();         Serial.println("IDLE");           break;
        case STATE_PARSING:  handleParsing();      Serial.println("PARSING");        break;
        case STATE_MOTION:   handleMotion();       Serial.println("MOTION");         break;
        case STATE_HOMING:   handleHoming();       Serial.println("HOMING");         break;
        case STATE_FAULT:    handleFault();        Serial.println("FAULT");          break;
    }


}

SystemState FSM_getCurrentState() {
    return currentState;
}

void FSM_triggerFault() {
    currentState = STATE_FAULT;
}
