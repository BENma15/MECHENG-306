#include <Arduino.h>
#include <avr/interrupt.h>

// Motor A encoder pins
const int L_ENCA = 18;   // INT3
const int L_ENCB = 19;   // INT2

// Motor B encoder pins
const int R_ENCA = 20;   // INT1
const int R_ENCB = 21;   // INT0

// Limit switch pins
const int L_LIMIT = 10;  // PCINT4 / PB4
const int R_LIMIT = 11;  // PCINT5 / PB5
const int D_LIMIT = 12;  // PCINT6 / PB6
const int U_LIMIT = 13;  // PCINT7 / PB7

// Motor pins
const int E1 = 5;
const int M1 = 4;
const int E2 = 6;
const int M2 = 7;

// Encoder variables
volatile long countA = 0;
volatile long countB = 0;
volatile uint8_t stateA = 0;
volatile uint8_t stateB = 0;

// Distance-to-encoder-count variables
const double COUNTS_PER_REV = 8256.0;
const double WHEEL_RADIUS_MM = 4.0;

// Move this far away after hitting a limit
const double LIMIT_BACKOFF_MM = 10.0;

// Current movement speed
int moveSpeed = 150;

// Limit-switch event values
enum LimitEvent : uint8_t
{
    LIMIT_NONE,
    LIMIT_LEFT,
    LIMIT_RIGHT,
    LIMIT_BOTTOM,
    LIMIT_TOP
};

// Only one switch event is recorded at a time
volatile uint8_t limitEvent = LIMIT_NONE;

// Cancels an encoder-controlled movement if another limit is hit
volatile bool limitAbortRequested = false;

// Previous electrical state of D10-D13
volatile uint8_t previousLimitPort = 0;

// Quadrature encoder lookup table
const int8_t encTable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

// Left encoder reading function
void updateA()
{
    uint8_t a = digitalRead(L_ENCA);
    uint8_t b = digitalRead(L_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateA << 2) | newState;

    countA += encTable[index];
    stateA = newState;
}

// Right encoder reading function
void updateB()
{
    uint8_t a = digitalRead(R_ENCA);
    uint8_t b = digitalRead(R_ENCB);

    uint8_t newState = (a << 1) | b;
    uint8_t index = (stateB << 2) | newState;

    countB += encTable[index];
    stateB = newState;
}

// Left motor movement
void setLeftMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E1, 0);
        return;
    }

    digitalWrite(M1, dir > 0 ? HIGH : LOW);
    analogWrite(E1, constrain(pwm, 0, 255));
}

// Right motor movement
void setRightMotor(int8_t dir, int pwm)
{
    if (dir == 0)
    {
        analogWrite(E2, 0);
        return;
    }

    digitalWrite(M2, dir > 0 ? HIGH : LOW);
    analogWrite(E2, constrain(pwm, 0, 255));
}

void stopLeftMotor()
{
    analogWrite(E1, 0);
}

void stopRightMotor()
{
    analogWrite(E2, 0);
}

void stopAllMotors()
{
    stopLeftMotor();
    stopRightMotor();
}

long distanceToCounts(double distance_mm)
{
    return (long)round(
        COUNTS_PER_REV * distance_mm /
        (2.0 * PI * WHEEL_RADIUS_MM)
    );
}

bool runAxis(double deltaA_mm, double deltaB_mm)
{
    long targetA = distanceToCounts(deltaA_mm);
    long targetB = distanceToCounts(deltaB_mm);

    long startA;
    long startB;

    noInterrupts();
    startA = countA;
    startB = countB;
    limitAbortRequested = false;
    interrupts();

    long absTargetA = labs(targetA);
    long absTargetB = labs(targetB);

    int8_t dirA =
        (targetA > 0) ? 1 :
        (targetA < 0) ? -1 : 0;

    int8_t dirB =
        (targetB > 0) ? 1 :
        (targetB < 0) ? -1 : 0;

    bool doneA = (absTargetA == 0);
    bool doneB = (absTargetB == 0);

    long largerTarget =
        (absTargetA > absTargetB) ? absTargetA : absTargetB;

    int pwmA = moveSpeed;
    int pwmB = moveSpeed;

    if (largerTarget > 0)
    {
        pwmA = (int)round(
            moveSpeed * ((double)absTargetA / largerTarget)
        );

        pwmB = (int)round(
            moveSpeed * ((double)absTargetB / largerTarget)
        );
    }

    if (!doneA)
    {
        setLeftMotor(dirA, pwmA);
    }

    if (!doneB)
    {
        setRightMotor(dirB, pwmB);
    }

    while (!doneA || !doneB)
    {
        // Another limit was hit during this movement
        if (limitAbortRequested)
        {
            stopAllMotors();
            return false;
        }

        long currentA;
        long currentB;

        noInterrupts();
        currentA = countA;
        currentB = countB;
        interrupts();

        if (!doneA &&
            labs(currentA - startA) >= absTargetA)
        {
            setLeftMotor(0, 0);
            doneA = true;
        }

        if (!doneB &&
            labs(currentB - startB) >= absTargetB)
        {
            setRightMotor(0, 0);
            doneB = true;
        }
    }

    stopAllMotors();
    return true;
}

bool moveTo(double dx, double dy)
{
    return runAxis(dx + dy, dx - dy);
}

void setup()
{
    Serial.begin(115200);

    pinMode(L_ENCA, INPUT_PULLUP);
    pinMode(L_ENCB, INPUT_PULLUP);
    pinMode(R_ENCA, INPUT_PULLUP);
    pinMode(R_ENCB, INPUT_PULLUP);

    pinMode(L_LIMIT, INPUT_PULLUP);
    pinMode(R_LIMIT, INPUT_PULLUP);
    pinMode(D_LIMIT, INPUT_PULLUP);
    pinMode(U_LIMIT, INPUT_PULLUP);

    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT);
    pinMode(E1, OUTPUT);
    pinMode(E2, OUTPUT);

    stopAllMotors();

    // Store initial encoder states
    stateA =
        (digitalRead(L_ENCA) << 1) |
        digitalRead(L_ENCB);

    stateB =
        (digitalRead(R_ENCA) << 1) |
        digitalRead(R_ENCB);

    // Store initial limit-switch states
    previousLimitPort = PINB;

    cli();

    // Enable encoder interrupts INT0-INT3
    EIMSK |=
        (1 << INT0) |
        (1 << INT1) |
        (1 << INT2) |
        (1 << INT3);

    // Trigger encoder interrupts on any logical change
    EICRA |=
        (1 << ISC00) |
        (1 << ISC10) |
        (1 << ISC20) |
        (1 << ISC30);

    // Enable D10-D13 as pin-change interrupt sources
    PCMSK0 |=
        (1 << PCINT4) |
        (1 << PCINT5) |
        (1 << PCINT6) |
        (1 << PCINT7);

    // Clear pending PCINT0 interrupt
    PCIFR |= (1 << PCIF0);

    // Enable PCINT0 group
    PCICR |= (1 << PCIE0);

    sei();

    Serial.println("System ready");
}

void loop()
{
    uint8_t event;

    // Copy and clear the event safely
    noInterrupts();
    event = limitEvent;
    limitEvent = LIMIT_NONE;
    interrupts();

    /*
     * Move 10 mm in the opposite direction.
     *
     * The switch release generates a PCINT, but the ISR ignores it
     * because it only acts on HIGH-to-LOW transitions.
     */
    if (event == LIMIT_LEFT)
    {
        Serial.println("LEFT limit switch hit");
        moveTo(+LIMIT_BACKOFF_MM, 0.0);
        Serial.println("LEFT back-off complete");
    }
    else if (event == LIMIT_RIGHT)
    {
        Serial.println("RIGHT limit switch hit");
        moveTo(-LIMIT_BACKOFF_MM, 0.0);
        Serial.println("RIGHT back-off complete");
    }
    else if (event == LIMIT_BOTTOM)
    {
        Serial.println("BOTTOM limit switch hit");
        moveTo(0.0, +LIMIT_BACKOFF_MM);
        Serial.println("BOTTOM back-off complete");
    }
    else if (event == LIMIT_TOP)
    {
        Serial.println("TOP limit switch hit");
        moveTo(0.0, -LIMIT_BACKOFF_MM);
        Serial.println("TOP back-off complete");
    }

    // Temporary test movement: starts only once
    static bool initialMovementStarted = false;

    if (!initialMovementStarted)
    {
        initialMovementStarted = true;

        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);

        analogWrite(E1, 128);
        analogWrite(E2, 128);

        Serial.println("Initial motor movement started");
    }
}

// Encoder interrupt service routines
ISR(INT0_vect)
{
    updateB();
}

ISR(INT1_vect)
{
    updateB();
}

ISR(INT2_vect)
{
    updateA();
}

ISR(INT3_vect)
{
    updateA();
}

// Limit-switch pin-change interrupt
ISR(PCINT0_vect)
{
    uint8_t currentPort = PINB;

    // Find which D10-D13 inputs changed
    uint8_t changedPins =
        (currentPort ^ previousLimitPort) & 0b11110000;

    /*
     * INPUT_PULLUP:
     * released = HIGH
     * pressed  = LOW
     *
     * This selects only pins that changed from HIGH to LOW.
     * LOW-to-HIGH switch releases are ignored.
     */
    uint8_t pressedPins =
        changedPins & (~currentPort) & 0b11110000;

    previousLimitPort = currentPort;

    if (pressedPins & (1 << PB4))
    {
        stopLeftMotor();
        limitEvent = LIMIT_LEFT;
        limitAbortRequested = true;
    }
    else if (pressedPins & (1 << PB5))
    {
        stopLeftMotor();
        limitEvent = LIMIT_RIGHT;
        limitAbortRequested = true;
    }
    else if (pressedPins & (1 << PB6))
    {
        stopRightMotor();
        limitEvent = LIMIT_BOTTOM;
        limitAbortRequested = true;
    }
    else if (pressedPins & (1 << PB7))
    {
        stopRightMotor();
        limitEvent = LIMIT_TOP;
        limitAbortRequested = true;
    }
}
