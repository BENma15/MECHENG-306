#include <Arduino.h>

int LEFT_LIMIT_PIN = 2;
int RIGHT_LIMIT_PIN = 3;
int UP_LIMIT_PIN = 14;
int DOWN_LIMIT_PIN = 10;

void testLimitSwitches() {
	Serial.print("Left: ");
	Serial.print(digitalRead(LEFT_LIMIT_PIN) == LOW ? "TRIGGERED" : "OPEN");
	Serial.print(" | Right: ");
	Serial.print(digitalRead(RIGHT_LIMIT_PIN) == LOW ? "TRIGGERED" : "OPEN");
	Serial.print(" | Up: ");
	Serial.print(digitalRead(UP_LIMIT_PIN) == LOW ? "TRIGGERED" : "OPEN");
	Serial.print(" | Down: ");
	Serial.println(digitalRead(DOWN_LIMIT_PIN) == LOW ? "TRIGGERED" : "OPEN");
}

void setup() {
	Serial.begin(115200);
	pinMode(LEFT_LIMIT_PIN, INPUT_PULLUP);
	pinMode(RIGHT_LIMIT_PIN, INPUT_PULLUP);
	pinMode(UP_LIMIT_PIN, INPUT_PULLUP);
	pinMode(DOWN_LIMIT_PIN, INPUT_PULLUP);
}

void loop() {
	testLimitSwitches();
	delay(200);
}
