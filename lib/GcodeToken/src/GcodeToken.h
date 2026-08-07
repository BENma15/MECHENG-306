#ifndef GCODETOKEN_H
#define GCODETOKEN_H

#include <Arduino.h>

#define MAX_TOKENS 4

class GcodeToken {
  private:
    char letter_;
    int value_;

  public:
    GcodeToken() : letter_(' '), value_(0) {};  // Constructor
    void Set(String token);                     // Set the token
    char GetLetter() { return letter_; }        // Getter for letter
    int GetValue() { return value_; }           // Getter for value
    bool CheckIfValid();                        // check token is valid i.e doesnt go out of bounds

};

void GcodeToken::Set(String token) {
    letter_ = token[0];                   // gets letter of instruction
    value_ = token.substring(1).toInt();  // converts number after letter to integer
}

bool GcodeToken::CheckIfValid() {

}


#endif