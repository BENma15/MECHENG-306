#include "GcodeToken.h"

void GcodeToken::Set(String token) {
    
    letter_ = token[0];                   // gets letter of instruction

    if (letter_ == 'F') {
        if (token.substring(1).toInt() >= MAX_SPEED) {
            value_ = MAX_SPEED;
        }
    } else {
        value_ = token.substring(1).toInt();  // converts number after letter to integer        
    }

}