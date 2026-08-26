#include "GcodeToken.h"

void GcodeToken::Set(String token) {
    
    letter_ = token[0];                   // gets letter of instruction

    if (letter_ == 'F') {
        int f = token.substring(1).toInt();
        f = (f >= MIN_SPEED) ? MIN_SPEED : f;
        value_ = (f >= MAX_SPEED) ? MAX_SPEED : f;
    } else {
        value_ = token.substring(1).toInt();  // converts number after letter to integer        
    }

}