#include "GcodeToken.h"

void GcodeToken::Set(String token) {
    letter_ = token[0];                   // gets letter of instruction
    value_ = token.substring(1).toInt();  // converts number after letter to integer
}