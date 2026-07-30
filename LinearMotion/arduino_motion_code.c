#include <Arduino.h>

const int LENCA = 18;     /* INT3 */
const int LENCB = 19;     /* INT2 */
const int RENCA = 20;     /* INT1 */
const int RENCB = 21;     /* INT0 */

StraightLinePID leftPID (1.0, 0.0, 0.0);
StraightLinePID rightPID (1.0, 0.0, 0.0);

void setup () {

}

void loop () {

}