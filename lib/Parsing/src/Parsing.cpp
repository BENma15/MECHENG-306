#include "Parsing.h"

String inputBuffer = "";
GcodeToken TokenArray[MAX_TOKENS];

/* inintialise tokenArray */
void initialiseTokenArray(void) {
    TokenArray[G].Set("G0");
    TokenArray[X].Set("X0");
    TokenArray[Y].Set("Y0");
    TokenArray[F].Set("F0");
}

/* reads line from arduino serial buffer into input buffer */
void readLine(void) {
    while(Serial.available()) {  // Serial.available() returns how many characters are waiting to be read in the arduinos serial buffer 
        char c = Serial.read();  // pulls the first (character) from the buffer
        if (c == '\n' || c == ';') {
            tokenise(inputBuffer); // when new line character is reahced send input buffer to parseGcode function
            inputBuffer = "";   // reset buffer
        } else {
            inputBuffer += c;
        }
    }
}


void tokenise(String line) {
    line = tidyString(line);   // remove spaces and make uppercase
    bool xToken = true, yToken = true;  // assuming there is an X and Y token

    int Gpos = line.indexOf('G');   // gets position of G in command
    int Mpos = line.indexOf('M');   // gets position of M in command
    
    if (Gpos != -1 && Mpos != -1) {
        //error (command contains G AND M)
        Serial.println("Invalid Gcode: Command contains G and M");    // error message
        return;
    } else if (Gpos == -1 && Mpos == -1) {
        //error (No G or M command)
        Serial.println("Invalid Gcode: Command does not contain G nor M");    // error message
        return;
    } else if (Gpos == -1 && Mpos != -1) {
        // command is M999

        String token = returnToken(line, 'M');
        if (token == "ERROR") {
            // error (no value)
            Serial.println("Invalid Gcode: no digits after M");    // error message
            return;
        }
        if (token.substring(1).toInt() != 999) {
            // Error (not 999 command)
            Serial.println("Invalid Gcode: Incorrect digits following M");    // error message
            return;
        }
        TokenArray[M].Set(token);
        // state change??                                   <---------------------- state change?
        return;

    } else if (Mpos == -1 && Gpos != -1) {
        // Command is either G01 or G28

        String token = returnToken(line, 'G');
        if (token == "ERROR") {
            // error (no value)
            Serial.println("Invalid Gcode: no digits after G");    // error message
            return;
        }
        int check = token.substring(1).toInt();

        if (check != 1 && check != 28) {
            // Error (no 01 or 28 command)
            Serial.println("Invalid Gcode: Incorrect digits following G");    // error message
            return;

        } else if (check == 28) {
            TokenArray[G].Set(token);
            // state change??                              <---------------------- state change?
            return;
        } else {    // check == 01
            TokenArray[G].Set(token);

            token = returnToken(line, 'X');     // gets token for X
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after X");    // error message
                return;
            } else if (token != "NoToken") {
                TokenArray[X].Set(token);
                xToken = true;
            } else {   
                xToken = false; 
            }


            token = returnToken(line, 'Y');     // gets toke for Y
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after Y");    // error message
                return;
            } else if (token != "NoToken") {
                TokenArray[Y].Set(token);
                yToken = true;
            } else {   
                yToken = false; 
            }


            if (!xToken && !yToken) {       // check that there is either an X or Y component
                // error (G01 command with no X and no Y)       <------------------------ state change?
                Serial.println("Error: G01 command called with no X nor Y coordinates");    // error message
                return;
            }

            token = returnToken(line, 'F');     // gets token for F
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after F");    // error message
                return;
            } else if (token != "NoToken") {       // if no F is present remains unchanged
                TokenArray[F].Set(token);
            }
        }
    }
}

String returnToken(String line, char Letter) {
    int letterPos = line.indexOf(Letter); 
    int tokenEnd = letterPos;
    if (letterPos == -1) {
        return "NoToken";   // returns no token
    }
    while (tokenEnd+1 < line.length() && isDigit(line[tokenEnd+1])) {
        tokenEnd++;
    }
    if (tokenEnd == letterPos) {
        // Error (No value after letter)
        return "ERROR";
    }
    String token = line.substring(letterPos, (tokenEnd + 1));   // substring excludes end, hence (tokenEnd + 1)
    return token;
}

String tidyString(String line) {
    line.trim();
    line.toUpperCase();  // any lowercase letters taken to upper
    line.replace(" ", "");  // removes spaces between tokens
    return line;
}
