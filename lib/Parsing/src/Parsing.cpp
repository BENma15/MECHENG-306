#include "Parsing.h"
#include "FSM.h"

String inputBuffer = "";
GcodeToken TokenArray[MAX_TOKENS];

//bugs: g01 x----25 y----25 f25 is a valid input
//a speed of 0 can be submitted, this shouldnt be possible
//sending m999 in idle state sends the state to motion


/* inintialise tokenArray */
void initialiseTokenArray(void) {
    TokenArray[G].Set("G0");
    TokenArray[X].Set("X0");
    TokenArray[Y].Set("Y0");
    TokenArray[F_token].Set("F20");
    //TokenArray[M].Set("M0");  // uncomment if random error comes up
}

/* reads line from arduino serial buffer into input buffer */
int readLine(void) {

    int flag = 1;   // set to 0 when tokensise gets called
    int validCommand = 2;   // 2 = incomplete command, 1 = invalid command, 0 = successful command

    while(Serial.available() && flag) {  // Serial.available() returns how many characters are waiting to be read in the arduinos serial buffer 

        char c = Serial.read();  // pulls the first (character) from the buffer

        if (c == '\n') {
            flag = 0;
            validCommand = tokenise(inputBuffer); // when new line character is reahced send input buffer to parseGcode function
            inputBuffer = "";   // reset buffer
        } else {
            inputBuffer += c;
        }
    }
    return (validCommand) ;    // will be 1 (error) if command reads complete line and command invalid
}


int tokenise(String line) {

    line = tidyString(line);   // remove spaces, make uppercase, and remove comment
    int err = additionalLetterCheck(line);

    if (err == 2) { // too many letters
        Serial.println("Invalid Gcode: Too many command letters");
        return 1;
    } else if (err == 1) {  // invalid -ve sign/s
        Serial.println("Invalid Gcode: Invalid negative sign placement");
        return 1;
    }

    bool xToken = true, yToken = true;  // assuming there is an X and Y token

    int Gpos = line.indexOf('G');   // gets position of G in command
    int Mpos = line.indexOf('M');   // gets position of M in command
    
    if (Gpos != -1 && Mpos != -1) {
        //error (command contains G AND M)
        Serial.println("Invalid Gcode: Command contains G and M");    // error message
        return 1;
    } else if (Gpos == -1 && Mpos == -1) {
        //error (No G or M command)
        Serial.println("Invalid Gcode: Command does not contain G nor M");    // error message
        return 1;
    } else if (Gpos == -1 && Mpos != -1) {
        // command is M999

        String token = returnToken(line, 'M');
        if (token == "ERROR") {
            // error (no value)
            Serial.println("Invalid Gcode: no digits after M");    // error message
            return 1;
        }
        if (token.substring(1).toInt() != 999) {
            // Error (not 999 command)
            Serial.println("Invalid Gcode: Incorrect digits following M");    // error message
            return 1;
        }
        TokenArray[M].Set(token);
        return 0;   // Valid token

    } else if (Mpos == -1 && Gpos != -1) {
        // Command is either G01 or G28

        if (Mpos == -1 && Gpos != -1) {
            if (FSM_getCurrentState() == STATE_FAULT) {
                return 0;
            }
        }
        
        String token = returnToken(line, 'G');
        if (token == "ERROR") {
            // error (no value)
            Serial.println("Invalid Gcode: no digits after G");    // error message
            return 1;
        }
        int check = token.substring(1).toInt();

        if (check != 1 && check != 28) {
            // Error (no 01 or 28 command)
            Serial.println("Invalid Gcode: Incorrect digits following G");    // error message
            return 1;

        } else if (check == 28) {
            TokenArray[G].Set(token);
            return 0;
        } else {    // check == 01
            TokenArray[G].Set(token);

            token = returnToken(line, 'X');     // gets token for X
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after X");    // error message
                return 1;
            } else if (token != "NoToken") {
                TokenArray[X].Set(token);
                xToken = true;
            } else {   
                xToken = false; 
            }


            token = returnToken(line, 'Y');     // gets token for Y
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after Y");    // error message
                return 1;
            } else if (token != "NoToken") {
                TokenArray[Y].Set(token);
                yToken = true;
            } else {   
                yToken = false; 
            }


            if (!xToken && !yToken) {       // check that there is either an X or Y component
                // error (G01 command with no X and no Y)
                Serial.println("Error: G01 command called with no X nor Y coordinates");    // error message
                return 1;
            }

            token = returnToken(line, 'F');     // gets token for F
            if (token == "ERROR") {
                // error (no value)
                Serial.println("Invalid Gcode: no digits after F");    // error message
                return 1;
            } else if (token != "NoToken") {       // if no F is present remains unchanged
                TokenArray[F_token].Set(token);
            }
            return 0;
        }
    }
    
    return 1;
}


String returnToken(String line, char Letter) {

    int letterPos = line.indexOf(Letter); 
    int tokenEnd = letterPos;

    if (letterPos == -1) {
        return "NoToken";   // returns no token
    }

    while (tokenEnd+1 < (int)line.length() && (isDigit(line[tokenEnd+1]) || (line[tokenEnd+1] == '-')) ) {  // runs while next char is a digit or -ve sign
        tokenEnd++;
    }

    if (tokenEnd == letterPos) {
        // Error (No value after letter)
        Serial.println("Error 1");
        return "ERROR";
    }


    String token = line.substring(letterPos, (tokenEnd + 1));   // substring excludes end, hence (tokenEnd + 1)

    if ((Letter == 'M' || Letter == 'G' || Letter == 'F') && token.substring(1).toDouble() <= 0) { //this one doesn't have a proper error code for 0 f
        Serial.println("Error 2");
        return "ERROR";
    }
    //Serial.print(token);
    return token;

}


String tidyString(String line) {

    int commaPos = line.indexOf(";");

    if (commaPos != -1) {
        line = line.substring(0, commaPos); // removes the comment from the line
    }

    line.trim();
    line.toUpperCase();  // any lowercase letters taken to upper
    line.replace(" ", "");  // removes spaces between tokens
    return line;

}


GcodeToken Parsing_getToken(int index) {

    return TokenArray[index];

}


void resetTokenArray() {
    TokenArray[G].Set("G0");
    TokenArray[M].Set("M0");
    TokenArray[X].Set("X0");
    TokenArray[Y].Set("Y0");
    // F token remains as it was in the previous intruction.
}

int additionalLetterCheck(String line) {   // pass tidied string into it to check there there is at most 4 letters and 2 -ve signs

    if (line.length() == 0) {
        return 2;   // line small (should never trigger due to previous code checks) // this is triggering
    }
    int numOfLetters = 0;
    int numOfSigns = 0;

    for (unsigned int i = 0; i < line.length(); i++) {
        char c = line[i];

        if (isAlpha(c)) {
            numOfLetters++;
        } else if (c == '-') {
            numOfSigns++;
        }
    }


    
    if (numOfLetters > 4) {
        return 2;   // error: too many letters
    } else if (numOfSigns > 2) {
        return 1;   // error: too many -ve signs
    } else if (numOfSigns > 0) {    // 1 or 2 -ves
        int indexOfSign[2] = {-1, -1};   // array for indices of -ve signs (should only be 2 -ve signs)
        int j = 0;

        for (unsigned int i = 0; i < line.length(); i++) {
            char c = line[i];
            if (c == '-') {
                indexOfSign[j] = i;
                j++;
            }
        }

        int err = checkValidSign(indexOfSign, line);
        if (err == 1) {
            return 1;   //error from signs not being valid
        } else {
            return 0;   //letter and signs correct
        }
    }

    return 0;   // again, shouldnt reach here 
}

int checkValidSign(int indices[], String line) {

    for (int i = 0; i < 2; i++) {
        unsigned int pos = indices[i];
        if (indices[i] == -1) { break;  }
        
        if (pos == 0 || pos == line.length() - 1) {
            return 1; // error
        } else {
            if (!isAlpha(line[pos-1]) || !isDigit(line[pos+1])) { // letter before and number after
                return 1;   // error (either no letter before or no number after)
            }
        }
    }
    return 0; // 1 or both are valid -ve signs

}