/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1
#define DEBUG_TEMP 1


#include <Arduino.h>

#include "Command.h"
#include "DS18B20.h"

#define ONE_WIRE_DATA_PIN A0


DS18B20<2> tempSensors( ONE_WIRE_DATA_PIN, 10, 1000 );

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    tempSensors.setup();

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static Command serialCmd;

void processCommand()
{

    int arg;
    int p;
    int i;
    bool error = false;
    Command* cmd = 0;
    if ( (cmd=serialParser.getCommand( &serialCmd, error )) ) {
        bool needResp = true;
        #if DEBUG_CMD
        Serial.print(F("Command: "));
        cmd->printName();
        Serial.println("");
        #endif
        switch ( cmd->kind() ) {
            case CmdNone:
                break;

            case CmdPing:
                break;

            case CmdSetRes: {
				int res;
				if (cmd->arg(0)->getInt(res) && (res >= 9) && (res <= 12) ) {
                    tempSensors.setResolution(res);
				}
                break;
            }
            case CmdGetTemp: {
				int unit;
				if (cmd->arg(0)->getInt(unit)) {
                    Serial.print("Temp #1:"); Serial.print(tempSensors.getTemp(0)); Serial.println("F");
                    Serial.print("Temp #2:"); Serial.print(tempSensors.getTemp(1)); Serial.println("F");
				}
                break;
            }
            default:
                #if DEBUG_CMD
                Serial.println(F("Unrecognized cmd\n"));
                #endif
                break;
        }
        if (needResp) {
            cmd->ack();
        }
        cmd->parser()->reset();
    } else if (error) {
        #if DEBUG_CMD
        Serial.println(F("Error in command\n"));
        #endif
    }
}



// the loop function runs over and over again forever
void loop() {
  processCommand();

  if (tempSensors.update()) {
    #if DEBUG_CHANGES
    Serial.print("Temp #1:"); Serial.print(tempSensors.getTemp(0)); Serial.println("F");
    Serial.print("Temp #2:"); Serial.print(tempSensors.getTemp(1)); Serial.println("F");
    #endif
  }
}

