/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1


#include <Arduino.h>

#include "Command.h"

static int level=0;


#define DIM_PIN 5

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    pinMode( DIM_PIN, OUTPUT );
    digitalWrite( DIM_PIN, 0 );
    level = 0;

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static Command serialCmd;

static void getStatus( Command* cmd, int ipump )
{
    Serial.print("Level=");
    Serial.println(level);
}

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

            case CmdStatus:
                getStatus(cmd,-1);
                break;

            case CmdDim: {
                int pct;
                if (cmd->arg(0)->getInt(pct)) {
                    int val = (pct * 255)/100;
                    if (val > 255) {
                        digitalWrite( DIM_PIN, 1);
                        Serial.print("Set pin ");Serial.print(DIM_PIN);Serial.println(" to digital 1");
                        level = 100;
                    } else if (val <= 0) {
                        digitalWrite( DIM_PIN, 0);
                        Serial.print("Set pin ");Serial.print(DIM_PIN);Serial.println(" to digital 0");
                        level = 0;
                    } else {
                        analogWrite( DIM_PIN, val);
                        Serial.print("Set pin ");Serial.print(DIM_PIN);Serial.write(" to ");Serial.println(val);
                        level = pct;
                    }
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
}
