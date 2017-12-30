/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1


#include <Arduino.h>
#include "Switch.h"

#include "Command.h"

#define FAN_CTRL 3
#define TACH   5
#define BUTTON 7
#define MOTION 9

#define ON_AFTER_MOTION  (15 * 60 * 1000L)
#define WAIT_AFTER_OFF   (15 * 1000L)

// Button
Switch button( BUTTON, LOW );
Switch motion( MOTION, HIGH );

// State vars
long lastExplOff = 0;
long fanOffTime = 0;
bool fanIsOn = false;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    pinMode( TACH, INPUT );
    pinMode( FAN_CTRL, OUTPUT );
    digitalWrite( FAN_CTRL, HIGH );

    button.setup();
    motion.setup();

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static Command serialCmd;


static void fanOn( long t=-1 ) {
    digitalWrite( FAN_CTRL, HIGH );
    fanIsOn = true;
    if (t > 0) {
        fanOffTime = (millis() + t);
    } else {
        fanOffTime = -1;
    }
}

static void fanOff() {
    digitalWrite( FAN_CTRL, LOW );
    if (fanIsOn) {
        lastExplOff = millis();
    }
    fanIsOn = false;
    fanOffTime = 0;
}

static void getStatus( Command* cmd, int ipump )
{
    if (fanIsOn) 
        Serial.println("Fan on");
    else
        Serial.println("Fan off");
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

            case CmdOn:
                fanOn();
                break;

            case CmdOff:
                fanOff();
                break;

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
  #if 1
  button.update();
  if (button.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Button ");
    Serial.println( button.isOn() ? "CLOSED" : "OPEN");
    #endif

    if ( button.isOn() ) {
        if (fanIsOn)
            fanOff();
        else
            fanOn();
    }
  }
  #endif
  #if 1
  motion.update();
  if (motion.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Motion ");
    Serial.println( motion.isOn() ? "ACTIVE" : "INACTIVE");
    #endif

    if ( 0 && motion.isOn() ) {
        if (!fanIsOn && ((millis() - lastExplOff) > WAIT_AFTER_OFF)) {
            #if DEBUG_CHANGES
            Serial.println("Turning on from motion");
            #endif
            fanOn( ON_AFTER_MOTION );
        }
    }
  }

  #endif
  #if 0
  if ((fanOffTime > 0) && (millis() > fanOffTime)) {
    #if DEBUG_CHANGES
    Serial.println("Turning off after timeout");
    #endif
    fanOff();
  }
  #endif
  processCommand();

}

