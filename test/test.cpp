/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1


#include <Arduino.h>

#include "Command.h"
//#include <DRV8825.h>
#include <EEPROM.h>
#include "DosingPump.h"

#define DIR_PIN 8
#define STEP_PIN 7
#define ENABLE_PIN 9

const unsigned short rpm = 350;
#define STEPS_PER_REV 6400

#define EE_SIG 123
#define SIG_EE_ADDR         0
#define SIG_EE_SIZE         sizeof(unsigned char)
#define PUMP_EE_ADDR        SIG_EE_ADDR + SIG_EE_SIZE

//BasicStepperDriver pump( STEPS_PER_REV, DIR_PIN, STEP_PIN );
#define NUM_PUMPS 1
DosingPump pump( STEPS_PER_REV, DIR_PIN, STEP_PIN, 0, PUMP_EE_ADDR );
DosingPump* pumps[NUM_PUMPS] = {&pump};

static void saveSettings() {
    for ( int ipump=0; ipump < NUM_PUMPS; ipump++ ) {
        pumps[ipump]->saveSettings();
    }
}

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif
    
    pinMode( ENABLE_PIN, OUTPUT );

    unsigned char sig = 0;
    EEPROM.get( SIG_EE_ADDR, sig );
    bool useSettings = (sig == EE_SIG);
    EEPROM.put( SIG_EE_ADDR, (char)EE_SIG );
    pump.init(rpm,useSettings);

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

            case CmdPumpOn: {
				int ipump = cmd->ID();
                int onOff = 0;
				cmd->arg(0)->getInt(onOff);
                Serial.print("Pump #");Serial.print(ipump);Serial.print(", on=");Serial.println(onOff);

                #if 1
                if (onOff)
                    pump.rotate(360);
                #endif

                break;
            }
            case CmdPumpSpeed: {
				int ipump = cmd->ID();
                int speed = 0;
				cmd->arg(0)->getInt(speed);
                Serial.print("Pump #");Serial.print(ipump);Serial.print(", speed=");Serial.println(speed);
                break;
            }
            case CmdDispense: {
                p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                cmd->arg(0)->getInt(arg);
                #if DEBUG_CMD
				Serial.print("CHANGE: Got dispense on ");
				Serial.print(p);
                Serial.print(" for ");
                Serial.print(arg);
                Serial.println("ml");
				#endif
                pumps[p]->startDispense(arg);
                break;
            }
            case CmdCal: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                #if DEBUG_CMD
				Serial.print("CHANGE: Got cal on ");
				Serial.println(p);
				#endif
                pumps[p]->startCal();
                break;
            }
            case CmdCalRslt: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                cmd->arg(0)->getInt(arg);
                #if DEBUG_CMD
				Serial.print("CHANGE: Got actual for ");
				Serial.print(p);
                Serial.print(" of ");
                Serial.print(arg);
                Serial.println("ml");
				#endif
                pumps[p]->setActualMl(arg);
                saveSettings();
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

    unsigned char en[1] = {ENABLE_PIN};

    pump.update(en);
    if (en[0] > 0)
        digitalWrite( en[0], LOW );
}

