/*
 */

#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "Command.h"
#include "DistanceSensor.h"
#include "ControllablePump.h"


#define RF24_CE          12
#define RF24_CSN         13
#define TRIG             39
#define ECHO             41
#define FLOAT_SW         37


// Debug prints.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1
//#define DEBUG_CONNECT 1



// Network objects
RF24IPInterface rf24( 7, RF24_CE, RF24_CSN );
RF24EthernetClass   RF24Ethernet( rf24.getRadio(), rf24.getNetwork(), rf24.getMesh() );
EthernetServer rf24EthernetServer(1000);
//EthernetClient rf24Client;

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 50 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

#define NPUMPS 4

#define EE_ADDR(prev) (prev##_EE_ADDR + prev##_EE_SIZE)

#define EE_SIG              123
#define SIG_EE_ADDR         0
#define SIG_EE_SIZE         sizeof(unsigned char)
#define PUMP_EE_ADDR        EE_ADDR(SIG)
#define PUMP_EE_ADDR_I(i)   (EE_ADDR(SIG) + (i * PUMP_EE_SIZE))
#define PUMP_EE_SIZE        (NPUMPS*ControllablePump::ee_size())


ControllablePump mainCirc( 4, 2, 20, PUMP_EE_ADDR_I(0), rf24 );
ControllablePump skimmer(  5, 3, 20, PUMP_EE_ADDR_I(1), rf24 );
ControllablePump ph1(      6, 0, 0,  PUMP_EE_ADDR_I(2), rf24 );
ControllablePump ph2(      7, 1, 0,  PUMP_EE_ADDR_I(3), rf24 );

ControllablePump* pumps[NPUMPS] = {&mainCirc,  &skimmer, &ph1, &ph2 };


// State vars

bool extPause = false;
//
// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(115200);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    // Check for current signature to determine if settings should be reset.
    unsigned char sig = 0;
    EEPROM.get( SIG_EE_ADDR, sig );
    bool useSettings = (sig == EE_SIG);
    EEPROM.put( SIG_EE_ADDR, EE_SIG );

    floatSwitch.setup();
  
    // Ethernet startup.
    rf24.init();

    for ( int ipump=0; ipump < NPUMPS; ipump++ )
        pumps[ipump]->setup( useSettings );

    // Initial state.
    extPause = false;

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

//static RF24SerialIO rfs( &rf24Network );
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24 );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<200> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    cmd->ack( json );
}

void doSwitch( int sw, bool onOff )
{
    char buf[8];
    if (onOff)
        strcpy( buf, "on " );
    else
        strcpy( buf, "off ");
    char* pc=&buf[0];
    while (*pc) pc++;
    *pc++ = '0' + sw;
    *pc++ = '\n';
    *pc++ = 0;
    rf24.sendToRadioClient( 5, buf, 2 );
}

static void saveSettings()
{
    for ( int ipump=0; ipump < NPUMPS; ipump++ )
        pumps[ipump]->saveSettings();
}

static void restoreSettings()
{
    for ( int ipump=0; ipump < NPUMPS; ipump++ )
        pumps[ipump]->restoreSettings();
}

void processCommand()
{

    bool error = false;
    Command* cmd = 0;
    if ( (cmd=serialParser.getCommand( &serialCmd, error )) ) {
    } else if ((cmd = rf24Parser.getCommand( &rf24Cmd, error )) ) {
    }
    if (cmd) {
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
                getStatus(cmd);
                break;

            case CmdSaveSettings:
                saveSettings();
                break;

            case CmdRestoreSettings:
                restoreSettings();
                break;

            case CmdSw: {
                int sw = cmd->ID();
                if ((sw >= 7) || (sw < 0))
                    break;
                int arg;
                cmd->arg(0)->getInt(arg);
                doSwitch( sw, arg ); 
                break;
            }
            case CmdPumpSpeed: {
                int ipump = cmd->ID();
                if ((ipump >= NPUMPS) || (ipump < 0))
                    break;
                int pct;
                cmd->arg(0)->getInt(pct);
                if (pct < 0)
                    pct = 0;
                else if (pct > 100)
                    pct = 100;
                pumps[ipump]->setSpeed( pct ); 
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

static uint32_t mesh_timer = 0;


// the loop function runs over and over again forever
void loop() {

  rf24.update();

  unsigned short dist = 0;
  if (distanceSensor.update(dist) ) {
    //Serial.print("Dist=");
    //Serial.println(dist);
  }
  floatSwitch.update();
  if (floatSwitch.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Float switch ");
    Serial.println( floatSwitch.isOn() ? "CLOSED" : "OPEN");
    #endif
  }
  processCommand();

  for ( int ipump=0; ipump < NPUMPS; ipump++ )
    pumps[ipump]->update();
}

