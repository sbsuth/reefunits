/*
 */
//
// Debug prints.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1
//#define DEBUG_CONNECT 1

#define RF24_RE_INIT_AFTER_NUM_EMPTY 2


#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <EEPROM.h>
#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "Command.h"
#include "relays.h"


#define RF24_CE          14
#define RF24_CSN         15



// Network objects
RF24IPInterface rf24( 5, RF24_CE, RF24_CSN, RF24_PA_LOW );
DEFINE_RF24IPInterface_STATICS(rf24);

// Relay
unsigned char relayPins[8] = {6, 5, 7, 4, 3, 9, 2, 10};
Relays<8> relays( relayPins, 1 );

#define EE_SIG 123


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  #if DEBUG_STARTUP
  Serial.println(F("Start"));
  #endif

  // Check for current signature to determine if settings should be reset.
  unsigned char sig = 0;
  EEPROM.get( 0, sig );
  bool useSettings = (sig == EE_SIG);
  EEPROM.put( 0, EE_SIG );

  relays.init(useSettings);
  
  // Ethernet startup.
  rf24.init();
  //rf24.getRadio().setDataRate( RF24_250KBPS );
  //rf24.getRadio().setPALevel(RF24_PA_MIN);
  
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
    JsonArray& on = json.createNestedArray("on");
    for (unsigned char p=0; p < 8; p++ ) {
        on.add( relays.isOn(p) );
    }
    cmd->ack( json );
}

static void saveSettings()
{
    EEPROM.put( 0, EE_SIG );
    relays.saveSettings();
}

static void restoreSettings()
{
    unsigned char sig = 0;
    EEPROM.get( 0, sig );
    if (sig == EE_SIG) 
        relays.restoreSettings();
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

            case CmdAllOff:
                relays.allOff();
                break;

            case CmdOn: {
                int p;
                cmd->arg(0)->getInt(p);
                relays.on(p);
                break;
            }
            case CmdOff: {
                int p;
                cmd->arg(0)->getInt(p);
                relays.off(p);
                break;
            }
            case CmdStatus:
                getStatus(cmd);
                break;

            case CmdSaveSettings:
                saveSettings();
                break;

            case CmdRestoreSettings:
                restoreSettings();
                break;
            case CmdRenewRadio:
                rf24.init();
                break;
            default:
                #if DEBUG_CMD
                Serial.println(F("Unrecognized cmd\n"));
                #endif
                break;
        }
        #if DEBUG_CONNECT
        if (rf24Parser.stream()->connected()) {Serial.println("Disconnecting after command");}
        #endif
        if (needResp) {
            cmd->ack();
        }
        cmd->parser()->reset();
        cmd->disconnect();
    } else if (error) {
        #if DEBUG_CMD
        Serial.println(F("Error in command\n"));
        #endif
    } else {
        #if DEBUG_CONNECT
        if (rf24Parser.stream()->connected()) {Serial.println("Leaving with connection, but incomplete command");}
        #endif
    }
}


// the loop function runs over and over again forever
void loop() {
  rf24.update();

  processCommand();
}

