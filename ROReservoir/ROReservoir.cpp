/*
 */

// Debug prints.
#define DEBUG_STARTUP 0
#define DEBUG_CHANGES 0
#define DEBUG_CMD 0
#define DEBUG_CONNECT 0

#define RF24_RE_INIT_AFTER_NUM_EMPTY 5

#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "Command.h"
#include "DistanceSensor.h"
#include "DecayingState.h"


#define RF24_CE          9
#define RF24_CSN         10
#define TRIG             3
#define ECHO             4
#define FLOAT_SW         2
#define TOP_PUMP         5
#define BOTTOM_PUMP      6
#define RO_SOLENOID      7




// Network objects
RF24IPInterface rf24( 6, RF24_CE, RF24_CSN, RF24_PA_LOW );
DEFINE_RF24IPInterface_STATICS;
RF24EthernetClass   RF24Ethernet( rf24.getRadio(), rf24.getNetwork(), rf24.getMesh() );
EthernetServer rf24EthernetServer(1000);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 40 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

// State vars

// Externally set pump on.
DecayingState<bool> bottomPumpOn( false, 10*1000UL );
DecayingState<bool> topPumpOn( false, 10*1000UL );
DecayingState<bool>* pumpOn[2] = {&topPumpOn, &bottomPumpOn};
char pumpPins[2] = {TOP_PUMP, BOTTOM_PUMP};

#define I_TOP 0
#define I_BOT 1

DecayingState<bool> extPause( false, 5 * 60 * 1000UL );

enum ROMode {
    ROForceOff = 0,
    ROForceOn = 1,
    ROKeepFull = 2
};

ROMode roMode = ROForceOff;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif


    pinMode( TOP_PUMP, OUTPUT );
    pinMode( BOTTOM_PUMP, OUTPUT );
    digitalWrite( TOP_PUMP, 1 );
    digitalWrite( BOTTOM_PUMP, 1 );

    digitalWrite( RO_SOLENOID, 0 );
    pinMode( RO_SOLENOID, OUTPUT );

    floatSwitch.setup();
  
    // Ethernet startup.
    rf24.init();

    // Initial state.
    roMode = ROKeepFull;

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
    StaticJsonBuffer<148> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["res_sw"] = floatSwitch.isOn();
    json["res_lev"] = distanceSensor.currentCM();
    json["top_on"] = pumpOn[I_TOP]->getVal() ? pumpOn[I_TOP]->timeAtValueSec() : 0;
    json["bot_on"] = pumpOn[I_BOT]->getVal() ? pumpOn[I_BOT]->timeAtValueSec() : 0;
    json["ro_mode"] = (int)roMode;
    json["pause"] = extPause.getVal() / 1000U;
    cmd->ack( json );
   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
   {
     "res_sw": true,
     "res_lev": 123,
     "top_on": 1234,
     "bot_on": 1234,
     "ro_mode": 1,
     "pause": false
    }
    134
   */
}

// Process a 'pon' cmd.
// Arg is number of seconds for decay.
// Response is json giving success and time since on.
static void setPumpOn( Command* cmd )
{
    bool ok = false;
    unsigned long t = 0;
    if ((cmd->ID() < 2) && !extPause.getVal()) {
        DecayingState<bool>* pump = pumpOn[cmd->ID()];
        t = pump->setVal( true );
        int decay = 0;
        cmd->arg(0)->getInt(decay);
        #if DEBUG_CHANGES
        Serial.print("PUMP: Turning pump #");Serial.print(cmd->ID());Serial.print(" on for ");Serial.println(decay);
        #endif
        if (decay)
            pump->setDecay(decay * 1000UL);

        ok = true;
    }

    StaticJsonBuffer<48> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["ton"] = t / 1000U;
    json["ok"] = ok;
    cmd->ack( json );
   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
   {
     "ton": 1234,
     "ok": true,
    }
    41
   */
}

static void updatePumps() 
{
    bool ep = extPause.getVal();
    for ( int i=0; i < 2; i++ ) {
        DecayingState<bool>* pump = pumpOn[i];
        pump->update();
        digitalWrite( pumpPins[i], (!ep && pump->getVal()) ? 0 : 1 );
    }
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

            case CmdAllOff:
                pumpOn[I_TOP]->setVal(false);
                pumpOn[I_BOT]->setVal(false);
                break;

            case CmdPumpOn: {
                setPumpOn( cmd );
                needResp = false;
                break;
            }
            case CmdPumpOff: {
                int p;
                cmd->arg(0)->getInt(p);
                if ((p >= 0) && (p < 2)) {
                    pumpOn[p]->setVal(false);
                }
                break;
            }
            case CmdROOn: {
                int p;
                cmd->arg(0)->getInt(p);

                roMode = (p ? ROForceOn : ROForceOff);
                break;
            }
            case CmdFill: {
                int p;
                cmd->arg(0)->getInt(p);
                roMode = (p ? ROKeepFull : ROForceOff);
                break;
            }
            case CmdExtPause: {
                int t = 0;
                cmd->arg(0)->getInt(t);
                extPause.setDecay(t * 1000UL);
                extPause.setVal(true);
                break;
            }
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
    }
}


// Turn on or off the solenoid based on state.
void updateROOn()
{
    int onOff = 0;

    if (extPause.getVal()) {
        // If in external pause mode, always stop.
        onOff = 0;
    } else {
        // Force on, force off, or on if not full.
        switch (roMode) {
            case ROForceOn: 
                onOff = 1; 
                break;
            case ROKeepFull: 
                onOff = !floatSwitch.isOn(); 
                break;
            case ROForceOff: 
            default:
                onOff = 0; 
                break;
        }
    }
    digitalWrite( RO_SOLENOID, onOff );
}

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

  updateROOn();
  updatePumps();
}

