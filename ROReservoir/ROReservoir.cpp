/*
 */

// Debug prints.
#define DEBUG_STARTUP 0
#define DEBUG_CHANGES 0
#define DEBUG_CMD 0
#define DEBUG_CONNECT 0

#define RF24_RE_INIT_AFTER_NUM_EMPTY 2
// 
// Set to 1 to enable commands on serial port.  Off to save memory.
#define SERIAL_COMMANDS 0


#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "Command.h"
#include "DistanceSensor.h"
#include "DecayingState.h"

#define OLD_BOARD 0

#define RF24_CE          9
#define RF24_CSN         10
#define TRIG             3
#define ECHO             4
#define FLOAT_SW         2
#if OLD_BOARD
#define TOP_PUMP         6
#define BOTTOM_PUMP      5
#else
#define TOP_PUMP         A0 // 18 // A0
#define BOTTOM_PUMP      A1 // 19 // A1
#define POND_PUMP_0      5
#define POND_PUMP_1      6
#endif
#define RO_SOLENOID      7
#define LED              8




// Network objects
RF24IPInterface rf24( 6, RF24_CE, RF24_CSN, RF24_PA_LOW );
DEFINE_RF24IPInterface_STATICS(rf24);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 40 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

// State vars

// Externally set pump on.
DecayingState<bool> bottomPumpOn( false, 10*1000UL );
DecayingState<bool> topPumpOn( false, 10*1000UL );
#if OLD_BOARD
#define NUM_PUMPS 2
DecayingState<bool>* pumpOn[NUM_PUMPS] = {&topPumpOn, &bottomPumpOn};
char pumpPins[NUM_PUMPS] = {TOP_PUMP, BOTTOM_PUMP};
char pumpAH[NUM_PUMPS] = {false, false};
#else
#define NUM_PUMPS 4
DecayingState<bool> pp0On( false, 10*1000UL );
DecayingState<bool> pp1On( false, 10*1000UL );
DecayingState<bool>* pumpOn[4] = {&topPumpOn, &bottomPumpOn, &pp0On, &pp1On};
char pumpPins[NUM_PUMPS] = {TOP_PUMP, BOTTOM_PUMP, POND_PUMP_0, POND_PUMP_1};
char pumpAH[NUM_PUMPS] = {false, false, true, true};
#endif

#define I_TOP 0
#define I_BOT 1
#define I_PP_0 2
#define I_PP_1 3

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


    for (int i=0; i < NUM_PUMPS; i++ ) {
        pinMode( pumpPins[i], OUTPUT );
        digitalWrite( pumpPins[i], pumpAH[i]?0:1 );
    }

    digitalWrite( RO_SOLENOID, 0 );
    pinMode( RO_SOLENOID, OUTPUT );

    pinMode( LED, OUTPUT );
    digitalWrite( LED, 0 );

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
#if SERIAL_COMMANDS
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
#endif
static EthernetSerialIO rfs( &rf24 );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<180> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["res_sw"] = floatSwitch.isOn();
    json["res_lev"] = distanceSensor.currentCM();
    json["top_on"] = pumpOn[I_TOP]->getVal() ? pumpOn[I_TOP]->timeAtValueSec() : 0;
    json["bot_on"] = pumpOn[I_BOT]->getVal() ? pumpOn[I_BOT]->timeAtValueSec() : 0;
    json["tpp0_on"] = pumpOn[I_PP_0]->getVal() ? pumpOn[I_PP_0]->timeAtValueSec() : 0;
    json["pp1_on"] = pumpOn[I_PP_1]->getVal() ? pumpOn[I_PP_1]->timeAtValueSec() : 0;
    json["ro_mode"] = (int)roMode;
    json["pause"] = extPause.getVal();
    cmd->ack( json );
   /*
   https://arduinojson.org/assistant/
   {
     "res_sw": true,
     "res_lev": 123,
     "top_on": 1234,
     "bot_on": 1234,
     "pp0_on": 1234,
     "pp1_on": 1234,
     "ro_mode": 1,
     "pause": false
    }
    178
   */
}

// Process a 'pon' cmd.
// Arg is number of seconds for decay.
// Response is json giving success and time since on.
static void setPumpOn( Command* cmd )
{
    bool ok = false;
    unsigned long t = 0;
    if ((cmd->ID() < NUM_PUMPS) && !extPause.getVal()) {
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

#if 1
    StaticJsonBuffer<41> jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    //json["ton"] = t / 1000U;
    json["ton"] = t;
    json["ok"] = ok;
    cmd->ack( json );
   /*
    https://arduinojson.org/assistant/
   {
     "ton": 1234,
     "ok": true,
    }
    41
   */
#endif
}

static void updatePumps() 
{
    bool ep = extPause.getVal();
    for ( int i=0; i < NUM_PUMPS; i++ ) {
        DecayingState<bool>* pump = pumpOn[i];
        pump->update();
        int onVal = pumpAH[i] ? 1 : 0;
        int offVal = pumpAH[i] ? 0 : 1;
        digitalWrite( pumpPins[i], (!ep && pump->getVal()) ? onVal : offVal );
    }
}

void processCommand()
{

    int p = 0;
    bool error = false;
    Command* cmd = 0;
    #if SERIAL_COMMANDS
    if ( (cmd=serialParser.getCommand( &serialCmd, error )) ) {
    } else 
    #endif
    cmd = rf24Parser.getCommand( &rf24Cmd, error );
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
                for (p=0; p < NUM_PUMPS; p++)
                    pumpOn[p]->setVal(false);
                break;

            case CmdPumpOn: {
                setPumpOn( cmd );
                needResp = false;
                break;
            }
            case CmdPumpOff: {
                cmd->arg(0)->getInt(p);
                if ((p >= 0) && (p < NUM_PUMPS)) {
                    pumpOn[p]->setVal(false);
                }
                break;
            }
            case CmdROOn: {
                cmd->arg(0)->getInt(p);

                roMode = (p ? ROForceOn : ROForceOff);
                break;
            }
            case CmdFill: {
                cmd->arg(0)->getInt(p);
                roMode = (p ? ROKeepFull : ROForceOff);
                break;
            }
            case CmdExtPause: {
                cmd->arg(0)->getInt(p);
                extPause.setDecay(p * 1000UL);
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

  extPause.update();

  #if 1
  unsigned short dist = 0;
  if (distanceSensor.update(dist) ) {
    //Serial.print("Dist=");
    //Serial.println(dist);
  }
  #endif
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

