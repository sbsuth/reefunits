/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 0
#define DEBUG_CHANGES 0
#define DEBUG_CMD 0
#define DEBUG_CONNECT 0
#define DEBUG_MEM 0


#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <ArduinoJson.h>
#include <DosingPump.h>

#include "RF24Interface.h"

#include "Command.h"
#include "DistanceSensor.h"
#if DEBUG_MEM
#include "MemUtils.h"
#endif

#define RF24_CE          9
#define RF24_CSN         10

#define TRIG             6
#define ECHO             7

#define FLOAT_SW         8

#define DOSING_DIR      18
#define DOSING_I_SLEEP   0
#define DOSING_SLEEP    14
#define CALC_STEP       19
#define ALK_STEP        17
#define MAG_STEP        15

#define H2OX_DIR        4
#define H2OX_I_SLEEP    1
#define H2OX_SLEEP      2
#define OLD_OUT_STEP    5
#define NEW_IN_STEP     3

#define CALC_PUMP   0
#define ALK_PUMP    1
#define MAG_PUMP    2
#define OLD_OUT_PUMP 3
#define NEW_IN_PUMP 4
#define NUM_PUMPS   5



// Network.
RF24IPInterface rf24( 8, RF24_CE, RF24_CSN );
RF24EthernetClass   RF24Ethernet( rf24.getRadio(), rf24.getNetwork(), rf24.getMesh() );

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 80 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

#define PUMPS 1

#if PUMPS
// Dosing pumps
DosingPump calcPump(  DOSING_DIR,   CALC_STEP,      DOSING_I_SLEEP);
DosingPump alkPump(   DOSING_DIR,   ALK_STEP,       DOSING_I_SLEEP);
DosingPump magPump(   DOSING_DIR,   MAG_STEP,       DOSING_I_SLEEP );
DosingPump oldOutPump(H2OX_DIR,     OLD_OUT_STEP,   H2OX_I_SLEEP);
DosingPump newInPump( H2OX_DIR,     NEW_IN_STEP,    H2OX_I_SLEEP);

DosingPump* pumps[NUM_PUMPS] = {&calcPump, &alkPump, &magPump, &oldOutPump, &newInPump};
unsigned short pumpRPM[NUM_PUMPS] = {200, 200, 200, 200, 200};
#endif

// State vars


// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    floatSwitch.setup();
  
    // Ethernet startup.
    rf24.init();

#if PUMPS
    // Init pumps.
    for ( int i=0; i < NUM_PUMPS; i++ ) {
        pumps[i]->init( pumpRPM[i] );
    }
    pinMode( DOSING_SLEEP, OUTPUT );
    pinMode( H2OX_SLEEP, OUTPUT );
#endif

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24 );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

static void getStatus( Command* cmd, int ipump )
{
    //StaticJsonBuffer<128> jsonBuffer;
    DynamicJsonBuffer jsonBuffer(128);

    JsonObject& json = jsonBuffer.createObject();

    if (ipump < 0) {
        json["dist"] = distanceSensor.lastSample();
        json["float_sw"] = floatSwitch.isOn();
        int numActive = 0;
        int numDisabled = 0;
        #if PUMPS
        for ( ipump=0; ipump < NUM_PUMPS; ipump++ ) {
            DosingPump* pump = pumps[ipump];
            if (!pump->isEnabled())
                numDisabled++;
            if (pump->isDispensing())
                numActive++;
        }
        #endif
        json["num_active"] = numActive;
        json["num_dis"] = numDisabled;
    } else if (ipump < NUM_PUMPS) {
        #if PUMPS
        DosingPump* pump = pumps[ipump];
        json["en"] = pump->isEnabled();
        json["is_disp"] = pump->isDispensing();
        json["tot_disp"] = pump->dispensedMl();
        json["to_disp"] = pump->toDispenseMl();
        json["spml"] = pump->stepsPerMl();
        #else
        json["en"] = true;
        json["is_disp"] = true;
        json["tot_disp"] = 100;
        json["to_disp"] = 100;
        json["spml"] = 100;
        #endif
    }
    cmd->ack( json );

    #if DEBUG_MEM
    MemChecker::log_mem();
    #endif
}

void processCommand()
{

    int arg;
    int p;
    int i;
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
                getStatus(cmd,-1);
                break;

            case CmdPumpStatus:
                getStatus(cmd, cmd->ID());
                break;

            #if PUMPS
            case CmdEnable: {
                // 1 means decr disable, 0 incr disable.
                cmd->arg(0)->getInt(arg);
                for ( i=0 ; i < NUM_PUMPS; i++ ) {
                    if (arg)
                        pumps[i]->enable();
                    else
                        pumps[i]->disable();
                }
                break;
            }
            case CmdResetAll: {
                for ( i=0 ; i < NUM_PUMPS; i++ ) {
                    pumps[i]->reset();
                }
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
                break;
            }
            case CmdStepsPerMl: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                int steps;
                cmd->arg(0)->getInt(steps);
                #if DEBUG_CMD
				Serial.print("CHANGE: Got step per ml for ");
				Serial.print(p);
                Serial.print(" of ");
                Serial.println(steps);
				#endif
                pumps[p]->setStepsPerMl(steps);
                break;
            }
            #endif
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
        cmd->disconnect();
    } else if (error) {
        #if DEBUG_CMD
        Serial.println(F("Error in command\n"));
        #endif
        cmd->disconnect();
    }
    #if DEBUG_MEM
    MemChecker::log_mem();
    #endif
}



// the loop function runs over and over again forever
void loop() {
  #if DEBUG_MEM
  MemChecker::reset();
  #endif

  rf24.update();

  floatSwitch.update();
  if (floatSwitch.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Float switch ");
    Serial.println( floatSwitch.isOn() ? "CLOSED" : "OPEN");
    #endif
  }

  int nPumping = 0;
  #if PUMPS
  // Let pumps be run if required.  
  // Each will enable itself, and clear en[i] if it does.
  int i;
  unsigned char en[2] = {DOSING_SLEEP,H2OX_SLEEP};
  for ( i=0 ; i < NUM_PUMPS; i++ ) {
    pumps[i]->update(en);
  }
  // Disable each bank if none enabled it.
  for ( i=0; i < 2; i++ ) {
    if (en[i] > 0)
        digitalWrite( en[i], LOW );
    else
        nPumping++;
  }
  #endif
  
  // Only check distance sensor while not pumping since it inserts a delay
  // that makes pumps run roughly.
  if (nPumping == 0) {
      unsigned short dist;
      if (distanceSensor.update(dist)) {
        #if DEBUG_CHANGES
        Serial.print("Dist=");
        Serial.println(dist);
        #endif
      }
    }
  
  processCommand();

  #if DEBUG_MEM
  static int last_gap = 0;
  int gap = MemChecker::gap();
  if (gap != last_gap) {
      Serial.print("Mem gap=");
      Serial.println(gap);
      last_gap = gap;
  }
  #endif
}

