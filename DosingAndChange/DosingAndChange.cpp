/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 0
#define DEBUG_CHANGES 0
#define DEBUG_CMD 0
#define DEBUG_CONNECT 0
#define DEBUG_MEM 0

#define RF24_RE_INIT_AFTER_NUM_EMPTY 2

// Set to 1 to enable commands on serial port.
// Turning this on will probably blow memory, and require turning off something
// else, like PUMPS
#define SERIAL_COMMANDS 0

// Enable if using paus cmd.
#define EXT_PAUSE 0


#include <Arduino.h>
#include "Switch.h"
#include <EEPROM.h>

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
#define OLD_OUT_STEP    3
#define NEW_IN_STEP     5

#define CALC_PUMP   0
#define ALK_PUMP    1
#define MAG_PUMP    2
#define OLD_OUT_PUMP 3
#define NEW_IN_PUMP 4
#define NUM_PUMPS   5



// Network.
RF24IPInterface rf24( 8, RF24_CE, RF24_CSN );
DEFINE_RF24IPInterface_STATICS(rf24);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 80 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

#if EXT_PAUSE
DecayingState<bool> extPause( false, 5 * 60 * 1000UL );
DecayingState<bool>* extPausePtr = &extPause;
#else
DecayingState<bool>* extPausePtr = 0;
#endif

#define PUMPS 1

#define EE_SIG 123
#define SIG_EE_ADDR         0
#define SIG_EE_SIZE         sizeof(unsigned char)

#define PUMP_EE_ADDR        SIG_EE_ADDR + SIG_EE_SIZE
#define PUMP_EE_ADDR_I(i)   (PUMP_EE_ADDR + (i * DosingPump::ee_size()))
#define PUMP_EE_SIZE        (NUM_PUMPS*DosingPump::ee_size())

#define SLOW_PUMP_STEPS 200
#define FAST_PUMP_STEPS 6400

#define SLOW_PUMP_RPM 200
#define FAST_PUMP_RPM 360

#if PUMPS
// Dosing pumps
DosingPump calcPump(  SLOW_PUMP_STEPS, DOSING_DIR,   CALC_STEP,      DOSING_I_SLEEP, PUMP_EE_ADDR_I(0), extPausePtr);
DosingPump alkPump(   SLOW_PUMP_STEPS, DOSING_DIR,   ALK_STEP,       DOSING_I_SLEEP, PUMP_EE_ADDR_I(1), extPausePtr);
DosingPump magPump(   SLOW_PUMP_STEPS, DOSING_DIR,   MAG_STEP,       DOSING_I_SLEEP, PUMP_EE_ADDR_I(2), extPausePtr);
DosingPump oldOutPump(FAST_PUMP_STEPS, H2OX_DIR,     OLD_OUT_STEP,   H2OX_I_SLEEP,   PUMP_EE_ADDR_I(3), extPausePtr);
DosingPump newInPump( FAST_PUMP_STEPS, H2OX_DIR,     NEW_IN_STEP,    H2OX_I_SLEEP,   PUMP_EE_ADDR_I(4), extPausePtr);

DosingPump* pumps[NUM_PUMPS] = {&calcPump, &alkPump, &magPump, &oldOutPump, &newInPump};
#define I_FIRST_FAST_PUMP 3
//unsigned short pumpRPM[NUM_PUMPS] = {200, 200, 200, 200, 200};
//const unsigned short pumpRPM = 200;
#endif


// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif
    
    // Check for current signature to determine if settings should be reset.
    unsigned char sig = 0;
    EEPROM.get( SIG_EE_ADDR, sig );
    bool useSettings = (sig == EE_SIG);
    EEPROM.put( SIG_EE_ADDR, (char)EE_SIG );

    floatSwitch.setup();
  
    // Ethernet startup.
    rf24.init();

#if PUMPS
    // Init pumps.
    for ( int i=0; i < NUM_PUMPS; i++ ) {
        pumps[i]->init( (i>=I_FIRST_FAST_PUMP) ? FAST_PUMP_RPM : SLOW_PUMP_RPM, useSettings );
    }
    pinMode( DOSING_SLEEP, OUTPUT );
    pinMode( H2OX_SLEEP, OUTPUT );
#endif

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

#if SERIAL_COMMANDS
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static Command serialCmd;
#endif
static EthernetSerialIO rfs( &rf24 );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command rf24Cmd;

static void saveSettings() {
    #if PUMPS
    for ( int ipump=0; ipump < NUM_PUMPS; ipump++ ) {
        pumps[ipump]->saveSettings();
    }
    #endif
}

static void getStatus( Command* cmd, int ipump )
{
    //StaticJsonBuffer<128> jsonBuffer;
    DynamicJsonBuffer jsonBuffer(128);

    JsonObject& json = jsonBuffer.createObject();

    if (ipump < 0) {
        json["dist"] = distanceSensor.currentCM();
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
        #if EXT_PAUSE
        json["pause"] = extPause.getVal();
        #endif
    } 
    #if 0
    else if (ipump < NUM_PUMPS) {
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
    #endif
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
                getStatus(cmd,-1);
                break;

            #if 0
            case CmdPumpStatus:
                getStatus(cmd, cmd->ID());
                break;
            #endif

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
                saveSettings();
                break;
            }
            #endif
            #if EXT_PAUSE
            case CmdExtPause: {
                int t = 0;
                cmd->arg(0)->getInt(t);
                extPause.setDecay(t * 1000UL);
                extPause.setVal(true);
                break;
            }
            #endif
            case CmdSaveSettings:
                saveSettings();
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
        cmd->disconnect();
    } else if (error) {
        #if DEBUG_CMD
        Serial.println(F("Error in command\n"));
        #endif
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

  #if EXT_PAUSE
  extPause.update();
  #endif

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

