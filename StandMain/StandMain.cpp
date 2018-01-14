/*
 */

// Debug prints.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1
//#define DEBUG_CONNECT 1
//#define DEBUG_PUMP 1

// Cause period re-init of RF24 subsystem.
#define AUTO_RE_INIT_INTERVAL 0
//#define AUTO_RE_INIT_INTERVAL 030000
//

// Setting prevents the periodic outbound cmds sent to the switch
#define DISABLE_PUMP_SWITCH_SYNC 1


#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "Command.h"
#include "DistanceSensor.h"
#include "ControllablePump.h"
#include "TempController.h"
#include "probe.h"


#define HEAT_ON          10
#define DISPLAY_TEMP     A1
#define SUMP_TEMP        A0
#define RF24_CE          12
#define RF24_CSN         13
#define TRIG             39
#define ECHO             41
#define FLOAT_SW         37




// Network objects
RF24IPInterface rf24( 7, RF24_CE, RF24_CSN, RF24_PA_LOW );
DEFINE_RF24IPInterface_STATICS;
RF24EthernetClass   RF24Ethernet( rf24.getRadio(), rf24.getNetwork(), rf24.getMesh() );
EthernetServer rf24EthernetServer(1000);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 50 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

#define NPUMPS 4

#define EE_SIG              129
#define SIG_EE_ADDR         0
#define SIG_EE_SIZE         sizeof(unsigned char)

#define T_CTRL_EE_ADDR      SIG_EE_ADDR + SIG_EE_SIZE
#define T_CTRL_EE_SIZE      TempController::ee_size()

#define PUMP_EE_ADDR        T_CTRL_EE_ADDR + T_CTRL_EE_SIZE
#define PUMP_EE_ADDR_I(i)   (PUMP_EE_ADDR + (i * ControllablePump::ee_size()))
#define PUMP_EE_SIZE        (NPUMPS*ControllablePump::ee_size())

#define NEXT_EE_ADDR        PUMP_EE_ADDR + PUMP_EE_SIZE

                        // PIN, SWITCH, MINPCT
ControllablePump mainCirc( 5,   2,      20, PUMP_EE_ADDR_I(0), rf24 );
ControllablePump skimmer(  4,   3,      20, PUMP_EE_ADDR_I(1), rf24 );
ControllablePump ph1(      6,   0,      0,  PUMP_EE_ADDR_I(2), rf24 );
ControllablePump ph2(      7,   1,      0,  PUMP_EE_ADDR_I(3), rf24 );

ControllablePump* pumps[NPUMPS] = {&mainCirc,  &skimmer, &ph1, &ph2 };

pHProbe pH_probe( Serial3 );
ConductivityProbe EC_probe( Serial2 );

TempController tempController( HEAT_ON, SUMP_TEMP, DISPLAY_TEMP, T_CTRL_EE_ADDR );


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
    pH_probe.setup();
    EC_probe.setup();
    tempController.setup( useSettings );

    // Ethernet startup.
    #if DEBUG_STARTUP
    Serial.println(F("Before rf24.init()"));
    #endif
    rf24.init();
    //rf24.getRadio().setDataRate( RF24_250KBPS );
    //rf24.getRadio().setDataRate( RF24_1MBPS );
    //rf24.getRadio().setPALevel(RF24_PA_LOW);
    #if DEBUG_STARTUP
    Serial.println(F("After rf24.init()"));

    Serial.print("DataRate=");Serial.println((int)rf24.getRadio().getDataRate()) ;
    #endif

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
    StaticJsonBuffer<330> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["pH"] = pH_probe.lastValue();
    json["EC"] = EC_probe.lastValue(0);
    json["TDS"] = EC_probe.lastValue(1);
    json["SAL"] = EC_probe.lastValue(2);
    json["SG"] = EC_probe.lastValue(3);
    json["sump_sw"] = floatSwitch.isOn();
    json["sump_lev"] = distanceSensor.currentCM();
    json["heat"] = tempController.heaterIsOn();
    json["theat"] = tempController.timeSinceLastOnOff(); // Dividing by 1000 fails.  Blows stack?
    float t = tempController.curTemp(0);
    json["temp0"] = isnan(t) ? 0.0 : t;
    t = tempController.curTemp(1);
    json["temp1"] = isnan(t) ? 0.0 : t;
    json["cal0"] = tempController.isCalibrated(0);
    json["cal1"] = tempController.isCalibrated(1);
    json["tset"] = tempController.getSetTemp();
    json["tper"] = tempController.getSamplePeriod();
    json["tsens"] = tempController.getSensitivity();
    cmd->ack( json );
   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
   {
     "pH": 1.23,
     "EC": 10,
     "SAL:": 10,
     "SG": 10,
     "TDS": 10,
     "sump_sw": true,
     "sump_lev": 123,
     "heat": true,
     "theat": 1000,
     "temp0": 1.23,
     "temp1": 1.23,
     "cal0": true,
     "cal1": true,
     "tset": 1.23,
     "tper": 10000,
     "tsens": 1.23
   }
    320
   */
}

static void getPumpStatus( Command* cmd )
{
    StaticJsonBuffer<936> jsonBuffer;

    JsonArray& array = jsonBuffer.createArray();
    for (int ipump=0; ipump < 4; ipump++ ) {
        ControllablePump* pump = pumps[ipump];
        JsonObject& jpump = array.createNestedObject();
        jpump["on"] = (pump->getCurSpeed() > 0) ? true : false;
        jpump["mode"] = (int)pump->getMode();
        jpump["cur_speed"] = pump->getCurSpeedPct();
        jpump["top_speed"] = pump->getTopSpeedPct();
        jpump["slow_speed"] = pump->getSlowSpeedPct();
        jpump["ramp_sec"] = pump->getRampSec();
        jpump["hold_sec"] = pump->getHoldSec();
        jpump["ramp_range"] = pump->getRampRangePct();
        jpump["hold_range"] = pump->getHoldRangePct();
        jpump["temp_shutdown"] = pump->tempShutdownRemainingSec();
    }
    cmd->ack(array);

   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
    {"on":false,"mode":1,"cur_speed":0,"top_speed":0,"slow_speed":0,"ramp_sec":1,"hold_sec":10,"ramp_range":0,"hold_range":0,"temp_shutdown":0}
    Copied into an array of 4 of these: 924
   */
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
    tempController.saveSettings();

    for ( int ipump=0; ipump < NPUMPS; ipump++ )
        pumps[ipump]->saveSettings();

}

static void restoreSettings()
{
    tempController.saveSettings();

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
        bool respDone = false;
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
                respDone = true;
                break;

            case CmdPumpStatus:
                getPumpStatus(cmd);
                respDone = true;
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
                for ( int i=0; i < 2; i++ ) {
                    cmd->arg(i)->getInt(pct);
                    if (pct < 0)
                        pct = 0;
                    else if (pct > 100)
                        pct = 100;
                    if (i==0)
                        pumps[ipump]->setTopSpeed( pct ); 
                    else
                        pumps[ipump]->setSlowSpeed( pct ); 
                }
                break;
            }
            case CmdTempShutdown: {
                int ipump = cmd->ID(); // -1 if all pumps.
                if (ipump >= NPUMPS) 
                    break;
                int secs;
                cmd->arg(0)->getInt(secs);
                for (int i=0; i < NPUMPS; i++) {
                    if ((ipump < 0) || (ipump == i)) {
                        if (secs > 0) 
                            pumps[i]->setTempShutoffInterval( secs ); 
                        else
                            pumps[i]->cancelTempShutoff();
                    }
                }
                break;
            }
            case CmdPumpMode: {
                int ipump = cmd->ID();
                if ((ipump >= NPUMPS) || (ipump < 0))
                    break;
                int mode=0;
                float holdArg, rampArg;
                cmd->arg(0)->getInt(mode);
                cmd->arg(1)->getFloat(holdArg);
                cmd->arg(2)->getFloat(rampArg);
                if ((mode < 0) || (mode >= ControllablePump::NumModes))
                    break;
                pumps[ipump]->setMode( (ControllablePump::Mode)mode, holdArg, rampArg );
                break;
            }
            case CmdCalEC: {
                int step=-1;
                cmd->arg(0)->getInt(step);
                cmd->ack( EC_probe.calStep( step ) );
                    break;
            }
            case CmdCalPH: {
                int step=-1;
                cmd->arg(0)->getInt(step);
                cmd->ack( pH_probe.calStep( step ) );
                break;
            }
            case CmdHeat: {
                int onOff=-1;
                cmd->arg(0)->getInt(onOff);
                tempController.setHeaterOn( onOff != 0 );
                break;
            }
            case CmdSetTargetTemp: {
                float TF;
                int sample;
                float sense;
                cmd->arg(0)->getFloat(TF);
                cmd->arg(1)->getInt(sample);
                cmd->arg(2)->getFloat(sense);
                if (TF > 0.0) tempController.setSetTemp(TF);
                if (sample > 0) tempController.setSamplePeriod(sample);
                if (sense > 0.0) tempController.setSensitivity(sense);
                break;
            }
            case CmdCalTemp: {
                int step = -1;
                float TF;
                cmd->arg(0)->getInt(step);
                cmd->arg(1)->getFloat(TF);
                cmd->ack( tempController.calStep( step, cmd->ID(), TF ) );
                break;
            }
            case CmdRenewRadio:
                rf24.init();
                break;
            case CmdCalConsts: 
                tempController.ackCalConsts( cmd );
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
        if (!respDone) {
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

static uint32_t mesh_timer = 0;


// the loop function runs over and over again forever
void loop() {
  #if DEBUG_CONNECT
  static bool rad_fail = false;
  if (!rad_fail && rf24.getRadio().failureDetected) {
    Serial.println("FAILURE DETECTED IN RADIO");
  }
  static unsigned long last=0;
  if ((millis() - last) > 5000) {
    last = millis();
    Serial.print("UPTIME: ");Serial.print(millis()/1000); Serial.println(" sec");
  }
  #endif
  rf24.update();

  SingleDistanceSensor::cm_type distCM = 0;
  if (distanceSensor.update(distCM) ) {
    //Serial.print("Dist=");
    //Serial.println(distCM);
  }
  floatSwitch.update();
  if (floatSwitch.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Float switch "); Serial.println( floatSwitch.isOn() ? "CLOSED" : "OPEN");
    #endif
  }

  if (pH_probe.update()) {
    #if DEBUG_CHANGES
    Serial.print("pH Meter: "); Serial.println(pH_probe.lastValue());
    #endif
  }
  if (EC_probe.update()) {
    #if DEBUG_CHANGES
    Serial.print("EC Meter: "); 
    for ( int i=0; i < 4; i++ ) 
        Serial.println(EC_probe.lastValue(i)); Serial.print(" ");
    Serial.println("");
    #endif
  }

  tempController.update();

  processCommand();

  #if 1
  for ( int ipump=0; ipump < NPUMPS; ipump++ )
    pumps[ipump]->update();
  #endif

}

