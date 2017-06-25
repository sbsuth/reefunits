/*
 */

#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <SPI.h>
#include <RF24.h>
#include <RF24Mesh.h>
#include <RF24Network.h>
#include <RF24Ethernet.h>
#include <ArduinoJson.h>
#include <DRV8825.h>

#include "Command.h"
#include "DistanceSensor.h"


#define RF24_CE          9
#define RF24_CSN         10
#define TRIG             6
#define ECHO             7
#define FLOAT_SW         8
#define TOP_PUMP         5
#define BOTTOM_PUMP      6
#define RO_SOLENOID      7


#define CALC_DIR        18
#define CALC_STEP       19
#define ALK_DIR         16
#define ALK_STEP        17
#define MAG_DIR         14
#define MAG_STEP        15
#define OLD_OUT_DIR     4
#define OLD_OUT_STEP    5
#define NEW_IN_DIR      2
#define NEW_IN_STEP     3

#define CALC_PUMP   0
#define ALK_PUMP    1
#define MAG_PUMP    2
#define OLD_OUT_PUMP 3
#define NEW_IN_PUMP 4
#define NUM_PUMPS   5


// Debug prints.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1
#define DEBUG_CONNECT 1



// Network objects
RF24 rf24Radio( RF24_CE, RF24_CSN);
RF24Network rf24Network(rf24Radio);
RF24Mesh rf24Mesh(rf24Radio,rf24Network);
RF24EthernetClass RF24Ethernet(rf24Radio,rf24Network,rf24Mesh);
IPAddress myIP(10, 10, 2, 8);
EthernetServer rf24EthernetServer(1000);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 80 );
unsigned short lastDist = 0;


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

// Dosing pumps
DosingPump calcPump(  CALC_DIR,    CALC_STEP);
DosingPump alkPump(   ALK_DIR,     ALK_STEP);
DosingPump magPump(   MAG_DIR,     MAG_STEP );
DosingPump oldOutPump(OLD_OUT_DIR, OLD_OUT_STEP);
DosingPump newInPump( NEW_IN_DIR,  NEW_IN_STEP);

DosingPump* pumps[NUM_PUMPS] = {&calcPump, &alkPump, &magPump, &oldOutPump, &newInPump};

// State vars

bool extPause = false;

// the setup function runs once when you press reset or power the board
void setup() {
    Serial.begin(57600);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif


    digitalWrite( TOP_PUMP, 1 );
    digitalWrite( BOTTOM_PUMP, 1 );
    pinMode( TOP_PUMP, OUTPUT );
    pinMode( BOTTOM_PUMP, OUTPUT );

    floatSwitch.setup();
  
    // Ethernet startup.
    Ethernet.begin(myIP);
    rf24Mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
    IPAddress gwIP(10, 10, 2, 2);
    Ethernet.set_gateway(gwIP);
    rf24EthernetServer.begin();

    // Init pumps.
    for ( int i=0; i < NUM_PUMPS; i++ ) {
        pumps[i]->init();
    }

    extPause = false;

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24EthernetServer );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<700> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["dist"] = lastDist;
    json["float_sw"] = floatSwitch.isOn();
    JsonArray& jsonPumps = json.createNestedArray("pumps");
    for (unsigned char p=0; p < NUM_PUMPS; p++ ) {
        DosingPump* pump = pumps[p];
        JsonObject& jsonPump = jsonBuffer.createObject();
        jsonPump["en"] = pump->enabled();
        jsonPump["is_disp"] = pump->isDispensing();
        jsonPump["tot_disp"] = pump->dispensedMl();
        jsonPump["to_disp"] = pump->toDispenseMl();
        jsonPump["spml"] = pump->stepsPerMl();
        jsonPumps.add( jsonPump );
    }
    cmd->ack( json );
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

            case CmdEnable: {
                // 1 means decr disable, 0 incr disable.
                int en;
                cmd->arg(0)->getInt(en);
                for ( int i=0 ; i < NUM_PUMPS; i++ ) {
                    if (en)
                        pumps[i]->enable();
                    else
                        pumps[i]->disable();
                }
                break;
            }
            case CmdResetAll: {
                for ( int i=0 ; i < NUM_PUMPS; i++ ) {
                    pumps[i]->reset();
                }
                break;
            }
            case CmdDispense: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                int ml;
                cmd->arg(0)->getInt(ml);
                #if DEBUG_CMD
				Serial.print("CHANGE: Got dispense on ");
				Serial.print(p);
                Serial.print(" for ");
                Serial.print(ml);
                Serial.println("ml");
				#endif
                pumps[p]->startDispense(ml);
                break;
            }
            case CmdCal: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                int ml;
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
                int ml;
                cmd->arg(0)->getInt(ml);
                #if DEBUG_CMD
				Serial.print("CHANGE: Got actual for ");
				Serial.print(p);
                Serial.print(" of ");
                Serial.print(ml);
                Serial.println("ml");
				#endif
                pumps[p]->setActualMl(ml);
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
static void renewMesh()
{
    uint32_t now = millis();
    if ((now - mesh_timer) > 30000) {
        mesh_timer  = now;
        if( ! rf24Mesh.checkConnection() ){
            #if DEBUG_CONNECT
            Serial.println(F("Lost connection. Begin nenew.."));
            #endif

            //refresh the network address
            rf24Mesh.renewAddress();

            #if DEBUG_CONNECT
            Serial.print(F("Connection renewal:"));
            Serial.println( rf24Mesh.checkConnection() );
            #endif
        }  else {
            #if DEBUG_CONNECT
            Serial.print(F("Connection good."));
            #endif
        }
    }
}


// the loop function runs over and over again forever
void loop() {
  renewMesh();
  unsigned short dist = distanceSensor.update();
  if (dist != lastDist) {
    Serial.print("Dist=");
    Serial.println(dist);
    lastDist = dist;
  }
  floatSwitch.update();
  if (floatSwitch.changed()) {
    #if DEBUG_CHANGES
    Serial.print("Float switch ");
    Serial.println( floatSwitch.isOn() ? "CLOSED" : "OPEN");
    #endif
  }
  for ( int i=0 ; i < NUM_PUMPS; i++ ) {
    pumps[i]->update();
  }
  
  processCommand();
}

