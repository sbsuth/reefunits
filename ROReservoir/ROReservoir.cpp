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

#include "Command.h"
#include "DistanceSensor.h"


#define RF24_CE          9
#define RF24_CSN         10
#define TRIG             3
#define ECHO             4
#define FLOAT_SW         2
#define TOP_PUMP         5
#define BOTTOM_PUMP      6
#define RO_SOLENOID      7


// Debug prints.
#define DEBUG_STARTUP 0
#define DEBUG_CHANGES 0
#define DEBUG_CMD 0
#define DEBUG_CONNECT 0



// Network objects
RF24 rf24Radio( RF24_CE, RF24_CSN);
RF24Network rf24Network(rf24Radio);
RF24Mesh rf24Mesh(rf24Radio,rf24Network);
RF24EthernetClass RF24Ethernet(rf24Radio,rf24Network,rf24Mesh);
IPAddress myIP(10, 10, 2, 6);
EthernetServer rf24EthernetServer(1000);

// Ultrasonic
SingleDistanceSensor distanceSensor( TRIG, ECHO, 40 );


// Float switch
Switch floatSwitch( FLOAT_SW, LOW );

// State vars

enum ROMode {
    ROForceOff = 0,
    ROForceOn = 1,
    ROKeepFull = 2
};

ROMode roMode = ROForceOff;
bool extPause = false;
bool targetReady[2] = {false,false};

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

    digitalWrite( RO_SOLENOID, 0 );
    pinMode( RO_SOLENOID, OUTPUT );

    floatSwitch.setup();
  
    // Ethernet startup.
    Ethernet.begin(myIP);
    rf24Mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
    IPAddress gwIP(10, 10, 2, 2);
    Ethernet.set_gateway(gwIP);
    rf24EthernetServer.begin();

    // Initial state.
    roMode = ROForceOff;
    extPause = false;
    targetReady[0] = targetReady[1] = false;

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

//static RF24SerialIO rfs( &rf24Network );
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24EthernetServer );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<200> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
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

            case CmdAllOff:
                digitalWrite( TOP_PUMP, 1 );
                digitalWrite( BOTTOM_PUMP, 1 );
                break;

            case CmdPumpOn: {
                int p;
                cmd->arg(0)->getInt(p);
                digitalWrite( p ? BOTTOM_PUMP : TOP_PUMP, 0 );
                break;
            }
            case CmdPumpOff: {
                int p;
                cmd->arg(0)->getInt(p);
                digitalWrite( p ? BOTTOM_PUMP : TOP_PUMP, 1 );
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

// Turn on or off the solenoid based on state.
void updateROOn()
{
    int onOff = 0;

    if (extPause) {
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

unsigned short lastDist = 0;

// the loop function runs over and over again forever
void loop() {
  //renewMesh();
  unsigned short dist = distanceSensor.update();
  if (dist != lastDist) {
    //Serial.print("Dist=");
    //Serial.println(dist);
    lastDist = dist;
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
}

