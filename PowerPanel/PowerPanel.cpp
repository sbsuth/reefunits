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
#include "relays.h"


#define RF24_CE          14
#define RF24_CSN         15


// Debug prints.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1
//#define DEBUG_CONNECT 1



// Network objects
RF24 rf24Radio( RF24_CE, RF24_CSN);
RF24Network rf24Network(rf24Radio);
RF24Mesh rf24Mesh(rf24Radio,rf24Network);
RF24EthernetClass RF24Ethernet(rf24Radio,rf24Network,rf24Mesh);
IPAddress myIP(10, 10, 2, 5);
EthernetServer rf24EthernetServer(1000);

// Relay
unsigned char relayPins[8] = {6, 5, 7, 4, 3, 9, 2, 10};
Relays<8> relays( relayPins );



// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  #if DEBUG_STARTUP
  Serial.println(F("Start"));
  #endif

  relays.init();
  
  // Ethernet startup.
  Ethernet.begin(myIP);
  rf24Mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
  IPAddress gwIP(10, 10, 2, 2);
  Ethernet.set_gateway(gwIP);
  rf24EthernetServer.begin();
  
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
    JsonArray& on = json.createNestedArray("on");
    for (unsigned char p=0; p < 8; p++ ) {
        on.add( relays.isOn(p) );
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
            //refresh the network address
            #if DEBUG_CONNECT
            Serial.println(F("Start reconnect..."));
            #endif
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

  processCommand();
}

