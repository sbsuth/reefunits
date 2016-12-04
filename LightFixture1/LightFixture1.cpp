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


#define UP_LED_OUT   	 5 
#define UP_BUTTON_IN 	 9
#define DOWN_LED_OUT 	 4 
#define DOWN_BUTTON_IN 2
#define SPEED_PWM      6
#define CURRENT_LED_OUT 3
#define CURRENT_IN    A0

// Debug prints.
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1




// Device status/.
bool goingUp = false;
bool goingDown = false;
AvgThresh<16> liftCurrent(518,502,525);

static Switch upButton( UP_BUTTON_IN );
static Switch downButton( DOWN_BUTTON_IN );


// Network objects
RF24 rf24Radio(7, 8);
RF24Network rf24Network(rf24Radio);
RF24Mesh rf24Mesh(rf24Radio,rf24Network);
RF24EthernetClass RF24Ethernet(rf24Radio,rf24Network,rf24Mesh);
IPAddress myIP(10, 10, 2, 4);
EthernetServer rf24EthernetServer(1000);


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  Serial.println(F("Start"));
  
  pinMode( UP_LED_OUT, OUTPUT);
  pinMode( DOWN_LED_OUT, OUTPUT);
  pinMode( SPEED_PWM, OUTPUT);
  pinMode( CURRENT_LED_OUT, OUTPUT);

  // Do these after network.being() to avoid mysterious interference....
  upButton.setup();
  downButton.setup();
  
  // Ethernet startup.
  Ethernet.begin(myIP);
  rf24Mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
  IPAddress gwIP(10, 10, 2, 2);
  Ethernet.set_gateway(gwIP);
  rf24EthernetServer.begin();
  
  Serial.println(F("Ready"));
}

static long lastTimeUp = 0;
static long lastTimeDown = 0;

void changeUp( bool newUp )
{
    goingDown = false;
    if (newUp != goingUp) {
        goingUp = newUp;
        #if DEBUG_CHANGES
        Serial.print(F("Change goingUp to "));
        Serial.println(goingUp);
        #endif
        if (newUp)
            lastTimeUp = millis();
    }
}

void changeDown( bool newDown )
{
    goingUp = false;
    if (newDown != goingDown) {
        goingDown = newDown;
        #if DEBUG_CHANGES
        Serial.print(F("Change goingDown to "));
        Serial.println(goingDown);
        #endif
        if (newDown)
            lastTimeDown = millis();
    }
}

static long curHeight = 0;
static bool fixtureRunning = false;

static void updateFixtureRunning()
{
  #if 1
  bool old = liftCurrent.isInRange();
  int newCurrent = analogRead(CURRENT_IN);
  liftCurrent.update( newCurrent );
  fixtureRunning = !liftCurrent.isInRange();
  if ( old != liftCurrent.isInRange() ) {
    #if DEBUG_CHANGES
    Serial.print(F("Fixture running: "));
    Serial.println(fixtureRunning);
    #endif
  }
  #else
  // Debug mode with fixture unplugged.
  fixtureRunning = (goingUp || goingDown);
  #endif
}
void updateHeight()
{
    long upPctPerSec = 4;
    long downPctPerSec = 5;
    long waitForStart = 500;
    long curTime = millis();
    if (goingUp) {
        long incr = (curTime - lastTimeUp);
        if (!fixtureRunning) {
            if (incr > waitForStart) {
                // Reached top.
                curHeight = 100000;
                #if DEBUG_CHANGES
                Serial.println(F("Reached top"));
                #endif
                changeUp(false);
            }
        } else {
            curHeight += (incr * upPctPerSec);
            if (curHeight > (100000))
                curHeight = 100000;
            lastTimeUp = curTime;
        }
    } else if (goingDown) {
        long incr = (curTime - lastTimeDown);
        if (!fixtureRunning) {
            if (incr > waitForStart) {
                // Reached bottom
                curHeight = 0;
                #if DEBUG_CHANGES
                Serial.println(F("Reached bottom"));
                #endif
                changeDown(false);
            }
        } else {
            curHeight -= (incr * downPctPerSec);
            if (curHeight < 0)
                curHeight = 0;
            lastTimeDown = curTime;
        }
    }
}

//static RF24SerialIO rfs( &rf24Network );
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24EthernetServer );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command serialCmd;
static Command rf24Cmd;

// Ack with the current height.
//  height: int
//  moving: 1/0
static void getHeightCmd( Command* cmd )
{
    StaticJsonBuffer<200> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["height"] = curHeight / 1000;
    json["moving"] = (int)((goingUp || goingDown) ? 1 : 0);
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
            case CmdStop:
                changeUp( false );
                changeDown( false );
                break;
            case CmdDown:  {
                changeDown( true );
                break;
            }
            case CmdUp: {
                int pct;
                cmd->arg(0)->getInt(pct);
                changeUp( true );
                break;
            }
            case CmdCalibrate:
                break;
            case CmdGetHeight:
                getHeightCmd( cmd );
                needResp = false;
                break;
            case CmdRFState:
                rf24EthernetServer.dumpstate();
                break;
            default:
                Serial.println(F("Unrecognized cmd\n"));
                break;
        }
        if (needResp) {
            cmd->ack();
        }
        cmd->parser()->reset();
    } else if (error) {
        Serial.println(F("Error in command\n"));
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
            rf24Mesh.renewAddress();

            Serial.print(F("Connection renewal:"));
            Serial.println( rf24Mesh.checkConnection() );
        } 
    }
}

// the loop function runs over and over again forever
void loop() {
  // This is causing a hang I don't understand...  Happens at radio.checkConnection().
  renewMesh();
  upButton.update();
  downButton.update();

  if (upButton.changed() && upButton.isOn()) {
    changeUp( !goingUp );
  }
  if (downButton.changed() && downButton.isOn()) {
    changeDown( !goingDown );
  }
  updateFixtureRunning();
  updateHeight();

  processCommand();

  digitalWrite(UP_LED_OUT, goingUp);  
  digitalWrite(DOWN_LED_OUT, goingDown);
  digitalWrite(SPEED_PWM, (goingUp | goingDown));
  digitalWrite(CURRENT_LED_OUT, fixtureRunning);
}

