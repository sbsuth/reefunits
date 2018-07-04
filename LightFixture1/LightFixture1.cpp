/*
 */

#include <Arduino.h>
#include "Switch.h"
#include "Avg.h"

#include <EEPROM.h>
#include <SPI.h>
#include "RF24Interface.h"
#include <ArduinoJson.h>

#include "leds.h"

#include "Command.h"


//#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
// Uno
//#define UP_LED_OUT   	 5 
//#define UP_BUTTON_IN 	 9
//#define DOWN_LED_OUT 	 4 
//#define DOWN_BUTTON_IN   2
//#define SPEED_PWM        6
//#define CURRENT_LED_OUT  3
//#define RF24_CE          7
//#define RF24_CSN         8
//#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
// Mega
#define UP_LED_OUT   	 27 
#define UP_BUTTON_IN 	 35
#define DOWN_LED_OUT 	 31 
#define DOWN_BUTTON_IN   33
#define SPEED_PWM        4
#define CURRENT_LED_OUT  29
#define BRIGHT_BUTTON_IN 49
#define DIM_BUTTON_IN    47
#define RF24_CE          25
#define RF24_CSN         23

//#else
//#error "Not an expected Arduino!"
//#endif
#define CURRENT_IN      A0

// Debug prints.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1
//#define DEBUG_CONNECT 1

#define LED_EE_ADDR 16


// Device status/.
bool goingUp = false;
bool goingDown = false;
AvgThresh<16,int> liftCurrent(500,495,515);

static Switch upButton( UP_BUTTON_IN );
static Switch downButton( DOWN_BUTTON_IN );


// Network objects
RF24IPInterface rf24( 7, RF24_CE, RF24_CSN, RF24_PA_LOW );
DEFINE_RF24IPInterface_STATICS(rf24);

// Leds
Leds leds(LED_EE_ADDR);

#define EE_SIG (unsigned char)123

// Set the prescale values on the timers:
// Avoid doing timer 0.
void setupTimers()
{
    // On mega for phase correct mode:
    //  1 : 31 kHz
    //  2 : 3.9 kHz
    //  3 : 490 Hz (default)
    //  4 : 30 Hz
    //  5 : less
    //
    // For fast mode, ~double these.
    //
    // Timers for pins:
    //  #0 : pins 4, 13
    //  #1 : pins 11, 12
    //  #2 : pins 9, 10
    //  #3:  pins 2, 3, 5
    //  #4:  pins 6, 7, 8
    //
    // I use pins 2->8, so timers 2, 3, 4.
    //int iScale = 1;
    int iScale = 4;
    PRESCALE_TIMER( 2, iScale );
    PRESCALE_TIMER( 3, iScale );
    PRESCALE_TIMER( 4, iScale );
}

static void saveSettings()
{
    EEPROM.put( (char)0, EE_SIG );
    leds.saveSettings();
}

static bool restoreSettings()
{
    unsigned char sig = 0;
    EEPROM.get( 0, sig );
    if (sig == EE_SIG) {
        leds.restoreSettings();
        return true;
    } else {
        return false;
    }
}



// the setup function runs once when you press reset or power the board
void setup() {
  #if DEBUG_STARTUP
  Serial.begin(115200);
  Serial.println(F("Start"));
  #endif

  setupTimers();
  
  pinMode( UP_LED_OUT, OUTPUT);
  pinMode( DOWN_LED_OUT, OUTPUT);
  pinMode( SPEED_PWM, OUTPUT);
  pinMode( CURRENT_LED_OUT, OUTPUT);


  leds.init();

  if (!restoreSettings())
    saveSettings();

  // Do these after network.being() to avoid mysterious interference....
  upButton.setup();
  downButton.setup();
  
  // Ethernet startup.
  rf24.init();
  
  #if DEBUG_STARTUP
  Serial.println(F("Ready"));
  #endif
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
  int newAvg = liftCurrent.update( newCurrent );
  fixtureRunning = !liftCurrent.isInRange();
  if ( old != liftCurrent.isInRange() ) {
    #if DEBUG_CHANGES
    //Serial.print(F("Fixture running: "));
    //Serial.print(newAvg);
    //Serial.print(F(": "));
    //Serial.println(fixtureRunning);
    #endif
  }
  #else
  // Debug mode with fixture unplugged.
  fixtureRunning = (goingUp || goingDown);
  #endif
}
void updateHeight()
{
    long upPctPerSec = 42;
    long downPctPerSec = 50;
    long waitForStart = 1000;
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
            curHeight += (incr * upPctPerSec)/10;
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
            curHeight -= (incr * downPctPerSec)/10;
            if (curHeight < 0)
                curHeight = 0;
            lastTimeDown = curTime;
        }
    }
}

//static RF24SerialIO rfs( &rf24Network );
static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static EthernetSerialIO rfs( &rf24 );
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

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<300> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["height"] = curHeight / 1000;
    json["moving"] = (int)((goingUp || goingDown) ? 1 : 0);
    json["lat"] = leds.getLatitude();
    json["lon"] = leds.getLongitude();
    json["tz"] = leds.getTimezone();
    json["high_pct"] = leds.getHighPct();
    json["low_pct"] = leds.getLowPct();
    json["timed_pct"] = leds.getTimedPct();
    json["mode"] = (int)leds.getMode();
    json["spec"] = leds.getCurSpectrum();
    json["sun_angle"] = leds.getSunAngle();
    json["am_factor"] = leds.getAMFactor();
    json["offset_sec"] = leds.getOffsetSec();
    cmd->ack( json );
    #if 0
    https://arduinojson.org/v5/assistant/
    277
    {
      "height":123,
      "moving":true,
      "lat": 1.234,
      "lon": 1.234,
      "tz": -8,
      "high_pct": 47,
      "low_pct": 47,
      "timed_pct": 58,
      "mode": 0,
      "spec": 0,
      "sun_angle": 45.87,
      "am_factor": 0.78,
      "offset_sec": 0
    }
    #endif
}

static void getCurVals( Command* cmd )
{
    StaticJsonBuffer<128> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    JsonArray& cur = json.createNestedArray("cur_pct");
    for (unsigned char iled=0; iled < NUM_LED_NUMS; iled++ ) {
        if ( leds.ledIndexUsed(iled)) {
            cur.add( leds.getCurVal(iled) );
        }
    }
    cmd->ack( json );

    #if 0
    https://arduinojson.org/v5/assistant/
    110
    {
      "cur_pct":[123,123,123,123,234,234,123],
    }
    #endif

}

static void getMaxPcts( Command* cmd, unsigned spec )
{
    StaticJsonBuffer<128> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    JsonArray& max = json.createNestedArray("max_pct");
    for (unsigned char iled=0; iled < NUM_LED_NUMS; iled++ ) {
        if ( leds.ledIndexUsed(iled)) {
            max.add( leds.getLedPct(spec,iled) );
        }
    }
    cmd->ack( json );

    #if 0
    https://arduinojson.org/v5/assistant/
    110
    {
      "max_pct":[123,123,123,123,234,234,123]
    }
    #endif

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
                rf24.getServer().dumpstate();
                break;
            case CmdDim: {
                int pct;
                if (cmd->arg(0)->getInt(pct)) {
                    leds.setMode( Leds::External );
                    leds.dimAll(pct);
                }
                break;
            }
            case CmdSetLoc: {
                // Latitude, longitude, timezone.
                float l;
                int tz;
                if (cmd->arg(0)->getFloat(l))
                    leds.setLatitude(l);
                if (cmd->arg(1)->getFloat(l))
                    leds.setLongitude(l);
                if (cmd->arg(2)->getInt(tz))
                    leds.setTimezone(tz);

                leds.invalidate();
                break;
            }

            case CmdSetTime: {
                // Read 32-bit unsigned, high word, then low word.
                unsigned time_h;
                unsigned time_l;
                cmd->arg(0)->getUnsigned(time_h);
                cmd->arg(1)->getUnsigned(time_l);
                unsigned long t = (((unsigned long)time_h) << 16) | time_l;
                leds.setTime(t);

                leds.invalidate();
                break;
            }
            case CmdStat:        {
                getStatus(cmd);
                break; 
            }
            case CmdGetCurVals:   {
                getCurVals(cmd);
                break; 
            }
            case CmdGetMaxPcts:   {
                getMaxPcts(cmd, cmd->ID());
                break; 
            }
            case CmdSetMaxPct:    {
                int ichan;
                int pct;
                cmd->arg(0)->getInt(ichan);
                if ((ichan < 0) || (ichan >= NUM_LED_NUMS) || SKIP_LED_NUM(ichan))
                    break;
                cmd->arg(1)->getInt(pct);
                if ((pct < 0) || (pct > 100))
                    break;
                
                leds.setLedPct( cmd->ID(), ichan, pct );

                leds.pushVals();

                break;  
            }
            case CmdSetLevel:   {
                int which = cmd->ID();
                int pct;

                cmd->arg(0)->getInt(pct);
                if ((pct < 0) || (pct > 100))
                    break;

                if (which == 0)
                    leds.setHighPct(pct);
                else
                    leds.setLowPct(pct);

                leds.invalidate();

                break; 
            }
            case CmdSetMode: {
                // Mode, spectrum.
                int mode;
                int spec;
                if ((mode >= 0) && (mode < Leds::NumModes))
                    leds.setMode( (Leds::Mode)mode );

                cmd->arg(1)->getInt(spec);
                if ((spec >= 0) && (spec < NUM_SPECTRUMS))
                    leds.setCurSpectrum(spec);

                leds.invalidate();

                break;
            }
            case CmdGetEdges:    {
                break;
            }
            case CmdSaveSettings: {
                saveSettings();
                break;
            } 
            case CmdRestoreSettings: {
                restoreSettings();
                leds.invalidate();
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
        cmd->disconnect();
    } else if (error) {
        #if DEBUG_CMD
        Serial.println(F("Error in command\n"));
        #endif
    }
}


// the loop function runs over and over again forever
void loop() {
  rf24.update();
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
  leds.update();

  processCommand();

  digitalWrite(UP_LED_OUT, goingUp);  
  digitalWrite(DOWN_LED_OUT, goingDown);
  digitalWrite(SPEED_PWM, (goingUp | goingDown));
  digitalWrite(CURRENT_LED_OUT, fixtureRunning);
}

