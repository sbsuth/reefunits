/*
 */

// These are above #includes so they can affect utilities in headers.
#define DEBUG_STARTUP 1
#define DEBUG_CHANGES 1
#define DEBUG_CMD 1

#define RF24_RE_INIT_AFTER_NUM_EMPTY 2

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "RF24Interface.h"

#include "Command.h"

#define RF24_CE          6
#define RF24_CSN         7

#define SPEED_PIN_0 9
#define SPEED_PIN_1 10
#define NUM_PUMPS 2

// Network.
RF24IPInterface rf24( 8, RF24_CE, RF24_CSN );
DEFINE_RF24IPInterface_STATICS(rf24);

class Pump {
  public:
    Pump( int pin ) 
        : m_pin(pin), m_on(false), m_speed(0), m_last_speed(1)
    {}
    void setup() {
       pinMode( m_pin, OUTPUT ); 
    }
    void setSpeed( int pct ) {
        if (pct > 100)
            pct = 100;
        else if (pct < 0)
            pct = 0;
        m_speed = 255 - (((unsigned long)pct * 255)/100);
    }
    void setOn( bool onOff ) {
        m_on = onOff;
    }
    void update() {
        unsigned value = m_on ? m_speed : 255;
        analogWrite( m_pin, value );
        if (value != m_last_speed) {
            Serial.print("Set speed for ");Serial.write(m_pin);Serial.print(" to ");Serial.println(value);
            m_last_speed = value;
        }
    }
  protected:
    unsigned char m_pin;
    unsigned char m_on;
    unsigned char m_speed;
    unsigned char m_last_speed;
};

Pump pump0(SPEED_PIN_0);
Pump pump1(SPEED_PIN_1);
Pump* pumps[NUM_PUMPS] = {&pump0, &pump1};

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
    // Timers for UNO pins:
    //  #0 : pins 5, 6
    //  #1 : pins 9, 10
    //  #2 : pins 3, 11
    //
    //int iScale = 2; // Smooth, but whines
    int iScale = 1; // Quiet, but out of spec rance of 20kHz
    PRESCALE_TIMER( 1, iScale );
}


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
    #if DEBUG_STARTUP
    Serial.println(F("Start"));
    #endif

    setupTimers();

    rf24.init();

    for ( int i=0; i < NUM_PUMPS; i++ ) {
        pumps[i]->setup();
    }

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static ArduinoSerialIO sis;
static CommandParser serialParser( g_commandDescrs, &sis );
static Command serialCmd;

static EthernetSerialIO rfs( &rf24 );
static CommandParser rf24Parser( g_commandDescrs, &rfs );
static Command rf24Cmd;

void processCommand()
{

    int arg;
    int p;
    int i;
    bool error = false;
    Command* cmd = 0;
    cmd=serialParser.getCommand( &serialCmd, error );
    if (!cmd)
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

            case CmdPumpOn: {
				int pump = cmd->ID();
                if ((pump < 0) || (pump >= NUM_PUMPS))
                    break;
                int onOff = 0;
				cmd->arg(0)->getInt(onOff);
                pumps[pump]->setOn( onOff );
                Serial.print("Pump #");Serial.print(pump);Serial.print(", on=");Serial.println(onOff);
                break;
            }
            case CmdPumpSpeed: {
				int pump = cmd->ID();
                if ((pump < 0) || (pump >= NUM_PUMPS))
                    break;
                int speed = 0;
				cmd->arg(0)->getInt(speed);
                pumps[pump]->setSpeed( speed );
                Serial.print("Pump #");Serial.print(pump);Serial.print(", speed=");Serial.println(speed);
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



// the loop function runs over and over again forever
void loop() {

  rf24.update();

  processCommand();

  for ( int i=0; i < NUM_PUMPS; i++ ) {
    pumps[i]->update();
  }
}

