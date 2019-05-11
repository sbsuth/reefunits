/*
 */

// These are above #includes so they can affect utilities in headers.
//#define DEBUG_STARTUP 1
//#define DEBUG_CHANGES 1
//#define DEBUG_CMD 1

#define RF24_RE_INIT_AFTER_NUM_EMPTY 2

#include <Arduino.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include "RF24Interface.h"

#include "Command.h"

#define RF24_CE          6
#define RF24_CSN         7

#define LED_PIN 2

#define SPEED_PIN_0 9
#define SPEED_PIN_1 10
#define NUM_PUMPS 2

#define EE_SIG 125
#define SIG_EE_ADDR         0
#define SIG_EE_SIZE         sizeof(unsigned char)

#define PUMP_EE_ADDR        SIG_EE_ADDR + SIG_EE_SIZE
#define PUMP_EE_ADDR_I(i)   (PUMP_EE_ADDR + (i * Pump::ee_size()))
#define PUMP_EE_SIZE        (NUM_PUMPS*Pump::ee_size())


#define DEFAULT_MS_PER_ML (70UL*1000UL)
#define MAX_MS_PER_ML (150UL*1000UL)

// Network.
RF24IPInterface rf24( 9, RF24_CE, RF24_CSN );
DEFINE_RF24IPInterface_STATICS(rf24);

class Pump {
  public:
    Pump( int pin, unsigned index ) 
        : m_pin(pin), m_index(index), m_on(false), m_speed(0), m_last_speed(255)
        , m_ms_per_ml(DEFAULT_MS_PER_ML), m_disp_start(0), m_disp_ml10(0)
    {}
    void setup( bool useSettings ) {
       pinMode( m_pin, OUTPUT ); 
        if (useSettings) {
            restoreSettings();
        } else {
            saveSettings();
        }
        setSpeed(20);
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
        m_disp_start = 0;
        m_disp_ml10 = 0;
    }
    bool isOn() {
        return m_on;
    }
    void update() {
        if (m_on) {
            // If there are params set, then turn off if limits reached.
            // If manually on, just leave it on.
            if (m_disp_start && m_disp_ml10 && m_ms_per_ml) {
                 unsigned long dt = (millis() - m_disp_start);
                 long ml10_pumped = (dt * 10)/ m_ms_per_ml;
                 if ((ml10_pumped >= m_disp_ml10) || (ml10_pumped < 0)) {
                    m_on = false;
                    m_disp_end = millis();
                }
            }
        }
        unsigned value;
        if (m_on) {
            value = m_speed;
        } else {
            value = 255;
            m_disp_ml10 = 0;
        }
        analogWrite( m_pin, value );

        if (value != m_last_speed) {
            #if DEBUG_CHANGES
            Serial.print(F("Set speed for "));Serial.write(m_pin);Serial.print(F(" to "));Serial.println(value);
            #endif
            m_last_speed = value;
        }
    }

    // Dispense the given number of 10ths of ml.
    bool startDispense( unsigned short ml10 ) {
        m_disp_start = millis();
        m_disp_ml10 = ml10;
        if (ml10 > 0)
            m_on = true;
    }

    void startCal() {
        startDispense(calMl10());
    }
    // Value is in tenths of ml.
    void setActualMl( unsigned short ml10 ) {
        if (m_disp_start && m_disp_end && (m_disp_end > m_disp_start) && ml10) {
            unsigned long t = m_disp_end - m_disp_start;
            m_ms_per_ml = (t * 10UL)/ (unsigned long)ml10;
        }
    }

    static unsigned ee_size() {
        return sizeof(m_ms_per_ml);
    }

    void restoreSettings() {
        unsigned addr = PUMP_EE_ADDR_I(m_index);

        EEPROM.get( addr, m_ms_per_ml );
        addr += sizeof(m_ms_per_ml);

        // Never allow m_ms_per_ml to be garbage.
        if ((m_ms_per_ml == 0) || (m_ms_per_ml > MAX_MS_PER_ML))
            m_ms_per_ml = DEFAULT_MS_PER_ML;
    }

    void saveSettings() {
        unsigned addr = PUMP_EE_ADDR_I(m_index);

        EEPROM.put( addr, m_ms_per_ml );
        addr += sizeof(m_ms_per_ml);
    }
    unsigned short calMl10() {
        return 100;
    }

  protected:
    unsigned char m_pin;
    unsigned char m_index;
    unsigned char m_on;
    unsigned char m_speed;
    unsigned char m_last_speed;
    unsigned long m_ms_per_ml;
    unsigned long m_disp_start;
    unsigned long m_disp_end;
    unsigned m_disp_ml10;
};

Pump pump0(SPEED_PIN_0,0);
Pump pump1(SPEED_PIN_1,1);
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

void updateLed()
{
    static long last = 0;
    static bool on = false;
    if ((millis() - last) > 1000) {
        on = !on;
        last = millis();
        digitalWrite( LED_PIN, on ? HIGH : LOW );
    }
}


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
    EEPROM.put( SIG_EE_ADDR, (char)EE_SIG );

    setupTimers();

    pinMode( LED_PIN, OUTPUT);

    rf24.init();

    for ( int i=0; i < NUM_PUMPS; i++ ) {
        pumps[i]->setup(useSettings);
    }

    #if DEBUG_STARTUP
    Serial.println(F("Ready"));
    #endif
}

static void getStatus( Command* cmd )
{
    StaticJsonBuffer<200> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    JsonArray& on = json.createNestedArray("on");
    int numActive = 0;
    for ( int i=0; i < NUM_PUMPS; i++ ) {
        if ( pumps[i]->isOn() ) 
            numActive++;
        on.add( pumps[i]->isOn() );
    }
    json["num_active"] = numActive;
    cmd->ack( json );
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
        Serial.print(F("Command: ")); cmd->printName(); Serial.println("");
        #endif
        switch ( cmd->kind() ) {
            case CmdNone:
                break;

            case CmdPing:
                break;

            case CmdStatus:
                getStatus(cmd);
                break;

            case CmdPumpOn: {
				int pump = cmd->ID();
                if ((pump < 0) || (pump >= NUM_PUMPS))
                    break;
                int onOff = 0;
				cmd->arg(0)->getInt(onOff);
                pumps[pump]->setOn( onOff );
                #if DEBUG_CMD
                Serial.print(F("Pump #"));Serial.print(pump);Serial.print(F(", on="));Serial.println(onOff);
                #endif
                break;
            }
            case CmdPumpSpeed: {
				int pump = cmd->ID();
                if ((pump < 0) || (pump >= NUM_PUMPS))
                    break;
                int speed = 0;
				cmd->arg(0)->getInt(speed);
                pumps[pump]->setSpeed( speed );
                #if DEBUG_CMD
                Serial.print("Pump #");Serial.print(pump);Serial.print(F(", speed="));Serial.println(speed);
                #endif
                break;
            }
            case CmdDispense: {
                p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                cmd->arg(0)->getInt(arg);
                #if DEBUG_CMD
				Serial.print(F("CHANGE: Got dispense on ")); Serial.print(p); Serial.print(F(" for ")); Serial.print(arg); Serial.println("ml");
				#endif
                pumps[p]->startDispense(arg);
                break;
            }
            case CmdCal: {
                int p = cmd->ID();
                if ((p >= NUM_PUMPS) || (p < 0))
                    break;
                #if DEBUG_CMD
				Serial.print("CHANGE: Got cal on "); Serial.println(p);
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
				Serial.print(F("CHANGE: Got actual for ")); Serial.print(p); Serial.print(" of "); Serial.print(arg); Serial.println("ml/10");
				#endif
                pumps[p]->setActualMl(arg);
                pumps[p]->saveSettings();
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

  updateLed();
}

