#ifndef CONTROLLABLE_PUMP_H
#define CONTROLLABLE_PUMP_H 1

#include <EEPROM.h>
#include "RF24Interface.h"

class ControllablePump
{
  public:
    ControllablePump(   unsigned char ctrlPin, unsigned char swAddr, unsigned char minPct, unsigned confAddr, RF24IPInterface& rf24 )
      : m_pin(ctrlPin), m_swAddr(swAddr), m_minPct(minPct), m_confAddr(confAddr)
      , m_curSpeed(0), m_newSpeed(0), m_mode(Simple)
      , m_rf24(rf24)
    {}

    enum Mode {
        Simple,
        Square,
        Ramp
    };

    void setup( bool useSettings ) {
      pinMode( m_pin, OUTPUT );
      analogWrite( m_pin, 0 );
      if (useSettings) {
        restoreSettings();
      } else {
        saveSettings();
      }
    }

    // Accepts pct.  Stored as 0-255.
    void setSpeed( unsigned char s ) {
        if (s > 100)
            m_newSpeed = 255;
        else if (s < m_minPct)
            m_newSpeed = 0;
        else
            m_newSpeed = (255U * s)/100U;
    }

    void update() {
        if (m_newSpeed == m_curSpeed)
            return;

        // If going from or to 0, turn on or off the switch.
        char cmd[8];
        char* np;
        if (m_newSpeed == 0) {
Serial.print("Turning off switch ");Serial.println(m_swAddr);
            strcpy( cmd, "off " );
        } else if (m_curSpeed == 0) {
Serial.print("Turning on switch ");Serial.println(m_swAddr);
            strcpy( cmd, "on " );
        } else {
            cmd[0] = 0;
        }
        if (cmd[0]) {
            char* pc = &cmd[0];
            while (*pc) pc++;
            *pc++ = '0' + m_swAddr;
            *pc++ = '\n';
            *pc++ = 0;
            m_rf24.sendToRadioClient( 5, cmd, 3 );
        }
            
Serial.print("Setting new speed: "); Serial.println(m_newSpeed);
        analogWrite( m_pin, m_newSpeed );

        m_curSpeed = m_newSpeed;
    }

    static unsigned ee_size() {
        return sizeof(m_mode) + sizeof(m_curSpeed);
    }

    void restoreSettings() {
        unsigned addr = m_confAddr;

        EEPROM.get( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.get( addr, m_newSpeed );
        addr += sizeof(m_newSpeed);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.put( addr, m_curSpeed );
        addr += sizeof(m_curSpeed);
    }
        
  protected:
    unsigned char m_pin;
    unsigned char m_swAddr;
    unsigned char m_minPct;
    unsigned      m_confAddr;
    unsigned char m_curSpeed;
    unsigned char m_newSpeed;
    Mode          m_mode;
    unsigned char m_initSpeed;
    RF24IPInterface& m_rf24;
};

#endif
