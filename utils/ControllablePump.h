#ifndef CONTROLLABLE_PUMP_H
#define CONTROLLABLE_PUMP_H 1

#include <EEPROM.h>
#include "RF24Interface.h"

class ControllablePump
{
  public:
    ControllablePump(   unsigned char ctrlPin, unsigned char swAddr, unsigned char minPct, unsigned confAddr, RF24IPInterface& rf24 )
      : m_pin(ctrlPin), m_swAddr(swAddr), m_minPct(minPct), m_confAddr(confAddr)
      , m_curSpeed(0), m_topSpeed(0), m_mode(Constant), m_rampSec(1), m_holdSec(10), m_lastChangeTime(0), m_upDown(0)
      , m_rf24(rf24)
    {}

    enum Mode {
        Constant = 0,
        Square   = 1,
        Ramp     = 2,
        NumModes = 3
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
    void setTopSpeed( unsigned char s ) {
        if (s > 100)
            m_topSpeed = 255;
        else if (s < m_minPct)
            m_topSpeed = 0;
        else
            m_topSpeed = (255U * s)/100U;
    }

    void setMode(   Mode m, unsigned char holdSec=0, unsigned rampSec=0 ) {
        m_mode = m;
        m_holdSec = holdSec;
        m_rampSec = rampSec;
        m_upDown = 0;
        m_lastChangeTime = 0;
    }

    void update() {

        unsigned char newSpeed = m_curSpeed;

        // Calculate a new speed in modes that do that.
        unsigned char minSpeed = (m_minPct ? (((m_minPct+1)*255)/100) : 1); // Never actually switch off.
        if (minSpeed > m_topSpeed)
            minSpeed = m_topSpeed;
        unsigned long curTime = millis();
        switch (m_mode) {
            case Square: {
                if ((curTime - m_lastChangeTime) > (m_holdSec * 1000)) {
                    if (m_upDown > 0) {
                        m_upDown = -1;
                        newSpeed = minSpeed;
                    } else {
                        m_upDown = 1;
                        newSpeed = m_topSpeed;
                    }
                }
                break;
            }
            case Ramp: {
                if (m_upDown == 0) {
                    if ((curTime - m_lastChangeTime) > (m_holdSec * 1000)) {
                        m_upDown = (m_curSpeed > ((m_topSpeed-minSpeed)/2)) ? -1 : 1;
                        m_lastChangeTime = curTime;
                    }
                } else {
                    unsigned long delta = ((unsigned long)(m_topSpeed - minSpeed) * (curTime - m_lastChangeTime))/(m_rampSec*1000UL);
                    if (delta) {
                        if (m_upDown > 0) {
                            newSpeed = m_curSpeed + delta;
                            if (newSpeed > m_topSpeed)
                                newSpeed = m_topSpeed;
                        } else if (delta < m_curSpeed) {
                            newSpeed = m_curSpeed - delta;
                            if (newSpeed < minSpeed)
                                newSpeed = minSpeed;
                        } else {
                            newSpeed = minSpeed;
                        }
                        if ((newSpeed==minSpeed) || (newSpeed==m_topSpeed))
                            m_upDown = 0;
                    }
                }
                break;
            }
            case Constant:
            default:
                newSpeed = m_topSpeed;
        }


        // Nothing to do if no change.
        if (newSpeed == m_curSpeed)
            return;

        m_lastChangeTime = curTime;

        // If going from or to 0, turn on or off the switch.
        char cmd[8];
        char* np;
        if (newSpeed == 0) {
            strcpy( cmd, "off " );
        } else if (m_curSpeed == 0) {
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
            
        m_curSpeed = newSpeed;
        analogWrite( m_pin, m_curSpeed );
    }

    static unsigned ee_size() {
        return sizeof(m_mode) + sizeof(m_curSpeed) + sizeof(m_holdSec) + sizeof(m_rampSec);
    }

    void restoreSettings() {
        unsigned addr = m_confAddr;

        EEPROM.get( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.get( addr, m_topSpeed );
        addr += sizeof(m_topSpeed);

        EEPROM.get( addr, m_holdSec );
        addr += sizeof(m_holdSec);

        EEPROM.get( addr, m_rampSec );
        addr += sizeof(m_rampSec);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.put( addr, m_curSpeed );
        addr += sizeof(m_curSpeed);

        EEPROM.put( addr, m_holdSec );
        addr += sizeof(m_holdSec);

        EEPROM.put( addr, m_rampSec );
        addr += sizeof(m_rampSec);
    }
        
  protected:
    unsigned char m_pin;
    unsigned char m_swAddr;
    unsigned char m_minPct;
    unsigned      m_confAddr;
    unsigned char m_curSpeed;
    unsigned char m_topSpeed;
    Mode          m_mode;
    unsigned char m_rampSec;
    unsigned char m_holdSec;
    unsigned long m_lastChangeTime;
    char          m_upDown;
    RF24IPInterface& m_rf24;
};

#endif
