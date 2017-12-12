#ifndef CONTROLLABLE_PUMP_H
#define CONTROLLABLE_PUMP_H 1

#include <EEPROM.h>
#include "RF24Interface.h"

class ControllablePump
{
  public:
    ControllablePump(   unsigned char ctrlPin, unsigned char swAddr, unsigned char minPct, unsigned confAddr, RF24IPInterface& rf24 )
      : m_pin(ctrlPin), m_swAddr(swAddr), m_minPct(minPct), m_confAddr(confAddr)
      , m_curSpeed(0), m_topSpeed(0), m_mode(Off)
      , m_rampSec(1), m_holdSec(10), m_rampRange(0), m_holdRange(0), m_rampOffset(0), m_holdOffset(0)
      , m_lastChangeTime(0), m_tempShutoffUntil(0), m_upDown(0)
      , m_rf24(rf24)
    {}

    enum Mode {
        Constant = 0,
        Square   = 1,
        Ramp     = 2,
        Off      = 4,
        NumModes = 4
    };

    const unsigned ReassertSwitchInterval = 10000;

    void setup( bool useSettings ) {
      pinMode( m_pin, OUTPUT );
      analogWrite( m_pin, 0 );
      if (useSettings) {
        restoreSettings();
      } else {
        saveSettings();
      }
    }
    unsigned char getCurSpeed() {
        return m_curSpeed;
    }
    unsigned char getTopSpeed() {
        return m_topSpeed;
    }
    Mode          getMode() {
        return m_mode;
    }
    unsigned char getRampSec() {
        return m_rampSec;
    }
    unsigned char getHoldSec() {
        return m_holdSec;
    }
    unsigned char getRampRange() {
        return m_rampRange;
    }
    unsigned char getHoldRange() {
        return m_holdRange;
    }
    unsigned long getLastChangeTime() {
        return m_lastChangeTime;
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

    void setMode(   Mode m, float holdArg, float rampArg=0.0 ) {

        m_mode = m;

        // times are set as "sec.pct" where pct gives the random range
        // as a percentage of 'sec', and where 'sec' is the centerpoint of the range.
        float sec = int(holdArg);
        float pct = (holdArg - sec);
        m_holdSec = sec;
        m_holdRange = int((pct * (float)sec) * 1000.0); // ms

        sec = int(rampArg);
        pct = (rampArg - sec);
        m_rampSec = sec;
        m_rampRange = int((pct * (float)sec) * 1000.0); // ms

        m_upDown = 0;
        m_lastChangeTime = 0;
        #if DEBUG_PUMP
        Serial.print("PUMP: ");Serial.print(m_swAddr);
        Serial.print(", mode=");Serial.print(m_mode);
        Serial.print(", holdArg=");Serial.print(holdArg);
        Serial.print(", holdSec=");Serial.print(m_holdSec);
        Serial.print(", rampSec=");Serial.print(m_rampSec);
        Serial.print(", holdRange=");Serial.print(m_holdRange);
        Serial.print(", rampRange=");Serial.print(m_rampRange);
        Serial.println("");
        #endif
    }

    void update() {

        unsigned char newSpeed = m_curSpeed;

        // Calculate a new speed in modes that do that.
        unsigned char minSpeed = (m_minPct ? (((m_minPct+1)*255)/100) : 1); // Never actually switch off.
        if (minSpeed > m_topSpeed)
            minSpeed = m_topSpeed;
        unsigned long curTime = millis();
        unsigned holdMsec = m_holdSec*1000;
        unsigned rampMsec = m_rampSec*1000;
        bool updateOffset = false;
        bool updateSwitch = false;

        if (inTempShutdown()) {
            if (curTime > m_tempShutoffUntil) {
                m_tempShutoffUntil = 0;
                #if DEBUG_PUMP
                Serial.print("Temp shutdown has expired at ");Serial.println(curTime);
                #endif
            }
        }
        
        if (!inTempShutdown()) {
            switch (m_mode) {
                case Square: {
                    if ((curTime - m_lastChangeTime) > (holdMsec + m_holdOffset)) {
                        if (m_upDown > 0) {
                            m_upDown = -1;
                            newSpeed = minSpeed;
                        } else {
                            m_upDown = 1;
                            newSpeed = m_topSpeed;
                        }
                        updateOffset = true;
                        updateSwitch = true;
                        #if DEBUG_PUMP
                        Serial.print("PUMP: ");Serial.print(m_swAddr);
                        Serial.print(": change at:");Serial.print(curTime);
                        Serial.print(", m_lastChangeTime=");Serial.print(m_lastChangeTime);
                        Serial.print(", m_holdSec=");Serial.print((int)m_holdSec);
                        Serial.print(", m_holdOffset=");Serial.print((int)m_holdOffset);
                        Serial.print(", inShutdown=");Serial.print(m_tempShutoffUntil);
                        Serial.println("");
                        #endif
                    }
                    break;
                }
                case Ramp: {
                    if (m_upDown == 0) {
                        if ((curTime - m_lastChangeTime) > (holdMsec + m_holdOffset)) {
                            m_upDown = (m_curSpeed > ((m_topSpeed-minSpeed)/2)) ? -1 : 1;
                            m_lastChangeTime = curTime;
                            updateOffset = true;
                            updateSwitch = true;
                            #if DEBUG_PUMP
                            Serial.print("PUMP: ");Serial.print(m_swAddr);
                            Serial.print(": change at:");Serial.print(curTime);
                            Serial.print(", m_lastChangeTime=");Serial.print(m_lastChangeTime);
                            Serial.print(", m_holdSec=");Serial.print((int)m_holdSec);
                            Serial.print(", m_holdOffset=");Serial.print((int)m_holdOffset);
                            Serial.print(", m_rampSec=");Serial.print((int)m_rampSec);
                            Serial.print(", m_rampOffset=");Serial.print((int)m_rampOffset);
                            Serial.print(", inShutdown=");Serial.print(m_tempShutoffUntil);
                            Serial.println("");
                            #endif
                        }
                    } else {
                        unsigned long delta = ((unsigned long)(m_topSpeed - minSpeed) * (curTime - m_lastChangeTime))
                                                / (rampMsec+m_rampOffset);
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
                    newSpeed = m_topSpeed;
                    updateSwitch = calcReassertSwitch( curTime );
                    break;
                case Off:
                    // Leave other settings in tact, but go to 0.
                    newSpeed = 0;
                    break;
                default:
                    newSpeed = m_topSpeed;
            }
        } else {
            // Temp shutdown.
            newSpeed = 0;
            updateSwitch |= calcReassertSwitch( curTime );
        }

        // If changing, update random offsets if set.
        if (updateOffset) {
            m_rampOffset = calcRandom( rampMsec, m_rampRange );
            m_holdOffset = calcRandom( holdMsec, m_holdRange );
        }

        syncSwitch( newSpeed, updateSwitch);

        // Nothing to do if no change.
        if (newSpeed == m_curSpeed)
            return;

        m_lastChangeTime = curTime;

        m_curSpeed = newSpeed;
        analogWrite( m_pin, m_curSpeed );
    }

    bool inTempShutdown() {
        return m_tempShutoffUntil;
    }
    void setTempShutoffInterval( unsigned tsec ) {
        m_tempShutoffUntil = millis() + (tsec * 1000);
        #if DEBUG_PUMP
        Serial.println("Setting shutdown with end time ");Serial.println(m_tempShutoffUntil);
        #endif
    }

    void cancelTempShutoff() {
        m_tempShutoffUntil = 0;
        #if DEBUG_PUMP
        Serial.println("Cancelling shutdown");
        #endif
    }
                    
    // Periodically force a switch update.
    // Skew delays between checks by pin of device so they don't all happen together.
    bool calcReassertSwitch( unsigned long curTime ) {
        if ((curTime - m_lastChangeTime) > (ReassertSwitchInterval + (m_swAddr * 1000))) {
            m_lastChangeTime = curTime;
            return true;
        } else {
            return false;
        }
    }

     static int calcRandom( unsigned int sec, unsigned int range ) {
        if (range == 0)
            return 0;

         // 16-bit signed random value.
         // Scale by multiplying to 32-bits, and scaling back down.
         m_rand = (100057  * m_rand + 1237);
         float pct = (float)m_rand/(float)0xefff;
         int rslt = int( pct * range );
         return rslt;
     }

     // If going from or to 0, turn on or off the switch.
     // Also force an "on" if force is set.
     void syncSwitch( unsigned char newSpeed, bool force ) {
         char cmd[8];
         char* np;
         bool needOn = false;
         bool changed = (newSpeed != m_curSpeed);
         if (changed || force) {
             if (newSpeed == 0) {
                 strcpy( cmd, "off " );
             } else if ((m_curSpeed == 0) || force) {
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
                 unsigned long tstart = millis();
                 m_rf24.sendToRadioClient( 5, cmd, 3 );
             }
         }
    }
            

    static unsigned ee_size() {
        return sizeof(m_mode) + sizeof(m_topSpeed) + sizeof(m_holdSec) + sizeof(m_rampSec)
                + sizeof(m_holdRange) + sizeof(m_rampRange);
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

        EEPROM.get( addr, m_holdRange );
        addr += sizeof(m_holdRange);

        EEPROM.get( addr, m_rampRange );
        addr += sizeof(m_rampRange);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.put( addr, m_topSpeed );
        addr += sizeof(m_topSpeed);

        EEPROM.put( addr, m_holdSec );
        addr += sizeof(m_holdSec);

        EEPROM.put( addr, m_rampSec );
        addr += sizeof(m_rampSec);

        EEPROM.put( addr, m_holdRange );
        addr += sizeof(m_holdRange);

        EEPROM.put( addr, m_rampRange );
        addr += sizeof(m_rampRange);
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
    unsigned int  m_rampRange;
    unsigned int  m_holdRange;
    int           m_rampOffset;
    int           m_holdOffset;
    unsigned long m_lastChangeTime;
    unsigned long m_tempShutoffUntil;
    char          m_upDown;
    static int    m_rand;
    RF24IPInterface& m_rf24;
};

int ControllablePump::m_rand __attribute__((weak)) = 12345;

#endif
