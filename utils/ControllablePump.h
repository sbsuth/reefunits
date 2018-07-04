#ifndef CONTROLLABLE_PUMP_H
#define CONTROLLABLE_PUMP_H 1

#include <EEPROM.h>
#include "RF24Interface.h"
#include "RemoteSetting.h"

class ControllablePump
{
  public:
    ControllablePump(   unsigned char ctrlPin, unsigned char swAddr, unsigned char minPct, unsigned confAddr, RF24IPInterface& rf24 )
      : m_pin(ctrlPin), m_swAddr(swAddr), m_minPct(minPct), m_confAddr(confAddr)
      , m_curSpeed(0), m_topSpeed(0), m_slowSpeed(0), m_mode(Off)
      , m_rampSec(5), m_holdSec(30), m_rampRangeMs(0), m_holdRangeMs(0)
      , m_rampOffset(0), m_holdOffset(0)
      , m_lastChangeTime(0), m_lastWriteTime(0), m_tempShutoffUntil(0), m_tempShutoffMode(ShutdownUnset)
      , m_upDown(0), m_retries(0)
      , m_rf24(rf24)
      , m_switch(this)
    {}

    enum Mode {
        Constant = 0,
        Slow     = 1,
        Square   = 2,
        Ramp     = 3,
        Off      = 4,
        Test     = 5,
        SinWave  = 6,
        NumModes = 7
    };

    enum ShutdownKind {
        ShutdownUnset   = 0,
        ShutdownCancel  = 1,
        ShutdownOff     = 2,
        ShutdownHalf    = 3,
        ShutdownFull    = 4,
        NumShutdowns    = 5
    };

    //static const unsigned ReassertSwitchInterval = 10 * 1000;
    static const unsigned ReassertSwitchInterval = 60 * 1000U;

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
    unsigned char getSlowSpeed() {
        return m_slowSpeed;
    }
    unsigned char getCurSpeedPct() {
        return getSpeedPct(m_curSpeed);
    }
    unsigned char getTopSpeedPct() {
        return getSpeedPct(m_topSpeed);
    }
    unsigned char getSlowSpeedPct() {
        return getSpeedPct(m_slowSpeed);
    }
    static unsigned char getSpeedPct( unsigned char speed ) {
        return ((speed * 100.0) + 50.0)/255.0;
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
    unsigned int getRampRangeMs() {
        return m_rampRangeMs;
    }
    unsigned int getHoldRangeMs() {
        return m_holdRangeMs;
    }
    unsigned char getRampRangePct() {
        return getRangePct(m_rampRangeMs,m_rampSec);
    }
    unsigned char getHoldRangePct() {
        return getRangePct(m_holdRangeMs,m_holdSec);
    }
    static unsigned char getRangePct( unsigned int range_ms, unsigned char hold_sec ) {
        // range is in ms, hold is in sec, we want the ration as a percent.
        return (range_ms * 1.0)/(hold_sec * 10.0); // ZERO DENOM!
    }

    unsigned long getLastChangeTime() {
        return m_lastChangeTime;
    }

    // Accepts pct.  Stored as 0-255.
    void setSpeedPct( unsigned char& speed, unsigned char s ) {
        if (s > 100)
            speed = 255;
        else if (s < m_minPct)
            speed = 0;
        else
            speed = (255U * (unsigned)s)/100U;
    }

    void setTopSpeed( unsigned char s ) {
        setSpeedPct( m_topSpeed, s );
    }

    void setSlowSpeed( unsigned char s ) {
        setSpeedPct( m_slowSpeed, s );
    }

    void setMode(   Mode m, float holdArg, float rampArg=0.0 ) {

        m_mode = m;

        // times are set as "sec.pct" where pct gives the random range
        // as a percentage of 'sec', and where 'sec' is the centerpoint of the range.
        float sec = int(holdArg);
        float pct = (holdArg - sec);
        m_holdSec = sec;
        m_holdRangeMs = int((pct * (float)sec) * 1000.0); // ms

        sec = int(rampArg);
        pct = (rampArg - sec);
        m_rampSec = sec;
        m_rampRangeMs = int((pct * (float)sec) * 1000.0); // ms

        m_upDown = 0;
        m_lastChangeTime = 0;
        #if DEBUG_PUMP
        Serial.print("PUMP: ");Serial.print(m_swAddr);
        Serial.print(", mode=");Serial.print(m_mode);
        Serial.print(", holdArg=");Serial.print(holdArg);
        Serial.print(", holdSec=");Serial.print(m_holdSec);
        Serial.print(", rampSec=");Serial.print(m_rampSec);
        Serial.print(", holdRange=");Serial.print(m_holdRangeMs);
        Serial.print(", rampRange=");Serial.print(m_rampRangeMs);
        Serial.println("");
        #endif
    }

    void update() {

        unsigned char newSpeed = m_curSpeed;

        // Calculate a new speed in modes that do that.
        unsigned char minSpeed = (m_minPct ? (((m_minPct+1)*255)/100) : 1); // Never actually switch off.
        if (m_slowSpeed > minSpeed)
            minSpeed = m_slowSpeed;
        if (minSpeed > m_topSpeed)
            minSpeed = m_topSpeed;
        unsigned long curTime = millis();
        unsigned holdMsec = m_holdSec*1000;
        unsigned rampMsec = m_rampSec*1000;
        bool updateOffset = false;
        bool updateSwitch = false;
        unsigned long elapsedTime = (curTime - m_lastChangeTime);

        if (inTempShutdown()) {
            if (curTime > m_tempShutoffUntil) {
                m_tempShutoffUntil = 0;
                m_tempShutoffMode = ShutdownUnset;
                #if DEBUG_PUMP
                Serial.print("Temp shutdown has expired at ");Serial.println(curTime);
                #endif
            }
        }
        
        if (!inTempShutdown()) {
            switch (m_mode) {
                case Square: {
                    if (elapsedTime > (holdMsec + m_holdOffset)) {
                        if (m_upDown > 0) {
                            m_upDown = -1;
                            newSpeed = minSpeed;
                        } else {
                            m_upDown = 1;
                            newSpeed = m_topSpeed;
                        }
                        updateOffset = true;
                        updateSwitch = true;
                        debugPrintPumpState();
                    }
                    break;
                }
                case Ramp: {
                    unsigned long speedRange = (m_topSpeed - minSpeed);
                    if (m_upDown == 0) {
                        if (elapsedTime > (holdMsec + m_holdOffset)) {
                            m_upDown = (m_curSpeed > (speedRange/2)) ? -1 : 1;
                            m_lastChangeTime = curTime;
                            updateOffset = true;
                            updateSwitch = true;
                            debugPrintPumpState();
                        }
                    } else if (elapsedTime > 250) { // Avoid unnecessary calculation.

                        unsigned long speedChange = ((unsigned long)speedRange * elapsedTime)
                                                      / (rampMsec + m_rampOffset); // ZERO DENOM!
                        if (speedChange) {
                            if (m_upDown > 0) {
                                newSpeed = m_curSpeed + speedChange;
                                if (newSpeed > m_topSpeed)
                                    newSpeed = m_topSpeed;
                            } else {
                                newSpeed = m_curSpeed - speedChange;
                                if (newSpeed < minSpeed)
                                    newSpeed = minSpeed;
                            }
                            if ((newSpeed==minSpeed) || (newSpeed==m_topSpeed))
                                m_upDown = 0;
                        }
                    }
                    break;
                }
                case SinWave: {
                    if (elapsedTime < 250) // Avoid unnecessary calculation.
                        break; 

                    float speedRange = (m_topSpeed - minSpeed);
                    unsigned long period = (2 * (rampMsec + m_rampOffset));
                    unsigned long tOffset = curTime % period;
                    float radians = ((float)tOffset / (float)period) * 6.283;
                    float speed = ((sin(radians) + 1.0) * (speedRange/2.0)) + (float)minSpeed;
                    unsigned newSpeed = (int)(speed + 0.5);

                    if (newSpeed > m_topSpeed) {
                        newSpeed = m_topSpeed;
                    } else if (newSpeed < minSpeed) {
                        newSpeed = minSpeed;
                    }
                    if (newSpeed != m_curSpeed) {
                        m_lastChangeTime = curTime;
                        if ((newSpeed < m_curSpeed) && (m_upDown > 0)) {
                            m_upDown = -1;
                            updateOffset = true;
                            updateSwitch = true;
                        } else if ((newSpeed > m_curSpeed) && (m_upDown <= 0)) {
                            m_upDown = 1;
                            updateOffset = true;
                            updateSwitch = true;
                        }
                    }
                    break;
                }
                case Slow:
                case Constant:
                    newSpeed = (m_mode == Slow) ? m_slowSpeed : m_topSpeed;
                    break;
                case Test:
                    // Turn off, but don't auto-switch
                    newSpeed = 0;
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
            switch (m_tempShutoffMode) {
                case ShutdownHalf:
                    newSpeed = m_topSpeed / 2;
                    break;
                case ShutdownFull:
                    newSpeed = m_topSpeed;
                    break;
                case ShutdownOff:
                default:
                    newSpeed = 0;
                    break;
            }
        }

        // If changing, update random offsets if set.
        if (updateOffset) {
            m_rampOffset = calcRandom( rampMsec, m_rampRangeMs );
            m_holdOffset = calcRandom( holdMsec, m_holdRangeMs );
        }

        syncSwitch( newSpeed, updateSwitch);

        bool speedChanged = (newSpeed != m_curSpeed);
        if (speedChanged)
            m_lastChangeTime = curTime;

        m_curSpeed = newSpeed;

        // Write the speed if its changed, or every 10 seconds.
        if (speedChanged || ((curTime - m_lastWriteTime) > (10 * 1000))) {
            analogWrite( m_pin, m_curSpeed );
            m_lastWriteTime = curTime;
        }
    }

    void debugPrintPumpState() {
        #if DEBUG_PUMP
        Serial.print("PUMP: ");Serial.print(m_swAddr);
        Serial.print(": change at:");Serial.print(millis());
        Serial.print(", m_lastChangeTime=");Serial.print(m_lastChangeTime);
        Serial.print(", m_holdSec=");Serial.print((int)m_holdSec);
        Serial.print(", m_holdOffset=");Serial.print((int)m_holdOffset);
        Serial.print(", m_rampSec=");Serial.print((int)m_rampSec);
        Serial.print(", m_rampOffset=");Serial.print((int)m_rampOffset);
        Serial.print(", inShutdown=");Serial.print(m_tempShutoffUntil);
        Serial.println("");
        #endif
    }

    bool inTempShutdown() {
        switch (m_tempShutoffMode) {
            case ShutdownHalf:
            case ShutdownFull:
            case ShutdownOff:
                return m_tempShutoffUntil;
            default:
                return false;
        }
    }
    unsigned long tempShutdownRemainingSec() {
        if (inTempShutdown())
            return ((m_tempShutoffUntil - millis()) + 500)/1000;
        else
            return 0;
    }
    void setTempShutoffInterval( unsigned long tsec ) {
        m_tempShutoffUntil = millis() + (tsec * 1000UL);
        #if DEBUG_PUMP
        Serial.print("At ");Serial.print(millis());Serial.print(", setting shutdown with end time ");Serial.println(m_tempShutoffUntil);
        #endif
    }

    void cancelTempShutoff() {
        m_tempShutoffUntil = 0;
        m_tempShutoffMode = ShutdownUnset;
        #if DEBUG_PUMP
        Serial.println("Cancelling shutdown");
        #endif
    }

    void setTempShutoff( unsigned long secs, ShutdownKind kind ) {
        switch (kind) {
            case ShutdownCancel:
                cancelTempShutoff();
                break;
            default:
                setTempShutoffInterval(secs);
                m_tempShutoffMode = kind;
                break;
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
         bool wantOn = (newSpeed > 0);
         m_switch.setWant( wantOn, force );

         #if !DISABLE_PUMP_SWITCH_SYNC
         m_switch.update();
         #endif 
    }
            

    static unsigned ee_size() {
        return sizeof(m_mode) + sizeof(m_topSpeed) + sizeof(m_slowSpeed) + sizeof(m_holdSec) + sizeof(m_rampSec)
                + sizeof(m_holdRangeMs) + sizeof(m_rampRangeMs);
    }

    void restoreSettings() {
        unsigned addr = m_confAddr;

        EEPROM.get( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.get( addr, m_topSpeed );
        addr += sizeof(m_topSpeed);

        EEPROM.get( addr, m_slowSpeed );
        addr += sizeof(m_slowSpeed);

        EEPROM.get( addr, m_holdSec );
        addr += sizeof(m_holdSec);

        EEPROM.get( addr, m_rampSec );
        addr += sizeof(m_rampSec);

        EEPROM.get( addr, m_holdRangeMs );
        addr += sizeof(m_holdRangeMs);

        EEPROM.get( addr, m_rampRangeMs );
        addr += sizeof(m_rampRangeMs);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_mode );
        addr += sizeof(m_mode);

        EEPROM.put( addr, m_topSpeed );
        addr += sizeof(m_topSpeed);

        EEPROM.put( addr, m_slowSpeed );
        addr += sizeof(m_slowSpeed);

        EEPROM.put( addr, m_holdSec );
        addr += sizeof(m_holdSec);

        EEPROM.put( addr, m_rampSec );
        addr += sizeof(m_rampSec);

        EEPROM.put( addr, m_holdRangeMs );
        addr += sizeof(m_holdRangeMs);

        EEPROM.put( addr, m_rampRangeMs );
        addr += sizeof(m_rampRangeMs);
    }
        
  protected:
    unsigned char m_pin;
    unsigned char m_swAddr;
    unsigned char m_minPct;
    unsigned      m_confAddr;
    unsigned char m_curSpeed;
    unsigned char m_topSpeed;
    unsigned char m_slowSpeed;
    Mode          m_mode;
    unsigned char m_rampSec;
    unsigned char m_holdSec;
    unsigned int  m_rampRangeMs;
    unsigned int  m_holdRangeMs;
    int           m_rampOffset;
    int           m_holdOffset;
    unsigned long m_lastChangeTime;
    unsigned long m_lastWriteTime;
    unsigned long m_tempShutoffUntil;
    ShutdownKind  m_tempShutoffMode;
    char          m_upDown;
    unsigned char m_retries;
    static int    m_rand;
    RF24IPInterface& m_rf24;

    class SwitchSettable : public RemoteSettable<char> 
    {
      public:
        SwitchSettable( ControllablePump* p ) 
            : m_pump(p)
        {
            m_wantValue = 0;
            m_confirmedValue = -1;
        }
        virtual const char* cmd() {
            static char buf[8];
            char* pc = &buf[0];
            strcpy( buf, m_wantValue ? "on " : "off " );
            while (*pc) pc++;
            *pc++ = '0' + m_pump->m_swAddr;
            *pc++ = '\n';
            *pc++ = 0;
            return buf;
        }
        virtual unsigned reassertInterval() {
            // Give different intervals for different switches to reduce collisions.
            return (ReassertSwitchInterval + (m_pump->m_swAddr * 1000));
        }
        virtual unsigned char instrAddr() {
            return 5;
        }
      protected:
        ControllablePump*   m_pump;
    };
    SwitchSettable  m_switch;
};

int ControllablePump::m_rand __attribute__((weak)) = 12345;

#endif
