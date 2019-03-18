#ifndef CONTROLLABLE_PUMP_H
#define CONTROLLABLE_PUMP_H 1

#include <EEPROM.h>
#include "RF24Interface.h"
#include "RemoteSetting.h"

class ControllablePumpBase 
{
  public:
    static int    m_rand;
};

template <int NUM_SETS>
class ControllablePump : public ControllablePumpBase
{
  public:
    ControllablePump(   unsigned char ctrlPin, unsigned char swAddr, unsigned char minPct, unsigned confAddr, RF24IPInterface& rf24 )
      : m_pin(ctrlPin), m_swAddr(swAddr), m_minPct(minPct), m_confAddr(confAddr)
      , m_curSpeed(0), m_curSet(0)
      , m_rampOffset(0), m_holdOffset(0)
      , m_lastChangeTime(0), m_lastWriteTime(0), m_tempShutoffUntil(0), m_tempShutoffMode(ShutdownUnset)
      , m_upDown(0), m_retries(0)
      , m_rf24(rf24)
      , m_switch(this)
    {
        for ( int i=0; i < NUM_SETS; i++ ) {
            m_topSpeed[i] = 0;
            m_slowSpeed[i] = 0;
            m_mode[i] = Off;
            m_rampSec[i] = 5;
            m_holdSec[i] = 30;
            m_rampRangeMs[i] = 0;
            m_holdRangeMs[i] = 0;
        }
    }

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

    enum {
        NumSets = NUM_SETS
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
    unsigned char getCurSpeed() const {
        return m_curSpeed;
    }
    unsigned char getTopSpeed() const {
        return m_topSpeed[m_curSet];
    }
    unsigned char getSlowSpeed() const {
        return m_slowSpeed[m_curSet];
    }
    unsigned char getCurSpeedPct() const {
        return getSpeedPct(m_curSpeed);
    }
    unsigned char getTopSpeedPct() const {
        return getSpeedPct(getTopSpeed());
    }
    unsigned char getSlowSpeedPct() const {
        return getSpeedPct(getSlowSpeed());
    }
    static unsigned char getSpeedPct( unsigned char speed ) {
        return ((speed * 100.0) + 50.0)/255.0;
    }

    Mode          getMode() const {
        return m_mode[m_curSet];
    }
    unsigned char getCurSet() const {
        return m_curSet;
    }
    unsigned char getRampSec() const {
        return m_rampSec[m_curSet];
    }
    unsigned char getHoldSec() const {
        return m_holdSec[m_curSet];
    }
    unsigned long getRampRangeMs() const {
        return m_rampRangeMs[m_curSet];
    }
    unsigned long getHoldRangeMs() const {
        return m_holdRangeMs[m_curSet];
    }
    unsigned char getRampRangePct() const {
        return getRangePct(getRampRangeMs(),getRampSec());
    }
    unsigned char getHoldRangePct() const {
        return getRangePct(getHoldRangeMs(),getHoldSec());
    }
    static unsigned char getRangePct( unsigned long range_ms, unsigned char hold_sec ) {
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
        setSpeedPct( m_topSpeed[m_curSet], s );
    }

    void setSlowSpeed( unsigned char s ) {
        setSpeedPct( m_slowSpeed[m_curSet], s );
    }

    void setCurSet( unsigned char cs ) {
        if (cs >= NUM_SETS)
            return;
        m_curSet = cs;
        setMode( getMode(), -1.0, -1.0 );
    }

    void setMode(   Mode m, float holdArg, float rampArg=0.0 ) {

        m_mode[m_curSet] = m;

        float sec;
        float pct;
        if (holdArg >= 0.0) {
            // times are set as "sec.pct" where pct gives the random range
            // as a percentage of 'sec', and where 'sec' is the centerpoint of the range.
            sec = int(holdArg);
            pct = (holdArg - sec);
            m_holdSec[m_curSet] = sec;
            m_holdRangeMs[m_curSet] = int((pct * (float)sec) * 1000.0); // ms
        }

        if (rampArg >= 0.0) {
            sec = int(rampArg);
            pct = (rampArg - sec);
            m_rampSec[m_curSet] = sec;
            m_rampRangeMs[m_curSet] = long((pct * (float)sec) * 1000.0); // ms
        }

        m_upDown = 0;
        m_lastChangeTime = 0;
        #if DEBUG_PUMP
        Serial.print("PUMP: ");Serial.print(m_swAddr);
        Serial.print(", curSet=");Serial.print(m_curSet);
        Serial.print(", mode=");Serial.print(m_mode[m_curSet]);
        Serial.print(", holdArg=");Serial.print(holdArg);
        Serial.print(", holdSec=");Serial.print(m_holdSec[m_curSet]);
        Serial.print(", rampSec=");Serial.print(m_rampSec[m_curSet]);
        Serial.print(", holdRange=");Serial.print(m_holdRangeMs[m_curSet]);
        Serial.print(", rampRange=");Serial.print(m_rampRangeMs[m_curSet]);
        Serial.println("");
        #endif
    }

    void update() {

        unsigned newSpeed = m_curSpeed;

        // Calculate a new speed in modes that do that.
        unsigned char minSpeed = (m_minPct ? (((m_minPct+1)*255)/100) : 1); // Never actually switch off.
        if (getSlowSpeed() > minSpeed)
            minSpeed = getSlowSpeed();
        if (minSpeed > getTopSpeed())
            minSpeed = getTopSpeed();
        unsigned long curTime = millis();
        unsigned long holdMsec = getHoldSec()*1000UL;
        unsigned long rampMsec = getRampSec()*1000UL;
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
            switch (getMode()) {
                case Square: {
                    if (elapsedTime > (holdMsec + m_holdOffset)) {
                        if (m_upDown > 0) {
                            m_upDown = -1;
                            newSpeed = minSpeed;
                        } else {
                            m_upDown = 1;
                            newSpeed = getTopSpeed();
                        }
                        updateOffset = true;
                        updateSwitch = true;
                        debugPrintPumpState();
                    }
                    break;
                }
                case Ramp: {
                    unsigned long speedRange = (getTopSpeed() - minSpeed);
                    if (m_upDown == 0) {
                        if (elapsedTime > (holdMsec + m_holdOffset)) {
                            m_upDown = (m_curSpeed > (minSpeed + (speedRange/2))) ? -1 : 1;
                            m_lastChangeTime = curTime;
                            updateOffset = true;
                            updateSwitch = true;
                            debugPrintPumpState();
                        }
                    } else if (elapsedTime > 1024) { // Avoid unnecessary calculation.

                        unsigned long speedChange = ((unsigned long)speedRange * elapsedTime * 16UL)
                                                     / (rampMsec + m_rampOffset); // ZERO DENOM!
                        if (speedChange) {
                            speedChange /= 16UL; // Denom was promoted to avoid 0 values above.
                            if (m_upDown > 0) {
                                newSpeed = (unsigned)m_curSpeed + speedChange;
                                if (newSpeed > getTopSpeed())
                                    newSpeed = getTopSpeed();
                            } else {
                                newSpeed = (unsigned)m_curSpeed - speedChange;
                                if (newSpeed < minSpeed)
                                    newSpeed = minSpeed;
                            }
                            if ((newSpeed==minSpeed) || (newSpeed==getTopSpeed()))
                                m_upDown = 0;
                        }
                    }
                    break;
                }
                case SinWave: {
                    if (elapsedTime < 1024) // Avoid unnecessary calculation.
                        break; 

                    float speedRange = (getTopSpeed() - minSpeed);
                    unsigned long period = (2 * (rampMsec + m_rampOffset));
                    //unsigned long tOffset = curTime % period;
                    unsigned long tOffset = (curTime - m_cycleStart);
                    float radians = ((float)tOffset / (float)period) * 6.283;
                    float speed = ((sin(radians) + 1.0) * (speedRange/2.0)) + (float)minSpeed;
                    newSpeed = (int)(speed + 0.5);

                    if (newSpeed > getTopSpeed()) {
                        newSpeed = getTopSpeed();
                    } else if (newSpeed < minSpeed) {
                        newSpeed = minSpeed;
                    }
                    if (newSpeed != m_curSpeed) {
                        if ((newSpeed < m_curSpeed) && (m_upDown > 0)) {
                            m_upDown = -1;
                            //updateOffset = true;
                            updateSwitch = true;
                        } else if ((newSpeed > m_curSpeed) && (m_upDown <= 0)) {
                            m_upDown = 1;
                            //updateOffset = true;
                            updateSwitch = true;
                        }
                    }
                    m_lastChangeTime = curTime;
                    if (tOffset > period) {
                        m_cycleStart = curTime;
                        updateOffset = true;
                    }
                    break;
                }
                case Slow:
                case Constant:
                    newSpeed = (getMode() == Slow) ? getSlowSpeed() : getTopSpeed();
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
                    newSpeed = getTopSpeed();
            }
        } else {
            // Temp shutdown.
            switch (m_tempShutoffMode) {
                case ShutdownHalf:
                    newSpeed = getTopSpeed() / 2;
                    break;
                case ShutdownFull:
                    newSpeed = getTopSpeed();
                    break;
                case ShutdownOff:
                default:
                    newSpeed = 0;
                    break;
            }
        }

        // If changing, update random offsets if set.
        if (updateOffset) {
            m_rampOffset = calcRandom( getRampRangeMs() );
            m_holdOffset = calcRandom( getHoldRangeMs() );
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
        Serial.print(F("PUMP: "));Serial.print(m_swAddr);
        Serial.print(F(": change at:"));Serial.print(millis());
        Serial.print(F(", m_lastChangeTime="));Serial.print(m_lastChangeTime);
        Serial.print(F(", m_holdSec="));Serial.print((int)getHoldSec());
        Serial.print(F(", m_holdOffset="));Serial.print((int)m_holdOffset);
        Serial.print(F(", m_rampSec="));Serial.print((int)getRampSec());
        Serial.print(F(", m_rampOffset="));Serial.print((int)m_rampOffset);
        Serial.print(F(", inShutdown="));Serial.print(m_tempShutoffUntil);
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
                    
     static long calcRandom( unsigned long range ) {
        if (range == 0)
            return 0;

         // 16-bit signed random value.
         // Scale by multiplying to 32-bits, and scaling back down.
         m_rand = (100057  * m_rand + 1237);
         float pct = (float)m_rand/(float)0xefff;
         long rslt = int( pct * range );
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
                + sizeof(m_holdRangeMs) + sizeof(m_rampRangeMs) + sizeof(m_curSet);
    }

    void restoreSettings( unsigned char i, unsigned& addr ) {

        EEPROM.get( addr, m_mode[i] );
        addr += sizeof(m_mode[0]);

        bool skip = false;
        if ((unsigned char)m_mode[i] == 0xFF) {
            m_mode[i] = Off;
            skip = true;
        }

        if (!skip)
            EEPROM.get( addr, m_topSpeed[i] );
        addr += sizeof(m_topSpeed[0]);

        if (!skip)
            EEPROM.get( addr, m_slowSpeed[i] );
        addr += sizeof(m_slowSpeed[0]);

        if (!skip)
            EEPROM.get( addr, m_holdSec[i] );
        addr += sizeof(m_holdSec[0]);

        if (!skip)
            EEPROM.get( addr, m_rampSec[i] );
        addr += sizeof(m_rampSec[0]);

        if (!skip)
            EEPROM.get( addr, m_holdRangeMs[i] );
        addr += sizeof(m_holdRangeMs[0]);

        if (!skip)
            EEPROM.get( addr, m_rampRangeMs[i] );
        addr += sizeof(m_rampRangeMs[0]);
    }

    void saveSettings( unsigned char i, unsigned& addr ) {

        EEPROM.put( addr, m_mode[i] );
        addr += sizeof(m_mode[0]);

        EEPROM.put( addr, m_topSpeed[i] );
        addr += sizeof(m_topSpeed[0]);

        EEPROM.put( addr, m_slowSpeed[i] );
        addr += sizeof(m_slowSpeed[0]);

        EEPROM.put( addr, m_holdSec[i] );
        addr += sizeof(m_holdSec[0]);

        EEPROM.put( addr, m_rampSec[i] );
        addr += sizeof(m_rampSec[0]);

        EEPROM.put( addr, m_holdRangeMs[i] );
        addr += sizeof(m_holdRangeMs[0]);

        EEPROM.put( addr, m_rampRangeMs[i] );
        addr += sizeof(m_rampRangeMs[0]);
    }

    void restoreSettings() {
        unsigned addr = m_confAddr;
        for ( int i=0; i < NUM_SETS; i++ ) {
            restoreSettings(i,addr);
        }

        EEPROM.get( addr, m_curSet );
        addr += sizeof(m_curSet);
        if (m_curSet == 0xFF)
            m_curSet = 0;
    }
        
    void saveSettings() {
        unsigned addr = m_confAddr;
        for ( int i=0; i < NUM_SETS; i++ ) {
            saveSettings(i,addr);
        }

        EEPROM.put( addr, m_curSet );
        addr += sizeof(m_curSet);
    }
        
  protected:
    unsigned char m_pin;
    unsigned char m_swAddr;
    unsigned char m_minPct;
    unsigned      m_confAddr;
    unsigned char m_curSpeed;
    unsigned char m_curSet;
    unsigned char m_topSpeed[NUM_SETS];
    unsigned char m_slowSpeed[NUM_SETS];
    Mode          m_mode[NUM_SETS];
    unsigned char m_rampSec[NUM_SETS];
    unsigned char m_holdSec[NUM_SETS];
    unsigned long m_rampRangeMs[NUM_SETS];
    unsigned long m_holdRangeMs[NUM_SETS];
    long           m_rampOffset;
    long           m_holdOffset;
    unsigned long m_lastChangeTime;
    unsigned long m_lastWriteTime;
    unsigned long m_cycleStart;
    unsigned long m_tempShutoffUntil;
    ShutdownKind  m_tempShutoffMode;
    char          m_upDown;
    unsigned char m_retries;
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

int ControllablePumpBase::m_rand __attribute__((weak)) = 12345;

#endif
