#ifndef DOSING_PUMP_H
#define DOSING_PUMP_H

#include <DRV8825.h>
#include "DecayingState.h"

class DosingPump : public DRV8825 {
  public: 
    DosingPump( int dir, int step, int isleep, unsigned confAddr, DecayingState<bool>* extDisable=0 ) 
        : DRV8825( 200, dir, step )
        , m_isleep(isleep)
        , m_disabled(true)
        , m_stepsPerMl(833)
        , m_stepsRemaining(0)
        , m_toDispenseMl(0)
        , m_confAddr(confAddr)
        #if EXT_PAUSE
        , m_extDisable(extDisable)
        #endif
    {}
    void init( unsigned rpm, bool useSettings ) {
        setRPM(rpm);
        if (useSettings) {
            restoreSettings();
        } else {
            saveSettings();
        }
        setMicrostep(1);
        reset();
    }
    void reset() {
        m_stepsRemaining = 0;
        m_disabled = 0;
        m_toDispenseMl = 0;
    }
    bool isEnabled() {
        #if EXT_PAUSE
        return (m_disabled == 0) && !m_extDisable->getVal();
        #else
        return (m_disabled == 0);
        #endif
    }
    void enable() {
        if (m_disabled > 0)
            m_disabled--;
    }
    void disable() {
        m_disabled++;
    }

    bool isDispensing() {
        return (m_stepsRemaining > 0);
    }


    // Setup a new dispense cycle.
    // Silently fails if already dispensing.
    bool startDispense( unsigned short ml ) {
        if (isDispensing())
            return false;
        
        // Reject crazy numbers.
        if (ml > 10000)
            return false;

        // Setting steps remaining causes update to run the pump and decrement.
        m_stepsRemaining = ((unsigned long)ml * (unsigned long)m_stepsPerMl);
        m_toDispenseMl = ml;
        return true;
    }

    // Begins to dispense by a fixed amount.
    void startCal() {
        startDispense(100);
    }

    // Gives the total ml dispensed in this session.
    unsigned short dispensedMl() {
        if (!m_stepsPerMl) {
            return 0;
        }
        unsigned long targetSteps = ((unsigned long)m_toDispenseMl * (unsigned long)m_stepsPerMl);
        unsigned long doneSteps = (targetSteps < m_stepsRemaining) ? 0 : (targetSteps - m_stepsRemaining);
        return (doneSteps / m_stepsPerMl);
    }

    // Does a spurt if there's something to do and we're not disabled.
    void update( unsigned char en[] ) {
        if (!isDispensing() || !isEnabled())
            return;
        static unsigned long stepsPerIter = 50;
        unsigned short steps;
        if (m_stepsRemaining > stepsPerIter) {
            steps = stepsPerIter;
            m_stepsRemaining -= stepsPerIter;
        } else {
            steps = m_stepsRemaining;
            m_stepsRemaining = 0;
        }

        unsigned char sleepPin = en[m_isleep];
        if (steps && sleepPin) {
            digitalWrite( sleepPin, 1 );
            en[m_isleep] = 0; // Indicates it was set.
        }

        move(steps);
    }

    // Sets the actual number of ml the were dispensed by the last run.
    // This is used to set m_stepsPerMl in a new calibration.
    void setActualMl( unsigned short ml ) {
        if (isDispensing() || !ml)
            return;
        unsigned long totalSteps = ((unsigned long)m_toDispenseMl * (unsigned long)m_stepsPerMl);
        m_stepsPerMl = totalSteps / ml;
    }

    // Directly set steps per ml. Used for stored calibrations.
    void setStepsPerMl( unsigned short spml ) {
        m_stepsPerMl = spml;
    }
    unsigned short stepsPerMl() {
        return m_stepsPerMl;
    }

    unsigned short toDispenseMl() {
        return m_toDispenseMl;
    }

    static unsigned ee_size() {
        return sizeof(m_stepsPerMl);
    }

    void restoreSettings() {
        unsigned addr = m_confAddr;

        EEPROM.get( addr, m_stepsPerMl );
        addr += sizeof(m_stepsPerMl);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_stepsPerMl );
        addr += sizeof(m_stepsPerMl);
    }
        
   
  protected:
    unsigned char  m_isleep;
    unsigned short m_disabled;
    unsigned long  m_stepsRemaining;
    unsigned short m_toDispenseMl;
    unsigned short m_stepsPerMl;
    unsigned       m_confAddr;
    #if EXT_PAUSE
    DecayingState<bool>* m_extDisable;
    #endif
};

#endif
