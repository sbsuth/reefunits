#ifndef DOSING_PUMP_H
#define DOSING_PUMP_H

class DosingPump : public DRV8825 {
  public: 
    DosingPump( int dir, int step ) 
        : DRV8825( 200, dir, step )
        , m_disabled(true)
        , m_stepsPerMl(833)
        , m_stepsRemaining(0)
        , m_toDispenseMl(0)
    {}
    void init() {
        setRPM(250);
        setMicrostep(1);
        reset();
    }
    void reset() {
        m_stepsRemaining = 0;
        m_disabled = 0;
        m_toDispenseMl = 0;
    }
    bool enabled() {
        return (m_disabled == 0);
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
    void update() {
        if (!isDispensing() || m_disabled)
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
   
  protected:
    unsigned short m_disabled;
    unsigned long  m_stepsRemaining;
    unsigned short m_toDispenseMl;
    unsigned short m_stepsPerMl;
};

#endif