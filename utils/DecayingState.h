#ifndef DECAYING_STATE
#define DECAYING_STATE 1

// Models a setting that must be periodically refresshed to avoid 
// decaying to a default value.
template <typename T> 
class DecayingState 
{
  public:
    DecayingState(  T defVal,  unsigned long t_decay ) 
        : m_defVal(defVal), m_curVal(defVal), m_decay(t_decay), m_lastRefresh(0)
    {}

    T getVal() {
        return m_curVal;
    }

    void update() {
        unsigned long now = millis();
        if (now > (m_lastRefresh + m_decay)) {
            if (m_curVal != m_defVal)
                m_lastChange = now;
            m_curVal = m_defVal;
        }
    }

    // Returns the length of time the setting has been this value.
    unsigned long setVal( T v ) {
        unsigned long now = millis();
        bool changed = (v != m_curVal);
        if (changed) {
            m_lastChange = now;
            m_curVal = v;
        }
        m_lastRefresh = now;
        return (now - m_lastChange);
    }

    unsigned long timeAtValue() {
        return (millis() - m_lastChange);
    }

    unsigned long timeAtValueSec() {
        return (millis() - m_lastChange) / 1000U;
    }

    void setDecay( unsigned long decay ) {
        m_decay = decay;
    }
                
  protected:
    T               m_defVal;
    T               m_curVal;
    unsigned long   m_decay;
    unsigned long   m_lastRefresh;
    unsigned long   m_lastChange;
};

#endif
