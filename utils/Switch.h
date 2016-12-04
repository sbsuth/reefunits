#ifndef SWITCH_H
#define SWITCH_H

class Switch {
  public:
    Switch(int pin) 
      : m_pin(pin), m_lastChange(millis()+m_debounce), m_lastVal(0), m_stableVal(0), m_changed(false)
    {}
    void setup() {
      pinMode( m_pin, INPUT );
      m_lastChange = millis()+m_debounce;
      m_lastVal = 0;
      m_stableVal = 0;
      m_changed = false;
      pinMode( m_pin, INPUT);
    }
    void update() {
      bool curVal = (digitalRead( m_pin ) == HIGH);
      unsigned long curTime = millis();
      if (curVal != m_lastVal) {
        m_lastChange = curTime;
        m_lastVal = curVal;
        m_changed = false;
      } else if ((curTime - m_lastChange) > m_debounce) {
        m_changed = (m_stableVal != curVal);
        m_stableVal = curVal;
      }
    }
    bool isOn() {
      return m_stableVal;
    }
    bool changed() {
      return m_changed;
    }
    static const int m_debounce = 10;
    int m_pin;
    long m_lastChange;
    bool m_lastVal;
    bool m_stableVal;
    bool m_changed;
};

#endif
