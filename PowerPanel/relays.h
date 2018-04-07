#ifndef RELAYS_H
#define RELAYS_H

#include <EEPROM.h>

// Class for managing banks of relays.

template <int N>
class Relays
{
  public:
    // Accepts an array of pins.  Expects static data, and keeps a reference.
    Relays( unsigned char (&pins)[N], unsigned confAddr, bool active=false )
        : m_pins(pins), m_confAddr(confAddr)
    {
        for (unsigned char i=0; i < N; i++ ) {
            m_on[i] = false;
        }
    }
    void init( bool useSettings ) {
        // Do an assertion to inactive state before setting as output.
        for (unsigned char i=0; i < N; i++ ) {
            digitalWrite( m_pins[i], !m_active );
            pinMode( m_pins[i], OUTPUT);
            m_on[i] = false;
        }

        if (useSettings) {
            restoreSettings();
        } else {
            saveSettings();
        }
    }
    void on(unsigned int p) {
        if (p < N) {
            digitalWrite( m_pins[p], m_active );
            m_on[p] = true;
        }
    }
    void off(unsigned int p) {
        if (p < N) {
            digitalWrite( m_pins[p], !m_active );
            m_on[p] = false;
        }
    }
    void allOff() {
        for (unsigned char i=0; i < N; i++ ) {
            off(i);
        }
    }
    bool isOn(unsigned char p) {
        if (p < N)
            return m_on[p];
        else
            return false;
    }
    
        
    unsigned saveSettings() {
        unsigned addr = m_confAddr;
        for (unsigned char i=0; i < N; i++ ) {
            unsigned char onOff = m_on[i];
            EEPROM.put( addr++, onOff );
        }
        return addr;
    }
        
    unsigned restoreSettings() {
        unsigned addr = m_confAddr;
        for (unsigned char i=0; i < N; i++ ) {
            unsigned char onOff = 0;
            EEPROM.get( addr++, onOff );
            if (onOff != m_on[i]) {
                if (onOff)
                    on(i);
                else
                    off(i);
            }
        }
        return addr;
    }
        
  protected:
    unsigned char (&m_pins)[N];
    bool            m_on[N];
    bool            m_active;
    unsigned        m_confAddr;
};

#endif
