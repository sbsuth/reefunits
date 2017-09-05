#ifndef HEATER_H
#define HEATER_H

class TempController
{
  public:
    TempController(   int ctrlPin, int temp1Pin, int temp2Pin, unsigned confAddr )
        : m_ctrlPin(ctrlPin), m_temp1Pin(temp1Pin), m_temp2Pin(temp2Pin), m_confAddr(confAddr), m_heatOn(false)
    {}

    void setup( bool useSettings )
    {
        if (useSettings) {
            restoreSettings();
        } else {
            saveSettings();
        }
        pinMode( m_ctrlPin, OUTPUT );
        digitalWrite( m_ctrlPin, 0 );

        pinMode( m_temp1, INPUT );
        pinMode( m_temp2, INPUT );
    }
    bool update()
    {
    }

    static unsigned ee_size() {
        return sizeof(m_setTemp);
    }
    void restoreSettings() {
        unsigned addr = m_confAddr;

        EEPROM.get( addr, m_setTemp );
        addr += sizeof(m_setTemp);
    }

    void saveSettings() {
        unsigned addr = m_confAddr;

        EEPROM.put( addr, m_setTemp );
        addr += sizeof(m_setTemp);
    }

    bool heaterIsOn() const {
        return m_heatOn;
    }

  protected:
    unsigned char   m_ctrlPin;
    unsigned char   m_temp1Pin;
    unsigned char   m_temp2Pin;
    unsigned        m_confAddr;
    Avg<16,float>   m_temp1Value;
    Avg<16,float>   m_temp2Value;
    bool            m_heatOn;
};

#endif
