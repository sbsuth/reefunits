#ifndef DS18B20_H
#define DS18B20_H

#include <Arduino.h>

#include "OneWire.h"
#include <DallasTemperature.h>
#include <Avg.h>

template <int N>
class DS18B20 {
  public:
    DS18B20( int pin, int res ) 
        : m_oneWire(pin)
        , m_sensors(&m_oneWire)
        , m_res(res)
        , m_nextTime(millis()+1000) // First action 1 sec after startup.
        , m_waitingTemp(false)
    {
    }
    void setup() {
        m_sensors.begin();
        m_sensors.setWaitForConversion(false);
    }
    bool update() {
        unsigned long now = millis();
        if (now >= m_nextTime) {
            if (m_waitingTemp) {
                // Read results and go idle for reset of second.
                m_nextTime = now + (1000 - curDelay());
                for (int i=0; i < N; i++ ) {
                    m_temp[i].update(m_sensors.getTempCByIndex(i));
                }
                m_waitingTemp = false;
                return true;
            } else {
                // Init reading, and come back after delay.
                m_sensors.requestTemperatures();
                m_nextTime = now + curDelay();
                m_waitingTemp = true;
            }
        }
        return false;
    }

    int getResolution() {
        return m_res;
    }
    void setResolution( int r ) {
        m_res = r;
    }
    float getTemp( int unit ) {
        return m_temp[unit].avg();
    }
  protected:
    OneWire m_oneWire;
    DallasTemperature m_sensors;
    int m_res;
    bool m_waitingTemp;
    unsigned long m_nextTime;
    Avg<16,float> m_temp[N];

    unsigned long curDelay() {
        return (750/ (1 << (12-m_res)));
    }
};

#endif
