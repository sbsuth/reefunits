#ifndef DS18B20_H
#define DS18B20_H

#include <Arduino.h>

#include "OneWire.h"
#include <DallasTemperature.h>
#include <Avg.h>

typedef uint8_t dsaddr_t[8];
    
template <int N>
class DS18B20 {
  public:

    DS18B20( int pin, int res, int period ) 
        : m_oneWire(pin)
        , m_sensors(&m_oneWire)
        , m_res(res)
        , m_period_ms(period)
        , m_nextMeasTime(0) 
        , m_nextSearchTime(millis()+5000)
        , m_waitingTemp(false)
        , m_initDone(false)
    {
        for (int i=0; i < N; i++ ) {
            m_devMap[i] = -1;
            m_numBad[i] = 0;
        }
            
    }
    void setup() {
        //m_nextMeasTime = (millis() + 3000);
        m_nextMeasTime = 0;
            //Serial.print(F("HEY: C: Set m_nextMeasTime to "));Serial.println(m_nextMeasTime);
    }
    bool update();
    int getResolution() {
        return m_res;
    }
    void setResolution( int r ) {
        m_res = r;
    }
    int getPeriodMs() {
        return m_period_ms;
    }
    void setPeriodMs( unsigned p ) {
        m_period_ms = p;
    }
    float getTemp( int devIndex ) {
        return m_temp[devIndex].avg();
    }
    unsigned char getNumDevices() {
        return m_numDevices;
    }
    void findSensors();
        
    void printAddress( dsaddr_t address );

    int addr2index( dsaddr_t address );
    uint8_t* index2addr( unsigned char index );

    int getDevIndexForAddrIndex( unsigned addrIndex ) {
        for ( unsigned di=0; di < N; di++ ) {
            if (m_devMap[di] == addrIndex)
                return di;
        }
        return -1;
    }
    int getAddrIndexForDevIndex( unsigned devIndex ) {
        return m_devMap[devIndex];
    }
  protected:
    OneWire m_oneWire;
    DallasTemperature m_sensors;
    int m_res;
    unsigned m_period_ms;
    bool m_waitingTemp;
    unsigned long m_nextMeasTime;
    unsigned long m_nextSearchTime;
    unsigned char m_numDevices;
    Avg<32,float> m_temp[N];
    char m_devMap[N]; // Holds addrIndexes.
    int m_numBad[N];
    bool m_initDone;

    unsigned long curDelay() {
        return (750/ (1 << (12-m_res)));
    }
};

#endif
