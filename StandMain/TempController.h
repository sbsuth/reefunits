#ifndef HEATER_H
#define HEATER_H

#define DEBUG_TEMP 0

#include <Arduino.h>
#include "Avg.h"
#include "DS18B20.h"

// Rejection values for readings.
#define LOWER_BAD_F 30.0
#define UPPER_BAD_F 100.0


class TempController
{
  public:
    TempController(   int ctrlPin, int tempPin, int tempRes, unsigned confAddr );
    void setup( bool useSettings );
    bool update();
    static unsigned ee_size();
    void restoreSettings();
    void saveSettings();

    void setHeaterOn( bool onOff ) {
        if (m_heatOn != onOff) {
            m_heatOn = onOff;
            m_lastOnOff = millis();
        }
    }

    bool heaterIsOn() const {
        return m_heatOn;
    }
    unsigned long timeSinceLastOnOff();

    float curTemp( int itherm=-1, bool raw=false );

    bool isGoodTemp( float TF ) {
        return (TF > LOWER_BAD_F) && (TF < UPPER_BAD_F);
    }

    void setSetTemp(   float TF ) {
        m_setTemp = TF;
    }
    float getSetTemp() {
        return m_setTemp;
    }
    void setSamplePeriod( unsigned sp ) {
        m_samplePeriod = sp;
        m_tempSensors.setPeriodMs(sp);
    }
    unsigned getSamplePeriod() {
        return m_samplePeriod; 
    }
    void setSensitivity( float s ) {
        m_sensitivity = s;
    }
    float getSensitivity() {
        return m_sensitivity;  
    }
    bool isCalibrated( int itherm=-1 );

    static float k2f( double K ) {
        return ((K - 273.15)/.5556) + 32;
    }
    static float f2k( double F ) {
        return 273.15 + ((F - 32)*.5556);
    }
    static float f2c( double F ) {
        return ((F - 32)*.5556);
    }
    float calcTemp( float R, unsigned itherm );
    void calcOnOff();

    bool setOffset( unsigned itherm, float TF ) {
        if (itherm < 2) {
            m_A[itherm] = TF;
            return true;
        } else {
            return false;
        }
    }
    bool calUsingTemp( float TF );
    void ackCalConsts( class Command* cmd );
    float getOffset( int devIndex );

  protected:
    unsigned char   m_ctrlPin;
    unsigned char   m_tempPin;
    unsigned        m_confAddr;
    DS18B20<2>      m_tempSensors;
    float           m_tempValue[2];
    bool            m_heatOn;
    unsigned long   m_lastOnOff;
    unsigned long   m_lastSampleTime; // Time of last read.
    unsigned char   m_lastSampledProbe;// Index of last read.

    //
    // EEPROM values.
    float           m_setTemp;  // Target temp.  Farenheit.
    unsigned        m_samplePeriod; // Time between samples.  ms.
    float           m_sensitivity;  // Threshhold relative to target for on/off.
    float           m_VCC;      // Reference VCC.
    float           m_A[2];      // Cal constant A for each thermistor.
    float           m_B[2];      // Cal constant B for each thermistor.
    float           m_C[2];      // Cal constant C for each thermistor.

};

#endif
