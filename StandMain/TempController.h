#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include "Avg.h"

#define DEBUG_TEMP 0

class TempController
{
  public:
    TempController(   int ctrlPin, int temp1Pin, int temp2Pin, unsigned confAddr );
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

    float curTemp( int itherm=-1 );

    unsigned char pinFor( unsigned itherm ) {
        if (itherm < 2)
            return m_tempPin[itherm];
        else
            return 0;
    }

    void setSetTemp(   float TF ) {
        m_setTemp = TF;
    }
    float getSetTemp() {
        return m_setTemp;
    }
    void setSamplePeriod( unsigned sp ) {
        m_samplePeriod = sp;
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

    float adc2R( unsigned adc );
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

    static constexpr float    VREF       = 2.2;
    static const unsigned R_pulldown = 5100;

    bool calStep( unsigned step, unsigned itherm, float TF );

    void ackCalConsts( class Command* cmd );
  protected:
    unsigned char   m_ctrlPin;
    unsigned char   m_tempPin[2];
    unsigned        m_confAddr;
    Avg<32,float>   m_tempValue[2];
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

    struct CalSession 
    {
        CalSession()
            : m_tc(0), m_nspec(0)
        {}
        void setTC( TempController* tc ) {
            m_tc = tc;
        }
        void reset() {
            m_nspec = 0;
        }
        unsigned nextStep() {
            return m_nspec;
        }
        bool addPoint( unsigned adc, float TF ) {
            if (m_nspec > 2)
                return false;

            m_R[m_nspec] = m_tc->adc2R(adc);
            m_TF[m_nspec] = TF;

            #if DEBUG_TEMP
            Serial.print("TEMP: Cal step ");Serial.print(m_nspec);Serial.print(", adc=");Serial.print(adc);Serial.print(", R=");Serial.print(m_R[m_nspec]);Serial.print(", TF="); Serial.println(m_TF[m_nspec]);
            #endif

            m_nspec++;
            return ready();
        }
        bool ready() {
            return (m_nspec == 3);
        }
    
        TempController* m_tc;
        unsigned        m_nspec;
        float           m_R[3];
        float           m_TF[3];
    };
    bool setCal( const CalSession& data, unsigned itherm );

    CalSession      m_cal[2];

};

#endif
