#ifndef HEATER_H
#define HEATER_H

#include <Arduino.h>
#include "Avg.h"

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
        m_heatOn = onOff;
    }

    bool heaterIsOn() const {
        return m_heatOn;
    }

    float adc2R( unsigned adc );
    static float k2f( double K ) {
        return ((K - 273.15)/.5556) + 32;
    }
    static float f2k( double F ) {
        return 273.15 + ((F - 32)*.5556);
    }
    float calcTemp( float R, unsigned itherm );

    static constexpr float    VREF       = 2.2;
    static const unsigned R_pulldown = 5100;

    struct CalSession 
    {
        CalSession( TempController* tc )
            : m_tc(tc), m_nspec(0)
        {}
        void reset() {
            m_nspec = 0;
        }
        bool addPoint( unsigned adc, float TF ) {
            if (m_nspec > 2)
                return false;

            m_R[m_nspec] = m_tc->adc2R(adc);
            m_TF[m_nspec] = TF;

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

  protected:
    unsigned char   m_ctrlPin;
    unsigned char   m_temp1Pin;
    unsigned char   m_temp2Pin;
    unsigned        m_confAddr;
    Avg<16,float>   m_temp1Value;
    Avg<16,float>   m_temp2Value;
    bool            m_heatOn;
    //
    // EEPROM values.
    unsigned        m_setTemp;  // Target temp.
    float           m_VCC;      // Reference VCC.
    float           m_A[2];      // Cal constant A for each thermistor.
    float           m_B[2];      // Cal constant B for each thermistor.
    float           m_C[2];      // Cal constant C for each thermistor.

};

#endif
