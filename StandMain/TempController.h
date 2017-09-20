#ifndef HEATER_H
#define HEATER_H

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

    static float adc2R( unsigned adc );

    const float    VREF       = 2.2;
    const unsigned R_pulldown = 5100;

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

    bool solve( float R1, float R2, float R3,
               float T1, float T2, float T3,
               float& A, float& B, float& C );
};

#endif
