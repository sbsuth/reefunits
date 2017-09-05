#ifndef HEATER_H
#define HEATER_H

class TempController
{
  public:
    TempController(   int ctrlPin, int temp1Pin, int temp2Pin )
        : m_ctrlPin(ctrlPin), m_temp1Pin(temp1Pin), m_temp2Pin(temp2Pin)
    {}

    void setup()
    {
        pinMode( m_ctrlPin, OUTPUT );
        digitalWrite( m_ctrlPin, 0 );

        pinMode( m_temp1, INPUT );
        pinMode( m_temp2, INPUT );
    }
  protected:
    unsigned char   m_ctrlPin;
    unsigned char   m_temp1Pin;
    unsigned char   m_temp2Pin;
    Avg<16,float>   m_temp1Value;
    Avg<16,float>   m_temp2Value;
};

#endif
