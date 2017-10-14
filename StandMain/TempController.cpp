#include <TempController.h>
#include <EEPROM.h>
#include <math.h>

TempController::TempController(   int ctrlPin, int temp1Pin, int temp2Pin, unsigned confAddr )
        : m_ctrlPin(ctrlPin), m_temp1Pin(temp1Pin), m_temp2Pin(temp2Pin), m_confAddr(confAddr), m_setTemp(0), m_heatOn(false)
    {}

void TempController::setup( bool useSettings )
{
    if (useSettings) {
        restoreSettings();
    } else {
        saveSettings();
    }
    pinMode( m_ctrlPin, OUTPUT );
    digitalWrite( m_ctrlPin, 0 );

    pinMode( m_temp1Pin, INPUT );
    pinMode( m_temp2Pin, INPUT );

    // The analog reference is global, so this is only valid if there are no other units doing analogRead().
    analogReference( INTERNAL2V56 );
}

bool TempController::update()
{
    digitalWrite( m_ctrlPin, m_heatOn ? HIGH : LOW );
}

unsigned TempController::ee_size() {
    return sizeof(m_setTemp)
        + sizeof(m_VCC)
        + sizeof(m_A)
        + sizeof(m_B)
        + sizeof(m_C);
}
void TempController::restoreSettings() {
    unsigned addr = m_confAddr;

    EEPROM.get( addr, m_setTemp );
    addr += sizeof(m_setTemp);

    EEPROM.get( addr, m_VCC );
    addr += sizeof(m_VCC);

    EEPROM.get( addr, m_A[0] );
    addr += sizeof(m_A[0]);

    EEPROM.get( addr, m_A[1] );
    addr += sizeof(m_A[1]);

    EEPROM.get( addr, m_B[0] );
    addr += sizeof(m_B[0]);

    EEPROM.get( addr, m_B[1] );
    addr += sizeof(m_B[1]);

    EEPROM.get( addr, m_C[0] );
    addr += sizeof(m_C[0]);

    EEPROM.get( addr, m_C[1] );
    addr += sizeof(m_C[1]);
}

void TempController::saveSettings() {
    unsigned addr = m_confAddr;

    EEPROM.put( addr, m_setTemp );
    addr += sizeof(m_setTemp);

    EEPROM.put( addr, m_VCC );
    addr += sizeof(m_VCC);

    EEPROM.put( addr, m_A[0] );
    addr += sizeof(m_A[0]);

    EEPROM.put( addr, m_A[1] );
    addr += sizeof(m_A[1]);

    EEPROM.put( addr, m_B[0] );
    addr += sizeof(m_B[0]);

    EEPROM.put( addr, m_B[1] );
    addr += sizeof(m_B[1]);

    EEPROM.put( addr, m_C[0] );
    addr += sizeof(m_C[0]);

    EEPROM.put( addr, m_C[1] );
    addr += sizeof(m_C[1]);
}

// Convert an ADC reading to a thermistor resistance based
// a fixed 5.1k pulldown resistor value, a fixed 2.2V reference,
// and a VCC that can be calibrated.
float TempController::adc2R( unsigned adc ) {
    float V = (adc * VREF)/1024;
    return (R_pulldown * (m_VCC - V))/V;
}

// Given thermistor 3 resistance values, 3 temperatures measured externally at
// the same time, solves the Steiner-Hart equation to for its 3 constants.
// Returns true if a solution was possible.
bool TempController::setCal(    const CalSession& data, unsigned itherm )
{
     bool success = false; 
     float& A = m_A[itherm];
     float& B = m_B[itherm];
     float& C = m_C[itherm];
     const float& R0 = data.m_R[0];
     const float& R1 = data.m_R[1];
     const float& R2 = data.m_R[2];
     
     float T0 = f2k(data.m_TF[0]);
     float T1 = f2k(data.m_TF[1]);
     float T2 = f2k(data.m_TF[2]);
     if (R0>5 && R1>5 && R2>5 && R0!=R1 && R1!=R2 && R0!=R2 && T0!=T1) {
        float lR0 = log(R0);
        float lR1 = log(R1);
        float lR2 = log(R2);
        float iT0 = 1/T0;
        float iT1 = 1/T1;
        float iT2 = 1/T2;
        float lR0_3 = pow(lR0,3);
        float lR1_3 = pow(lR1,3);
        float lR2_3 = pow(lR2,3);
        C =   ((iT0-iT1)     - (((lR0-lR1) * (iT0-iT2))/(lR0-lR2))) 
            / ((lR0_3-lR1_3) - (((lR0-lR1) * (lR0_3-lR2_3))/(lR0-lR2)));
        B = ((iT0-iT1) - (C * (lR0_3-lR1_3))) / (lR0-lR1);
        A = iT0 - C*lR0_3 - B*lR0;

        saveSettings();

        return true;
    } else {
        A = 0;
        B = 0;
        C = 0;
        return false;
    }
}

// Given a thermistor resistance value, and a thermistor index
// to specify cal constants, uses the Steiner-Hart equation to
// calculate a temperature in Farenheit.
float TempController::calcTemp( float R, unsigned itherm )
{
     float lR = log(R);
     float T = m_A[itherm] + m_B[itherm] * lR + m_C[itherm] * pow(lR,3);
     T = 1.0/T;
     return k2f(T);
}
