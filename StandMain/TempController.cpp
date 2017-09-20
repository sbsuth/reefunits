#include <TempController.h>

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

static unsigned TempController::ee_size() {
    return sizeof(m_setTemp);
}
void TempController::restoreSettings() {
    unsigned addr = m_confAddr;

    EEPROM.get( addr, m_setTemp );
    addr += sizeof(m_setTemp);
}

void TempController::saveSettings() {
    unsigned addr = m_confAddr;

    EEPROM.put( addr, m_setTemp );
    addr += sizeof(m_setTemp);
}

// Convert an ADC reading to a thermistor resistance based
// a fixed 5.1k pulldown resistor value, a fixed 2.2V reference,
// and a VCC that can be calibrated.
float TempController::adc2R( unsigned adc ) {
    float V = (adc * VREF)/1024;
    return (R_pulldown * (m_VCC - V))/V;
}

bool TempController::solve( float R1, float R2, float R3,
                            float T1, float T2, float T3,
                            float& A, float& B, float& C )
{
     bool success = false; 
     C = 0;
     B = 0;
     A = 0;
     
     T1 = 273.15 + ((T1 - 32)*.5556);
     T2 = 273.15 + ((T2 - 32)*.5556);
     T3 = 273.15 + ((T3 - 32)*.5556);
     if (R1>5 && R2>5 && R3>5 && R1!=R2 && R2!=R3 && R1!=R3 && T1!=T2) {
        float lR1 = log(R1);
        float lR2 = log(R2);
        float lR3 = log(R3);
        float iT1 = 1/T1;
        float iT2 = 1/T2;
        float iT3 = 1/T3;
        float lR1_3 = pow(lR1,3);
        float lR2_3 = pow(lR2,3);
        float lR3_3 = pow(lR3,3);
        C =   ((iT1-iT2)     - (((lR1-lR2) * (iT1-iT3))/(lR1-lR3))) 
            / ((lR1_3-lR2_3) - (((lR1-lR2) * (lR1_3-lR3_3))/(lR1-lR3)));
        B = ((iT1-iT2) - (C * (lR1_3-lR2_3))) / (lR1-lR2);
        A = iT1 - C*lR1_3 - B*lR1;

        success = true;
    }
    return success;
}
