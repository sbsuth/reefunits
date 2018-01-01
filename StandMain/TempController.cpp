#include <TempController.h>
#include <EEPROM.h>
#include <math.h>
#include <ArduinoJson.h>
#include "Command.h"


TempController::TempController(   int ctrlPin, int temp1Pin, int temp2Pin, unsigned confAddr )
        : m_ctrlPin(ctrlPin), m_confAddr(confAddr)
        , m_setTemp(0), m_heatOn(false), m_lastOnOff(millis()),
        m_lastSampleTime(0), m_lastSampledProbe(0)
{
    m_cal[0].setTC(this);
    m_cal[1].setTC(this);
    m_tempPin[1] = temp2Pin;
    m_tempPin[0] = temp1Pin;
    m_tempPin[1] = temp2Pin;

    // Defaults for EEPROM values.  Should be overwritten except for first clean exec.
    m_setTemp = 79;
    m_samplePeriod = 500;
    m_sensitivity = 0.25;
    m_VCC = 4.88;
    
    // Taken from an initial calibration.  But they don't seem right, so update on next cal.
    m_A[0] =  0.0023715203;
    m_B[0] =  0.0000152698;
    m_C[0] =  0.0000010045;
    m_A[1] =  0.0002206226;
    m_B[1] =  0.0003594418;
    m_C[1] = -0.0000002995;
}

void TempController::setup( bool useSettings )
{
    if (useSettings) {
        restoreSettings();
    } else {
        saveSettings();
    }
    m_heatOn = false;
    m_lastOnOff = millis();

    pinMode( m_ctrlPin, OUTPUT );
    digitalWrite( m_ctrlPin, 0 );

    pinMode( m_tempPin[0], INPUT );
    pinMode( m_tempPin[1], INPUT );

    // The analog reference is global, so this is only valid if there are no other units doing analogRead().
    analogReference( INTERNAL2V56 );
}

static double scaleFloat( float v ) {
    return v * 1000.0;
}

bool TempController::update()
{
    // Sample temp on one sensor every m_samplePeriod ms.
    unsigned long curTime = millis();
    if ((curTime - m_lastSampleTime) > m_samplePeriod) {
        m_lastSampledProbe = (m_lastSampledProbe ? 0 : 1);
        m_lastSampleTime = curTime;
        if (isCalibrated(m_lastSampledProbe)) {
            unsigned adc = analogRead( pinFor(m_lastSampledProbe) );
            float R = adc2R(adc);
            float TF = calcTemp( R, m_lastSampledProbe );
            #if DEBUG_TEMP
            Serial.print("TEMP: ");Serial.print(m_lastSampledProbe);Serial.print(": adc=");Serial.print(adc);
            Serial.print(", R=");Serial.print(R);
            Serial.print(", TF=");Serial.print(TF);
            Serial.println("");
            #endif
            m_tempValue[m_lastSampledProbe].update(TF);

            // Set heater on/off based on current temps.
            calcOnOff();
        }
    }
    
    // Push heat on.
    digitalWrite( m_ctrlPin, m_heatOn ? HIGH : LOW );

}



unsigned long TempController::timeSinceLastOnOff() {
    return (millis() - m_lastOnOff);
}

bool TempController::isCalibrated( int itherm ) {
    if ((itherm >= 0) && (itherm < 2)) {
        return m_A[itherm] != 0.0 && m_B[itherm] != 0.0 && m_C[itherm] != 0.0;
    } else {
        return isCalibrated(0) && isCalibrated(1);
    }
}

// Turns the header off if the ave temp is > m_thresh above set temp
// off if below.
void TempController::calcOnOff()
{
    if (!isCalibrated())
        return;

    float ave = curTemp();
    float diff = (ave - m_setTemp);
    if (diff > m_sensitivity) {
        setHeaterOn(false);
    } else if (diff < -m_sensitivity) {
        setHeaterOn(true);
    }
}

unsigned TempController::ee_size() {
    return sizeof(m_setTemp)
        + sizeof(m_samplePeriod)
        + sizeof(m_sensitivity)
        + sizeof(m_VCC)
        + sizeof(m_A)
        + sizeof(m_B)
        + sizeof(m_C);
}
void TempController::restoreSettings() {
    unsigned addr = m_confAddr;

    EEPROM.get( addr, m_setTemp );
    addr += sizeof(m_setTemp);

    EEPROM.get( addr, m_samplePeriod );
    addr += sizeof(m_samplePeriod);

    EEPROM.get( addr, m_sensitivity );
    addr += sizeof(m_sensitivity);

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

   #if DEBUG_TEMP
   Serial.print("TEMP: A[0]=");Serial.println(String(scaleFloat(m_A[0]),7));
   Serial.print("TEMP: B[0]=");Serial.println(String(scaleFloat(m_B[0]),7));
   Serial.print("TEMP: C[0]=");Serial.println(String(scaleFloat(m_C[0]),7));
   Serial.print("TEMP: A[1]=");Serial.println(String(scaleFloat(m_A[1]),7));
   Serial.print("TEMP: B[1]=");Serial.println(String(scaleFloat(m_B[1]),7));
   Serial.print("TEMP: C[1]=");Serial.println(String(scaleFloat(m_C[1]),7));
   #endif
}

void TempController::saveSettings() {
    unsigned addr = m_confAddr;

    EEPROM.put( addr, m_setTemp );
    addr += sizeof(m_setTemp);

    EEPROM.put( addr, m_samplePeriod );
    addr += sizeof(m_samplePeriod);

    EEPROM.put( addr, m_sensitivity );
    addr += sizeof(m_sensitivity);

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
// 
//  VCC --- R_therm --- ADC --- R_pulldown --- GND
//  
float TempController::adc2R( unsigned adc ) {
    float V = (adc * VREF)/1024;
    return (R_pulldown * (m_VCC - V))/V; // ZERO DENOM!
}

// Adds a calibration point given a step and an externally measure temp.
// Measures the current ADC reading to go with it.
// If step is 0, resets.
// Otherwise, if step is not the next expected step, returns false.
// If step is 2, sets the calibration.
bool TempController::calStep( unsigned step, unsigned itherm, float TF )
{
    if (step == 0) {
        m_cal[itherm].reset();
    } else if (step != m_cal[itherm].nextStep()) {
        return false;
    }
    unsigned adc = analogRead( pinFor(itherm) );
    if (m_cal[itherm].addPoint( adc, TF ))
        return setCal( m_cal[itherm], itherm );
    else
        return true;
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
        float iT0 = 1/T0; // ZERO DENOM!
        float iT1 = 1/T1; // ZERO DENOM!
        float iT2 = 1/T2; // ZERO DENOM!
        float lR0_3 = pow(lR0,3);
        float lR1_3 = pow(lR1,3);
        float lR2_3 = pow(lR2,3);
        C =   ((iT0-iT1)     - (((lR0-lR1) * (iT0-iT2))/(lR0-lR2)))  // ZERO DENOM!
            / ((lR0_3-lR1_3) - (((lR0-lR1) * (lR0_3-lR2_3))/(lR0-lR2))); // ZERO DENOM!
        B = ((iT0-iT1) - (C * (lR0_3-lR1_3))) / (lR0-lR1); // ZERO DENOM!
        A = iT0 - C*lR0_3 - B*lR0;

        #if DEBUG_TEMP
        Serial.print("TEMP: Finish cal for probe ");Serial.print(itherm);Serial.print(", A=");Serial.print(A);Serial.print(",B=");Serial.print(B);Serial.print(", C=");Serial.println(C);
        #endif

        saveSettings();

        return true;
    } else {
        A = 0;
        B = 0;
        C = 0;
        #if DEBUG_TEMP
        Serial.print("TEMP: Finish cal for probe: failed: bad input");
        #endif
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
     T = 1.0/T; // ZERO DENOM!
     return k2f(T);
}

void TempController::ackCalConsts( Command* cmd )
{
    StaticJsonBuffer<160> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["A0"] = String(scaleFloat(m_A[0]),7);
    json["B0"] = String(scaleFloat(m_B[0]),7);
    json["C0"] = String(scaleFloat(m_C[0]),7);
    json["A1"] = String(scaleFloat(m_A[1]),7);
    json["B1"] = String(scaleFloat(m_B[1]),7);
    json["C1"] = String(scaleFloat(m_C[1]),7);
    cmd->ack( json );
   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
   {
     "A0": "0.0001234",
     "B0": "0.0001234",
     "C0": "0.0001234",
     "A1": "0.0001234",
     "B1": "0.0001234",
     "C1": "0.0001234",
   }
    142
   */
}
