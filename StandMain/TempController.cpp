#include <TempController.h>
#include <EEPROM.h>
#include <math.h>
#include <ArduinoJson.h>
#include "Command.h"


TempController::TempController(   int ctrlPin, int tempPin, int tempRes, unsigned confAddr )
        : m_ctrlPin(ctrlPin), m_confAddr(confAddr)
        , m_samplePeriod(1000)
        , m_tempSensors(tempPin,tempRes,m_samplePeriod)
        , m_setTemp(0), m_heatOn(false), m_lastOnOff(millis()),
        m_lastSampleTime(0), m_lastSampledProbe(0)
{
    m_tempPin = tempPin;

    // Defaults for EEPROM values.  Should be overwritten except for first clean exec.
    m_setTemp = 79;
    m_sensitivity = 0.25;
    m_VCC = 4.88; // Unusued
    
    // taken from good calibratrion.
    m_A[0] =  0.0;
    m_B[0] =  0.0; // Unusued
    m_C[0] =  0.0; // Unusued
    m_A[1] =  0.0;
    m_B[1] =  0.0; // Unusued
    m_C[1] =  0.0; // Unusued
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

    m_tempSensors.setup();

    // The analog reference is global, so this is only valid if there are no other units doing analogRead().
    analogReference( INTERNAL2V56 );
}

static double scaleFloat( float v ) {
    return v * 1000.0;
}

bool TempController::update()
{
    if (m_tempSensors.update()) {
        for ( int i=0; (i < 2) && (i < m_tempSensors.getNumDevices()); i++ ) {
            // Read temp F and store.  Reject any obviously bad.
            float TF = m_tempSensors.getTemp(i);
            if ( isGoodTemp(TF) ) {
                m_tempValue[i] = TF;
            } else {
                m_tempValue[i] = 0.0;
            }

        }
        // Set heater on/off based on current temps.
        calcOnOff();
    }
    
    // Push heat on.
    digitalWrite( m_ctrlPin, m_heatOn ? HIGH : LOW );

}



unsigned long TempController::timeSinceLastOnOff() {
    return (millis() - m_lastOnOff);
}

bool TempController::isCalibrated( int itherm ) {
    if ((itherm >= 0) && (itherm < 2)) {
        return m_A[itherm];
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
        + sizeof(m_VCC) // unusued
        + sizeof(m_A)
        + sizeof(m_B) // unusued
        + sizeof(m_C); // unusued
}
void TempController::restoreSettings() {
    unsigned addr = m_confAddr;

    EEPROM.get( addr, m_setTemp );
    addr += sizeof(m_setTemp);

    EEPROM.get( addr, m_samplePeriod );
    addr += sizeof(m_samplePeriod);
    m_tempSensors.setPeriodMs(m_samplePeriod);

    EEPROM.get( addr, m_sensitivity );
    addr += sizeof(m_sensitivity);

    EEPROM.get( addr, m_VCC ); // unusued
    addr += sizeof(m_VCC);

    EEPROM.get( addr, m_A[0] );
    addr += sizeof(m_A[0]);

    EEPROM.get( addr, m_A[1] );
    addr += sizeof(m_A[1]);

    EEPROM.get( addr, m_B[0] ); // unused
    addr += sizeof(m_B[0]);

    EEPROM.get( addr, m_B[1] ); // unused
    addr += sizeof(m_B[1]);

    EEPROM.get( addr, m_C[0] ); // unused
    addr += sizeof(m_C[0]);

    EEPROM.get( addr, m_C[1] ); // unused
    addr += sizeof(m_C[1]);

   #if DEBUG_TEMP
   Serial.print("TEMP: A[0]=");Serial.println(String(scaleFloat(m_A[0]),7));
   Serial.print("TEMP: A[1]=");Serial.println(String(scaleFloat(m_A[1]),7));
   //Serial.print("TEMP: B[0]=");Serial.println(String(scaleFloat(m_B[0]),7));
   //Serial.print("TEMP: B[1]=");Serial.println(String(scaleFloat(m_B[1]),7));
   //Serial.print("TEMP: C[0]=");Serial.println(String(scaleFloat(m_C[0]),7));
   //Serial.print("TEMP: C[1]=");Serial.println(String(scaleFloat(m_C[1]),7));
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

    EEPROM.put( addr, m_VCC ); // unused
    addr += sizeof(m_VCC);

    EEPROM.put( addr, m_A[0] );
    addr += sizeof(m_A[0]);

    EEPROM.put( addr, m_A[1] );
    addr += sizeof(m_A[1]);

    EEPROM.put( addr, m_B[0] ); // unused
    addr += sizeof(m_B[0]);

    EEPROM.put( addr, m_B[1] ); // unused
    addr += sizeof(m_B[1]);

    EEPROM.put( addr, m_C[0] ); // unused
    addr += sizeof(m_C[0]);

    EEPROM.put( addr, m_C[1] ); // unused
    addr += sizeof(m_C[1]);
}

float TempController::curTemp( int itherm, bool raw ) {
    if ((itherm >= 0) && (itherm < 2)) {
        if (itherm < m_tempSensors.getNumDevices()) {
            return m_tempValue[itherm] + ((raw ? 0.0 : getOffset(itherm)));
        } else {
            return 0.0;
        }
    } else if (m_tempSensors.getNumDevices() >= 2) {
        float tf0 = curTemp(0,raw);
        float tf1 = curTemp(1,raw);
        if (isGoodTemp(tf0) && isGoodTemp(tf1)) {
            return (tf0 + tf1)/2.0;
        } else if (isGoodTemp(tf0)) {
            return tf0;
        } else {
            return tf1;
        }
    } else if (m_tempSensors.getNumDevices() == 1) {
        return curTemp(0,raw);
    } else {
        return 0.0;
    }
}

float TempController::getOffset( int devIndex )
{
    int addrIndex = m_tempSensors.getAddrIndexForDevIndex(devIndex);
    for ( int i=0; i < 2; i++ ) {
        if ( int(m_B[i]) == addrIndex )
            return m_A[i];
    }
    return 0.0;
}

// Uses the current "correct" temp to set the offset for each current sensor.
// Stores in EEPROM.
bool TempController::calUsingTemp( float TF )
{
    int nDev = m_tempSensors.getNumDevices();
    for ( int devIndex=0; devIndex < nDev; devIndex++ ) {
        int addrIndex = m_tempSensors.getAddrIndexForDevIndex(devIndex);
        float rawTemp = curTemp(devIndex,true);
        float offset = TF - rawTemp;
        m_A[devIndex] = offset;
        m_B[devIndex] = addrIndex; // Tag with addrIndex so not dependent on order.
    }
    saveSettings();
    return (nDev > 0);
}

void TempController::ackCalConsts( Command* cmd )
{
    StaticJsonBuffer<64> jsonBuffer;

    JsonObject& json = jsonBuffer.createObject();
    json["A0"] = String(scaleFloat(m_A[0]),7);
    json["A1"] = String(scaleFloat(m_A[1]),7);
    cmd->ack( json );
   /*
    https://bblanchon.github.io/ArduinoJson/assistant/
   {
     "A0": "0.0001234",
     "A1": "0.0001234"
   }
    50
   */
}
