#include "DistanceSensor.h"



// static data decl
SingleDistanceSensor*    SingleDistanceSensor::m_sensor = 0;

SingleDistanceSensor::SingleDistanceSensor( int trig, int echo, int maxDist )
    : NewPing( trig, echo, maxDist )
    , m_nextPing(millis())
    , m_waiting(false)
    , m_lastSample(0)
{
    m_sensor = this; // only one!  Could make a vector to expand
}

// Called from main loop to update and get newest value.
// Sets 'changed' to true if the value is different to the last time
// update() was called.
bool SingleDistanceSensor::update( unsigned short& val ) {

    unsigned long curTime = millis();
    bool timedOut = (curTime > (m_nextPing + PING_TIMEOUT_MS));

    if (!m_waiting || timedOut) {
        if (curTime > m_nextPing) {
            ping_timer(getEchoCB);
            m_waiting = true;
            m_nextPing = curTime + PING_INTERVAL_MS;
        }
    }
    val = m_values.avg();
    bool changed = (val != m_lastSample);
    m_lastSample = val;
    return changed;
}

// Non-static timer callback.
void SingleDistanceSensor::getEcho() {
    if (check_timer()) {
        unsigned short distanceCM = (ping_result / US_ROUNDTRIP_CM);
        m_values.update(distanceCM);
        m_waiting = false;
    }
}
