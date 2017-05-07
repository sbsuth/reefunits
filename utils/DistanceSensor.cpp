#include "DistanceSensor.h"

// static data decl
SingleDistanceSensor*    SingleDistanceSensor::m_sensor = 0;

SingleDistanceSensor::SingleDistanceSensor( int trig, int echo )
    : NewPing( trig, echo, 40 )
    , m_nextPing(millis())
    , m_waiting(false)
{
    m_sensor = this; // only one!  Could make a vector to expand
}

// Called from main loop to update and get newest value.
unsigned short SingleDistanceSensor::update() {
    if (!m_waiting) {
        unsigned long curTime = millis();
        if (curTime > m_nextPing) {
            ping_timer(getEchoCB);
            m_waiting = true;
            m_nextPing = curTime + PING_INTERVAL_MS;
        }
    }
    return m_values.avg();
}

// Non-static timer callback.
void SingleDistanceSensor::getEcho() {
    if (check_timer()) {
        unsigned short distanceCM = (ping_result / US_ROUNDTRIP_CM);
        m_values.update(distanceCM);
        m_waiting = false;
    }
}