#include "DistanceSensor.h"

#define DEAD_AFTER_TIMEOUTS 16
#define MIN_CHANGE_CM        5
#define MIN_CHANGE_MS        (MIN_CHANGE_CM * US_ROUNDTRIP_CM)

#define DEBUG_DISTANCE 0


// static data decl
SingleDistanceSensor*    SingleDistanceSensor::m_sensor = 0;

SingleDistanceSensor::SingleDistanceSensor( int trig, int echo, int maxDist )
    : NewPing( trig, echo, maxDist )
    , m_pingStarted(millis())
    , m_waiting(false)
    , m_lastAvgCM(0)
    , m_numTimeouts(0)
{
    m_sensor = this; // only one!  Could make a vector to expand

	//pinMode( echo, INPUT_PULLUP);     // SBS: More predicatable when disconnected.
}

// Called from main loop to update and get newest value.
// Sets 'changed' to true if the value is different to the last time
// update() was called.
bool SingleDistanceSensor::update( cm_type& val ) {

    unsigned long curTime = millis();
    bool timedOut = m_waiting && (curTime > (m_pingStarted + PING_TIMEOUT_MS));
    if (!m_waiting || timedOut) {
        if (curTime > (m_pingStarted + PING_INTERVAL_MS)) {
            ping_timer(getEchoCB);
            m_waiting = true;
            m_pingStarted =curTime;
        } else if (timedOut) {
            m_waiting = false;
        }
    }
    value_type rawAvg = m_values.avg();
    val = (rawAvg / (value_type)US_ROUNDTRIP_CM); // Reduce.

    // After a certain number of timeouts seen, val becomes 0.
    if (timedOut && (m_numTimeouts < DEAD_AFTER_TIMEOUTS)) {
        m_numTimeouts++;
        #if DEBUG_DISTANCE
        if (isDead()) {
            Serial.print(F("DISTANCE: Got max number timeouts"));
        }
        #endif
    }
    if (isDead())
        val = 0; 

    bool changed = (val != m_lastAvgCM);
    m_lastAvgCM = val;

    return changed;
}

// Non-static timer callback.
void SingleDistanceSensor::getEcho() {
    if (check_timer() && m_waiting) {

        if (   (ping_result > (3 * US_ROUNDTRIP_CM)) 
            && (   !m_values.isFull()  // Starting up.
                || (abs((long)m_values.avg() - (long)ping_result) <= MIN_CHANGE_MS))) { // Non-spurious change.

            #if DEBUG_DISTANCE
            Serial.print(F("DISTANCE: New value: "));Serial.print(ping_result);Serial.print(F(", avg="));Serial.println(m_values.avg());
            #endif
            m_values.update(ping_result);
            m_numTimeouts = 0;
        } else {
            #if DEBUG_DISTANCE
            Serial.print(F("DISTANCE: Reject value: "));Serial.print(ping_result);Serial.print(F(", avg="));Serial.println(m_values.avg());
            #endif
        }
        m_waiting = false;
    }
}
bool SingleDistanceSensor::isDead() {
    return (m_numTimeouts >= DEAD_AFTER_TIMEOUTS);
}
