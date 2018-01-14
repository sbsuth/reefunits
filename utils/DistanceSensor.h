#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "Avg.h"
#include "NewPing.h"

// Min interval for pings.
#define PING_INTERVAL_MS 1000

// If we go longer than this many ms, presume we won't get an answer,
// which is what happens if you're over range.
#define PING_TIMEOUT_MS 200


// Class to collect data from a single ultrasonic distance sensor.
// Keeps a moving avg of 16 values.  Presumes there is only 1 instance
// in the program!
class SingleDistanceSensor : public NewPing
{
  public:
    typedef unsigned long value_type; // Enough resolution for raw values added up.
    typedef unsigned short cm_type;   // Enough to hold reduced values.

    SingleDistanceSensor( int trig, int echo, int maxDist );

    bool update( cm_type & val );
    cm_type currentCM() {
        return m_lastAvgCM;
    }
    bool isDead();

  protected:
    static SingleDistanceSensor*    m_sensor;
    static void getEchoCB() {
        m_sensor->getEcho();
    }
    void getEcho();

    Avg<16,value_type> m_values;
    unsigned long m_pingStarted;
    bool m_waiting;
    cm_type m_lastAvgCM;
    unsigned char m_numTimeouts;

};

#endif
