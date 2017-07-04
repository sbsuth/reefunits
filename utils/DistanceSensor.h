#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "Avg.h"
#include "NewPing.h"

// Min interval for pings.
#define PING_INTERVAL_MS 20

// If we go longer than this many ms, presume we won't get an answer,
// which is what happens if you're over range.
#define PING_TIMEOUT_MS 100


// Class to collect data from a single ultrasonic distance sensor.
// Keeps a moving avg of 16 values.  Presumes there is only 1 instance
// in the program!
class SingleDistanceSensor : public NewPing
{
  public:
    SingleDistanceSensor( int trig, int echo, int maxDist );

    bool update( unsigned short & val );
    unsigned short lastSample() {
        return m_lastSample;
    }

  protected:
    static SingleDistanceSensor*    m_sensor;
    static void getEchoCB() {
        m_sensor->getEcho();
    }
    void getEcho();

    Avg<16> m_values;
    unsigned long m_nextPing;
    bool m_waiting;
    unsigned short m_lastSample;

};

#endif
