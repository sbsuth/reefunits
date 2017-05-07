#ifndef DISTANCE_SENSOR_H
#define DISTANCE_SENSOR_H

#include "Avg.h"
#include "NewPing.h"

#define PING_INTERVAL_MS 20


// Class to collect data from a single ultrasonic distance sensor.
// Keeps a moving avg of 16 values.  Presumes there is only 1 instance
// in the program!
class SingleDistanceSensor : public NewPing
{
  public:
    SingleDistanceSensor( int trig, int echo );
    unsigned short update();

  protected:
    static SingleDistanceSensor*    m_sensor;
    static void getEchoCB() {
        m_sensor->getEcho();
    }
    void getEcho();

    Avg<16> m_values;
    unsigned long m_nextPing;
    bool m_waiting;

};

#endif
