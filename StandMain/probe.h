#ifndef PROBE_H
#define PROBE_H 1

#include "TempController.h"

class AtlasProbe
{
  public:
    AtlasProbe( HardwareSerial& serial, TempController& tc )
        : m_serial(serial), m_tc(tc), m_lastTC(0)
    {
    }

    void setup() {
        m_serial.begin(9600);
        m_buf.reserve(40);                            

        // set to continuous reading mode.
        m_serial.print("C,1");
        m_serial.print('\r');


        // Turn off LED
        m_serial.print("L,0");
        m_serial.print('\r');
    }
    void sendCmd( const char* cmd, String& resp ) {
        if (m_serial.available()) 
            m_buf = m_serial.readStringUntil(13);        
        m_serial.print(cmd);
        m_serial.print('\r');
        unsigned long start = millis();
        bool done = false;
        while ( !done && (millis() < (start + 1000))) {
            if (m_serial.available()) {
                resp = m_serial.readStringUntil(13);        
                done = true;
            }
        }
        if (!done) 
            resp = "Timed out!";
    }
    void sendCmd( const String& cmd, String& resp ) {
        sendCmd( cmd.c_str(), resp );
    }

    bool update() {
        // Update temp compensation once per minute.
        if ((millis() - m_lastTC) > (60UL*1000UL)) {
            String cmd("T,");
            cmd.concat( TempController::f2c(m_tc.curTemp()));
            String rslt;
            sendCmd(cmd,rslt);
            m_lastTC = millis();
        }

        if (m_serial.available()) {
            m_buf = m_serial.readStringUntil(13);        
            return true;
        } else {
            return false;
        }
    }
    String& lastValueRead() {
        return m_buf;
    }

    bool sendCalCmd( const char* cmd )
    {
        String resp;
        
        // Exit continuous read mode.
        sendCmd("C,0",resp);
        
        // Send.
        sendCmd( cmd, resp );

        // Look for *OK
        bool ok = (resp[0] == '*') && (resp[1] == 'O') && (resp[2] == 'K');
        
        // Back to continuous read mode.
        sendCmd("C,1",resp);

        return ok;
    }
  protected:
    HardwareSerial& m_serial;
    String  m_buf;
    TempController& m_tc;
    unsigned long m_lastTC;
};

class pHProbe : public AtlasProbe
{
  public:
    pHProbe( HardwareSerial& serial, TempController& tc )
        : AtlasProbe(serial,tc)
        , m_lastValue(0.0)
    {}

    bool update() {
        bool changed = false;
        if (AtlasProbe::update()) {
            float val = m_buf.toFloat();
            if (val != 0.0) {
                changed = (m_lastValue != val);
                m_lastValue = val;
            }
        }
        return changed;
    }

    bool calStep( int step )
    {
        const char* cmd;
        bool ok = true;
        switch (step) {
            case 0:
                cmd = "Cal,mid,7.0"; 
                break;
            case 1:
                cmd = "Cal,low,4.0"; 
                break;
            case 2:
                cmd = "Cal,high,10.0"; 
                break;
            default:
                ok = false;
        }
        if (ok) {
            ok = sendCalCmd(cmd);
        }
        return ok;
    }

    float lastValue() {
        return m_lastValue;
    }

  protected:
    float   m_lastValue;
};

class ConductivityProbe : public AtlasProbe
{
  public:
    ConductivityProbe( HardwareSerial& serial, TempController& tc )
        : AtlasProbe(serial,tc)
    {
        for ( int i=0; i < 4; i++ )
            m_lastValues[i] = 0.0;
    }

    void setup() {
        AtlasProbe::setup();
    }

    bool update() {
        bool changed = false;
        if (AtlasProbe::update()) {
            // If the first char is an integer, expect 4 comma-separted integers
            // that are EC, TDS, SALINITY, SPEC_GRAVITY.
            char sbuf[40];
            m_buf.toCharArray(sbuf, sizeof(sbuf));
            if (isdigit(sbuf[0])) {
                int i=0;
                for ( char* tok = strtok( sbuf, ","); (i<4) && tok; i++ ) {
                    float val = atof(tok);
                    changed |= (val != m_lastValues[i]);
                    m_lastValues[i] = val;
                    tok = strtok(0,",");
                }
            }
        }
        return changed;
    }

    bool calStep( int step )
    {
        const char* cmd;
        bool ok = true;
        switch (step) {
            case 0:
                cmd = "Cal,dry"; 
                break;
            case 1:
                cmd = "Cal,low,12880"; 
                break;
            case 2:
                cmd = "Cal,high,80000"; 
                break;
            default:
                ok = false;
        }
        if (ok) {
            ok = sendCalCmd(cmd);
        }
        return ok;
    }


    float lastValue(int i) {
        return m_lastValues[i];
    }

  protected:
    float     m_lastValues[4];
};

#endif
