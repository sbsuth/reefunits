#ifndef PROBE_H
#define PROBE_H 1

class AtlasProbe
{
  public:
    AtlasProbe( HardwareSerial& serial )
        : m_serial(serial)
    {
    }

    void setup() {
        m_serial.begin(9600);
        m_buf.reserve(30);                            

        // Explicitly set to continuous reading mode.
        m_serial.print("C,1");
        m_serial.print('\r');
    }

    bool update() {
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
  protected:
    HardwareSerial& m_serial;
    String  m_buf;
};

class pHProbe : public AtlasProbe
{
  public:
    pHProbe( HardwareSerial& serial )
        : AtlasProbe(serial)
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

    float lastValue() {
        return m_lastValue;
    }

  protected:
    float   m_lastValue;
};

class ConductivityProbe : public AtlasProbe
{
  public:
    ConductivityProbe( HardwareSerial& serial )
        : AtlasProbe(serial)
    {
        for ( int i=0; i < 4; i++ )
            m_lastValues[i] = 0;
    }

    bool update() {
        bool changed = false;
        if (AtlasProbe::update()) {
            // If the first char is an integer, expect 4 comma-separted integers
            // that are EC, TDS, SALINITY, SPEC_GRAVITY.
            char sbuf[30];
            m_buf.toCharArray(sbuf, sizeof(sbuf));
            if (isdigit(sbuf[0])) {
                int i=0;
                for ( char* tok = strtok( sbuf, ","); (i<4) && (tok=strtok(0,",")); i++ ) {
                    int val = atoi(tok);
                    changed |= (val != m_lastValues[i]);
                    m_lastValues[i] = val;
                }
            }
        }
        return changed;
    }

    int lastValue(int i) {
        return m_lastValues[i];
    }

  protected:
    int     m_lastValues[4];
};

#endif
