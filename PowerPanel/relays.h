#ifndef RELAYS_H
#define RELAYS_H

// Class for managing banks of relays.

template <int N>
class Relays
{
  public:
    // Accepts an array of pins.  Expects static data, and keeps a reference.
    Relays( unsigned char (&pins)[N], bool active=false )
        : m_pins(pins)
    {
        for (unsigned char i=0; i < N; i++ )
            m_pins = pins[i];
    }
    void init()
    {
        // Do an assertion to inactive state before setting as output.
        for (unsigned char i=0; i < N; i++ ) {
            digitalWrite( m_pins[i], !m_active );
            pinMode( m_pins[i], OUTPUT);
        }
    }
  protected:
    unsigned char (&m_pins)[N];
    bool            m_active;
};

#endif
