#ifndef LEDS_H
#define LEDS_H

// LEDs
#define FIRST_LED       2
#define LAST_LED        9
#define SKIP_LED(id)    (id == 4)
#define WHITE_LED       8
#define VIOLET_LED      7
#define ROYAL_BLUE_LED  6
#define BLUE_LED        5
#define CYAN_LED        9
#define RED_LED         3
#define AMBER_LED       2

class Leds 
{
  public:
    Leds()
      {}

    void init() {
        for ( char led=FIRST_LED; led <= LAST_LED; led++ ) {
            if (SKIP_LED(led))
                continue;
            pinMode( led, OUTPUT );
        }
        dimAll(0);
    }
    void dimAll( unsigned char pct ) {
        if (pct > 100)
            pct = 100;
        unsigned int val = ((unsigned)pct * 255)/100;
        for ( char led=FIRST_LED; led <= LAST_LED; led++ ) {
            if (SKIP_LED(led)) 
                continue;
            int i = (led - FIRST_LED);
            m_val[i] = val;
            analogWrite( led, val );
        }
    }
    void dimOne( int led, unsigned char pct ) {
        if (pct > 100)
            pct = 100;
        if ((led < FIRST_LED) || (led > LAST_LED) || SKIP_LED(led))
            return;
        unsigned char val = ((unsigned)pct * 255)/100;
        int i = (led - FIRST_LED);
        m_val[i] = val;
        analogWrite( led, val );
    }
  protected:
    unsigned char   m_val[(LAST_LED - FIRST_LED) + 1];
};

#endif

