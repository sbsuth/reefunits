#include <Arduino.h>
#include "leds.h"
#include <EEPROM.h>
#include <Time.h>

#define RECALC_TIMED_MS 5000

void Leds::init() {
    for ( unsigned char iled=0; iled < NUM_LED_NUMS; iled++ ) {
        unsigned led = iled + FIRST_LED;
        if (SKIP_LED(led)) {
            m_curVals[iled] = -1;
            for (unsigned ispec=0; ispec < NUM_SPECTRUMS; ispec++)
                m_ledPcts[ispec][iled] = -1;
        } else {
            m_curVals[iled] = 0;
            for (unsigned ispec=0; ispec < NUM_SPECTRUMS; ispec++)
                m_ledPcts[ispec][iled] = 100;
            pinMode( led, OUTPUT );
        }
    }
    setTime( 1530514800 ); // A midnight
    dimAll(0);

}

unsigned Leds::saveSettings()
{
    unsigned addr = m_confAddr;

    for (unsigned char spec=0; spec < NUM_SPECTRUMS; spec++ ) {
        for ( unsigned char ichan=0; ichan < NUM_LED_NUMS; ichan++ ) {
            unsigned char pct = m_ledPcts[spec][ichan];
            EEPROM.put( addr++, pct );
        }
    }
    EEPROM.put( addr++, m_curSpectrum );
    EEPROM.put( addr++, m_highPct );
    EEPROM.put( addr++, m_lowPct );
    EEPROM.put( addr, m_latitude );
    addr += sizeof(float);
    EEPROM.put( addr, m_longitude );
    addr += sizeof(float);
    EEPROM.put( addr++, m_timezone );
    EEPROM.put( addr, m_offsetSec );
    addr += sizeof(int);
    EEPROM.put( addr, (unsigned char)m_mode );
}

unsigned Leds::restoreSettings()
{
    unsigned addr = m_confAddr;

    for (unsigned char spec=0; spec < NUM_SPECTRUMS; spec++ ) {
        for ( unsigned char ichan=0; ichan < NUM_LED_NUMS; ichan++ ) {
            unsigned char pct = 0;
            EEPROM.get( addr++, pct );
            m_ledPcts[spec][ichan] = pct;
        }
    }
    EEPROM.get( addr++, m_curSpectrum );
    EEPROM.get( addr++, m_highPct );
    EEPROM.get( addr++, m_lowPct );
    EEPROM.get( addr, m_latitude );
    addr += sizeof(float);
    EEPROM.get( addr, m_longitude );
    addr += sizeof(float);
    EEPROM.get( addr++, m_timezone );
    EEPROM.get( addr, m_offsetSec );
    addr += sizeof(int);
    unsigned char mode = 0;
    EEPROM.get( addr, mode );
    m_mode = (mode < NumModes) ? (Mode)mode : Timed;
}


// The given time is as returned by time(), but adjusted for timezone.
void Leds::setTime( unsigned long t ) 
{
    ::setTime( t );
}

// Set level of all channels to the given pct, scaled by m_ledPcts.
void Leds::dimAll( unsigned char pct ) {
    if (pct > 100)
        pct = 100;
    unsigned int val = ((unsigned)pct * 255)/100;
    for ( char led=FIRST_LED; led <= LAST_LED; led++ ) {
        if (SKIP_LED(led)) 
            continue;
        int i = (led - FIRST_LED);
        unsigned int ledVal = ((unsigned)m_ledPcts[m_curSpectrum][led] * val)/100;
        m_curVals[i] = ledVal;
        analogWrite( led, ledVal );
    }
}


// Set level of the given led index to the given pct.
// Ignores the m_ledPcts for the LED.
void Leds::dimOne( int led, unsigned char pct ) {
    if (pct > 100)
        pct = 100;
    if ((led < FIRST_LED) || (led > LAST_LED) || SKIP_LED(led))
        return;
    unsigned char val = ((unsigned)pct * 255)/100;
    int i = (led - FIRST_LED);
    m_curVals[i] = val;
    analogWrite( led, val );
}

float d2r( float deg )
{
    return (deg * M_PI) / 180.0;
}

//#define d2r(deg) (deg * M_PI) / 180.0

float r2d( float rad )
{
    return (rad * 180.0) / M_PI;
}

//#define r2d(rad)  (rad*180.0)/M_PI

float dcos( float deg )
{
    return cos(d2r(deg));
}

float dsin( float deg )
{
    return sin(d2r(deg));
}

float dtan( float deg )
{
    return tan(d2r(deg));
}


void Leds::updateSunAngle()
{
    float a = floor((14.0 - month()) / 12.0);   
    float y = year() + 4800.0 - a;   
    float m = month() + (12.0 * a) - 3.0;   
    float AH;   
    int result;   

    float JC = (((day() + floor(((153.0 * m) + 2.0) / 5.0) + (365.0 * y) + floor(y / 4.0) - floor(y / 100.0) + floor(y / 400.0) - 32045.0) + ((hour() / 24.0) + (minute() / 1444.0) + (second() / 86400.0))) - 2451556.08) / 36525.0;   

    float GMLS = fmod(280.46646 + JC*(36000.76983 + JC * 0.0003032), 360.0);   

    float GMAS = 357.52911 + JC * (35999.05029 - 0.0001537 * JC);   

    float EEO = 0.016708634 - JC * (0.000042037 + 0.0000001267 * JC);   

    float SEoC = dsin(GMAS)*(1.914602 - JC * (0.004817 + 0.000014 * JC)) + dsin(2.0 * GMAS) * (0.019993 - 0.000101 * JC) + dsin(3.0 * JC) * 0.000289;   

    float STL = GMLS + SEoC;   

    float STA = GMAS + SEoC;   

    float SRV = (1.000001018 * (1.0 - EEO * EEO)) / (1.0 + EEO * dcos(STA));   

    float SAL = STL - 0.00569 - 0.00478 * dsin(125.04 - 1934.136 * JC);   

    float MOE = 23.0 + (26.0 + ((21.448 - JC * (46.815 + JC * (0.00059 - JC * 0.001813)))) / 60.0) / 60.0;   

    float OC = MOE + 0.00256 * dcos(215.04 - 1934.136 * JC);   

    float SD = r2d(asin(   dsin(OC)    * dsin(SAL)   ));

    float vy = dtan(OC / 2.0) * dtan(OC / 2.0);

    float EQoT = r2d(4.0 * (vy * (    dsin(2.0 * GMLS)       - 2.0 * EEO * dsin(GMAS) + 4.0 * EEO * vy * dsin(GMAS) * dcos(2.0 * GMLS) - 0.5 * vy * vy * dsin(4.0 * GMLS) - 1.25 * EEO * EEO * 
    
        dsin(2 * GMAS)   )));

    float HAS = r2d( acos( dcos(90.833) / dcos(m_latitude) * dcos(SD) - dtan(m_latitude) * dtan(SD)) );

    float SN = (720.0 - 4.0 * m_longitude - EQoT + (int)m_timezone * 60.0);   

    float SR = SN - HAS * 4.0;   

    float SS = SN + HAS * 4.0;   

    float STD = 8.0 * HAS;   

    float TST = fmod((((hour())+(minute() / 60.0) + (second() / 3600.0)) / 24.0) * 1440.0 + EQoT + 4.0 * m_longitude - 60.0 * (int)m_timezone, 1440.0) + m_offsetSec;   

    if (TST / 4 < 0.0)   
    {   
        AH = ((TST / 4.0) + 180.0);   
    }   
    else   
    {   
        AH = ((TST / 4.0) - 180.0);   
    }   

    float SZA = r2d(acos(    dsin(m_latitude) * dsin(SD) + dcos(m_latitude) * dcos(SD) * dcos(AH)));

    float SEA = 90.0 - SZA;   

    m_sunAngle = 90.0 - SZA;   
}


// Air mass reference: http://www.ftexploring.com/solar-energy/air-mass-and-insolation2.htm


// Given the current solar angle and the m_highPct value, given the current global percentage for the lights.
void Leds::updateTimedPct()
{
    if (m_sunAngle < 0) {
        m_timedPct = 0;
        return;
    }

    float angleFactor = dcos(m_sunAngle);

    float AM;
    float SZA = 90.0 - m_sunAngle;
    if (SZA < 5)
        SZA = 5.0;
    AM = 1.0 / dcos(SZA); // Air Mass: ratio of mass of air traversed relative to overhead.
    m_amFactor = 1.353 * 1.1 * pow( 0.7, pow( AM, 0.678 ) );

    m_timedPct = floor((angleFactor * m_amFactor * m_highPct) + 0.5); 
}

unsigned Leds::getLightPct()
{
    switch (m_mode) {
        case Timed: 
            return m_timedPct;
        case Off:
            return 0;
        case Low:
            return m_lowPct;
        case High:
            return m_highPct;
        default:
            return 0;
    }
}

void Leds::update()
{
    if ((m_lastUpdate + RECALC_TIMED_MS) < millis()) {
        if (m_mode == Timed) {
            updateSunAngle();
            updateTimedPct();
        }
        m_lastUpdate = millis();
        pushVals();
    }
}

