#include <Arduino.h>
#include "leds.h"
#include <EEPROM.h>
#undef USE_JSON
#include "Command.h"
#include <Time.h>

//#define DEBUG_LIGHT 1

#define RECALC_TIMED_MS (60UL * 1000UL)

void Leds::init() {
    for ( unsigned char iled=0; iled < NUM_LED_NUMS; iled++ ) {
        unsigned led = iled + FIRST_LED;
        if (SKIP_LED(led)) {
            m_curVals[iled] = -1;
            for (unsigned ispec=0; ispec < NUM_SPECTRUMS; ispec++)
                m_ledPcts[ispec][iled] = 0;
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
    EEPROM.put( addr, m_sunriseSec );
    addr += sizeof(unsigned long);
    EEPROM.put( addr, m_periodSec );
    addr += sizeof(unsigned long);
    EEPROM.put( addr++, (unsigned char)m_mode );
    EEPROM.put( addr, m_normFactor );
    addr += sizeof(float);
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
    EEPROM.get( addr, m_sunriseSec );
    addr += sizeof(unsigned long);
    EEPROM.get( addr, m_periodSec );
    addr += sizeof(unsigned long);
    unsigned char mode = 0;
    EEPROM.get( addr++, mode );
    m_mode = (mode < NumModes) ? (Mode)mode : Timed;
    EEPROM.get( addr, m_normFactor );
    addr += sizeof(float);
}


// The given time is as returned by time(), but adjusted for timezone.
void Leds::setTime( unsigned long t ) 
{
    ::setTime( t );
    m_timeIsSet = true;
}
// Sets time temporarily, not affecting m_timeIsSet.
void Leds::tempSetTime( unsigned long t ) 
{
    ::setTime( t );
}

unsigned long Leds::getTimeSec() const
{
    return now();
}
// Get number of seconds since start of day.
// Optionally, adjust as is done for dimming
unsigned long Leds::getTimeOfDaySec( bool adjust ) 
{
    unsigned long timeDelta = 0;
    if (adjust) {
        timeDelta = adjustTime();
    }
    unsigned long t = (hour() * (60UL*60UL)) + (minute() * 60UL) + second();

    if (timeDelta)
        restoreTime(timeDelta);

    return t;
}

// Set level of all channels to the given pct, scaled by m_ledPcts.
void Leds::dimAll( unsigned char pct ) {
    if (pct > 100)
        pct = 100;
    unsigned int val = ((unsigned long)pct * 255UL)/100UL;
    for ( char led=FIRST_LED; led <= LAST_LED; led++ ) {
        if (SKIP_LED(led)) 
            continue;
        int i = (led - FIRST_LED);
        unsigned char ledPct = m_ledPcts[m_curSpectrum][i];
        unsigned int ledVal = ((unsigned)ledPct * val)/100;
        
        // If we would be off, but we have a positive value, then set 
        // colors >=50% to 1.
        // This gives us dominant colors on at a min when they would round down to 0.
        if ((ledVal == 0) && (ledPct >= 50) && (val > 0))
            ledVal = 1;

        m_curVals[i] = ledVal;
        analogWrite( led, ledVal );
    }
}


// Set level of the given led index to the given pct.
// Ignores the m_ledPcts for the LED.
void Leds::dimOne( int iled, unsigned char pct ) {
    int led = (iled + FIRST_LED);
    if (pct > 100)
        pct = 100;
    if ((led < FIRST_LED) || (led > LAST_LED) || SKIP_LED(led))
        return;
    unsigned int val = ((unsigned long)pct * 255UL)/100UL;
    m_curVals[iled] = val;
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
    unsigned long timeDelta = adjustTime();
    float a = floor((14.0 - month()) / 12.0);   
    float y = year() + 4800.0 - a;   
    float m = month() + (12.0 * a) - 3.0;   
    float AH;   

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

    float TST = fmod((((hour())+(minute() / 60.0) + (second() / 3600.0)) / 24.0) * 1440.0 + EQoT + 4.0 * m_longitude - 60.0 * (int)m_timezone, 1440.0);

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

    restoreTime( timeDelta );

    #if DEBUG_LIGHT
    Serial.print("DEBUG: updateSunAngle="); Serial.println(m_sunAngle);
    #endif
}



// Calculates the period data for the day.
// This is time consuming, and only needs to be done once per day, or when location or time or offset or scale are changed.
void Leds::updateCycle( bool force )
{
    if (!force && !m_cycleInvalid)
        return;
    m_cycleInvalid = false;

    // Save and restore time and sun angle around searches since the search has a side effect.
    unsigned long oldTime = now();
    unsigned long tbegin = millis();

    // Reset 'useCycle' to 'calcCycle' so adjustments are not done during sunrise/sunset calc.
    m_useCycle.sunriseSec = m_calcCycle.sunriseSec;
    m_useCycle.sunsetSec = m_calcCycle.sunsetSec;
    m_useCycle.peakSec = m_calcCycle.peakSec;
    m_offsetSec = 0;

    unsigned long hourSecs = 60UL * 60UL;
    m_dayStart = now() - ((hour()*hourSecs) + (minute()*60) + second());
    unsigned long t4AM = m_dayStart + (4 * hourSecs);
    unsigned long t9AM = m_dayStart + (9 * hourSecs);
    unsigned long t4PM = m_dayStart + (16 * hourSecs);
    unsigned long t10PM = m_dayStart + (22 * hourSecs);

    // Binsearch for sunrise and sunset.
    m_calcCycle.sunriseSec = binsearchTime( t4AM, t9AM, 0.0, &Leds::getSunAngleForTime );
    #if DEBUG_LIGHT
    Serial.print("DEBUG: sunrise=");Serial.println(toDaySec(m_calcCycle.sunriseSec));
    #endif
    m_calcCycle.sunsetSec = binsearchTime( t4PM, t10PM, 0.0, &Leds::getSunAngleForTime );
    #if DEBUG_LIGHT
    Serial.print("DEBUG: sunset=");Serial.println(toDaySec(m_calcCycle.sunsetSec));
    #endif

        // Presume max is right between.
        // Get the natural intensity pct, considering angle and air mass.
        m_calcCycle.peakSec = m_calcCycle.sunriseSec + ((m_calcCycle.sunsetSec - m_calcCycle.sunriseSec)/2);
        float peakAngle = getSunAngleForTime( m_calcCycle.peakSec );
        m_calcCycle.peakPct = floor( calcAngleFactor( peakAngle ) * calcAMFactor( peakAngle ) * 100.0);
        

        // calculate the used sunrise and sunset and set m_offsetSec to the linear offset for all 
        // times.  Any offset for specified photo period will be calcualted based on the current time.
        // The sense of m_offsetSec is such that usedTimed after calc'd times are positive.
        m_useCycle.sunriseSec = m_calcCycle.sunriseSec;
        m_useCycle.sunsetSec = m_calcCycle.sunsetSec;
        m_useCycle.peakPct = m_calcCycle.peakPct;

        // Calc change in period.  La=onger period is positive.
        long deltaPeriod = 0;
        if (m_periodSec != 0)
            deltaPeriod = ((long)m_periodSec - ((long)m_calcCycle.sunsetSec - (long)m_calcCycle.sunriseSec));

        if (m_sunriseSec) {
            // Fix sunrise, and use any specified period delta to adjust sunset.
            m_offsetSec = (m_sunriseSec - toDaySec(m_calcCycle.sunriseSec));
            m_useCycle.sunriseSec += m_offsetSec;
            m_useCycle.sunsetSec += m_offsetSec;
            if (m_periodSec) {
                m_useCycle.sunsetSec += deltaPeriod;
            }
        } else if (m_periodSec) {
            // Adjust sunrise and sunset so that period change is split before sunrise and after sunset.
            m_useCycle.sunriseSec -= deltaPeriod/2;
            m_useCycle.sunsetSec += deltaPeriod/2;
            m_offsetSec = -deltaPeriod/2;
        }
        m_useCycle.peakSec = m_useCycle.sunriseSec + ((m_useCycle.sunsetSec - m_useCycle.sunriseSec)/2);
        m_useCycle.peakPct = 100;
    #if 0
    Serial.print("HEY: m_sunriseSec=");Serial.println(m_sunriseSec);
    Serial.print("HEY: m_periodSec=");Serial.println(m_periodSec);
    Serial.print("HEY: delta=");Serial.println(delta);
    Serial.print("HEY: m_calcCycle.sunriseSec=");Serial.println(m_calcCycle.sunriseSec);
    Serial.print("HEY: m_calcCycle.sunsetSec=");Serial.println(m_calcCycle.sunsetSec);
    Serial.print("HEY: m_useCycle.sunriseSec=");Serial.println(m_useCycle.sunriseSec);
    Serial.print("HEY: toDaySec(m_useCycle.sunriseSec)=");Serial.println(toDaySec(m_useCycle.sunriseSec));
    Serial.print("HEY: Set m_offsetSec=");Serial.println(m_offsetSec);
    #endif
        
        // Calculate m_peakFactor, which will scale between values calculated during the say, and the
        // peak percent (which is fixed at 100).
        // If the m_normFactor is 1.0 seasonal max diffs are eliminated, and the peak is m_maxPct.
        // If its 0.0, we use the natural values for the location, and will usually never reach m_maxPct.
        float fullNorm = (m_calcCycle.peakPct > 0) ? ((float)m_useCycle.peakPct / (float)m_calcCycle.peakPct) : 1.0;
        if (fullNorm > 1.0) {
            m_peakFactor = 1.0 + ((fullNorm - 1.0) * m_normFactor);
        } else {
            m_peakFactor = 1.0; // Never decrease.
        }

        // Restore time, and recalc sun angle with ajusted time.
        tempSetTime(oldTime);
        updateSunAngle();
        updateTimedPct();

        #if DEBUG_LIGHT
        Serial.print("DEBUG: updateCycle, sunrise="); 
        Serial.print( toDaySec(m_calcCycle.sunriseSec));
        Serial.print(", sunset="); 
        Serial.print( toDaySec(m_calcCycle.sunsetSec));
        Serial.print(", peak="); 
        Serial.println( toDaySec(m_calcCycle.peakPct));
        Serial.print(", delay(ms)="); 
        Serial.println( millis() - tbegin );
        #endif
    }

    // Account for the offset time, and if within the photo period, the period time.
    // If the usedTime is after the calcualted time, a negative number should be returned.
    // The m_offsetSec value is a psotive number if used sunrise is later than calc'd sunrise,
    // so the base value is -m_offsetSec.  If there's a spec'd period, then the ratio between
    // the two is used, together with the dietance we are into the day to give an offset for the 
    // current time.  
    long Leds::calcTimeAdjustment()
    {
        // If outide of both ranges, don't worry about scaling.
        unsigned long nowSec = now();

        // If before use sunrise, adjustment is just offset.
        if (nowSec < m_useCycle.sunriseSec) {
            return -m_offsetSec;
        }

        unsigned long useRange = (m_useCycle.sunsetSec - m_useCycle.sunriseSec);
        unsigned long calcRange = (m_calcCycle.sunsetSec - m_calcCycle.sunriseSec);
        if (calcRange == useRange) {
            return -m_offsetSec;
        }

        if (useRange == 0) {
            return -m_offsetSec; // div by zero protection.
        }

        // Ratio is the difference between the ranges relative to the used range 
        // so that we get a use distance given a calc distance.
        // The adjustment is based on the distance past calc'd sunrise, adjusted for offset.
        float ratio = ((float)useRange - (float)calcRange)/(float)useRange;
        long scaleAdj = (((long)nowSec - m_offsetSec) - (long)m_calcCycle.sunriseSec) * ratio;
        long result = -(m_offsetSec + scaleAdj);
    #if 0
    Serial.print("HEY: ratio=");Serial.println(ratio);
    Serial.print("HEY: scaleAdj=");Serial.println(scaleAdj);
    Serial.print("HEY: m_offsetSec=");Serial.println(m_offsetSec);
    Serial.print("HEY: result=");Serial.println(result);
    #endif


        return result;
    }

    long Leds::adjustTime()
    {
        if (m_cycleInvalid || !m_timeIsSet)
            return 0;

        long adj = calcTimeAdjustment();

        ::adjustTime(adj);

        return adj;
    }

    void Leds::restoreTime( long delta )
    {
        ::adjustTime( -delta );
}

float Leds::calcAngleFactor( float sunAngle )
{
    return dcos(90.0-sunAngle);
}

float Leds::calcAMFactor( float sunAngle )
{
    float AM;
    float SZA = 90.0 - sunAngle;
    if (SZA < 5)
        SZA = 5.0;
    AM = 1.0 / dcos(SZA); // Air Mass: ratio of mass of air traversed relative to overhead.
    return 1.353 * 1.1 * pow( 0.7, pow( AM, 0.678 ) );
} 

// Given the current solar angle and the m_highPct value, given the current global percentage for the lights.
// Air mass reference: http://www.ftexploring.com/solar-energy/air-mass-and-insolation2.htm
void Leds::updateTimedPct()
{
    if (m_sunAngle < 0) {
        m_timedPct = 0;
        return;
    }

    m_angleFactor = calcAngleFactor(m_sunAngle);

    m_amFactor = calcAMFactor(m_sunAngle);
    
    m_timedPct = floor((m_angleFactor * m_amFactor * m_peakFactor * m_highPct ) + 0.5); 
    if (m_timedPct > 100) {
        m_timedPct = 100;
    }

    //  Special case near 0 such that if there is a positive angle, and non-zero
    //  factors, we use 1%.
    if ((m_timedPct == 0) && (m_sunAngle > 0.0) && (m_highPct > 0.0) && (m_peakFactor > 0.0))
        m_timedPct = 1;

    #if DEBUG_LIGHT
    //Serial.print("DEBUG: updateTimedPct="); Serial.println(m_timedPct);
    #endif
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
    if (hour() != m_lastHour) {
        // Update cycle at the start of every day.
        m_cycleInvalid = true;
        m_timeIsSet = false;
        m_lastHour = hour();
    }
    if ((((m_lastUpdate + RECALC_TIMED_MS) < millis()) || m_cycleInvalid) && m_timeIsSet) {
        updateAll();
        m_lastUpdate = millis();
        pushVals();
    }
}

void Leds::dumpDay( int numPoints, InStream* stream)
{
    if (numPoints <= 0)
        return;

    unsigned long tod = getTimeOfDaySec();
    unsigned long torig =  now();
    unsigned long incr = (24UL * 60UL * 60UL)/(unsigned long)numPoints;

    stream->write("(");
    
    invalidate(true);

    unsigned long t = torig-tod;
    for ( int i=0; i < numPoints; i++, t+=incr) {

        unsigned long markStart = millis();
        tempSetTime(t);
        updateAll();

        if (i==0) {
            const PeriodData* datas[2] = {&getCalculatedCycle(), &getUsedCycle()};
            for ( int i=0; i < 2; i++ ) {
                const PeriodData& data = *datas[i];
                stream->write( String( toDaySec(data.sunriseSec) ).c_str() );
                stream->write(",");
                stream->write( String( toDaySec(data.sunsetSec) ).c_str() );
                stream->write(",");
                stream->write( String( toDaySec(data.peakSec) ).c_str() );
                stream->write(",");
                stream->write( String( data.peakPct).c_str() );
                stream->write("\n");
            }
        }

        stream->write(String(i).c_str());
        stream->write(",");
        stream->write( String( toDaySec((unsigned long)t)).c_str() );
        stream->write(",");
        stream->write(String(m_sunAngle).c_str());
        stream->write(",");
        stream->write(String(m_angleFactor).c_str());
        stream->write(",");
        stream->write(String(m_amFactor).c_str());
        stream->write(",");
        stream->write(String(m_peakFactor).c_str());
        stream->write(",");
        stream->write(String(m_timedPct).c_str());
        stream->write(",");
        stream->write( String(millis() - markStart).c_str());
        stream->write("\n");
    }


    tempSetTime(torig);
    invalidate(true);
    updateAll();
}

// has side effects!  Sets time and m_sunAngle.
// Intended to be used during binsearch, which should save/restore on outside.
float Leds::getSunAngleForTime( unsigned long t )
{
    tempSetTime(t);
    updateSunAngle();
    return m_sunAngle;
}

// Searches for the closest point to target between start and end.
// Assumes the slope is the same sign over the entire range.
// Uses the given func to calculate the value.
// Presums the func has a side effect on 
unsigned long Leds::binsearchTime( unsigned long start, unsigned long end, float target, float (Leds::*func)( unsigned long )  )
{
    unsigned long minSep = 60UL * 2; // 2 min resolution 
    
    int dir = -1; 
    bool dirKnown = false;

    float vstart = (this->*func)(start);
    float vend = (this->*func)(end);

    int maxIters = (end - start) / minSep;

    for ( int iter=0; iter < maxIters; iter++ ) {
        unsigned long tsep = (end - start);
        if (tsep < minSep)
            break;

        if (dir < 0) {
            dir = (vstart < vend) ? 1 : 0;
        }
        unsigned long mid = start + ((end - start)/2UL);
        float vmid = (this->*func)(mid);
        
        if (dir) {
            if (vmid > target)  {
                end = mid;
                vend = vmid;
            } else {
                start = mid;
                vstart = vmid;
            }
        } else {
            if (vmid < target)  {
                end = mid;
                vend = vmid;
            } else {
                start = mid;
                vstart = vmid;
            }
        }
    }
    return start + ((end - start)/2UL);
}

