#ifndef LEDS_H
#define LEDS_H

// LEDs
#define AMBER_LED       2
#define RED_LED         3
#define SKIPPED_LED     4
#define BLUE_LED        5
#define ROYAL_BLUE_LED  6
#define VIOLET_LED      7
#define WHITE_LED       8
#define CYAN_LED        9 
// Cyan sometimes labeled turquois

#define FIRST_LED       2
#define LAST_LED        9
#define NUM_LED_NUMS    ((LAST_LED - FIRST_LED) + 1)
#define NUM_LEDS        (LAST_LED - FIRST_LED)
#define SKIP_LED(id)    (id == SKIPPED_LED)
#define SKIP_LED_NUM(i)    SKIP_LED(i + FIRST_LED)


#define NUM_SPECTRUMS   2

struct PeriodData 
{
    PeriodData() 
        : isSet(false), sunriseSec(0), sunsetSec(0), peakSec(0)
    {}
    bool isSet;
    unsigned long sunriseSec;
    unsigned long sunsetSec;
    unsigned long peakSec;
    unsigned char peakPct;
};

class Leds 
{
  public:
    enum Mode {
        Timed=0,
        Low,
        High,
        Off,
        External,
        NumModes
    };

    Leds( unsigned confAddr) 
        : m_confAddr(confAddr)
        , m_lastUpdate(0)
        , m_lastHour(0)
        , m_cycleInvalid(true)
        , m_dayStart(0)
        , m_sunAngle(0)
        , m_amFactor(0)
        , m_peakFactor(0)
        , m_timedPct(0)
        , m_angleFactor(0)
        , m_latitude(-21.534847)
        , m_longitude(174.287109)
        , m_timezone(12)
        , m_offsetSec(0)
        , m_periodSec(0)
        , m_sunriseSec(0)
        , m_normFactor(1.0)
        , m_highPct(50)
        , m_lowPct(20)
        , m_curSpectrum(0)
        , m_mode(Timed)
        , m_timeIsSet(false)
    {
    }

    void init();

    float sunAngle() const {
        return m_sunAngle;
    }

    void dimAll( unsigned char pct );

    void dimOne( int led, unsigned char pct );

    void setTime( unsigned long );
    void tempSetTime( unsigned long );
    bool timeIsSet() const { return m_timeIsSet; }

    float getLatitude() const { return m_latitude; }
    void setLatitude( float l ) { m_latitude = l; }
    float getLongitude() const { return m_longitude; }
    void setLongitude( float l ) { m_longitude = l; }
    int getTimezone() const {return m_timezone;}
    void setTimezone( int tz ) {m_timezone = tz; }

    char getCurVal( unsigned char ichan ) const {
        return m_curVals[ichan];
    }
    bool ledIndexUsed( unsigned char ichan) const {
        return !SKIP_LED_NUM(ichan);
    }
    char getLedPct( char spec, unsigned char ichan ) const {
        return m_ledPcts[(spec<0)?m_curSpectrum:spec][ichan];
    }
    void setLedPct( char spec, unsigned char ichan, unsigned char val ) {
        m_ledPcts[(spec<0)?m_curSpectrum:spec][ichan] = val;
    }
    unsigned char getCurSpectrum() const { return m_curSpectrum; }
    void setCurSpectrum( unsigned char spec ) {m_curSpectrum = spec; }

    void update();

    unsigned long getTimeSec() const;
    unsigned long getTimeOfDaySec() const;
    unsigned long getDayTime() const;
    unsigned char getHighPct() const { return m_highPct; }
    void setHighPct( unsigned char pct ) {m_highPct = pct; }
    unsigned char getLowPct() const { return m_lowPct; }
    void setLowPct( unsigned char pct ) {m_lowPct = pct; }
    unsigned char getTimedPct() const { return m_timedPct; }
    float getSunAngle() const { return m_sunAngle; }
    float getAMFactor() const { return m_amFactor; }
    float getPeakFactor() const { return m_peakFactor; }
    float getNormFactor() const { return m_normFactor; }
    void setNormFactor( float f ) {m_normFactor = f; }
    float getAngleFactor() const { return m_angleFactor; }
    unsigned long getOffsetSec() const { return m_offsetSec; }
    unsigned long getSunriseSec() const { return m_sunriseSec; }
    void setSunriseSec( unsigned long s ) {m_sunriseSec = s;}
    unsigned long getPeriodSec() const { return m_periodSec; }
    void setPeriodSec( unsigned long s ) {m_periodSec = s;}
    unsigned getDayStartSec() const { return m_dayStart; }
    unsigned long toDaySec( unsigned long sec ) { return sec - m_dayStart; }
    Mode getMode() const {return m_mode;}
    void setMode( Mode m ) {m_mode = m;}

    const PeriodData& getCalculatedCycle() const { return m_calcCycle; }
    const PeriodData& getUsedCycle() const { return m_useCycle; }
    float calcAngleFactor( float sunAngle );
    float calcAMFactor( float sunAngle );

    void pushVals() {
        dimAll( getLightPct() );
    }
    void invalidate( bool cycleToo=false ) {
        m_lastUpdate = 0;
        if (cycleToo)
            m_cycleInvalid = true;
    }

    void dumpDay( int numPoints, class InStream* stream );

    unsigned saveSettings();
    unsigned restoreSettings();
  protected:
    unsigned        m_confAddr;
    char            m_curVals[NUM_LED_NUMS];
    char            m_ledPcts[NUM_SPECTRUMS][NUM_LED_NUMS];
    unsigned char   m_curSpectrum;
    unsigned char   m_highPct;
    unsigned char   m_lowPct;
    unsigned char   m_timedPct;
    unsigned long   m_lastUpdate;
    unsigned char   m_lastHour;
    unsigned long   m_dayStart;
    bool            m_cycleInvalid;
    float           m_sunAngle;
    float           m_amFactor;
    float           m_peakFactor;
    float           m_angleFactor;
    float           m_latitude;
    float           m_longitude;
    char            m_timezone; // Timezone delta for lat and long.
    unsigned long   m_offsetSec;
    unsigned long   m_sunriseSec;
    unsigned long   m_periodSec;
    PeriodData      m_calcCycle;
    PeriodData      m_useCycle;
    Mode            m_mode;
    float           m_normFactor;
    bool            m_timeIsSet;

    void            updateSunAngle();
    void            updateTimedPct();
    void            updateCycle( bool force=false );
    void            updateAll() {
                        updateSunAngle();
                        updateTimedPct();
                        updateCycle();
                    }
    long            calcTimeAdjustment();
    long            adjustTime();
    void            restoreTime( unsigned long delta );
    unsigned        getLightPct();
    float           getSunAngleForTime( unsigned long t );
    unsigned long   binsearchTime( unsigned long start, unsigned long end, float target, float (Leds::*func)( unsigned long )  );
};

#endif

