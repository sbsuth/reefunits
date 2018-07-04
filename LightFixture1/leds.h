#ifndef LEDS_H
#define LEDS_H

// LEDs
#define FIRST_LED       2
#define LAST_LED        9
#define NUM_LED_NUMS    ((LAST_LED - FIRST_LED) + 1)
#define NUM_LEDS        (LAST_LED - FIRST_LED)
#define SKIP_LED(id)    (id == 4)
#define SKIP_LED_NUM(i)    SKIP_LED(i - FIRST_LED)
#define WHITE_LED       8
#define VIOLET_LED      7
#define ROYAL_BLUE_LED  6
#define BLUE_LED        5
#define CYAN_LED        9
#define RED_LED         3
#define AMBER_LED       2

#define NUM_SPECTRUMS   2

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
        , m_sunAngle(0)
        , m_amFactor(0)
        , m_latitude(-21.534847)
        , m_longitude(174.287109)
        , m_timezone(12)
        , m_offsetSec(0)
        , m_highPct(50)
        , m_lowPct(20)
        , m_timedPct(0)
        , m_curSpectrum(0)
        , m_mode(Timed)
    {
    }

    void init();

    float sunAngle() const {
        return m_sunAngle;
    }

    void dimAll( unsigned char pct );

    void dimOne( int led, unsigned char pct );

    void setTime( unsigned long );

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
        return !SKIP_LED(ichan+FIRST_LED);
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

    unsigned char getHighPct() const { return m_highPct; }
    void setHighPct( unsigned char pct ) {m_highPct = pct; }
    unsigned char getLowPct() const { return m_lowPct; }
    void setLowPct( unsigned char pct ) {m_lowPct = pct; }
    unsigned char getTimedPct() const { return m_timedPct; }
    float getSunAngle() const { return m_sunAngle; }
    float getAMFactor() const { return m_amFactor; }
    unsigned getOffsetSec() const { return m_offsetSec; }
    Mode getMode() const {return m_mode;}
    void setMode( Mode m ) {m_mode = m;}

    void pushVals() {
        dimAll( getLightPct() );
    }
    void invalidate() {
        m_lastUpdate = 0;
    }

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
    float           m_sunAngle;
    float           m_amFactor;
    float           m_latitude;
    float           m_longitude;
    char            m_timezone; // Timezone delta for lat and long.
    unsigned        m_offsetSec;
    Mode            m_mode;

    void            updateSunAngle();
    void            updateTimedPct();
    unsigned        getLightPct();
};

#endif

