#include <math.h>
#include <time.h>
#include <stdio.h>

typedef unsigned char byte;
#if 1
//float latitude = 47.612;
float latitude = 0;
float longitude = -122.104;
int TimeZone = -7; 
#else
//float latitude = -19.770621;   // + to N  Defualt - (-19.770621) Heart Reef, Great Barrier Reef, QLD, Australia    
//float longitude = 149.238532;  // + to E  Defualt - (149.238532)   
//int TimeZone = 10;             // + to E  Defulat - (10)   
float latitude = -21.534847; // + to S Defualt - (-21.534847) Heart Reef, Great Barrier Reef, QLD, Australia
float longitude = 174.287109; // + to E Defualt - (174.287109)
int TimeZone = 12; // + to E Defulat - (12)

#endif
int delayTime = 0;     // start time delay in minutes,  - will push the day back, + will bring the day forward   
int MyTimeZone = -7;             

// Main Sun Cycle  
//

float d2r( float deg )
{
    (deg * M_PI) / 180.0;
}

float r2d( float rad )
{
    (rad * 180.0) / M_PI;
}

float SunLight(byte year, byte month, byte day, byte hour, byte min, byte sec)   
{   
    float a = floor((14.0 - month) / 12.0);   
    float y = year + 4800.0 - a;   
    float m = month + (12.0 * a) - 3.0;   
    float AH;   
    int result;   

    float JC = (((day + floor(((153.0 * m) + 2.0) / 5.0) + (365.0 * y) + floor(y / 4.0) - floor(y / 100.0) + floor(y / 400.0) - 32045.0) + ((hour / 24.0) + (min / 1444.0) + (sec / 86400.0))) - 2451556.08) / 36525.0;   

    float GMLS = fmod(280.46646 + JC*(36000.76983 + JC * 0.0003032), 360.0);   

    float GMAS = 357.52911 + JC * (35999.05029 - 0.0001537 * JC);   

    float EEO = 0.016708634 - JC * (0.000042037 + 0.0000001267 * JC);   

    float SEoC = sin((GMAS * M_PI) / 180.0)*(1.914602 - JC * (0.004817 + 0.000014 * JC)) + sin(((2.0 * GMAS) * M_PI) / 180.0) * (0.019993 - 0.000101 * JC) + sin(((3.0 * JC) * M_PI) / 180.0) * 0.000289;   

    float STL = GMLS + SEoC;   

    float STA = GMAS + SEoC;   

    float SRV = (1.000001018 * (1.0 - EEO * EEO)) / (1.0 + EEO * cos((STA * M_PI) / 180.0));   

    float SAL = STL - 0.00569 - 0.00478 * sin(((125.04 - 1934.136 * JC) * M_PI) / 180.0);   

    float MOE = 23.0 + (26.0 + ((21.448 - JC * (46.815 + JC * (0.00059 - JC * 0.001813)))) / 60.0) / 60.0;   

    float OC = MOE + 0.00256 * cos(((215.04 - 1934.136 * JC) * M_PI) / 180.0);   

    float SD = (asin(sin((OC * M_PI) / 180.0) * sin((SAL * M_PI) / 180.0))) * (180.0 / M_PI);   

    float vy = tan(((OC / 2.0) * M_PI) / 180.0) * tan(((OC / 2.0) * M_PI) / 180.0);   

    float EQoT = (4.0 * (vy * (sin(2.0 * ((GMLS * M_PI) / 180.0)) - 2.0 * EEO * sin((GMAS * M_PI) / 180.0) + 4.0 * EEO * vy * sin((GMAS * M_PI) / 180.0) * cos(2.0 * ((GMLS * M_PI) / 180.0)) - 0.5 * vy * vy * sin(4.0 * ((GMLS * M_PI) / 180.0)) - 1.25 * EEO * EEO * sin(2 * ((GMAS * M_PI) / 180))))) * (180 / M_PI);   

    float HAS = acos(cos((90.833 * M_PI) / 180.0) / (cos((latitude * M_PI) / 180.0) * cos((SD * M_PI) / 180.0)) - tan((latitude * M_PI) / 180.0) * tan((SD * M_PI) / 180.0)) * (180.0 / M_PI);   

    float SN = (720.0 - 4.0 * longitude - EQoT + TimeZone * 60.0);   

    float SR = SN - HAS * 4.0;   

    float SS = SN + HAS * 4.0;   

    float STD = 8.0 * HAS;   

    float TST = fmod((((hour)+(min / 60.0) + (sec / 3600.0)) / 24.0) * 1440.0 + EQoT + 4.0 * longitude - 60.0 * TimeZone, 1440.0) + delayTime;   

    if (TST / 4 < 0.0)   
    {   
        AH = ((TST / 4.0) + 180.0);   
    }   
    else   
    {   
        AH = ((TST / 4.0) - 180.0);   
    }   

    float SZA = (acos(sin((latitude * M_PI) / 180.0) * sin((SD * M_PI) / 180.0) + cos((latitude * M_PI) / 180.0) * cos((SD * M_PI) / 180.0) * cos((AH * M_PI) / 180.0))) * (180.0 / M_PI);   

    float SEA = 90.0 - SZA;   


    return SEA;   

}   


int main( int argc, char** argv )
{
    int offset = (argc > 1) ? atoi(argv[1]) : 0;
    time_t now = time(0) + offset;
    //now = 1530514800;
    struct tm* now_tm = localtime(&now);
    float sun = SunLight( now_tm->tm_year, now_tm->tm_mon, now_tm->tm_mday, now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec)   ;
    printf("time=%s: sun=%f\n", asctime(now_tm), sun);

    now = 1530514800;
    for ( int i=0; i < 24; i++ ) {
        now += (60 * 60);
        struct tm* now_tm = localtime(&now);
        float sun = SunLight( now_tm->tm_year, now_tm->tm_mon, now_tm->tm_mday, now_tm->tm_hour, now_tm->tm_min, now_tm->tm_sec)   ;
        printf("time=%s: sun=%f\n", asctime(now_tm), sun);
    }
}
