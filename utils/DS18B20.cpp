#include "DS18B20.h"

//#define DEBUG_TEMP 1

// Interval between retries for missing sensors
#define DEVICE_SEARCH_INTERVAL_MS (5 * 1000)

// After this many bad measurements in a row, consider the sensor dead.
#define MAX_BAD_MEASUREMENTS 16

// The order of this array defines the addrIndexes used as handles
// for devices, so once an address has occupied an index, it should never move
// to another index.
#define NUM_DEVICES 3
static dsaddr_t g_devices[NUM_DEVICES] = {
    {0x28, 0xC8, 0x3C, 0x77, 0x91, 0x03, 0x02, 0xC1}, // A
    {0x28, 0x5C, 0xE4, 0x77, 0x91, 0x09, 0x02, 0x2B}, // B
    {0x28, 0xB4, 0x12, 0x77, 0x91, 0x04, 0x02, 0x10}  // C
};

template <int N>
bool DS18B20<N>::update() {
    bool changed = false;
    if (!m_initDone) {
        if (millis() < 5000)
            return false;
        m_sensors.begin();
        m_sensors.setWaitForConversion(false); 
        m_initDone = true;
    }
    bool goneBad = false;
    unsigned long now = millis();
    if (now >= m_nextMeasTime) {
        if ((m_nextSearchTime > 0) && (now > m_nextSearchTime)) {
            #if DEBUG_TEMP
            Serial.print(F("Do search for devices since there are only "));Serial.println(m_numDevices);
            #endif
            findSensors();
            m_nextSearchTime = 0;
        } else if (m_waitingTemp) {
            // Read results and go idle for reset of second.
            m_nextMeasTime = now + (m_period_ms - curDelay());
            for (int i=0; (i < m_numDevices) && (i < N); i++ ) {
                float newTemp = m_sensors.getTempF( g_devices[m_devMap[i]] );
                // Bad readings are something like -196, so reject negative numbers.
                #if DEBUG_TEMP
                Serial.print(F("New #"));Serial.print(i);Serial.print("=");Serial.println(newTemp);
                #endif
                if (newTemp > 0.0) {
                    m_temp[i].update(newTemp);
                    m_numBad[i] = 0;
                } else {
                    m_numBad[i]++;
                }
                if (m_numBad[i] > MAX_BAD_MEASUREMENTS) {
                    // Count it as dead.
                    m_temp[i].reset();
                    goneBad = true;
                    m_numBad[i] = 0;
                    #if DEBUG_TEMP
                    Serial.print(F("Device #"));Serial.print(i);Serial.println(F(" is dead"));
                    #endif
                }
            }
            m_waitingTemp = false;
            changed  = true;
        } else {
            // Init reading, and come back after delay.
            if (m_numDevices > 0) {
                m_sensors.requestTemperatures();
                m_waitingTemp = true;
            }
            m_nextMeasTime = now + curDelay();
        }
    }
    if ((m_numDevices < N) || goneBad){
        if (!m_nextSearchTime) {
            #if DEBUG_TEMP
            Serial.print(F("Only "));Serial.print(m_numDevices);Serial.print(F(" devices.  Will start search in "));Serial.println(DEVICE_SEARCH_INTERVAL_MS);
            #endif
            m_nextSearchTime = now + DEVICE_SEARCH_INTERVAL_MS;
        }
    }
    return changed;
}

// Return the index for the given address, or -1.
template <int N>
int DS18B20<N>::addr2index( dsaddr_t address ) {
    for ( unsigned index=0; index < NUM_DEVICES; index++ ) {
        unsigned char i=0;
        for ( i=0; i < 8; i++ ) {
            if (g_devices[index][i] != address[i])
                break;
        }
        if (i==8)
            return index;
    }
    return -1;
}

// Returns a pointer to address for the given index or 0.
template <int N>
uint8_t* DS18B20<N>::index2addr( unsigned char index ) {
    if (index < NUM_DEVICES) {
        return g_devices[index];
    } else {
        return 0;
    }
}

template <int N>
void DS18B20<N>::findSensors() {
    dsaddr_t addr;
    for ( unsigned i=0; i < N; i++ ) {
        m_devMap[i] = -1;
    }
    int devIndex = 0;
    m_oneWire.reset_search();
    while (m_oneWire.search(addr)) {
        int addrIndex = addr2index(addr);
        if (addrIndex >= 0) {
            m_devMap[devIndex] = addrIndex;
            #if DEBUG_TEMP 
            Serial.print(("Recognized device @index="));Serial.print(devIndex);
            Serial.print(F(", addrIndex="));Serial.println(addrIndex);
            printAddress(addr);
            #endif
            devIndex++;
        } else {
            #if DEBUG_TEMP
            Serial.println(F("ERROR: Unrecognized address:"));
            printAddress(addr);
            #endif
        }
    }
    m_numDevices = devIndex;
    #if DEBUG_TEMP
    Serial.print(F("Locating devices..."));
    Serial.print(F("Found "));
    Serial.print(m_numDevices, DEC);
    Serial.println(F(" devices."));
    #endif
}
    

template <int N>
void DS18B20<N>::printAddress( dsaddr_t address ) {
    Serial.print("  {");
    for (uint8_t i = 0; i < 8; i++) {
        Serial.print("0x");
        if (address[i] < 0x10) Serial.print("0");
        Serial.print(address[i], HEX);
        if (i < 7) Serial.print(", ");
    }
    Serial.println("  }");
}

template class DS18B20<2>;
