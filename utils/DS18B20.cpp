#include "DS18B20.h"

#define DEBUG_TEMP 1

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
    unsigned long now = millis();
    if (now >= m_nextTime) {
        if (m_waitingInit) {
            m_sensors.begin();
            findSensors();
            #if DEBUG_TEMP
              Serial.print("Locating devices...");
              Serial.print("Found ");
              Serial.print(m_numDevices, DEC);
              Serial.println(" devices.");
              Serial.print((int)sizeof(float));Serial.println(" bytes for float");
              Serial.print((int)sizeof(int));Serial.println(" bytes for int");
            #endif

            m_sensors.setWaitForConversion(false);
            m_waitingInit = false;
        } else if (m_waitingTemp) {
            // Read results and go idle for reset of second.
            m_nextTime = now + (m_period_ms - curDelay());
            for (int i=0; (i < m_numDevices) && (i < N); i++ ) {
                float newTemp = m_sensors.getTempF( g_devices[m_devMap[i]] );
                newTemp += m_offset[i];
                m_temp[i].update(newTemp);
            }
            m_waitingTemp = false;
            return true;
        } else {
            // Init reading, and come back after delay.
            m_sensors.requestTemperatures();
            m_nextTime = now + curDelay();
            m_waitingTemp = true;
        }
    }
    return false;
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
    while (m_oneWire.search(addr)) {
        int addrIndex = addr2index(addr);
        if (addrIndex >= 0) {
            m_devMap[devIndex] = addrIndex;
            devIndex++;
            #if DEBUG_TEMP
            Serial.print("Recognized device @index=");Serial.print(devIndex);
            Serial.print(", addrIndex=");Serial.println(addrIndex);
            printAddress(addr);
            #endif
        } else {
            #if DEBUG_TEMP
            Serial.println("ERROR: Unrecognized address:");
            printAddress(addr);
            #endif
        }
    }
    m_numDevices = devIndex;
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
