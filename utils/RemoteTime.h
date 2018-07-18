#ifndef REMOTE_TIME_H
#define REMOTE_TIME_H 1


class RemoteTime 
{
  public:
    RemoteTime() 
        : m_gotTime(false)
        , m_queriedTime(false)
        , m_lastTimeAttempt(0)
        , m_time(0)
    {}
    bool update( bool force=false ) {

        if (m_gotTime && !force)
            return false;

        if ((millis() - m_lastTimeAttempt) < 5000) 
            return false;

        // Don't try while processing a command.
        if (EthernetSerialIO::client().connected())
            return false;

        bool gotNewTime = false;

        RF24IPInterface* rf24 = RF24IPInterface::inst();
        EthernetClient& client = rf24->getOutgoingClient();

        if (m_queriedTime) {
            // The outgoing connection should be ours.
            if (client.connected()) {
                // Try to read a response.
                int n;
                if ((n = client.waitAvailable( m_timeoutMs ))) {
                    char str[20];
                    int i;
                    for ( i=0; (i < n) && (i < sizeof(str)); i++ ) {
                        str[i] = client.read();
                    }
                    str[i] =0;
                    client.stop();
                    rf24->pingHeartbeat();
                    #if DEBUG_CONNECT
                    Serial.print(F("Got time str: ")); Serial.println(str);
                    #endif
                    m_gotTime = decodeTime(str);
                    if (m_gotTime)
                        gotNewTime = true;

                    #if DEBUG_CONNECT
                    Serial.print(F("Good time="));Serial.print(m_gotTime);Serial.print(", m_time=");Serial.println( m_time );
                    #endif
                    m_queriedTime = false;
                } else {
                    m_lastTimeAttempt = millis();
                    #if DEBUG_CONNECT
                    Serial.println(F("Waiting for response"));
                    #endif
                }
            } else {
                m_lastTimeAttempt = millis();
                #if DEBUG_CONNECT
                Serial.println(F("Disconnected"));
                #endif
            }
        } else {
            #if DEBUG_CONNECT
            Serial.println("Connecting to client");
            #endif
            if ( rf24->connectToClient( Ethernet.gatewayIP(), 3001, 1 ))  {
                client.write( "curtime", 7 );
                rf24->pingHeartbeat();
                m_queriedTime = true;
                m_lastTimeAttempt = 0;
                #if DEBUG_CONNECT
                Serial.print(F("Sent time cmd"));
                    #endif
            } else {
                m_lastTimeAttempt = millis();
                #if DEBUG_CONNECT
                Serial.println(F("Failed getting time."));
                #endif
            }
        }
        return gotNewTime;
    }
    bool haveTime() const {
        return m_gotTime;
    }
    unsigned long getTime() const {
        return m_time;
    }
    void invalidateTime() {
        m_gotTime = false;
        m_queriedTime = false;
        m_time = 0;
    }
  protected:
    bool m_gotTime = false;
    bool m_queriedTime = false;
    unsigned long m_lastTimeAttempt;
    unsigned long m_time;
    static const unsigned m_timeoutMs = 1000;

    bool decodeTime( const String& str ) {
        int X = str.indexOf('X');
        if (X <= 0)
            return false;
        m_time = str.substring(0,X).toInt();
        return (m_time > 0);
    }

};


#endif
