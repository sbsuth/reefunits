#ifndef REMOTE_SETTING_H
#define REMOTE_SETTING_H

// Base class for settable or gettable remote state.
// Manages execution of a command to set or retrieve
// state.
class RemoteState 
{
  public:
    enum State {
        Idle,
        Submitted,
        Sent,
        Completed,
        TimedOut
    };
    RemoteState()
        : m_state(Idle)
        , m_lastConfirmed(0)
        , m_nextTry(0)
        , m_tries(0)
    {}
    State               getState() {
        return m_state;
    }
    bool                isActive() {
        return (m_state==Sent);
    }
    bool                initiate(   const char* cmd );
    bool                reset();

    virtual const char* cmd()=0;
    virtual unsigned    reassertInterval() {return 0;} // Defaults to no reassert.
    virtual bool        isCandidateForUpdate() {
        // If its never been confirmed, or if there's a reassertion interval and its passed.
        if (!m_lastConfirmed)
            return true;
        unsigned interval = reassertInterval();
        return interval && ((m_lastConfirmed + interval) < millis());
    }
    virtual unsigned char instrAddr()=0;
    virtual void        handleResponse( EthernetClient& client, int n ) {client.flush();}
    virtual void        beginAttempt() {}

    void                update() {
        if (!isCandidateForUpdate())
            return;

        // Move to submitted if there's a demand for a new value and we haven't started asking.
        if ((m_state == Idle) || (m_state == Completed) || (m_state == TimedOut)) {
            m_state = Submitted;
            m_tries = 0;
            m_nextTry = 0;
            beginAttempt();
            #if DEBUG_CONNECT
            Serial.println(F("RemoteState: Begin an attempt"));
            #endif
        }
        
        // Don't try anything if there is a connected command client.
        if (EthernetSerialIO::client().connected())
            return;

        // Yeild between tries.
        if (millis() < m_nextTry) {
            return;
        }

        RF24IPInterface* rf24 = RF24IPInterface::inst();
        EthernetClient& client = rf24->getOutgoingClient();
        
        if (m_state == Submitted) {
            // Try to get a connection.
            if (rf24->connectToClient( IPAddress(10,10,2,instrAddr()), commsTimeout(), 1)) {
                // Send the command and go to Sent.
                const char* c = cmd();
                #if DEBUG_CONNECT
                Serial.print(F("RemoteState: send command: "));Serial.println(c);
                #endif
                client.write( c, strlen(c) );
                rf24->pingHeartbeat();
                m_state = Sent;
                m_tries = 0;
                m_nextTry = 0;
            } else if (++m_tries > commsRetries()) {
                setTimedOut();
                #if DEBUG_CONNECT
                Serial.println(F("RemoteState: timed out on connect"));
                #endif
            } else {
                m_nextTry = millis() + commsTimeout();
                #if DEBUG_CONNECT
                Serial.println(F("RemoteState: yield before connect retry."));
                #endif
            }
        }
        if (m_state == Sent) {
            // The outgoing connection should be ours.
            if (client.connected()) {
                // Try to read a response.
                int n;
                if ((n = client.waitAvailable( commsTimeout() ))) {
                    handleResponse( client, n );
                    setCompleted();
                    client.stop();
                    rf24->pingHeartbeat();
                    #if DEBUG_CONNECT
                    Serial.println(F("RemoteState: got response"));
                    #endif
                } else if (++m_tries > commsRetries()) {
                    setTimedOut();
                    client.stop();
                    #if DEBUG_CONNECT
                    Serial.println(F("RemoteState: timed out on response"));
                    #endif
                } else {
                    m_nextTry = millis() + commsTimeout();
                    #if DEBUG_CONNECT
                    Serial.println(F("RemoteState: yield before response retry."));
                    #endif
                }
            } else {
                // If its disconnected, go to timed out.
                setTimedOut();
                #if DEBUG_CONNECT
                Serial.println(F("RemoteState: timed out with external disconnect"));
                #endif
            }
        }
    }
    void setTimedOut() {
        m_state = TimedOut;
        m_lastConfirmed = millis(); // Prevent immediate retry.
        m_nextTry = 0;
        m_tries = 0;
    }
    void setCompleted() {
        m_state = Completed;
        m_lastConfirmed = millis();
        m_nextTry = 0;
        m_tries = 0;
    }

    static const unsigned   commsTimeout() { return 1000; }
    static const unsigned   commsRetries() { return 3; }
  protected:
    State               m_state;
    unsigned long       m_lastConfirmed;
    unsigned long       m_nextTry;
    unsigned char       m_tries;
};

// A setting that has a desired, and a confirmed state.
template <typename T>
class RemoteSettable  : public RemoteState
{
  public:
    RemoteSettable()
    {}
    void                setWant(    const T& newWant, bool force ) {
        m_wantValue = newWant;
        if (force)
            m_lastConfirmed = 0;
    }
    virtual bool        isCandidateForUpdate() {
        // Either the values don't match, or the timer says its time.
        if (m_wantValue != m_confirmedValue)
            return true;
        return RemoteState::isCandidateForUpdate();
    }
    virtual void        handleResponse( EthernetClient& client, int n ) {
        // Don't care about content.  Just set confirmed value.
        m_confirmedValue = m_wantValue;
        client.flush();
    }
  protected:
    T                   m_wantValue;
    T                   m_confirmedValue;
};

// A value to be queried from another unit.
template <typename T>
class RemoteGettable  : public RemoteState
{
  public:
    RemoteGettable()
        : m_confirmed(false)
    {}
    virtual void        beginAttempt() {
        m_confirmed = false;
    }
    virtual void        handleResponse( EthernetClient& client, int n ) {
        // Derived class must overload to interpret value, but mark as confirmed.
        m_confirmed = true;
        client.flush();
    }
  protected:
    T                   m_value;
    bool                m_confirmed;
};

#endif
