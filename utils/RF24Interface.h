#ifndef RF24_INTERFACE 
#define RF24_INTERFACE 

#include <SPI.h>
#include <RF24.h>
#include <RF24Mesh.h>
#include <RF24Network.h>
#include <RF24Ethernet.h>

// There must be a global variable named RF24Ethernet.
// This is accessed directly by the library.
extern RF24EthernetClass   RF24Ethernet;

class RF24IPInterface
{
  public:
    static RF24IPInterface* inst() {
        return m_inst;
    }

    RF24IPInterface( int addr, int ce, int csn, uint8_t level=RF24_PA_MAX )
        : m_radio(ce,csn)
        , m_network(m_radio)
        , m_mesh(m_radio,m_network)
        , m_ip( 10, 10, 2, addr )
        , m_server(1000)
        , m_heartbeat(0)
        , m_emptyRefreshes(0)
        , m_unnatural(0)
        , m_pa_level(level)
    {
        m_inst = this; // There's one global one.
    }

    void init() {
        Ethernet.begin(m_ip);
        m_mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
        //m_mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS, 5000);
        m_radio.setPALevel(m_pa_level);
        IPAddress gwIP(10, 10, 2, 2);
        Ethernet.set_gateway(gwIP);
        m_server.begin();
    }

    void pingHeartbeat() {
        m_heartbeat = millis();
        m_emptyRefreshes = 0;
    }
    unsigned long lastHeartbeat() {
        return m_heartbeat;
    }


    // Check mesh connection this often.
    const unsigned long m_timeout = (1 * 60000);

    void update() {
        #if 0
        // If we've seen a large number of unnatural disconnects, re-init.
        if (m_unnatural > m_maxUnnatural) {
            #if 0 
            // This isn't working well..
            init();
            m_unnatural = 0;
            #endif
            delay(100);
            return;
        }
        #endif
        unsigned long now = millis();
        if ((now - m_heartbeat) > m_timeout) {
            #if RF24_RE_INIT_AFTER_NUM_EMPTY
            // If there have been the specified number of re-checks
            // without any activity, restart subsystem
            // Only do this if there are no open connections.
            if (   (m_emptyRefreshes > RF24_RE_INIT_AFTER_NUM_EMPTY) 
                #if ENABLE_OUTGOING_ETHERNET
                && !m_outgoing.connected()
                #endif
                && !m_incoming.connected() ) {

                #if DEBUG_CONNECT
                Serial.print(F("Re-init RF24 system"));
                #endif
                init();
                return;
            }
            #endif
            // If 
            m_heartbeat  = now;
            if( ! m_mesh.checkConnection() ){
                #if DEBUG_CONNECT
                Serial.println(F("Lost connection. Begin nenew.."));
                #endif

                //refresh the network address
                m_mesh.renewAddress();

                #if DEBUG_CONNECT
                Serial.print(F("Connection renewal:"));
                Serial.println( m_mesh.checkConnection() );
                #endif
            }  else {
                #if DEBUG_CONNECT
                Serial.println(F("Connection good."));
                #endif
            }
        }
        #if AUTO_RE_INIT_INTERVAL
        static unsigned long to = AUTO_RE_INIT_INTERVAL;
        if ((millis() > to) && !m_incoming.connected() && !m_outgoing.connected()) {
        Serial.println("DOing RF24 init()");
            to += 30000;
            init();
        }
        #endif
    }
    #if ENABLE_OUTGOING_ETHERNET
    bool sendToRadioClient( unsigned char addr, const char* cmd, unsigned tries=1 )
    {
        return sendToClient( IPAddress(10,10,2,addr), 1000, cmd, tries );
    }
    bool connectToClient( const IPAddress& addr, uint16_t port, unsigned tries=1 )
    {
        bool success = false;
        while (tries-- && !success) {
            #if DEBUG_CONNECT
            Serial.println("Starting connect");
            #endif
            if (m_outgoing.connect(addr, port)) {
                #if DEBUG_CONNECT
                Serial.println("Connected");
                #endif
                pingHeartbeat();
                success = true;
            } else {
                #if DEBUG_CONNECT
                Serial.println("Not Connected");
                #endif
                success = false;
                delay(100);
            }
        }

        return success;
    }
    bool sendToClient( const IPAddress& addr, uint16_t port, const char* cmd, unsigned tries=1 )
    {
        bool success = false;
        if (connectToClient( addr, port, tries )) {
            #if DEBUG_CONNECT
            Serial.print("Sending "); Serial.println(cmd);
            #endif
            m_outgoing.write( cmd, strlen(cmd) );
            pingHeartbeat();
            if (m_outgoing.waitAvailable(1000)) {
                #if DEBUG_CONNECT
                Serial.println("GOt response");
                #endif
                m_outgoing.flush();
                pingHeartbeat();
                success = true;
            } else {
                #if DEBUG_CONNECT
                Serial.println("No response");
                #endif
                success = false;
            }
            m_outgoing.stop();
        }
        logDisconnect( !success );

        return success;
    }
    EthernetClient& getOutgoingClient() {
        return m_outgoing;
    }
    #endif
    void setIncomingClient(EthernetClient c) {
        // This is pushed by readers so others can check globally
        // if there's a reader with an open connection.  Relies
        // on their only being one active.
        m_incoming = c;
    }
    EthernetClient& getIncomingClient() {
        return m_incoming;
    }
    EthernetServer& getServer() {
        return m_server;
    }
    RF24& getRadio() {
        return m_radio;
    }
    RF24Network& getNetwork() {
        return m_network;
    }
    RF24Mesh& getMesh() {
        return m_mesh;
    }
    IPAddress& getAddress() {
        return m_ip;
    }
    void logDisconnect( bool unnatural ) {
        if (unnatural) {
            m_unnatural++;
            #if DEBUG_CONNECT
            Serial.print("Logging unnatural disconnect #");Serial.println(m_unnatural);
            #endif
        } else {
            m_unnatural = 0;
        }
    }
  protected:
    RF24                m_radio;
    RF24Network         m_network;
    RF24Mesh            m_mesh;
    IPAddress           m_ip;
    EthernetServer      m_server;
    #if ENABLE_OUTGOING_ETHERNET
    EthernetClient      m_outgoing;
    #endif
    EthernetClient      m_incoming;
    uint32_t            m_heartbeat;
    unsigned            m_emptyRefreshes;
    unsigned            m_unnatural;
    uint8_t             m_pa_level;
    static const unsigned m_maxUnnatural = 8;
    static RF24IPInterface* m_inst;
};



#endif
