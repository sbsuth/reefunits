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
    RF24IPInterface( int addr, int ce, int csn )
        : m_radio(ce,csn)
        , m_network(m_radio)
        , m_mesh(m_radio,m_network)
        , m_ip( 10, 10, 2, addr )
        , m_server(1000)
        , m_heartbeat(0)
        , m_unnatural(0)
    {}

    void init() {
        Ethernet.begin(m_ip);
        m_mesh.begin(MESH_DEFAULT_CHANNEL, RF24_1MBPS, 5000);
        IPAddress gwIP(10, 10, 2, 2);
        Ethernet.set_gateway(gwIP);
        m_server.begin();
    }

    void pingHeartbeat() {
        m_heartbeat = millis();
    }
    unsigned long lastHeartbeat() {
        return m_heartbeat;
    }


    // Check mesh connection this often.
    const unsigned long m_timeout = (1 * 60000);

    void update() {
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
        unsigned long now = millis();
        if ((now - m_heartbeat) > m_timeout) {
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
    }
    #if ENABLE_OUTGOING_ETHERNET
    bool sendToRadioClient( unsigned char addr, const char* cmd, unsigned tries=1 )
    {
        return sendToClient( IPAddress(10,10,2,addr), 1000, cmd, tries );
    }
    bool sendToClient( const IPAddress& addr, uint16_t port, const char* cmd, unsigned tries=1 )
    {
        bool success = false;
        while (tries-- && !success) {
            #if DEBUG_CONNECT
            Serial.println("Starting connect");
            #endif
            if (m_outgoing.connect(addr, port)) {
                #if DEBUG_CONNECT
                Serial.println("Connected");
                Serial.print("Sending "); Serial.println(cmd);
                #endif
                m_outgoing.write( cmd, strlen(cmd) );
                if (m_outgoing.waitAvailable()) {
                    #if DEBUG_CONNECT
                    Serial.println("GOt response");
                    #endif
                    m_outgoing.flush();
                    success = true;
                } else {
                    #if DEBUG_CONNECT
                    Serial.println("No response");
                    #endif
                    success = false;
                }
                m_outgoing.stop();
            } else {
                #if DEBUG_CONNECT
                Serial.println("Not Connected");
                #endif
                success = false;
                delay(100);
            }
        }
        logDisconnect( !success );

        return success;
    }
    #endif
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
    uint32_t            m_heartbeat;
    unsigned            m_unnatural;
    static const unsigned m_maxUnnatural = 8;
};

#endif
