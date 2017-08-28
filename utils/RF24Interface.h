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


    const unsigned long m_timeout = (2 * 60000);

    void update() {
        uint32_t now = millis();
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
                Serial.print(F("Connection good."));
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
            Serial.println("Starting connect");
            if (m_outgoing.connect(addr, port)) {
            Serial.println("Connected");
            Serial.print("Sending "); Serial.println(cmd);
                m_outgoing.write( cmd, strlen(cmd) );
                if (m_outgoing.waitAvailable()) {
            Serial.println("GOt response");
                    m_outgoing.flush();
                    success = true;
                } else {
            Serial.println("No response");
                    success = false;
                }
            } else {

            Serial.println("Not Connected");
                success = false;
            }
            m_outgoing.stop();
        }

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
};

#endif
