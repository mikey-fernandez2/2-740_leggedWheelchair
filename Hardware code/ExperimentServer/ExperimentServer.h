#include "EthernetInterface.h"
#include "mbed.h"

#define _MAX_BUFFER_SIZE 200

/**
 * Experiment Server
 */
 
class ExperimentServer
{
public:

    /**
     * Constructor.
     *
     * Constructs an empty experiment server. Server objects require additional
     * initialization before being used.
     * Constructs an empty experiment server. Server objects require further
     * initialization before use.
     */
    ExperimentServer();
    
     /**
     * Link a serial terminal to the server for debugging purposes.
     *
     * @param terminal Serial terminal to be used for debugging messages
     */
    void attachTerminal( Serial & terminal); 
    
    /**
     * Initialize the server.
     * 
     * Following initialization, the getParams, sendData, and setExperimentComplete
     * methods may be used.
     *
     * Applies default server configuration with ip <tt>192.168.1.100</tt>, 
     * subnet <tt>255.255.255.0</tt>, gateway <tt>192.168.1.1</tt>, port <tt>11223</tt>.
     */    
    void init();
    
    /**
     * Initialize the server with supplied configuration.
     *
     * Following initialization, the getParams, sendData, and setExperimentComplete
     * methods may be used.
     *
     * @param addr      IP address of the server
     * @param subnet    Subnet mask of the IP address
     * @param gateway   Gateway/router IP address
     * @param port      Port to listen on for connections from MATLAB.
     */
    void init(const char * addr, const char * subnet, const char * gateway, unsigned int port);
    
    /**
     * Listen for parameters passed to the server. This operation blocks until any data
     * is recieved.
     *
     * @param params        Float array of size num_params used to store incoming data
     * @param num_params    Expected length of incoming data (# of floats)
     *
     * @return Success of the operation (0 - Failure, 1 - Success)
     *         The operation is considered failure if the correct number of floats
     *         is not recieved.
     *    
     */
    int getParams(float params[], int num_params);
    
    /**
     * Send data back to host machine. To alleviate the processing burden on the host
     * machine, data is actually buffered on the chip and sent to the host in batches.
     *
     * @param data_output  Float array of size data_size containing data to send to host
     * @param data_size    Length of outgoing data (# of floats)
     *    
     */
    void sendData(float data_output[], int data_size);
    
    /**
     * Inform the host that the experiment is complete.
     * 
     * This function also sends any final buffered output data that has yet 
     * to be relayed to the host.
     *
     */
    void setExperimentComplete();
    
private:

    void flushBuffer(); // Sends all data in buffer to the host
    
    EthernetInterface _eth; // Low level ethernet object
    SocketAddress _client;       // Stores client connected to server
    UDPSocket _server;      // Low level UDP server object
    Serial * _terminal;     // Pointer to attached terminal for debugging
    
    float _buffer[_MAX_BUFFER_SIZE]; // Buffer for batch data storage before sending
    int _data_cnt;                   // Current occupied length of the buffer
};