#include "ExperimentServer.h"
#include "EthernetInterface.h"

ExperimentServer::ExperimentServer() {
    _terminal = NULL;  
    _data_cnt = 0;
}

void ExperimentServer::attachTerminal( Serial & terminal) {
    _terminal = &terminal;
} 

void ExperimentServer::init() {
    // default configuration  
    char ip_address[] = "192.168.1.100";
    char subnet_mask[]= "255.255.255.0";
    char gateway[]    = "192.168.1.1";
    int server_port   = 11223;
    init(ip_address,subnet_mask,gateway, server_port);
}   
    
void ExperimentServer::init(const char * ip_addr, const char * subnet_mask, const char * gateway, unsigned int port) {
    if(_terminal!=NULL) {
        _terminal->printf("\r\n==============================\r\nStarting Server\r\n");
        _terminal->printf("...Intializing Ethernet\r\n");
    }
    
    int code = _eth.set_network(ip_addr,subnet_mask,gateway);
    if(_terminal!=NULL) {
        _terminal->printf("...Connecting\r\n");
        if(code!=0) 
            _terminal->printf("Error Code = %d\r\n",code);
    }
       
    code = _eth.connect();
    if(_terminal!=NULL) {
        _terminal->printf("...Ethernet IP Address is %s\r\n",_eth.get_ip_address());
        if(code!=0) 
            _terminal->printf("Error Code = %d\r\n",code);    
    }
    
    SocketAddress sock;
    sock.set_port(port);
    sock.set_ip_address(ip_addr);
    
    code = _server.open(&_eth);
    if(_terminal!=NULL) {
        _terminal->printf("...Opened\n");
        if(code!=0) 
            _terminal->printf("Error Code = %d\r\n",code);    
    }
    
    code = _server.bind(sock);
    if(_terminal!=NULL) {
        _terminal->printf("...Listening on Port %d\r\n",port);
        if(code!=0) 
            _terminal->printf("Error Code = %d\r\n",code);
    }
}

int ExperimentServer::getParams(float params[], int num_params) {
    if(_terminal!=NULL) {
        _terminal->printf("\r\n========================\r\nNew Experiment\r\n");
        _terminal->printf("...Waiting for parameters...\r\n");
    }
        
    int n = _server.recvfrom(&_client,(char *) params, num_params*sizeof(float));    
    if ( (n% 4) > 0 ) {
        if(_terminal!=NULL) {
            _terminal->printf("ERROR: input data bad size\r\n");
            _terminal->printf("ERROR: Expected %d got %d\r\n",4*num_params,n);
        }
        return false;    
    }
    if ( n / 4 != num_params) {
        if(_terminal!=NULL) {
            _terminal->printf("ERROR: input data bad size\r\n");
            _terminal->printf("ERROR: Expected %d got %d\r\n",4*num_params,n);
        }
        return false;    
    }
    
    if(_terminal!=NULL) {
        _terminal->printf("...Received input from: %s\r\n", _client.get_ip_address());
        _terminal->printf("...Parameters: \r\n");
        for ( int j = 0 ; j < n/sizeof(float) ; j++) {
            _terminal->printf("      %d) %f\r\n",j,params[j]);
        }
    }
    return true;
}
void ExperimentServer::flushBuffer() {
    if(_data_cnt > 0) {
         _server.sendto(_client,(char*) _buffer, 4*_data_cnt );    
        _data_cnt = 0;
    }
}

void ExperimentServer::sendData(float data_output[], int data_size) {
    if( data_size + _data_cnt > _MAX_BUFFER_SIZE) {
        flushBuffer();
    }
    for(int j = 0; j < data_size; j++) {
        _buffer[ _data_cnt++ ] = data_output[j];    
    }
}
void ExperimentServer::setExperimentComplete() {
    flushBuffer();
    char buff[] = {'0'};
    _server.sendto(_client,buff,1);
}   