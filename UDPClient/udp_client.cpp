#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <iostream>
#define JOINTNUM 6
typedef struct UDPSTATE{
	double q[JOINTNUM];
	double dq[JOINTNUM];
	double ddq[JOINTNUM];
	double torque[JOINTNUM];
	double quat[4];
	double pos[4];
	double V[6];
	double dV[6];
    double s_time;
}UDPstate;
typedef struct UDPIndy7Info{
	UDPSTATE act;
	UDPSTATE des;
}UDPIndy7Info;

int main() {
    Poco::Net::DatagramSocket client;
    UDPIndy7Info data;
    Poco::Net::SocketAddress receiver("192.168.0.9", 9911);
    // Send data and receive the echoed value
    client.sendTo(&data, sizeof(UDPIndy7Info), receiver);
    client.receiveFrom(&data, sizeof(UDPIndy7Info), receiver);
    std::cout<<data.act.q[0]<<std::endl;
    return 0;
}
