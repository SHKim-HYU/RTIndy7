#include <Poco/Net/DatagramSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <iostream>

#define MAX_BUFFER_SIZE 1024
struct Data {
    int jointNum;
    double q[6];
    double dq[6];
    double ddq[6];
};



int main() {
    Poco::Net::DatagramSocket server(Poco::Net::SocketAddress("127.0.0.1", 2000));
    while (true) {
        Data recv_data;
        Data send_data;
        send_data.q[0]= 0;
        send_data.q[1]= 1;
        send_data.q[2]= 2;
        send_data.q[3]= 3;
        Poco::Net::SocketAddress sender;
        server.receiveFrom(&recv_data, sizeof(Data), sender);
        server.sendTo(&send_data, sizeof(Data), sender);
    }
    return 0;
}
