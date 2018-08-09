/*
 * Ethernet.h
 *
 *  Created on: 25.04.2018
 *      Author: yitao
 */

#ifndef UDP_H_
#define UDP_H_

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <string>

class Udp {
  public:
    Udp();
    Udp(const std::vector<unsigned char> &ip, const int port);
    ~Udp();

    void setRemoteDevice(const std::vector<unsigned char> &ip, const int port);

    void sendMsg(const std::vector<unsigned char> &msg);
    void sendMsg(const std::vector<unsigned char> &msg, const std::vector<unsigned char> &ip, const int port);
    void recvMsg(std::vector<unsigned char> &msg);
    std::vector<unsigned char> msgToChar(const std::string &msg);
	std::string msgToString(const std::vector<unsigned char> &msg);

    void setRemoteIp(const std::vector<unsigned char> &ip);
    void setRemotePort(const int port);

    std::vector<unsigned char> getRemoteIp();
    int getRemotePort() const;

    void setLocalIp(const std::vector<unsigned char> &ip);
    void setLocalPort(const int port);

    std::vector<unsigned char> getLocalIp();
    int getLocalPort() const;

  private:
    void setIp(const std::vector<unsigned char> &ip, sockaddr_in &);
    void setPort(const int &port, sockaddr_in &);
    std::vector<unsigned char> getIp(sockaddr_in &);
    unsigned int getPort(sockaddr_in &);
    int fd, port, localDeviceLen, extDeviceLen;
    struct sockaddr_in localDevice, extDevice, remoteDevice;
};

#endif /* UDP_H_ */
