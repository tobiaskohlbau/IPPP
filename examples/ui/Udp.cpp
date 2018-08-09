/*
 * Udp.cpp
 *
 *  Created on: 25.04.2018
 *      Author: yitao
 */

#include "Udp.h"
#include <iostream>

Udp::Udp() : localDeviceLen(sizeof(localDevice)), extDeviceLen(sizeof(extDevice)) {
    std::vector<unsigned char> ip{0, 0, 0, 0};
    setLocalIp(ip);
    setLocalPort(30000);

    std::cout << "UDP-connection on port " << port << ": ";
    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Done" << std::endl;
    localDevice.sin_family = AF_INET;
    localDevice.sin_addr.s_addr = htonl(INADDR_ANY);

    std::cout << "Binding UDP-connection to socket " << fd << ": ";
    if (bind(fd, (struct sockaddr *)&localDevice, localDeviceLen) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Done" << std::endl;
}

Udp::Udp(const std::vector<unsigned char> &ip, const int port)
    : localDeviceLen(sizeof(localDevice)), extDeviceLen(sizeof(extDevice)) {
    setLocalPort(port);
    std::cout << "UDP-connection on port " << port << ": ";
    if ((fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == 0) {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Done" << std::endl;
    localDevice.sin_family = AF_INET;
    localDevice.sin_addr.s_addr = htonl(INADDR_ANY);

    std::cout << "Binding UDP-connection to socket " << fd << ": ";
    if (bind(fd, (struct sockaddr *)&localDevice, sizeof(localDevice)) < 0) {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    std::cout << "Done" << std::endl;
}

Udp::~Udp() {
    close(fd);
    std::cout << "Closing UDP-socket: " << fd << std::endl;
}

// only allow data coming from specified address and port. 0 = don't care
void Udp::setRemoteDevice(const std::vector<unsigned char> &ip, const int port) {
    remoteDevice.sin_family = AF_INET;
    setIp(ip, remoteDevice);
    setPort(port, remoteDevice);
    connect(fd, (struct sockaddr *)&remoteDevice, sizeof(remoteDevice));
}

void Udp::sendMsg(const std::vector<unsigned char> &msg) {
    sendto(fd, msg.data(), msg.size(), 0, (struct sockaddr *)&extDevice, (socklen_t)extDeviceLen);
}

void Udp::sendMsg(const std::vector<unsigned char> &msg, const std::vector<unsigned char> &ip, const int port) {
    struct sockaddr_in tmpExtDevice;
    setIp(ip, tmpExtDevice);
    setPort(port, tmpExtDevice);
    tmpExtDevice.sin_family = AF_INET;
    sendto(fd, msg.data(), msg.size(), 0, (struct sockaddr *)&tmpExtDevice, (socklen_t)sizeof(tmpExtDevice));
}

void Udp::recvMsg(std::vector<unsigned char> &msg) {
    msg.resize(1024);
    std::vector<unsigned char> ip{127, 0, 0, 1};
    setRemoteIp(ip);
    int length = recvfrom(fd, msg.data(), msg.size(), 0, (struct sockaddr *)&extDevice, (socklen_t *)&extDeviceLen);
    msg.resize(length);
}

std::vector<unsigned char> Udp::msgToChar(const std::string &msg) {
    std::vector<unsigned char> chars;
    for (auto &elem : msg)
        chars.push_back(elem);
    return chars;
}

std::string Udp::msgToString(const std::vector<unsigned char> &msg) {
    std::string str;
    for (auto &elem : msg)
        str.push_back(elem);
    return str;
}

void Udp::setRemoteIp(const std::vector<unsigned char> &ip) {
    setIp(ip, extDevice);
}

void Udp::setRemotePort(const int port) {
    setPort(port, extDevice);
}

std::vector<unsigned char> Udp::getRemoteIp() {
    return getIp(extDevice);
}

int Udp::getRemotePort() const {
    return (int)ntohs(extDevice.sin_port);
}

void Udp::setLocalIp(const std::vector<unsigned char> &ip) {
    setIp(ip, localDevice);
}

void Udp::setLocalPort(const int port) {
    setPort(port, localDevice);
}

std::vector<unsigned char> Udp::getLocalIp() {
    return getIp(localDevice);
}

int Udp::getLocalPort() const {
    return (int)ntohs(localDevice.sin_port);
}

void Udp::setIp(const std::vector<unsigned char> &ip, sockaddr_in &device) {
    device.sin_addr.s_addr =
        static_cast<in_addr_t>(((0xff & ip[3]) << 24) | ((0xff & ip[2]) << 16) | ((0xff & ip[1]) << 8) | (0xff & ip[0]));
}

void Udp::setPort(const int &port, sockaddr_in &device) {
    device.sin_port = htons(port);
}

std::vector<unsigned char> Udp::getIp(sockaddr_in &device) {
    return (std::vector<unsigned char>){static_cast<unsigned char>((device.sin_addr.s_addr >> 0) & 0xff),
                                        static_cast<unsigned char>((device.sin_addr.s_addr >> 8) & 0xff),
                                        static_cast<unsigned char>((device.sin_addr.s_addr >> 16) & 0xff),
                                        static_cast<unsigned char>((device.sin_addr.s_addr >> 24) & 0xff)};
}

unsigned int getPort(sockaddr_in &device) {
    return static_cast<unsigned int>(ntohs(device.sin_port));
}
