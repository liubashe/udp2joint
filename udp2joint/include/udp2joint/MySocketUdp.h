/********************************************************************
˵����
��1��ͨ��UDPͨѶ�������ݣ������н�����������������������ģ��������
��2��UDPͨѶ����Ե�ͨѶ���������ӡ�
��3���ͻ���ʱ���Է�����Ϊ��һ��ͨѶ��������ʱ���Խ�����Ϊ��һ��ͨѶ��

*********************************************************************/
#pragma once
#ifndef MYSOCKETUDP_H
#define MYSOCKETUDP_H

#define BUFFERLEN_MAX 25600
#define UDP_TIMEOUT_SEC 0  //��
#define UDP_TIMEOUT_USEC 100000  //΢��

// #include <WinSock2.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <cstdint>
typedef int SOCKET;
class MySocketUdp
{
public:
	MySocketUdp() { };
	~MySocketUdp() { };

private:
	//���ڳ�ʱ����
	struct timeval tv;
	fd_set readfds;

	SOCKET socket_udp;

private:
	bool JudgeIsInRecvTime(SOCKET socket, int time_sec, int time_usec);

public:
	static sockaddr_in ToSockaddr(const char *ip, unsigned short port);
	static const char* ToIpPort(sockaddr_in sockaddr, unsigned short &port);
	
	bool Initial(unsigned short localPort);
	void Close();
	int Recv(sockaddr_in &dstAddr, char *recvBuf, int recvBufLen);
	int Send(sockaddr_in dstAddr, char *sendBuf, int sendBufLen);
	int Send(sockaddr_in dstAddr, unsigned char *sendBuf, int sendBufLen);

};

#endif
