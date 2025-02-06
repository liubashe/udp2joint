#include "MySocketUdp.h"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>  // for close
#include <errno.h> 

#define SOCKET_ERROR -1
/**
* @brief
*   ��ip��ַ���˿ں�ת��Ϊsocket_udpʶ���sockaddr_in���͵�ַ
* @param[in] ip: ip��ַ���磺"10.0.27.68","192.168.1.2"
* @param[in] port: �˿ں�
* @return socket_udpʶ��ĵ�ַ
*/
sockaddr_in MySocketUdp::ToSockaddr(const char *ip, unsigned short port)
{
	// Set up the RecvAddr structure with the IP address of the receiver
	// and the specified port number.
	sockaddr_in addr;
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port);
	inet_pton(AF_INET, ip, (void *)&(addr.sin_addr.s_addr));
	return addr;
}

/**
* @brief
*   ��sockaddr_in���͵ĵ�ַת��Ϊip��ַ���˿ں�
* @param[in] sockaddr_in���͵�ַ
* @param[out] ip: ip��ַ���磺"10.0.27.68","192.168.1.2"
* @param[out] port: �˿ں�
*/
const char* MySocketUdp::ToIpPort(sockaddr_in sockaddr, unsigned short &port)
{
	char buff[INET_ADDRSTRLEN] = { 0 };
	const char* ip = inet_ntop(AF_INET, &(sockaddr.sin_addr), buff, sizeof(buff));
	port = ntohs(sockaddr.sin_port);
	return ip;
}

/**
* @brief
*   ���ö�ȡ��ʱ�����жϸ�ʱ�����Ƿ������ݹ�����ÿ��recvfromǰ�����жϣ�δ��ʱʱ����recvfrom
* @param[in] socket: socket����
* @param[in] time_sec: ��ʱʱ�䣬��λ��
* @param[in] time_usec: ��ʱʱ�䣬��λ΢��
* @return false: ��ʱ���ܳ�ʱ�������ݿɶ�ȡ����true: δ��ʱ���ܳ�ʱ�������ݿɶ�ȡ��
* @remark
*   �ܳ�ʱʱ�� = time_sec + time_usec
*/
bool MySocketUdp::JudgeIsInRecvTime(SOCKET socket, int time_sec, int time_usec)
{
	//struct timeval tv;
	//fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(socket, &readfds);
	tv.tv_sec = time_sec;  //��λ��
	tv.tv_usec = time_usec;  //��λ΢��
	select((int)socket + 1, &readfds, NULL, NULL, &tv); //�ȴ�ʱ��t = tv_sec + tv_usec;
	if (FD_ISSET(socket, &readfds)) //����ʱ�䳬ʱ��ÿ��recvfromǰ��Ҫ���ã����漰�Ĳ�����Ҫ���¸�ֵ
	{
		//δ��ʱ����������� recvfrom();
		return true;  //δ��ʱ
	}
	else
	{
		return false;   //��ʱ
	}
}

/**
* @brief
*   UDP��ʼ��
* @param[in] localPort: Ҫ�󶨵ı��ض˿�
* @return false: ��ʼ��ʧ�ܣ�true: ��ʼ���ɹ�
*/
bool MySocketUdp::Initial(unsigned short localPort)
{
	int iResult = 0;
	sockaddr_in localAddr;  //���ص�ַ

	socket_udp = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (socket_udp == -1) {
		//wprintf(L"socket failed with error %d\n", WSAGetLastError());
		// WSACleanup();
		return false;
	}
	//-----------------------------------------------
	// Bind the socket to any address and the specified port.
	localAddr.sin_family = AF_INET;
	localAddr.sin_port = htons(localPort);
	localAddr.sin_addr.s_addr = htonl(INADDR_ANY);

	iResult = bind(socket_udp, (struct sockaddr  *)& localAddr, sizeof(localAddr));
	if (iResult != 0) {
		//wprintf(L"bind failed with error %d\n", WSAGetLastError());
		if (close(socket_udp) == SOCKET_ERROR) {
			//wprintf(L"closesocket failed with error %d\n", WSAGetLastError());
		}
		// WSACleanup();
		return false;
	}
	return true;
}

/**
* @brief
*   �ر�
*/
void MySocketUdp::Close()
{
	if (close(socket_udp) == -1) {
            // perror("close failed");
    }
}

/**
* @brief
*   �����Ը�socket�󶨵�ַΪĿ�ĵ�ַ��������ַ���͹���������
* @param[out] desAddr: �������ݵĵ�ַ
* @param[out] recvBuf: ���յ�������
* @param[in] recvBufLen: recvBuf�����С
* @return -1����ȡ������>=0���ɹ���ȡ���ֽ���
*/
int MySocketUdp::Recv(sockaddr_in &dstAddr, char *recvBuf, int recvBufLen)
{
	int recvLen = 0;
	socklen_t desAddrLen = sizeof(dstAddr);
	if (JudgeIsInRecvTime(socket_udp, UDP_TIMEOUT_SEC, UDP_TIMEOUT_USEC)) //δ��ʱ
	{
		recvLen = recvfrom(socket_udp, recvBuf, recvBufLen, 0, (struct sockaddr *)&dstAddr, &desAddrLen);
		//recvLen: >= 0��ʾ��ȡ���ֽ�����С��0Ϊ������
		if (SOCKET_ERROR == recvLen)
		{
			
			if (errno == ECONNRESET) { return -ECONNRESET; }  // 对方强制关闭连接
                // perror("recvfrom failed");
                close(socket_udp);
			return -1;
		}
	}
	return recvLen;
}


/**
* @brief
*   ��Ŀ�ĵ�ַ��������
* @param[in] desAddr: Ŀ�ĵ�ַ������IP���˿���Ϣ��
* @param[in] sendBuf: Ҫ���͵��ֽ�����
* @param[in] sendBufLen: Ҫ�����ֽ�������ֽ���
* @return -1�����ͳ�����>=0���ɹ����͵��ֽ���
*/
int MySocketUdp::Send(sockaddr_in dstAddr, char *sendBuf, int sendBufLen)
{
	int senLen = 0;
	senLen = sendto(socket_udp, sendBuf, sendBufLen, 0, (struct sockaddr *)&dstAddr, sizeof(dstAddr));
	if (senLen == SOCKET_ERROR)
	{
		//wprintf(L"sendto failed with error: %d\n", WSAGetLastError());
		close(socket_udp);
		return -1;
	}
	return senLen;
}

/**
* @brief
*   ��Ŀ�ĵ�ַ��������
* @param[in] desAddr: Ŀ�ĵ�ַ������IP���˿���Ϣ��
* @param[in] sendBuf: Ҫ���͵��ֽ�����
* @param[in] sendBufLen: Ҫ�����ֽ�������ֽ���
* @return -1�����ͳ�����>=0���ɹ����͵��ֽ���
*/
int MySocketUdp::Send(sockaddr_in dstAddr, unsigned char *sendBuf, int sendBufLen)
{
	char sendData[BUFFERLEN_MAX] = { 0 };
	for (int i = 0; i < sendBufLen; i++)
	{
		sendData[i] = (char)sendBuf[i];
	}
	return Send(dstAddr, sendData, sendBufLen);
}
