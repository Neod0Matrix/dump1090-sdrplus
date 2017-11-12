#ifndef __NETPRO_H__
#define __NETPRO_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

#define ANET_OK                 0u
#define ANET_ERR                -1
#define ANET_ERR_LEN            256u
#define ANET_CONNECT_NONE       0
#define ANET_CONNECT_NONBLOCK   1

#if defined(__sun)
#define AF_LOCAL AF_UNIX
#endif

int anetNonBlock(char *err, int fd);
int anetTcpNoDelay(char *err, int fd);
int anetSetSendBuffer(char *err, int fd, int buffsize);
int anetTcpKeepAlive(char *err, int fd);
int anetResolve(char *err, char *host, char *ipbuf);
int anetTcpConnect(char *err, char *addr, int port);
int anetTcpNonBlockConnect(char *err, char *addr, int port);
int anetUnixGenericConnect(char *err, char *path, int flags);
int anetUnixConnect(char *err, char *path);
int anetUnixNonBlockConnect(char *err, char *path);
int anetRead(int fd, char *buf, int count);
int anetWrite(int fd, char *buf, int count);
int anetTcpServer(char *err, int port, char *bindaddr);
int anetUnixServer(char *err, char *path, mode_t perm);
int anetTcpAccept(char *err, int s, char *ip, int *port);
int anetUnixAccept(char *err, int s);
int anetPeerToString(int fd, char *ip, int *port);
int anetSockName(int fd, char *ip, int *port);

#endif

//===============================================================================
//code by </MATRIX>@Neod Anderjon
