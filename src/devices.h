#ifndef __DEVICES_H__
#define __DEVICES_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

#define MODES_CONTENT_TYPE_HTML     "text/html;charset=utf-8"
#define MODES_CONTENT_TYPE_JSON     "application/json;charset=utf-8"
typedef enum {
    MODES_NET_SERVICE_RAWO  = 0u,
    MODES_NET_SERVICE_RAWI  = 1u,
    MODES_NET_SERVICE_HTTP  = 2u,
    MODES_NET_SERVICE_SBS   = 3u,
} ModesNetSvrArray;
#define MODES_NET_SERVICES_NUM      4u

//port server elementscle
typedef struct {
    char *descr;            //string describe
    int *socket;            //stack of port
    int port;               //port number
} modesNetServices;
extern modesNetServices modesNetServer[MODES_NET_SERVICES_NUM];

void modesInitConfig(void);
void modesInit(void);
int modesInitRTLSDR(void);
int modesInitAirSpy(void);
int modesInitHackRF(void);
int modesInitSDRplay(void);
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx);
int hackrfCallback (hackrf_transfer *transfer);
int airspyCallback (airspy_transfer *transfer);
int sdrplay_start_rx(void);
void modesInitNet(void);
void modesWaitReadableClients(int timeout_ms);
int handleHTTPRequest(client *c);
void modesReadFromClient(client *c, char *sep, int(*handler)(client *));
void modesReadFromClients(void);
void sigWinchCallback(void);
void modesAcceptClients(void);

#endif

//===============================================================================
//code by </MATRIX>@Neod Anderjon
