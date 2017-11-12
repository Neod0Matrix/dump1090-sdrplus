#ifndef __DATA_H__
#define __DATA_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

#define WebMapFile      "gmap.html"
#define DefaultBrowser  "google-chrome"
#define StdUserName     "neod-anderjon"

extern uint32_t modes_checksum_table[112];
extern char *ca_str[8];
extern char *fs_str[8];

void readDataFromFile(void);
void *readerThreadEntryPoint(void *arg);
uint32_t modesChecksum(unsigned char *msg, int bits);
int modesMessageLenByType(int type);
int fixSingleBitErrors(unsigned char *msg, int bits);
int fixTwoBitsErrors(unsigned char *msg, int bits);
uint32_t ICAOCacheHashAddress(uint32_t a);
void addRecentlySeenICAOAddr(uint32_t addr);
int ICAOAddressWasRecentlySeen(uint32_t addr);
int bruteForceAP(unsigned char *msg, modesMessage *mm);
int decodeAC13Field(unsigned char *msg, int *unit);
int decodeAC12Field(unsigned char *msg, int *unit);
char *getMEDescription(int metype, int mesub);
void decodeModesMessage(modesMessage *mm, unsigned char *msg);
void displayModesMessage(modesMessage *mm);
void useModesMessage(modesMessage *mm);
int cprNLFunction(double lat);
void decodeCPR(aircraft *a);
void interactiveShowData(void);
void modesSendSBSOutput(modesMessage *mm, aircraft *a);
int hexDigitVal(int c);
int decodeHexMessage(client *c);
char *aircraftsToJson(int *len);
int getTermRows();
void INTHandler(int sig);
int detectOutOfPhase(uint16_t *m);
void applyPhaseCorrection(uint16_t *m);
void modesSendAllClients(int service, void *msg, int len);
void modesSendRawOutput(modesMessage *mm);
void modesFreeClient(int fd);
void dumpRawMessageJS(char *descr, unsigned char *msg, uint16_t *m, uint32_t offset, int fixable);
void dumpRawMessage(char *descr, unsigned char *msg, uint16_t *m, uint32_t offset);
char* stradd(char *s1, char *s2);
char* itoa_l(int num, char* str, int radix);
void systemCmdExecute(char* cmd);
void mapWebPageOpen(int http_port, char* username);
void flightAwareWebPageOpen (char* flight, char* username);

#endif

//===============================================================================
//code by </MATRIX>@Neod Anderjon
