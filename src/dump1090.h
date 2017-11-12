#ifndef __DUMP1090_H__
#define __DUMP1090_H__
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

#define MODES_DEFAULT_RATE              2000000
#define MODES_DEFAULT_FREQ              1090000000          //dump 1090MHz, flight wave range
#define MODES_DEFAULT_WIDTH             1000
#define MODES_DEFAULT_HEIGHT            700
#define MODES_ASYNC_BUF_NUMBER          12
#define MODES_DATA_LEN                  (16*16384)          /* 256k */
#define MODES_AUTO_GAIN                 -100                /* Use automatic gain. */
#define MODES_MAX_GAIN                  999999              /* Use max available gain. */
/* HackRF One Defaults */
#define HACKRF_RF_GAIN	 	            0
#define HACKRF_LNA_GAIN		            32
#define HACKRF_VGA_GAIN		            48
/* AirSpy defaults */
#define AIRSPY_RF_GAIN		            11
#define AIRSPY_LNA_GAIN		            11
#define AIRSPY_VGA_GAIN		            11

#define MODES_PREAMBLE_US               8                   /* microseconds */
#define MODES_LONG_MSG_BITS             112
#define MODES_SHORT_MSG_BITS            56
#define MODES_FULL_LEN                  (MODES_PREAMBLE_US+MODES_LONG_MSG_BITS)
#define MODES_LONG_MSG_BYTES            (112/8)
#define MODES_SHORT_MSG_BYTES           (56/8)

#define MODES_ICAO_CACHE_LEN            1024                /* Power of two required. */
#define MODES_ICAO_CACHE_TTL            60                  /* Time to live of cached addresses. */
#define MODES_UNIT_FEET                 0
#define MODES_UNIT_METERS               1

#define MODES_DEBUG_DEMOD               (1<<0)
#define MODES_DEBUG_DEMODERR            (1<<1)
#define MODES_DEBUG_BADCRC              (1<<2)
#define MODES_DEBUG_GOODCRC             (1<<3)
#define MODES_DEBUG_NOPREAMBLE          (1<<4)
#define MODES_DEBUG_NET                 (1<<5)
#define MODES_DEBUG_JS                  (1<<6)

/* When debug is set to MODES_DEBUG_NOPREAMBLE, the first sample must be
 * at least greater than a given level for us to dump the signal. */
#define MODES_DEBUG_NOPREAMBLE_LEVEL    25

#define MODES_INTERACTIVE_REFRESH_TIME  250                 /* Milliseconds */
#define MODES_INTERACTIVE_ROWS          15                  /* Rows on screen */
#define MODES_INTERACTIVE_TTL           60                  /* TTL before being removed */

#define MODES_NET_MAX_FD                1024
#define MODES_NET_OUTPUT_RAW_PORT       30000
#define MODES_NET_OUTPUT_SBS_PORT       40000
#define MODES_NET_HTTP_PORT             8081                //apache or tomcat may use 8080
#define MODES_NET_INPUT_RAW_PORT        50000
#define MODES_CLIENT_BUF_SIZE           1024
#define MODES_NET_SNDBUF_SIZE           (1024*64)

#ifndef MODES_NOTUSED
#define MODES_NOTUSED(Value)            ((void) Value)
#endif

/* Structure used to describe a networking client. */
typedef struct {
    int fd;                                                 /* File descriptor. */
    int service;                                            /* TCP port the client is connected to. */
    char buf[MODES_CLIENT_BUF_SIZE+1];                      /* Read buffer. */
    int buflen;                                             /* Amount of data on buffer. */
} client;

/* Structure used to describe an aircraft in iteractive mode. */
typedef struct airtag {
    uint32_t addr;                                          /* ICAO address */
    char hexaddr[7];                                        /* Printable ICAO address */
    char flight[9];                                         /* Flight number */
    int altitude;                                           /* Altitude */
    int speed;                                              /* Velocity computed from EW and NS components. */
    int track;                                              /* Angle of flight. */
    time_t seen;                                            /* Time at which the last packet was received. */
    long messages;                                          /* Number of Mode S messages received. */

    /* Encoded latitude and longitude as extracted by odd and even
     * CPR encoded messages.
     * */
    int odd_cprlat;
    int odd_cprlon;
    int even_cprlat;
    int even_cprlon;
    double lat, lon;                                        /* Coordinated obtained from CPR encoded data. */
    long long odd_cprtime, even_cprtime;

    struct airtag *next;                                    /* make a list for this struct. */
} aircraft;

/* Program global state. */
typedef struct {
    /* Internal state */
    pthread_t reader_thread;
    pthread_mutex_t data_mutex;                             /* Mutex to synchronize buffer access. */
    pthread_cond_t data_cond;                               /* Conditional variable associated. */
    unsigned char *data;                                    /* Raw IQ samples buffer */
    uint16_t *magnitude;                                    /* Magnitude vector */
    uint32_t data_len;                                      /* Buffer length. */
    int fd;                                                 /* --ifile option file descriptor. */
    int data_ready;                                         /* Data ready to be processed. */
    uint32_t *icao_cache;                                   /* Recently seen ICAO addresses cache. */
    uint16_t *maglut;                                       /* I/Q -> Magnitude lookup table. */
    int exit;                                               /* Exit from the main loop when true. */

    /* Drivers */
    int prefer_airspy;
    int prefer_hackrf;
    int prefer_rtlsdr;
    int prefer_sdrplay;

    /* RTLSDR */
    int rtl_enabled;
    int dev_index;
    int gain;
    int enable_agc;
    rtlsdr_dev_t *dev;

    /* HackRF One and Airspy are very similar... */
    int hackrf_enabled;
    int rf_gain;
    int lna_gain;
    int vga_gain;
    hackrf_device *hackrf;

    /* ... but AirSpy needs to be resampled */
    int airspy_enabled;
    struct airspy_device *airspy;
    soxr_t resampler;
    char *airspy_bytes, *airspy_scratch;

    /* SDRplay */
    int sdrplay_enabled;
    int sdrplaySamplesPerPacket;
    short *sdrplay_i;
    short *sdrplay_q;

    /* SDR Common */
    long long int freq;

    /* Networking */
    char aneterr[ANET_ERR_LEN];
    client *clients[MODES_NET_MAX_FD];                      /* Our clients. */
    int maxfd;                                              /* Greatest fd currently active. */
    int sbsos;                                              /* SBS output listening socket. */
    int ros;                                                /* Raw output listening socket. */
    int ris;                                                /* Raw input listening socket. */
    int https;                                              /* HTTP listening socket. */

    /* Configuration */
    char *filename;                                         /* Input form file, --ifile option. */
    int fix_errors;                                         /* Single bit error correction if true. */
    int check_crc;                                          /* Only display messages with good CRC. */
    int raw;                                                /* Raw output format. */
    int debug;                                              /* Debugging mode. */
    int net;                                                /* Enable networking. */
    int net_only;                                           /* Enable just networking. */
    int interactive;                                        /* Interactive mode */
    int interactive_rows;                                   /* Interactive mode: max number of rows. */
    int interactive_ttl;                                    /* Interactive mode: TTL before deletion. */
    int stats;                                              /* Print stats at exit in --ifile mode. */
    int onlyaddr;                                           /* Print only ICAO addresses. */
    int metric;                                             /* Use metric units. */
    int aggressive;                                         /* Aggressive detection algorithm. */

    /* Interactive mode */
    aircraft *aircrafts;
    long long interactive_last_update;                      /* Last screen update in milliseconds */

    /* Statistics */
    long long stat_valid_preamble;
    long long stat_demodulated;
    long long stat_goodcrc;
    long long stat_badcrc;
    long long stat_fixed;
    long long stat_single_bit_fix;
    long long stat_two_bits_fix;
    long long stat_http_requests;
    long long stat_sbs_connections;
    long long stat_out_of_phase;
} Modes;

extern Modes ms;

/* The struct we use to store information about a decoded message. */
typedef struct {
    /* Generic fields */
    unsigned char msg[MODES_LONG_MSG_BYTES];                /* Binary message. */
    int msgbits;                                            /* Number of bits in message */
    int msgtype;                                            /* Downlink format # */
    int crcok;                                              /* True if CRC was valid */
    uint32_t crc;                                           /* Message CRC */
    int errorbit;                                           /* Bit corrected. -1 if no bit corrected. */
    int aa1, aa2, aa3;                                      /* ICAO Address bytes 1 2 and 3 */
    int phase_corrected;                                    /* True if phase correction was applied. */

    /* DF 11 */
    int ca;                                                 /* Responder capabilities. */

    /* DF 17 */
    int metype;                                             /* Extended squitter message type. */
    int mesub;                  /* Extended squitter message subtype. */
    int heading_is_valid;
    int heading;
    int aircraft_type;
    int fflag;                  /* 1 = Odd, 0 = Even CPR message. */
    int tflag;                  /* UTC synchronized? */
    int raw_latitude;           /* Non decoded latitude */
    int raw_longitude;          /* Non decoded longitude */
    char flight[9];             /* 8 chars flight number. */
    int ew_dir;                 /* 0 = East, 1 = West. */
    int ew_velocity;            /* E/W velocity. */
    int ns_dir;                 /* 0 = North, 1 = South. */
    int ns_velocity;            /* N/S velocity. */
    int vert_rate_source;       /* Vertical rate source. */
    int vert_rate_sign;         /* Vertical rate sign. */
    int vert_rate;              /* Vertical rate. */
    int velocity;               /* Computed from EW and NS velocity. */

    /* DF4, DF5, DF20, DF21 */
    int fs;                     /* Flight status for DF4,5,20,21 */
    int dr;                     /* Request extraction of downlink request. */
    int um;                     /* Request extraction of downlink request. */
    int identity;               /* 13 bits identity (Squawk). */

    /* Fields used by multiple message types. */
    int altitude, unit;
} modesMessage;

extern float buf[16384];

void dumpMagnitudeBar(int index, int magnitude);
void dumpMagnitudeVector(uint16_t *m, uint32_t offset);
void computeMagnitudeVector(void);
void detectModeS(uint16_t *m, uint32_t mlen);
aircraft *interactiveCreateAircraft(uint32_t addr);
aircraft *interactiveFindAircraft(uint32_t addr);
int cprModFunction(int a, int b);
int cprNFunction(double lat, int isodd);
double cprDlonFunction(double lat, int isodd);
aircraft *interactiveReceiveData(modesMessage *mm);
void interactiveRemoveStaleAircrafts(void);
void snipMode(int level);
void showHelp(void);
void backgroundTasks(void);
int main(int argc, char **argv);

#endif

//===============================================================================
//code by </MATRIX>@Neod Anderjon
