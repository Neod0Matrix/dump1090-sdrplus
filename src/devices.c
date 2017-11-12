#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

modesNetServices modesNetServer[MODES_NET_SERVICES_NUM] = {
        {"Raw TCP output",          &ms.ros,     MODES_NET_OUTPUT_RAW_PORT},
        {"Raw TCP input",           &ms.ris,     MODES_NET_INPUT_RAW_PORT},
        {"HTTP server",             &ms.https,   MODES_NET_HTTP_PORT},
        {"Basestation TCP output",  &ms.sbsos,   MODES_NET_OUTPUT_SBS_PORT}
};

/* =============================== Initialization =========================== */
void modesInitConfig(void) {
    signal(SIGINT, INTHandler);     //signal init

    ms.gain = MODES_MAX_GAIN;
    ms.dev_index = 0;
    ms.enable_agc = 0;
    ms.rf_gain = 0;
    ms.lna_gain = 0;
    ms.vga_gain = 0;
    ms.freq = MODES_DEFAULT_FREQ;
    ms.filename = NULL;
    ms.fix_errors = 1;
    ms.check_crc = 1;
    ms.raw = 0;
    ms.net = 0;
    ms.net_only = 0;
    ms.onlyaddr = 0;
    ms.debug = 0;
    ms.interactive = 0;
    ms.interactive_rows = MODES_INTERACTIVE_ROWS;
    ms.interactive_ttl = MODES_INTERACTIVE_TTL;
    ms.aggressive = 0;
    ms.interactive_rows = getTermRows();
}

void modesInit(void) {
    int i, q;

    pthread_mutex_init(&ms.data_mutex,NULL);
    pthread_cond_init(&ms.data_cond,NULL);
    /* We add a full message minus a final bit to the length, so that we
     * can carry the remaining part of the buffer that we can't process
     * in the message detection loop, back at the start of the next data
     * to process. This way we are able to also detect messages crossing
     * two reads. */
    ms.data_len = MODES_DATA_LEN + (MODES_FULL_LEN-1)*4;
    ms.data_ready = 0;
    /* Allocate the ICAO address cache. We use two uint32_t for every
     * entry because it's a addr / timestamp pair for every entry. */
    ms.icao_cache = malloc(sizeof(uint32_t)*MODES_ICAO_CACHE_LEN*2);
    memset(ms.icao_cache,0,sizeof(uint32_t)*MODES_ICAO_CACHE_LEN*2);
    ms.aircrafts = NULL;
    ms.interactive_last_update = 0;
    if ((ms.data = malloc(ms.data_len)) == NULL ||
        (ms.magnitude = malloc(ms.data_len*2)) == NULL) {
        fprintf(stderr, "Out of memory allocating data buffer.\n");
        exit(1);
    }
    memset(ms.data,127,ms.data_len);

    /* Populate the I/Q -> Magnitude lookup table. It is used because
     * sqrt or round may be expensive and may vary a lot depending on
     * the libc used.
     *
     * We scale to 0-255 range multiplying by 1.4 in order to ensure that
     * every different I/Q pair will result in a different magnitude value,
     * not losing any resolution. */
    ms.maglut = malloc(129*129*2);
    for (i = 0; i <= 128; i++) {
        for (q = 0; q <= 128; q++) {
            ms.maglut[i*129+q] = round(sqrt(i*i+q*q)*360);
        }
    }

    /* Statistics */
    ms.stat_valid_preamble = 0;
    ms.stat_demodulated = 0;
    ms.stat_goodcrc = 0;
    ms.stat_badcrc = 0;
    ms.stat_fixed = 0;
    ms.stat_single_bit_fix = 0;
    ms.stat_two_bits_fix = 0;
    ms.stat_http_requests = 0;
    ms.stat_sbs_connections = 0;
    ms.stat_out_of_phase = 0;
    ms.exit = 0;
}

/* =============================== RTLSDR handling ========================== */
int modesInitRTLSDR(void) {
    int j;
    int device_count;
    int ppm_error = 0;
    char vendor[256], product[256], serial[256];

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        fprintf(stderr, "No supported RTLSDR devices found.\n");
        return(1);
    }

    fprintf(stderr, "Found %d device(s):\n", device_count);
    for (j = 0; j < device_count; j++) {
        rtlsdr_get_device_usb_strings(j, vendor, product, serial);
        fprintf(stderr, "%d: %s, %s, SN: %s %s\n", j, vendor, product, serial,
                (j == ms.dev_index) ? "(currently selected)" : "");
    }

    if (rtlsdr_open(&ms.dev, ms.dev_index) < 0) {
        fprintf(stderr, "Error opening the RTLSDR device: %s\n",
                strerror(errno));
        return(1);
    }

    /* Set gain, frequency, sample rate, and reset the device. */
    rtlsdr_set_tuner_gain_mode(ms.dev,
                               (ms.gain == MODES_AUTO_GAIN) ? 0 : 1);
    if (ms.gain != MODES_AUTO_GAIN) {
        if (ms.gain == MODES_MAX_GAIN) {
            /* Find the maximum gain available. */
            int numgains;
            int gains[100];

            numgains = rtlsdr_get_tuner_gains(ms.dev, gains);
            ms.gain = gains[numgains-1];
            fprintf(stderr, "Max available gain is: %.2f\n", ms.gain/10.0);
        }
        rtlsdr_set_tuner_gain(ms.dev, ms.gain);
        fprintf(stderr, "Setting gain to: %.2f\n", ms.gain/10.0);
    } else {
        fprintf(stderr, "Using automatic gain control.\n");
    }
    rtlsdr_set_freq_correction(ms.dev, ppm_error);
    if (ms.enable_agc) rtlsdr_set_agc_mode(ms.dev, 1);
    rtlsdr_set_center_freq(ms.dev, ms.freq);
    rtlsdr_set_sample_rate(ms.dev, MODES_DEFAULT_RATE);
    rtlsdr_reset_buffer(ms.dev);
    fprintf(stderr, "Gain reported by device: %.2f\n",
            rtlsdr_get_tuner_gain(ms.dev)/10.0);
    ms.rtl_enabled = 1;
    ms.hackrf_enabled = 0;
    ms.airspy_enabled = 0;
    ms.sdrplay_enabled = 0;
    return (0);
}

/* =============================== AirSpy handling ========================== */
int modesInitAirSpy(void) {
#define AIRSPY_STATUS(status, message) \
        if (status != 0) { \
            fprintf(stderr, "%s\n", message); \
            airspy_close(ms.airspy); \
            airspy_exit(); \
            return (1); \
        } \

    int status;
    soxr_error_t	sox_err = NULL;
    soxr_io_spec_t	ios;
    soxr_quality_spec_t	qts;
    soxr_runtime_spec_t	rts;

    ms.airspy_scratch = calloc(2*MODES_DATA_LEN, sizeof(int16_t));
    ms.airspy_bytes = malloc(2*MODES_DATA_LEN);
    if ((ms.airspy_bytes == NULL) || (ms.airspy_scratch == NULL))
        err(1, NULL);

    ios = soxr_io_spec(SOXR_INT16_I, SOXR_INT16_I);
    qts = soxr_quality_spec(SOXR_MQ, 0);
    rts = soxr_runtime_spec(2);

    ms.resampler = soxr_create(10, 2, 2, &sox_err, &ios, &qts, &rts);
    if (sox_err) {
        int e = errno;
        fprintf(stderr, "soxr_create: %s; %s\n", soxr_strerror(sox_err), strerror(errno));
        return e;
    }

    status = airspy_init();
    AIRSPY_STATUS(status, "airspy_init failed.");

    status = airspy_open(&ms.airspy);
    AIRSPY_STATUS(status, "No AirSpy compatible devices found.");

    if ((ms.rf_gain + ms.lna_gain + ms.vga_gain) == 0) {
        ms.rf_gain = AIRSPY_RF_GAIN;
        ms.lna_gain = AIRSPY_LNA_GAIN;
        ms.vga_gain = AIRSPY_VGA_GAIN;
    }

    status = airspy_set_freq(ms.airspy, ms.freq);
    AIRSPY_STATUS(status, "airspy_set_freq failed.");

    status = airspy_set_sample_type(ms.airspy, AIRSPY_SAMPLE_INT16_IQ);
    AIRSPY_STATUS(status, "airspy_set_sample_type failed.");

    status = airspy_set_samplerate(ms.airspy, AIRSPY_SAMPLERATE_10MSPS);
    AIRSPY_STATUS(status, "airspy_set_samplerate failed.");

    status = airspy_set_mixer_gain(ms.airspy, ms.rf_gain != 0);
    AIRSPY_STATUS(status, "airspy_set_mixer_gain failed.");

    status = airspy_set_lna_gain(ms.airspy, ms.lna_gain);
    AIRSPY_STATUS(status, "airspy_set_lna_gain failed.");

    status = airspy_set_vga_gain(ms.airspy, ms.vga_gain);
    AIRSPY_STATUS(status, "airspy_set_vga_gain failed");

    if (ms.enable_agc) {
        airspy_set_mixer_agc(ms.airspy, 1);
        AIRSPY_STATUS(status, "airspy_set_mixer_agc failed");
        airspy_set_lna_agc(ms.airspy, 1);
        AIRSPY_STATUS(status, "airspy_set_lna_agc failed");
    }
    fprintf (stderr, "AirSpy successfully initialized "
                     "(RF Gain: %i, LNA Gain: %i, VGA Gain: %i, AGC: %i).\n",
             ms.rf_gain, ms.lna_gain, ms.vga_gain, ms.enable_agc);

    ms.airspy_enabled = 1;
    ms.rtl_enabled = 0;
    ms.hackrf_enabled = 0;
    ms.sdrplay_enabled = 0;

    return (0);
}

/* =============================== HackRF One handling ========================== */
int modesInitHackRF(void) {
#define HACKRF_STATUS(status, message) \
        if (status != 0) { \
            fprintf(stderr, "%s\n", message); \
            hackrf_close(ms.hackrf); \
            hackrf_exit(); \
            return (1); \
        } \

    int status;

    status = hackrf_init();
    HACKRF_STATUS(status, "hackrf_init failed.");

    status = hackrf_open(&ms.hackrf);
    HACKRF_STATUS(status, "No HackRF compatible devices found.");

    if ((ms.lna_gain + ms.vga_gain) == 0) {
        ms.lna_gain = HACKRF_LNA_GAIN;
        ms.vga_gain = HACKRF_VGA_GAIN;
    }

    status = hackrf_set_freq(ms.hackrf, ms.freq);
    HACKRF_STATUS(status, "hackrf_set_freq failed.");

    status = hackrf_set_sample_rate(ms.hackrf, MODES_DEFAULT_RATE);
    HACKRF_STATUS(status, "hackrf_set_sample_rate failed.");

    status = hackrf_set_amp_enable(ms.hackrf, ms.rf_gain != 0);
    HACKRF_STATUS(status, "hackrf_set_amp_enable failed.");

    status = hackrf_set_lna_gain(ms.hackrf, ms.lna_gain);
    HACKRF_STATUS(status, "hackrf_set_lna_gain failed.");

    status = hackrf_set_vga_gain(ms.hackrf, ms.vga_gain);
    HACKRF_STATUS(status, "hackrf_set_vga_gain failed");

    fprintf (stderr, "HackRF successfully initialized "
                     "(AMP Enable: %i, LNA Gain: %i, VGA Gain: %i).\n",
             ms.rf_gain, ms.lna_gain, ms.vga_gain);

    ms.hackrf_enabled = 1;
    ms.airspy_enabled = 0;
    ms.rtl_enabled = 0;
    ms.sdrplay_enabled = 0;

    return (0);
}

/* =============================== SDRplay handling ========================== */
int modesInitSDRplay(void) {

    mir_sdr_ErrT err;
    float ver;

    /* Check API version */
    err = mir_sdr_ApiVersion(&ver);
    if (err ||  (ver != MIR_SDR_API_VERSION)) {
        fprintf(stderr, "Incorrect API version %f\n", ver);
        return (1);
    }

    mir_sdr_SetParam(201,1);
    mir_sdr_SetParam(202,0);

    /* Initialize SDRplay device */
    err = mir_sdr_Init (9, 8.000, 1090.048, mir_sdr_BW_1_536, mir_sdr_IF_2_048, &ms.sdrplaySamplesPerPacket);

    if (err){
        fprintf(stderr, "Unable to initialize RSP\n");
        return (1);
    }
    /* Allocate 16-bit I and Q buffers */

    ms.sdrplay_i = malloc (ms.sdrplaySamplesPerPacket * sizeof(short));
    ms.sdrplay_q = malloc (ms.sdrplaySamplesPerPacket * sizeof(short));

    if ((ms.sdrplay_i == NULL) || (ms.sdrplay_q == NULL)){
        fprintf(stderr, "Insufficient memory for buffers\n");
        return (1);
    }

    /* Configure DC tracking in tuner */
    err = mir_sdr_SetDcMode(4,0);
    err |= mir_sdr_SetDcTrackTime(63);
    if (err){
        fprintf(stderr, "Set DC tracking failed, %d\n", err);
        return (1);
    }

    ms.sdrplay_enabled = 1;
    ms.hackrf_enabled = 0;
    ms.airspy_enabled = 0;
    ms.rtl_enabled = 0;

    return (0);
}

/* We use a thread reading data in background, while the main thread
 * handles decoding and visualization of data to the user.
 *
 * The reading thread calls the RTLSDR API to read data asynchronously, and
 * uses a callback to populate the data buffer.
 * A Mutex is used to avoid races with the decoding thread. */
void rtlsdrCallback(unsigned char *buf, uint32_t len, void *ctx) {
    MODES_NOTUSED(ctx);

    pthread_mutex_lock(&ms.data_mutex);
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;
    /* Move the last part of the previous buffer, that was not processed,
     * on the start of the new buffer. */
    memcpy(ms.data, ms.data+MODES_DATA_LEN, (MODES_FULL_LEN-1)*4);
    /* Read the new data. */
    memcpy(ms.data+(MODES_FULL_LEN-1)*4, buf, len);
    ms.data_ready = 1;
    /* Signal to the other thread that new data is ready */
    pthread_cond_signal(&ms.data_cond);
    pthread_mutex_unlock(&ms.data_mutex);
}

int hackrfCallback (hackrf_transfer *transfer) {
    uint32_t i;
    pthread_mutex_lock(&ms.data_mutex);
    uint32_t len = transfer-> buffer_length;
    /* HackRF One returns signed IQ values, convert them to unsigned */
    for (i = 0; i < len; i++) {
        transfer->buffer[i] ^= (uint8_t)0x80;
    }
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;
    /* Move the last part of the previous buffer, that was not processed,
     * on the start of the new buffer. */
    memcpy(ms.data, ms.data+MODES_DATA_LEN, (MODES_FULL_LEN-1)*4);
    /* Read the new data. */
    memcpy(ms.data+(MODES_FULL_LEN-1)*4, transfer->buffer, len);
    ms.data_ready = 1;
    /* Signal to the other thread that new data is ready */
    pthread_cond_signal(&ms.data_cond);
    pthread_mutex_unlock(&ms.data_mutex);
    return (0);
}

int airspyCallback (airspy_transfer *transfer) {
    pthread_mutex_lock(&ms.data_mutex);
    int16_t *inptr = (int16_t *)transfer->samples;
    int16_t *outptr = (int16_t *)ms.airspy_scratch;
    size_t i, i_done, o_done, i_len, len;

    i_len = transfer->sample_count;
    len = 4 * i_len / 5; // downsample from 2.5Msps to 2Msps
    soxr_process(ms.resampler, inptr, i_len, &i_done, outptr, len, &o_done);
    for(i = 0; i < o_done; i++)
        ms.airspy_bytes[i] = (int8_t)(outptr[i]>>4)+127;
    len = o_done;
    if (len > MODES_DATA_LEN) len = MODES_DATA_LEN;
    /* Move the last part of the previous buffer, that was not processed,
     * on the start of the new buffer. */
    memcpy(ms.data, ms.data+MODES_DATA_LEN, (MODES_FULL_LEN-1)*4);
    /* Read the new data. */
    memcpy(ms.data+(MODES_FULL_LEN-1)*4, ms.airspy_bytes, len);
    ms.data_ready = 1;
    /* Signal to the other thread that new data is ready */
    pthread_cond_signal(&ms.data_cond);
    pthread_mutex_unlock(&ms.data_mutex);
    return (0);
}

int sdrplay_start_rx(void) {
    unsigned int data_index, firstSampleNum;
    int grChanged, rfChanged, fsChanged;
    int input_index = ms.sdrplaySamplesPerPacket;
    mir_sdr_ErrT err = 0;

    pthread_mutex_lock(&ms.data_mutex);
    while(1)
    {

        if (ms.data_ready) {
            pthread_cond_wait(&ms.data_cond,&ms.data_mutex);
            continue;
        }

        /* Move the last part of the previous buffer, that was not processed,
         * on the start of the new buffer. */

        memcpy(ms.magnitude, ms.magnitude+MODES_DATA_LEN, (MODES_FULL_LEN-1)*4);

        /* now read new data buffer */

        data_index = (MODES_FULL_LEN-1)*2;
        while (data_index < ((MODES_DATA_LEN/2) + (MODES_FULL_LEN-1)*2))
        {
            /* copy available data into buffer */

            while ((data_index < (MODES_DATA_LEN/2 + (MODES_FULL_LEN-1)*2)) && (input_index < ms.sdrplaySamplesPerPacket))
            {
                int sum = abs(ms.sdrplay_i[input_index++]);
                sum += abs(ms.sdrplay_i[input_index++]);
                sum += abs(ms.sdrplay_i[input_index++]);
                sum += abs(ms.sdrplay_i[input_index++]);
                sum = sum >> 2;
                if (sum > 32767) sum = 32767;
                ms.magnitude[data_index++] = sum;
            }

            if (input_index > ms.sdrplaySamplesPerPacket) {
                fprintf(stderr, "ERROR packet size not divisible by 4\n");
                ms.exit = 1; /* Signal the other thread to exit. */
                break;
            }


            if (input_index == ms.sdrplaySamplesPerPacket)
            {
                input_index = 0;
                err = mir_sdr_ReadPacket (ms.sdrplay_i, ms.sdrplay_q,
                                          &firstSampleNum, &grChanged, &rfChanged, &fsChanged);

                if (err){
                    fprintf(stderr, "sdrplay data read failed\n");
                    ms.exit = 1; /* Signal the other thread to exit. */
                    break;
                }
            }
        }

        ms.data_ready = 1;
        /* Signal to the other thread that new data is ready */
        pthread_cond_signal(&ms.data_cond);
    }
    return (err)? 1 : 0;
}

/* Networking "stack" initialization. */
void modesInitNet(void) {
    memset(ms.clients,0,sizeof(ms.clients));
    ms.maxfd = -1;

    for (uint8_t j = 0; j < MODES_NET_SERVICES_NUM; j++) {
        int s = anetTcpServer(ms.aneterr, modesNetServer[j].port, NULL);
        //try more to map port
        while (s == -1) {
            fprintf(stderr, "Port %d(%s) opened failure: %s, add port number retry\n", \
                modesNetServer[j].port, \
                modesNetServer[j].descr, \
                strerror(errno));
            modesNetServer[j].port += 1;
            s = anetTcpServer(ms.aneterr, modesNetServer[j].port, NULL);//update value
        }
        anetNonBlock(ms.aneterr, s);
        *modesNetServer[j].socket = s;
        printf("Map no.%d port last port-number: %d\n", j, modesNetServer[j].port);
        getchar();      //wait user check port number
    }
    signal(SIGPIPE, SIG_IGN);

    //open map web page to watch airplane
    //mapWebPageOpen(modesNetServer[MODES_NET_SERVICE_HTTP].port, StdUserName);
}

/* This function gets called from time to time when the decoding thread is
 * awakened by new data arriving. This usually happens a few times every
 * second. */
void modesAcceptClients(void) {
    int fd, port;
    uint32_t j;
    client *c;

    for (j = 0; j < MODES_NET_SERVICES_NUM; j++) {
        fd = anetTcpAccept(ms.aneterr, *modesNetServer[j].socket, NULL, &port);
        if (fd == -1) {
            if (ms.debug & MODES_DEBUG_NET && errno != EAGAIN)
                printf("Accept %d: %s\n", *modesNetServer[j].socket, strerror(errno));
            continue;
        }

        if (fd >= MODES_NET_MAX_FD) {
            close(fd);
            return; /* Max number of clients reached. */
        }

        anetNonBlock(ms.aneterr, fd);
        c = malloc(sizeof(*c));
        c->service = *modesNetServer[j].socket;
        c->fd = fd;
        c->buflen = 0;
        ms.clients[fd] = c;
        anetSetSendBuffer(ms.aneterr,fd,MODES_NET_SNDBUF_SIZE);

        if (ms.maxfd < fd) ms.maxfd = fd;
        if (*modesNetServer[j].socket == ms.sbsos)
            ms.stat_sbs_connections++;

        j--; /* Try again with the same listening port. */

        if (ms.debug & MODES_DEBUG_NET)
            printf("Created new client %d\n", fd);
    }
}

/* This function is used when "net only" mode is enabled to know when there
 * is at least a new client to serve. Note that the dump1090 networking model
 * is extremely trivial and a function takes care of handling all the clients
 * that have something to serve, without a proper event library, so the
 * function here returns as long as there is a single client ready, or
 * when the specified timeout in milliesconds elapsed, without specifying to
 * the caller what client requires to be served. */
void modesWaitReadableClients(int timeout_ms) {
    struct timeval tv;
    fd_set fds;
    int maxfd = ms.maxfd;
    uint8_t j;

    FD_ZERO(&fds);

    /* Set client FDs */
    for (j = 0; j <= ms.maxfd; j++) {
        if (ms.clients[j]) FD_SET(j,&fds);
    }

    /* Set listening sockets to accept new clients ASAP. */
    for (j = 0; j < MODES_NET_SERVICES_NUM; j++) {
        int s = *modesNetServer[j].socket;
        FD_SET(s,&fds);
        if (s > maxfd) maxfd = s;
    }

    tv.tv_sec = timeout_ms/1000;
    tv.tv_usec = (timeout_ms%1000)*1000;
    /* We don't care why select returned here, timeout, error, or
     * FDs ready are all conditions for which we just return. */
    select(maxfd+1, &fds, NULL, NULL, &tv);
}

/* Get an HTTP request header and write the response to the client.
 * Again here we assume that the socket buffer is enough without doing
 * any kind of userspace buffering.
 *
 * Returns 1 on error to signal the caller the client connection should
 * be closed. */
int handleHTTPRequest(client *c) {
    char hdr[512];
    int clen, hdrlen;
    int httpver, keepalive;
    char *p, *url, *content;
    char *ctype;

    if (ms.debug & MODES_DEBUG_NET)
        printf("\nHTTP request: %s\n", c->buf);

    /* Minimally parse the request. */
    httpver = (strstr(c->buf, "HTTP/1.1") != NULL) ? 11 : 10;
    if (httpver == 10) {
        /* HTTP 1.0 defaults to close, unless otherwise specified. */
        keepalive = strstr(c->buf, "Connection: keep-alive") != NULL;
    } else if (httpver == 11) {
        /* HTTP 1.1 defaults to keep-alive, unless close is specified. */
        keepalive = strstr(c->buf, "Connection: close") == NULL;
    }

    /* Identify he URL. */
    p = strchr(c->buf,' ');
    if (!p) return 1; /* There should be the method and a space... */
    url = ++p; /* Now this should point to the requested URL. */
    p = strchr(p, ' ');
    if (!p) return 1; /* There should be a space before HTTP/... */
    *p = '\0';

    if (ms.debug & MODES_DEBUG_NET) {
        printf("\nHTTP keep alive: %d\n", keepalive);
        printf("HTTP requested URL: %s\n\n", url);
    }

    /* Select the content to send, we have just two so far:
     * "/" -> Our google map application.
     * "/data.json" -> Our ajax request to update planes. */
    if (strstr(url, "/data.json")) {
        content = aircraftsToJson(&clen);
        ctype = MODES_CONTENT_TYPE_JSON;
    } else {
        struct stat sbuf;
        int fd = -1;

        //confirm gmap.html file
        if (stat(WebMapFile,&sbuf) != -1 &&
            (fd = open(WebMapFile,O_RDONLY)) != -1)
        {
            content = malloc(sbuf.st_size);
            if (read(fd,content,sbuf.st_size) == -1) {
                snprintf(content,sbuf.st_size,"Error reading from local-map file: %s",
                         strerror(errno));
            }
            clen = sbuf.st_size;
        } else {
            char buf[128];

            clen = snprintf(buf,sizeof(buf),"Error opening local-map html file: %s",
                            strerror(errno));
            content = strdup(buf);
        }
        if (fd != -1) close(fd);
        ctype = MODES_CONTENT_TYPE_HTML;
    }

    /* Create the header and send the reply. */
    hdrlen = snprintf(hdr, sizeof(hdr),
                      "HTTP/1.1 200 OK\r\n"
                              "Server: Dump1090\r\n"
                              "Content-Type: %s\r\n"
                              "Connection: %s\r\n"
                              "Content-Length: %d\r\n"
                              "Access-Control-Allow-Origin: *\r\n"
                              "\r\n",
                      ctype,
                      keepalive ? "keep-alive" : "close",
                      clen);

    if (ms.debug & MODES_DEBUG_NET)
        printf("HTTP Reply header:\n%s", hdr);

    /* Send header and content. */
    if (write(c->fd, hdr, hdrlen) != hdrlen ||
        write(c->fd, content, clen) != clen)
    {
        free(content);
        return 1;
    }
    free(content);
    ms.stat_http_requests++;
    return !keepalive;
}

/* This function polls the clients using read() in order to receive new
 * messages from the net.
 *
 * The message is supposed to be separated by the next message by the
 * separator 'sep', that is a null-terminated C string.
 *
 * Every full message received is decoded and passed to the higher layers
 * calling the function 'handler'.
 *
 * The handelr returns 0 on success, or 1 to signal this function we
 * should close the connection with the client in case of non-recoverable
 * errors. */
void modesReadFromClient(client *c, char *sep, int(*handler)(client *)) {
    while(1) {
        int left = MODES_CLIENT_BUF_SIZE - c->buflen;
        int nread = read(c->fd, c->buf+c->buflen, left);
        int i, fullmsg = 0;
        char *p;

        if (nread <= 0) {
            if (nread == 0 || errno != EAGAIN) {
                /* Error, or end of file. */
                modesFreeClient(c->fd);
            }
            break; /* Serve next client. */
        }
        c->buflen += nread;

        /* Always null-term so we are free to use strstr() */
        c->buf[c->buflen] = '\0';

        /* If there is a complete message there must be the separator 'sep'
         * in the buffer, note that we full-scan the buffer at every read
         * for simplicity. */
        while ((p = strstr(c->buf, sep)) != NULL) {
            i = p - c->buf; /* Turn it as an index inside the buffer. */
            c->buf[i] = '\0'; /* Te handler expects null terminated strings. */
            /* Call the function to process the message. It returns 1
             * on error to signal we should close the client connection. */
            if (handler(c)) {
                modesFreeClient(c->fd);
                return;
            }
            /* Move what's left at the start of the buffer. */
            i += strlen(sep); /* The separator is part of the previous msg. */
            memmove(c->buf,c->buf+i,c->buflen-i);
            c->buflen -= i;
            c->buf[c->buflen] = '\0';
            /* Maybe there are more messages inside the buffer.
             * Start looping from the start again. */
            fullmsg = 1;
        }
        /* If our buffer is full discard it, this is some badly
         * formatted shit. */
        if (c->buflen == MODES_CLIENT_BUF_SIZE) {
            c->buflen = 0;
            /* If there is garbage, read more to discard it ASAP. */
            continue;
        }
        /* If no message was decoded process the next client, otherwise
         * read more data from the same client. */
        if (!fullmsg) break;
    }
}

/* Read data from clients. This function actually delegates a lower-level
 * function that depends on the kind of service (raw, http, ...). */
void modesReadFromClients(void) {
    int j;
    client *c;

    for (j = 0; j <= ms.maxfd; j++) {
        if ((c = ms.clients[j]) == NULL) continue;
        if (c->service == ms.ris)
            modesReadFromClient(c, "\n", decodeHexMessage);
        else if (c->service == ms.https)
            modesReadFromClient(c, "\r\n\r\n", handleHTTPRequest);
    }
}

/* ============================ Terminal handling  ========================== */

/* Handle resizing terminal. */
void sigWinchCallback(void) {
    signal(SIGWINCH, SIG_IGN);
    ms.interactive_rows = getTermRows();
    interactiveShowData();
    signal(SIGWINCH, (__sighandler_t)sigWinchCallback);//from signal.h type __sighandler_t
}

//===============================================================================
//code by </MATRIX>@Neod Anderjon
