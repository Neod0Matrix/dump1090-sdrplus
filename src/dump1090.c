#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================

float buf[16384];
Modes ms;

/* ============================= Utility functions ========================== */
static long long mstime(void) {
    struct timeval tv;
    long long mst;

    gettimeofday(&tv, NULL);
    mst = ((long long)tv.tv_sec)*1000;
    mst += tv.tv_usec/1000;
    return mst;
}

/* ============================== Debugging ================================= */

/* Helper function for dumpMagnitudeVector().
 * It prints a single bar used to display raw signals.
 *
 * Since every magnitude sample is between 0-255, the function uses
 * up to 63 characters for every bar. Every character represents
 * a length of 4, 3, 2, 1, specifically:
 *
 * "O" is 4
 * "o" is 3
 * "-" is 2
 * "." is 1
 */
void dumpMagnitudeBar(int index, int magnitude) {
    char *set = " .-o";
    char buf[256];
    int div = magnitude / 256 / 4;
    int rem = magnitude / 256 % 4;

    memset(buf,'O',div);
    buf[div] = set[rem];
    buf[div+1] = '\0';

    if (index >= 0)
        printf("[%.3d] |%-66s %d\n", index, buf, magnitude);
    else
        printf("[%.2d] |%-66s %d\n", index, buf, magnitude);
}

/* Display an ASCII-art alike graphical representation of the undecoded
 * message as a magnitude signal.
 *
 * The message starts at the specified offset in the "m" buffer.
 * The function will display enough data to cover a short 56 bit message.
 *
 * If possible a few samples before the start of the messsage are included
 * for context. */

void dumpMagnitudeVector(uint16_t *m, uint32_t offset) {
    uint32_t padding = 5; /* Show a few samples before the actual start. */
    uint32_t start = (offset < padding) ? 0 : offset-padding;
    uint32_t end = offset + (MODES_PREAMBLE_US*2)+(MODES_SHORT_MSG_BITS*2) - 1;
    uint32_t j;

    for (j = start; j <= end; j++) {
        dumpMagnitudeBar(j-offset, m[j]);
    }
}

/* Turn I/Q samples pointed by Modes.data into the magnitude vector
 * pointed by Modes.magnitude. */
void computeMagnitudeVector(void) {
    uint16_t *m = ms.magnitude;
    unsigned char *p = ms.data;
    uint32_t j;

    /* Compute the magnitudo vector. It's just SQRT(I^2 + Q^2), but
     * we rescale to the 0-255 range to exploit the full resolution. */
    for (j = 0; j < ms.data_len; j += 2) {
        int i = p[j]-127;
        int q = p[j+1]-127;

        if (i < 0) i = -i;
        if (q < 0) q = -q;
        m[j/2] = ms.maglut[i*129+q];
    }

#ifdef XX
    static FILE *fdump = NULL; int i;
    if (fdump == NULL) fdump = fopen ("fdump", "w");
    for (i = 0; i < 16384; i++) { buf[i] = m[i]; }
    fwrite (buf, sizeof(float), 16384, fdump);
#endif
}

/* Detect a Mode S messages inside the magnitude buffer pointed by 'm' and of
 * size 'mlen' bytes. Every detected Mode S message is convert it into a
 * stream of bits and passed to the function to display it. */
void detectModeS(uint16_t *m, uint32_t mlen) {
    unsigned char bits[MODES_LONG_MSG_BITS];
    unsigned char msg[MODES_LONG_MSG_BITS/2];
    uint16_t aux[MODES_LONG_MSG_BITS*2];
    uint32_t j;
    int use_correction = 0;

    /* The Mode S preamble is made of impulses of 0.5 microseconds at
     * the following time offsets:
     *
     * 0   - 0.5 usec: first impulse.
     * 1.0 - 1.5 usec: second impulse.
     * 3.5 - 4   usec: third impulse.
     * 4.5 - 5   usec: last impulse.
     * 
     * Since we are sampling at 2 Mhz every sample in our magnitude vector
     * is 0.5 usec, so the preamble will look like this, assuming there is
     * an impulse at offset 0 in the array:
     *
     * 0   -----------------
     * 1   -
     * 2   ------------------
     * 3   --
     * 4   -
     * 5   --
     * 6   -
     * 7   ------------------
     * 8   --
     * 9   -------------------
     */
    for (j = 0; j < mlen - MODES_FULL_LEN*2; j++) {
        int low, high, delta, i, errors;
        int good_message = 0;

        if (use_correction) goto good_preamble; /* We already checked it. */

        /* First check of relations between the first 10 samples
         * representing a valid preamble. We don't even investigate further
         * if this simple test is not passed. */
        if (!(m[j] > m[j+1] &&
            m[j+1] < m[j+2] &&
            m[j+2] > m[j+3] &&
            m[j+3] < m[j]   &&
            m[j+4] < m[j]   &&
            m[j+5] < m[j]   &&
            m[j+6] < m[j]   &&
            m[j+7] > m[j+8] &&
            m[j+8] < m[j+9] &&
            m[j+9] > m[j+6]))
        {
            if (ms.debug & MODES_DEBUG_NOPREAMBLE &&
                m[j] > MODES_DEBUG_NOPREAMBLE_LEVEL)
                dumpRawMessage("Unexpected ratio among first 10 samples",
                    msg, m, j);
            continue;
        }

        /* The samples between the two spikes must be < than the average
         * of the high spikes level. We don't test bits too near to
         * the high levels as signals can be out of phase so part of the
         * energy can be in the near samples. */
        high = (m[j]+m[j+2]+m[j+7]+m[j+9])/6;
        if (m[j+4] >= high ||
            m[j+5] >= high)
        {
            if (ms.debug & MODES_DEBUG_NOPREAMBLE &&
                m[j] > MODES_DEBUG_NOPREAMBLE_LEVEL)
                dumpRawMessage(
                    "Too high level in samples between 3 and 6",
                    msg, m, j);
            continue;
        }

        /* Similarly samples in the range 11-14 must be low, as it is the
         * space between the preamble and real data. Again we don't test
         * bits too near to high levels, see above. */
        if (m[j+11] >= high ||
            m[j+12] >= high ||
            m[j+13] >= high ||
            m[j+14] >= high)
        {
            if (ms.debug & MODES_DEBUG_NOPREAMBLE &&
                m[j] > MODES_DEBUG_NOPREAMBLE_LEVEL)
                dumpRawMessage(
                    "Too high level in samples between 10 and 15",
                    msg, m, j);
            continue;
        }
        ms.stat_valid_preamble++;

        good_preamble:
        /* If the previous attempt with this message failed, retry using
         * magnitude correction. */
        if (use_correction) {
            memcpy(aux,m+j+MODES_PREAMBLE_US*2,sizeof(aux));
            if (j && detectOutOfPhase(m+j)) {
                applyPhaseCorrection(m+j);
                ms.stat_out_of_phase++;
            }
            /* TODO ... apply other kind of corrections. */
        }

        /* Decode all the next 112 bits, regardless of the actual message
         * size. We'll check the actual message type later. */
        errors = 0;
        for (i = 0; i < MODES_LONG_MSG_BITS*2; i += 2) {
            low = m[j+i+MODES_PREAMBLE_US*2];
            high = m[j+i+MODES_PREAMBLE_US*2+1];
            delta = low-high;
            if (delta < 0) delta = -delta;

            if (i > 0 && delta < 256) {
                bits[i/2] = bits[i/2-1];
            } else if (low == high) {
                /* Checking if two adiacent samples have the same magnitude
                 * is an effective way to detect if it's just random noise
                 * that was detected as a valid preamble. */
                bits[i/2] = 2; /* error */
                if (i < MODES_SHORT_MSG_BITS*2) errors++;
            } else if (low > high) {
                bits[i/2] = 1;
            } else {
                /* (low < high) for exclusion  */
                bits[i/2] = 0;
            }
        }

        /* Restore the original message if we used magnitude correction. */
        if (use_correction)
            memcpy(m+j+MODES_PREAMBLE_US*2,aux,sizeof(aux));

        /* Pack bits into bytes */
        for (i = 0; i < MODES_LONG_MSG_BITS; i += 8) {
            msg[i/8] =
                bits[i]<<7      |
                bits[i+1]<<6    |
                bits[i+2]<<5    |
                bits[i+3]<<4    |
                bits[i+4]<<3    |
                bits[i+5]<<2    |
                bits[i+6]<<1    |
                bits[i+7];
        }

        int msgtype = msg[0]>>3;
        int msglen = modesMessageLenByType(msgtype)/8;

        /* Last check, high and low bits are different enough in magnitude
         * to mark this as real message and not just noise? */
        delta = 0;
        for (i = 0; i < msglen*8*2; i += 2)
            delta += abs(m[j+i+MODES_PREAMBLE_US*2]- m[j+i+MODES_PREAMBLE_US*2+1]);
        delta /= msglen*4;

        /* Filter for an average delta of three is small enough to let almost
         * every kind of message to pass, but high enough to filter some
         * random noise. */
        if (delta < 10*255) {
            use_correction = 0;
            continue;
        }

        /* If we reached this point, and error is zero, we are very likely
         * with a Mode S message in our hands, but it may still be broken
         * and CRC may not be correct. This is handled by the next layer. */
        if (errors == 0 || (ms.aggressive && errors < 3)) {
            modesMessage mm;

            /* Decode the received message and update statistics */
            decodeModesMessage(&mm,msg);

            /* Update statistics. */
            if (mm.crcok || use_correction) {
                if (errors == 0) ms.stat_demodulated++;
                if (mm.errorbit == -1) {
                    if (mm.crcok)
                        ms.stat_goodcrc++;
                    else
                        ms.stat_badcrc++;
                } else {
                    ms.stat_badcrc++;
                    ms.stat_fixed++;
                    if (mm.errorbit < MODES_LONG_MSG_BITS)
                        ms.stat_single_bit_fix++;
                    else
                        ms.stat_two_bits_fix++;
                }
            }

            /* Output debug mode info if needed. */
            if (use_correction == 0) {
                if (ms.debug & MODES_DEBUG_DEMOD)
                    dumpRawMessage("Demodulated with 0 errors", msg, m, j);
                else if (ms.debug & MODES_DEBUG_BADCRC
                         && mm.msgtype == 17
                         && (!mm.crcok || mm.errorbit != -1))
                    dumpRawMessage("Decoded with bad CRC", msg, m, j);
                else if (ms.debug & MODES_DEBUG_GOODCRC
                         && mm.crcok
                         && mm.errorbit == -1)
                    dumpRawMessage("Decoded with good CRC", msg, m, j);
            }

            /* Skip this message if we are sure it's fine. */
            if (mm.crcok) {
                j += (MODES_PREAMBLE_US+(msglen*8))*2;
                good_message = 1;
                if (use_correction)
                    mm.phase_corrected = 1;
            }

            /* Pass data to the next layer */
            useModesMessage(&mm);
        } else {
            if (ms.debug & MODES_DEBUG_DEMODERR && use_correction) {
                printf("The following message has %d demod errors\n", errors);
                dumpRawMessage("Demodulated with errors", msg, m, j);
            }
        }

        /* Retry with phase correction if possible. */
        if (!good_message && !use_correction) {
            j--;
            use_correction = 1;
        } else
            use_correction = 0;
    }
}

/* ========================= Interactive mode =============================== */
/* Return a new aircraft structure for the interactive mode linked list
 * of aircrafts. */
aircraft *interactiveCreateAircraft(uint32_t addr) {
    aircraft *a = malloc(sizeof(*a));

    a->addr = addr;
    snprintf(a->hexaddr,sizeof(a->hexaddr),"%06x",(int)addr);
    a->flight[0] = '\0';
    a->altitude = 0;
    a->speed = 0;
    a->track = 0;
    a->odd_cprlat = 0;
    a->odd_cprlon = 0;
    a->odd_cprtime = 0;
    a->even_cprlat = 0;
    a->even_cprlon = 0;
    a->even_cprtime = 0;
    a->lat = 0;
    a->lon = 0;
    a->seen = time(NULL);
    a->messages = 0;
    a->next = NULL;
    return a;
}

/* Return the aircraft with the specified address, or NULL if no aircraft
 * exists with this address. */
aircraft *interactiveFindAircraft(uint32_t addr) {
    aircraft *a = ms.aircrafts;

    while(a) {
        if (a->addr == addr) return a;
        a = (aircraft *)a->next;
    }
    return NULL;
}

/* Always positive MOD operation, used for CPR decoding. */
int cprModFunction(int a, int b) {
    int res = a % b;

    if (res < 0) res += b;
    return res;
}

int cprNFunction(double lat, int isodd) {
    int nl = cprNLFunction(lat) - isodd;

    if (nl < 1) nl = 1;
    return nl;
}

double cprDlonFunction(double lat, int isodd) {
    return 360.0 / cprNFunction(lat, isodd);
}

/* Receive new messages and populate the interactive mode with more info. */
aircraft *interactiveReceiveData(modesMessage *mm) {
    uint32_t addr;
    aircraft *a, *aux;

    if (ms.check_crc && mm->crcok == 0) return NULL;
    addr = (mm->aa1 << 16) | (mm->aa2 << 8) | mm->aa3;

    /* Loookup our aircraft or create a new one. */
    a = interactiveFindAircraft(addr);
    if (!a) {
        a = interactiveCreateAircraft(addr);
        a->next = ms.aircrafts;
        ms.aircrafts = a;
    } else {
        /* If it is an already known aircraft, move it on head
         * so we keep aircrafts ordered by received message time.
         *
         * However move it on head only if at least one second elapsed
         * since the aircraft that is currently on head sent a message,
         * othewise with multiple aircrafts at the same time we have an
         * useless shuffle of positions on the screen. */
        if (0 && ms.aircrafts != a && (time(NULL) - a->seen) >= 1) {
            aux = ms.aircrafts;
            while(aux->next != a) aux = (aircraft *)aux->next;
            /* Now we are a node before the aircraft to remove. */
            aux->next = aux->next->next;    /* removed. */
            /* Add on head */
            a->next = ms.aircrafts;
            ms.aircrafts = a;
        }
    }

    a->seen = time(NULL);
    a->messages++;

    if (mm->msgtype == 0 || mm->msgtype == 4 || mm->msgtype == 20)
        a->altitude = mm->altitude;
    else if (mm->msgtype == 17) {
        if (mm->metype >= 1 && mm->metype <= 4)
            memcpy(a->flight, mm->flight, sizeof(a->flight));
        else if (mm->metype >= 9 && mm->metype <= 18) {
            a->altitude = mm->altitude;
            if (mm->fflag) {
                a->odd_cprlat = mm->raw_latitude;
                a->odd_cprlon = mm->raw_longitude;
                a->odd_cprtime = mstime();
            } else {
                a->even_cprlat = mm->raw_latitude;
                a->even_cprlon = mm->raw_longitude;
                a->even_cprtime = mstime();
            }
            /* If the two data is less than 10 seconds apart, compute
             * the position. */
            if (abs(a->even_cprtime - a->odd_cprtime) <= 10000)
                decodeCPR(a);
        } else if (mm->metype == 19) {
            if (mm->mesub == 1 || mm->mesub == 2) {
                a->speed = mm->velocity;
                a->track = mm->heading;
            }
        }
    }
    return a;
}

/* When in interactive mode If we don't receive new nessages within
 * MODES_INTERACTIVE_TTL seconds we remove the aircraft from the list. */
void interactiveRemoveStaleAircrafts(void) {
    aircraft *a = ms.aircrafts;
    aircraft *prev = NULL;
    time_t now = time(NULL);

    while(a) {
        if ((now - a->seen) > ms.interactive_ttl) {
            aircraft *next = (aircraft *)a->next;
            /* Remove the element from the linked list, with care
             * if we are removing the first element. */
            free(a);
            if (!prev)
                ms.aircrafts = next;
            else
                prev->next = next;
            a = next;
        } else {
            prev = a;
            a = (aircraft *)a->next;
        }
    }
}

/* ============================== Snip mode ================================= */

/* Get raw IQ samples and filter everything is < than the specified level
 * for more than 256 samples in order to reduce example file size. */
void snipMode(int level) {
    int i, q;
    long long c = 0;

    while ((i = getchar()) != EOF && (q = getchar()) != EOF) {
        if (abs(i-127) < level && abs(q-127) < level) {
            c++;
            if (c > MODES_PREAMBLE_US*4)
                continue;
        } else
            c = 0;

        putchar(i);
        putchar(q);
    }
}

/* ================================ Main ==================================== */

void showHelp(void) {
    printf(
"code by </MATRIX>@Neod Anderjon\n"
"--device-index <index>   Select RTL device (default: 0).\n"
"--dev-rtl                use RTLSDR device.\n"
"--dev-hackrf             use HackRF device.\n"
"--dev-airspy             use AirSpy device.\n"
"--dev-sdrplay            use RSP device.\n"
"--gain <db>              Set RTLSDR gain (default: max gain. Use -100 for auto-gain).\n"
"--enable-agc             Enable RTLSDR Automatic Gain Control (default: off).\n"
"--enable-amp             Enable HackRF RX/TX RF amplifier (default: off).\n"
"--rf-gain                Set RX AMP (RF) gain\n"
"                         HackRF 0 or 14, default 0\n"
"                         AirSpy 0-14, step 1, default 11\n"
"--lna-gain               Set RX LNA (IF) gain\n"
"                         HackRF 0-40, step 8, default: 40\n"
"                         AirSpy 0-14, step 1, default: 11\n"
"--vga-gain               Set RX VGA (baseband) gain\n"
"                         HackRF 0-62, step 2, default: 62\n"
"                         AirSpy 0-14, step 1, default: 11\n"
"--freq <hz>              Set frequency (default: 1090 Mhz).\n"
"--ifile <filename>       Read data from file (use '-' for stdin).\n"
"--interactive(--irv)     Interactive mode refreshing data on screen.\n"
"--interactive-rows <num> Max number of rows in interactive mode (default: 15).\n"
"--interactive-ttl <sec>  Remove from list if idle for <sec> (default: 60).\n"
"--raw                    Show only messages hex values.\n"
"--net                    Enable networking.\n"
"--net-only               Enable just networking, no RTL device or file used.\n"
"--net-ro-port <port>     TCP listening port for raw output (default: 30000).\n"
"--net-ri-port <port>     TCP listening port for raw input (default: 40000).\n"
"--net-http-port <port>   HTTP server port (default: 8081).\n"
"--net-sbs-port <port>    TCP listening port for BaseStation format output (default: 50000).\n"
"--ian                    use --interactive and --net two option.\n"
"--no-fix                 Disable single-bits error correction using CRC.\n"
"--no-crc-check           Disable messages with broken CRC (discouraged).\n"
"--aggressive             More CPU for more messages (two bits fixes, ...).\n"
"--stats                  With --ifile print stats at exit. No other output.\n"
"--onlyaddr               Show only ICAO addresses (testing purposes).\n"
"--metric                 Use metric units (meters, km/h, ...).\n"
"--snip <level>           Strip IQ file removing samples < level.\n"
"--debug <flags>          Debug mode (verbose), see README for details.\n"
"--help(-h)               Show this help page.\n"
"\n"
"Debug mode flags: d = Log frames decoded with errors\n"
"                  D = Log frames decoded with zero errors\n"
"                  c = Log frames with bad CRC\n"
"                  C = Log frames with good CRC\n"
"                  p = Log frames with bad preamble\n"
"                  n = Log network debugging info\n"
"                  j = Log frames to frames.js, loadable by debug.html.\n"
    );
}

/* This function is called a few times every second by main in order to
 * perform tasks we need to do continuously, like accepting new clients
 * from the net, refreshing the screen in interactive mode, and so forth.
 * */
void backgroundTasks(void) {
    if (ms.net) {
        modesAcceptClients();
        modesReadFromClients();
        interactiveRemoveStaleAircrafts();
    }

    /* Refresh screen when in interactive mode. */
    if (ms.interactive
        && mstime() - ms.interactive_last_update > MODES_INTERACTIVE_REFRESH_TIME) {
        interactiveRemoveStaleAircrafts();
        interactiveShowData();
        ms.interactive_last_update = mstime();
    }
}

int main(int argc, char **argv) {
    printf(
    "code by </MATRIX>@Neod Anderjon.\n"
    "dump1090-sdrplus, more devices support base on dump1090 with hackrf etc.\n"
    "visit github page [Neod0Matrix]https://github.com/Neod0Matrix to fork."
    "test devices support by T.WKVER.\n"
    );

    //device init config
    modesInitConfig();

    /* Parse the command line options */
    for (uint16_t j = 1; j < argc; j++) {
        /* There are more arguments. */
        int more = j+1 < argc;

        if (!strcmp(argv[j],"--device-index") && more) {
            ms.dev_index = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--dev-sdrplay")) {
            ms.prefer_sdrplay = 1;
        } else if (!strcmp(argv[j],"--dev-airspy")) {
            ms.rf_gain = AIRSPY_RF_GAIN;
            ms.lna_gain = AIRSPY_LNA_GAIN;
            ms.vga_gain = AIRSPY_VGA_GAIN;
            ms.prefer_airspy = 1;
        } else if (!strcmp(argv[j],"--dev-hackrf")) {
            ms.rf_gain = HACKRF_RF_GAIN;
            ms.lna_gain = HACKRF_LNA_GAIN;
            ms.vga_gain = HACKRF_VGA_GAIN;
            ms.prefer_hackrf = 1;
        } else if (!strcmp(argv[j],"--dev-rtlsdr")) {
            ms.prefer_rtlsdr = 1;
        } else if (!strcmp(argv[j],"--gain") && more) {
            ms.gain = atof(argv[++j])*10; /* Gain is in tens of DBs */
        } else if (!strcmp(argv[j],"--enable-agc")) {
            ms.enable_agc++;
        } else if (!strcmp(argv[j],"--enable-amp")) {
            ms.rf_gain = 14;
        } else if (!strcmp(argv[j],"--rf-gain")) {
            ms.rf_gain =  atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--lna-gain")) {
            ms.lna_gain =  atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--vga-gain")) {
            ms.vga_gain =  atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--freq") && more) {
            ms.freq = strtoll(argv[++j],NULL,10);
        } else if (!strcmp(argv[j],"--ifile") && more) {
            ms.filename = strdup(argv[++j]);
        } else if (!strcmp(argv[j],"--no-fix")) {
            ms.fix_errors = 0;
        } else if (!strcmp(argv[j],"--no-crc-check")) {
            ms.check_crc = 0;
        } else if (!strcmp(argv[j],"--raw")) {
            ms.raw = 1;
        } else if (!strcmp(argv[j],"--net")) {
            ms.net = 1;
        } else if (!strcmp(argv[j],"--net-only")) {
            ms.net = 1;
            ms.net_only = 1;
        } else if (!strcmp(argv[j],"--net-ro-port") && more) {
            modesNetServer[MODES_NET_SERVICE_RAWO].port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-ri-port") && more) {
            modesNetServer[MODES_NET_SERVICE_RAWI].port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-http-port") && more) {
            modesNetServer[MODES_NET_SERVICE_HTTP].port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--net-sbs-port") && more) {
            modesNetServer[MODES_NET_SERVICE_SBS].port = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--onlyaddr")) {
            ms.onlyaddr = 1;
        } else if (!strcmp(argv[j],"--metric")) {
            ms.metric = 1;
        } else if (!strcmp(argv[j],"--aggressive")) {
            ms.aggressive++;
        } else if (!strcmp(argv[j],"--interactive")||!strcmp(argv[j],"--irv")) {
            ms.interactive = 1;
        } else if (!strcmp(argv[j],"--ian")) {
            //union option
            ms.interactive = 1;
            ms.net = 1;
        } else if (!strcmp(argv[j],"--interactive-rows")) {
            ms.interactive_rows = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--interactive-ttl")) {
            ms.interactive_ttl = atoi(argv[++j]);
        } else if (!strcmp(argv[j],"--debug") && more) {
            char *f = argv[++j];
            while(*f) {
                switch(*f) {
                case 'D': ms.debug |= MODES_DEBUG_DEMOD; break;
                case 'd': ms.debug |= MODES_DEBUG_DEMODERR; break;
                case 'C': ms.debug |= MODES_DEBUG_GOODCRC; break;
                case 'c': ms.debug |= MODES_DEBUG_BADCRC; break;
                case 'p': ms.debug |= MODES_DEBUG_NOPREAMBLE; break;
                case 'n': ms.debug |= MODES_DEBUG_NET; break;
                case 'j': ms.debug |= MODES_DEBUG_JS; break;
                default:
                    fprintf(stderr, "Unknown debugging flag: %c\n", *f);
                    exit(1);
                    break;
                }
                f++;
            }
        } else if (!strcmp(argv[j],"--stats")) {
            ms.stats = 1;
        } else if (!strcmp(argv[j],"--snip") && more) {
            snipMode(atoi(argv[++j]));
            exit(0);
        } else if (!strcmp(argv[j],"--help")||!strcmp(argv[j],"-h")) {
            showHelp();
            exit(0);
        } else {
            fprintf(stderr, "Unknown or not enough arguments for option '%s'.\n\n", argv[j]);
            showHelp();
            exit(1);
        }
    }

    if ((ms.prefer_airspy + ms.prefer_hackrf + ms.prefer_rtlsdr + ms.prefer_sdrplay) > 1) {
        showHelp();
        fprintf(stderr, "\n\n[Error] dev{sdrplay, airspy, hackrf, rtlsdr} are mutually exclusive.\n");
        exit(1);
    }

    /* Setup for SIGWINCH for handling lines */
    if (ms.interactive == 1)
        signal(SIGWINCH, (__sighandler_t)sigWinchCallback);//signal return

    /* Initialization */
    modesInit();

    //try to open all response devices
    if (ms.net_only) {
        fprintf(stderr,"Net-only mode, no RTL device or file open.\n");
    } else if (ms.filename == NULL) {
        if ((ms.prefer_airspy + ms.prefer_hackrf + ms.prefer_sdrplay) == 0 && modesInitRTLSDR() == 0)
            ms.rtl_enabled = 1;
        else if ((ms.prefer_hackrf + ms.prefer_airspy + ms.prefer_rtlsdr ) == 0 &&  modesInitSDRplay() == 0 )
            ms.sdrplay_enabled =  1;
        else if ((ms.prefer_sdrplay + ms.prefer_airspy + ms.prefer_rtlsdr ) == 0 &&  modesInitHackRF() == 0 )
            ms.hackrf_enabled =  1;
        else if ((ms.prefer_sdrplay + ms.prefer_rtlsdr + ms.prefer_hackrf ) == 0 &&  modesInitAirSpy() == 0 )
            ms.airspy_enabled =  1;
        else {
            fprintf(stderr,"No compatible SDR device found.\n");
            exit (1);
        }
    } else {
        if (ms.filename[0] == '-' && ms.filename[1] == '\0')
            ms.fd = STDIN_FILENO;
        else if ((ms.fd = open(ms.filename,O_RDONLY)) == -1) {
            perror("Opening data file");
            exit(1);
        }
    }
    if (ms.net == 1) modesInitNet();

    /* If the user specifies --net-only, just run in order to serve network
     * clients without reading data from the RTL device. */
    while (ms.net_only) {
        backgroundTasks();
        modesWaitReadableClients(100);
    }

    /* Create the thread that will read the data from the device. */
    pthread_create(&ms.reader_thread, NULL, readerThreadEntryPoint, NULL);
    pthread_mutex_lock(&ms.data_mutex);

    while(1) {
        if (!ms.data_ready) {
            pthread_cond_wait(&ms.data_cond,&ms.data_mutex);
            continue;
        }
        //computeMagnitudeVector();

        /* Signal to the other thread that we processed the available data
         * and we want more (useful for --ifile). */
        ms.data_ready = 0;
        pthread_cond_signal(&ms.data_cond);

        /* Process data after releasing the lock, so that the capturing
         * thread can read data while we perform computationally expensive
         * stuff * at the same time. (This should only be useful with very
         * slow processors). */
        pthread_mutex_unlock(&ms.data_mutex);
        detectModeS(ms.magnitude, ms.data_len/2);
        backgroundTasks();
        pthread_mutex_lock(&ms.data_mutex);
        if (ms.exit)
            break;
    }

    /* If --ifile and --stats were given, print statistics. */
    if (ms.stats && ms.filename) {
        printf("%lld valid preambles\n", ms.stat_valid_preamble);
        printf("%lld demodulated again after phase correction\n", ms.stat_out_of_phase);
        printf("%lld demodulated with zero errors\n", ms.stat_demodulated);
        printf("%lld with good crc\n", ms.stat_goodcrc);
        printf("%lld with bad crc\n", ms.stat_badcrc);
        printf("%lld errors corrected\n", ms.stat_fixed);
        printf("%lld single bit errors\n", ms.stat_single_bit_fix);
        printf("%lld two bits errors\n", ms.stat_two_bits_fix);
        printf("%lld total usable messages\n", ms.stat_goodcrc + ms.stat_fixed);
    }
    rtlsdr_close(ms.dev);
    hackrf_close(ms.hackrf);

    return 0;
}

//===============================================================================
//code by </MATRIX>@Neod Anderjon
