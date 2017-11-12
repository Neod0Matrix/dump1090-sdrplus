#pragma once
//sys header
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/un.h>
//net lib
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <netdb.h>
//standard lib
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
//process handler lib
#include <unistd.h>
#include <pthread.h>
#include <err.h>
#include <errno.h>
#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <fcntl.h>
#include <ctype.h>
//sdr lib header(/usr/include or /usr/local/include)
#include "rtl-sdr.h"
#include "libhackrf/hackrf.h"
#include "libairspy/airspy.h"
#include "mirsdrapi-rsp.h"
#include "soxr.h"
//local header(array compile cannot change)
#include "netpro.h"
#include "dump1090.h"
#include "devices.h"
#include "data.h"
//code by </MATRIX>@Neod Anderjon
//===============================================================================
/* Mode1090, a Mode S messages decoder for RTLSDR devices.
 *
 * Copyright (C) 2012 by Salvatore Sanfilippo <antirez@gmail.com>
 *
 * HackRF One support added by Ilker Temir <ilker@ilkertemir.com>
 * AirSpy support added by Chris Kuethe <chris.kuethe+github@gmail.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  *  Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  *  Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Update and refacte by </MATRIX>@Neod Anderjon.
 */

#define __Organization__    "</MATRIX>"
#define __Laboratory__      "T.WKVER"
#define __ProjectType__     "SDR"
#define __Author__          "Neod Anderjon"
#define __Version__         "v0p3_develop"

//===============================================================================
//code by </MATRIX>@Neod Anderjon
