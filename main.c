/*********************************************************************
*                                                                    *
*                   Copyright (C) 2022 SWS Inc.                      *
*                                                                    *
*                      All rights reserved                           *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer for LAUTERBACH TRACE32       *
*                                                                    *
**********************************************************************
*/

/*********************************************************************
*
*       Include Section
*
**********************************************************************
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>

#ifdef __linux__
#include <getopt.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#include <time.h>
#include <winsock2.h>
#include "getopt.h"
#endif

#include "t32.h"

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/

#define BEL    0x07
#define BS     0x08
#define TAB    0x09
#define LF     0x0A
#define VT     0x0B
#define FF     0x0C
#define CR     0x0D
#define SO     0x0E
#define SI     0x0F
#define CAN    0x18
#define SUB    0x1A
#define ESC    0x1B
#define SS2    0x8E
#define SS3    0x8F
#define DCS    0x90
#define CSI    0x9B
#define ST     0x9C
#define OSC    0x9D
#define PM     0x9E
#define APC    0x9F

//
// RTT CB offset calc
//
#define RTTCB_SIZEOF_BASE                                                (0x00)
#define RTTCB_SIZEOF_ACID                                                (0x10)
#define RTTCB_SIZEOF_MAXNUMUPBUFFERS                                     (0x04)
#define RTTCB_SIZEOF_MAXNUMDOWNBUFFERS                                   (0x04)
#define RTTCB_SIZEOF_AUP                                                 (0x18)
#define RTTCB_SIZEOF_ADOWN                                               (0x18)

#define RTTCB_SIZEOF_SNAME                                               (0x04)
#define RTTCB_SIZEOF_PBUFFER                                             (0x04)
#define RTTCB_SIZEOF_SIZEOFBUFFER                                        (0x04)
#define RTTCB_SIZEOF_WROFF                                               (0x04)
#define RTTCB_SIZEOF_RDOFF                                               (0x04)
#define RTTCB_SIZEOF_FLAGS                                               (0x04)

#define RTTCB_SIZEOF_AUP_INDEX(                             bufindex)    (bufindex * RTTCB_SIZEOF_AUP)
#define RTTCB_SIZEOF_AUP_MAXNUM(                  maxupnum)              (maxupnum * RTTCB_SIZEOF_AUP)
#define RTTCB_SIZEOF_ADOWN_INDEX(                           bufindex)    (bufindex * RTTCB_SIZEOF_ADOWN)

#define RTTCB_OFFSET_ACID(               address)                        (address                                   + RTTCB_SIZEOF_BASE)
#define RTTCB_OFFSET_MAXNUMUPBUFFERS(    address)                        (RTTCB_OFFSET_ACID(               address) + RTTCB_SIZEOF_ACID)
#define RTTCB_OFFSET_MAXNUMDOWNBUFFERS(  address)                        (RTTCB_OFFSET_MAXNUMUPBUFFERS(    address) + RTTCB_SIZEOF_MAXNUMUPBUFFERS)
#define RTTCB_OFFSET_AUP(                address)                        (RTTCB_OFFSET_MAXNUMDOWNBUFFERS(  address) + RTTCB_SIZEOF_MAXNUMDOWNBUFFERS)
#define RTTCB_OFFSET_ADOWN(              address, maxupnum)              (RTTCB_OFFSET_AUP(                address) + RTTCB_SIZEOF_AUP_MAXNUM(maxupnum))

#define RTTCB_OFFSET_AUP_INDEX(          address          , bufindex)    (RTTCB_OFFSET_AUP(                address)                     + (RTTCB_SIZEOF_AUP_INDEX(   bufindex)))
#define RTTCB_OFFSET_ADOWN_INDEX(        address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN(              address, maxupnum)           + (RTTCB_SIZEOF_ADOWN_INDEX( bufindex)))

#define RTTCB_OFFSET_AUP_SNAME(          address          , bufindex)    (RTTCB_OFFSET_AUP_INDEX(          address,           bufindex) + RTTCB_SIZEOF_BASE   )
#define RTTCB_OFFSET_AUP_PBUFFER(        address          , bufindex)    (RTTCB_OFFSET_AUP_SNAME(          address,           bufindex) + RTTCB_SIZEOF_SNAME)
#define RTTCB_OFFSET_AUP_SIZEOFBUFFER(   address          , bufindex)    (RTTCB_OFFSET_AUP_PBUFFER(        address,           bufindex) + RTTCB_SIZEOF_PBUFFER)
#define RTTCB_OFFSET_AUP_WROFF(          address          , bufindex)    (RTTCB_OFFSET_AUP_SIZEOFBUFFER(   address,           bufindex) + RTTCB_SIZEOF_SIZEOFBUFFER)
#define RTTCB_OFFSET_AUP_RDOFF(          address          , bufindex)    (RTTCB_OFFSET_AUP_WROFF(          address,           bufindex) + RTTCB_SIZEOF_WROFF)
#define RTTCB_OFFSET_AUP_FLAGS(          address          , bufindex)    (RTTCB_OFFSET_AUP_RDOFF(          address,           bufindex) + RTTCB_SIZEOF_RDOFF)

#define RTTCB_OFFSET_ADOWN_SNAME(        address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_INDEX(        address, maxupnum, bufindex) + RTTCB_SIZEOF_BASE)
#define RTTCB_OFFSET_ADOWN_PBUFFER(      address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_SNAME(        address, maxupnum, bufindex) + RTTCB_SIZEOF_SNAME)
#define RTTCB_OFFSET_ADOWN_SIZEOFBUFFER( address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_PBUFFER(      address, maxupnum, bufindex) + RTTCB_SIZEOF_PBUFFER)
#define RTTCB_OFFSET_ADOWN_WROFF(        address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_SIZEOFBUFFER( address, maxupnum, bufindex) + RTTCB_SIZEOF_SIZEOFBUFFER)
#define RTTCB_OFFSET_ADOWN_RDOFF(        address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_WROFF(        address, maxupnum, bufindex) + RTTCB_SIZEOF_WROFF)
#define RTTCB_OFFSET_ADOWN_FLAGS(        address, maxupnum, bufindex)    (RTTCB_OFFSET_ADOWN_RDOFF(        address, maxupnum, bufindex) + RTTCB_SIZEOF_RDOFF)

//
// Ring Buffer offset calc
//
#define RTTBUFFER_SIZEOF_BASE                                            (0x00)
#define RTTBUFFER_SIZEOF_SNAME                                           (0x04)
#define RTTBUFFER_SIZEOF_PBUFFER                                         (0x04)
#define RTTBUFFER_SIZEOF_SIZEOFBUFFER                                    (0x04)
#define RTTBUFFER_SIZEOF_WROFF                                           (0x04)
#define RTTBUFFER_SIZEOF_RDOFF                                           (0x04)
#define RTTBUFFER_SIZEOF_FLAGS                                           (0x04)

#define RTTBUFFER_OFFSET_SNAME(              address)                    (                               address    + RTTBUFFER_SIZEOF_BASE)
#define RTTBUFFER_OFFSET_PBUFFER(            address)                    (RTTBUFFER_OFFSET_SNAME(        address)   + RTTBUFFER_SIZEOF_SNAME)
#define RTTBUFFER_OFFSET_SIZEOFBUFFER(       address)                    (RTTBUFFER_OFFSET_PBUFFER(      address)   + RTTBUFFER_SIZEOF_PBUFFER)
#define RTTBUFFER_OFFSET_WROFF(              address)                    (RTTBUFFER_OFFSET_SIZEOFBUFFER( address)   + RTTBUFFER_SIZEOF_SIZEOFBUFFER)
#define RTTBUFFER_OFFSET_RDOFF(              address)                    (RTTBUFFER_OFFSET_WROFF(        address)   + RTTBUFFER_SIZEOF_WROFF)
#define RTTBUFFER_OFFSET_FLAGS(              address)                    (RTTBUFFER_OFFSET_RDOFF(        address)   + RTTBUFFER_SIZEOF_RDOFF)

#define RTTBUFFER_SIZEOF_AUP                                             (0x18)
#define RTTBUFFER_SIZEOF_AUP_INDEX(                   bufindex)          (bufindex * RTTBUFFER_SIZEOF_AUP)
#define RTTBUFFER_OFFSET_AUP_INDEX(          address, bufindex)          (address                                                 + (RTTBUFFER_SIZEOF_AUP_INDEX(bufindex)))

#define RTTBUFFER_OFFSET_AUP_SNAME(          address, bufindex)          (RTTBUFFER_OFFSET_AUP_INDEX(        address, bufindex)   + RTTBUFFER_SIZEOF_BASE)
#define RTTBUFFER_OFFSET_AUP_PBUFFER(        address, bufindex)          (RTTBUFFER_OFFSET_AUP_SNAME(        address, bufindex)   + RTTBUFFER_SIZEOF_SNAME)
#define RTTBUFFER_OFFSET_AUP_SIZEOFBUFFER(   address, bufindex)          (RTTBUFFER_OFFSET_AUP_PBUFFER(      address, bufindex)   + RTTBUFFER_SIZEOF_PBUFFER)
#define RTTBUFFER_OFFSET_AUP_WROFF(          address, bufindex)          (RTTBUFFER_OFFSET_AUP_SIZEOFBUFFER( address, bufindex)   + RTTBUFFER_SIZEOF_SIZEOFBUFFER)
#define RTTBUFFER_OFFSET_AUP_RDOFF(          address, bufindex)          (RTTBUFFER_OFFSET_AUP_WROFF(        address, bufindex)   + RTTBUFFER_SIZEOF_WROFF)
#define RTTBUFFER_OFFSET_AUP_FLAGS(          address, bufindex)          (RTTBUFFER_OFFSET_AUP_RDOFF(        address, bufindex)   + RTTBUFFER_SIZEOF_RDOFF)

#define RTTBUFFER_SIZEOF_ADOWN                                           (0x18)
#define RTTBUFFER_SIZEOF_ADOWN_INDEX(                 bufindex)          (bufindex * RTTBUFFER_SIZEOF_ADOWN)
#define RTTBUFFER_OFFSET_ADOWN_INDEX(        address, bufindex)          (address                                                 + (RTTBUFFER_SIZEOF_ADOWN_INDEX(bufindex)))

#define RTTBUFFER_OFFSET_ADOWN_SNAME(        address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_INDEX(        address, bufindex) + RTTBUFFER_SIZEOF_BASE)
#define RTTBUFFER_OFFSET_ADOWN_PBUFFER(      address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_SNAME(        address, bufindex) + RTTBUFFER_SIZEOF_SNAME)
#define RTTBUFFER_OFFSET_ADOWN_SIZEOFBUFFER( address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_PBUFFER(      address, bufindex) + RTTBUFFER_SIZEOF_PBUFFER)
#define RTTBUFFER_OFFSET_ADOWN_WROFF(        address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_SIZEOFBUFFER( address, bufindex) + RTTBUFFER_SIZEOF_SIZEOFBUFFER)
#define RTTBUFFER_OFFSET_ADOWN_RDOFF(        address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_WROFF(        address, bufindex) + RTTBUFFER_SIZEOF_WROFF)
#define RTTBUFFER_OFFSET_ADOWN_FLAGS(        address, bufindex)          (RTTBUFFER_OFFSET_ADOWN_RDOFF(        address, bufindex) + RTTBUFFER_SIZEOF_RDOFF)

//
// Operating modes. Define behavior if buffer is full (not enough space for entire message)
//
#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0)     // Skip. Do not block, output nothing. (Default)
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1)     // Trim: Do not block, output as much as fits.
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2)     // Block: Wait until there is space in the buffer.
#define SEGGER_RTT_MODE_MASK                  (3)

#define _SYS_SOCKET_INVALID_HANDLE            (-1)
#define _SYS_SOCKET_IP_ADDR_ANY               (0)
#define _SYS_SOCKET_IP_ADDR_LOCALHOST         (0x7F000001)                  // 127.0.0.1 (localhost)
#define _SYS_SOCKET_PORT_ANY                  (0)

#define _SYS_SOCKET_ERR_UNSPECIFIED           (-1)
#define _SYS_SOCKET_ERR_WOULDBLOCK            (-2)
#define _SYS_SOCKET_ERR_TIMEDOUT              (-3)
#define _SYS_SOCKET_ERR_CONNRESET             (-4)
#define SYS_SOCKET_ERR_INTERRUPT              (-5)

#define _SYS_SOCKET_SHUT_RD                   (0)
#define _SYS_SOCKET_SHUT_WR                   (1)
#define _SYS_SOCKET_SHUT_RDWR                 (2)

#define SOCKET_ERROR                          (-1)    // General socket error returned from send(), sendto(), ...
#ifdef __linux__
#define INVALID_SOCKET                        (-1)
#endif

#define  HEX_BYTES_PER_LINE   16


/*********************************************************************
*
*       defines, configurable
*
**********************************************************************
*/

//#define _TELNET_RTT_DEBUG

/*********************************************************************
*
*       RTT_IDLE_DELAY
*  Maximum delay after which available data from the SystemView RTT
*  Buffer is sent to SystemView App.
*
*/
#ifndef   RTT_IDLE_DELAY
  #define RTT_IDLE_DELAY       20
#endif

/*********************************************************************
*
*       RTT_COMM_SEND_THRESHOLD
*  Threshold for the SystemView RTT Buffer level,
*  after which the data is sent to SystemView App.
*
*/
#ifndef   RTT_SEND_THRESHOLD
  #define RTT_SEND_THRESHOLD   512
#endif

/*********************************************************************
*
*       RTT_COMM_POLL_INTERVAL
*  Polling interval to check for send threshold or idle delay timeout.
*
*/
#ifndef   RTT_COMM_POLL_INTERVAL
  #define RTT_COMM_POLL_INTERVAL    2
#endif

/*********************************************************************
*
*       Function-like macros
*
**********************************************************************
*/

#define COUNTOF(a)            (sizeof((a))/sizeof((a)[0]))
#define MIN(a,b)              (((a) < (b)) ? (a) : (b))
#define MAX(a,b)              (((a) > (b)) ? (a) : (b))
#define ERR_TO_STR(e)         (x==(e)) ? #e  //lint !e9024 !e773

#define USE_PARA(Para)        (void)Para                           // Some compiler complain about unused parameters. This works for most compilers.
#define ADDR2PTR(Type, Addr)  (((Type*)((unsigned int)(Addr))))    // Allow cast from address to pointer.
#define PTR2ADDR(p)           (((unsigned int)(p)))                // Allow cast from pointer to address.
#define PTR2PTR(Type, p)      (((Type*)(p)))                       // Allow cast from one pointer type to another (ignore different size).
#define PTR_DISTANCE(p0, p1)  (PTR2ADDR(p0) - PTR2ADDR(p1))

#ifdef _TELNET_RTT_DEBUG
    #define Log_Print(...)  SYS_Log("%s:%d, %s(): ", __FILE__, __LINE__, __FUNCTION__); SYS_Log(__VA_ARGS__)
#else
    #define Log_Print(...)
#endif

/*********************************************************************
*
*       Types
*
**********************************************************************
*/

typedef int _SYS_SOCKET_HANDLE;

typedef enum _VT_STATE_T {
  Normal,
  Esc,
  Csi,
  Dcs,
  DcsString,
  DropOne
} VT_STATE_T;

/*********************************************************************
*
*       static data
*
**********************************************************************
*/

static int            _int1   = 1;
static const char hexchar[] = "0123456789ABCDEF";
static       char telnetCmd[] = {0xff, 0xfb, 0x01, 0xff, 0xfb, 0x03, 0xff, 0xfc, 0x1f};

/*********************************************************************
*
*       Static const data
*
**********************************************************************
*/

/*********************************************************************
*
*       Global data
*
**********************************************************************
*/

/*********************************************************************
*
*       Static functions
*
**********************************************************************
*/
static void T32_DefaultState(int exit);

/*********************************************************************
*
*       util log functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SYS_Log()
*
*  Function description
*    Outputs a formatted log message.
*
*  Parameters
*    sFormat : String to output that might contain placeholders.
*/
void SYS_Log(const char* sFormat, ...) {
  va_list         ParamList;
  char            ac[256];
  char            acTime[40];
  char            acFileName[64];
  static FILE    *pFile = NULL;
  struct tm      *tm;
  struct timeval  tv;

  //
  // Replace placeholders (%d, %x, etc.) by values and call output routine.
  //
  va_start(ParamList, sFormat);
  (void)vsnprintf(ac, (int)sizeof(ac), sFormat, ParamList);
  va_end(ParamList);

  if (pFile == NULL) {
    gettimeofday(&tv, NULL);
    tm = localtime(&tv.tv_sec);
    strftime(acTime, sizeof(acTime), "%Y-%m-%dT%H:%M:%S", tm);
    sprintf(acFileName, "main_%s.log", acTime);

    //
    // call output routine.
    //
    pFile = fopen(acFileName, "a+");
  }

  if (pFile != NULL) {
    fprintf(pFile,  "%s", ac);
    fflush(pFile);
  }
  fprintf(stderr, "%s", ac);
  fflush(stderr);
}

/*********************************************************************
*
*       system task functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SYS_Sleep()
*
*  Function description
*    Yield the execution of current thread for msec miliseconds.
*/
#ifdef __linux__
int32_t SYS_Sleep(uint32_t msec) {
  if (msec) {
    struct timeval timeout;
    timeout.tv_sec  = msec / 1000;
    timeout.tv_usec = (msec % 1000) * 1000;
    select(0, NULL, NULL, NULL, &timeout); // won't get interrupted by signals if no FD_SETs are given (says libc's manual...)
  } else {
    sched_yield();
  }
  return 0;
}
#endif
#ifdef _WIN32
int32_t SYS_Sleep(uint32_t msec) {
    Sleep(msec);
    return 0;
}
#endif

/*********************************************************************
*
*       system signal functions
*
**********************************************************************
*/

/*********************************************************************
*
*       T32_Err2Str()
*
*  Function description
*    Converts trace2 error code to a readable string by simply using
*    the defines name.
*
*  Parameters
*    x: Error code returned by API of the stack.
*
*  Return value
*    Pointer to string of the define name.
*/
const char* T32_Err2Str(int x) {
  return (
    ERR_TO_STR(T32_OK)                                  :
    ERR_TO_STR(T32_ERR_COM_RECEIVE_FAIL)                :
    ERR_TO_STR(T32_ERR_COM_TRANSMIT_FAIL)               :
    ERR_TO_STR(T32_ERR_COM_PARA_FAIL)                   :
    ERR_TO_STR(T32_ERR_COM_SEQ_FAIL)                    :
    ERR_TO_STR(T32_ERR_NOTIFY_MAX_EVENT)                :
    ERR_TO_STR(T32_ERR_MALLOC_FAIL)                     :
    ERR_TO_STR(T32_ERR_STD_RUNNING)                     :
    ERR_TO_STR(T32_ERR_STD_NOTRUNNING)                  :
    ERR_TO_STR(T32_ERR_STD_RESET)                       :
    ERR_TO_STR(T32_ERR_STD_ACCESSTIMEOUT)               :
    ERR_TO_STR(T32_ERR_STD_INVALID)                     :
    ERR_TO_STR(T32_ERR_STD_REGUNDEF)                    :
    ERR_TO_STR(T32_ERR_STD_VERIFY)                      :
    ERR_TO_STR(T32_ERR_STD_BUSERROR)                    :
    ERR_TO_STR(T32_ERR_STD_NOMEM)                       :
    ERR_TO_STR(T32_ERR_STD_RESETDETECTED)               :
    ERR_TO_STR(T32_ERR_STD_FDXBUFFER)                   :
    ERR_TO_STR(T32_ERR_STD_RTCKTIMEOUT)                 :
    ERR_TO_STR(T32_ERR_STD_INVALIDLICENSE)              :
    ERR_TO_STR(T32_ERR_STD_CORENOTACTIVE)               :
    ERR_TO_STR(T32_ERR_STD_USERSIGNAL)                  :
    ERR_TO_STR(T32_ERR_STD_NORAPI)                      :
    ERR_TO_STR(T32_ERR_STD_FAILED)                      :
    ERR_TO_STR(T32_ERR_STD_LOCKED)                      :
    ERR_TO_STR(T32_ERR_STD_POWERFAIL)                   :
    ERR_TO_STR(T32_ERR_STD_DEBUGPORTFAIL)               :
    ERR_TO_STR(T32_ERR_STD_DEBUGPORTTIMEOUT)            :
    ERR_TO_STR(T32_ERR_STD_NODEVICE)                    :
    ERR_TO_STR(T32_ERR_STD_RESETFAIL)                   :
    ERR_TO_STR(T32_ERR_STD_EMUTIMEOUT)                  :
    ERR_TO_STR(T32_ERR_STD_NORTCK)                      :
    ERR_TO_STR(T32_ERR_STD_ATTACH)                      :
    ERR_TO_STR(T32_ERR_STD_FATAL)                       :
    ERR_TO_STR(T32_ERR_GETRAM_INTERNAL)                 :
    ERR_TO_STR(T32_ERR_READREGBYNAME_NOTFOUND)          :
    ERR_TO_STR(T32_ERR_READREGBYNAME_FAILED)            :
    ERR_TO_STR(T32_ERR_WRITEREGBYNAME_NOTFOUND)         :
    ERR_TO_STR(T32_ERR_WRITEREGBYNAME_FAILED)           :
    ERR_TO_STR(T32_ERR_READREGOBJ_PARAFAIL)             :
    ERR_TO_STR(T32_ERR_READREGOBJ_MAXCORE)              :
    ERR_TO_STR(T32_ERR_READREGOBJ_NOTFOUND)             :
    ERR_TO_STR(T32_ERR_READREGSETOBJ_PARAFAIL)          :
    ERR_TO_STR(T32_ERR_READREGSETOBJ_NUMREGS)           :
    ERR_TO_STR(T32_ERR_WRITEREGOBJ_PARAFAIL)            :
    ERR_TO_STR(T32_ERR_WRITEREGOBJ_MAXCORE)             :
    ERR_TO_STR(T32_ERR_WRITEREGOBJ_NOTFOUND)            :
    ERR_TO_STR(T32_ERR_WRITEREGOBJ_FAILED)              :
    ERR_TO_STR(T32_ERR_SETBP_FAILED)                    :
    ERR_TO_STR(T32_ERR_READMEMOBJ_PARAFAIL)             :
    ERR_TO_STR(T32_ERR_WRITEMEMOBJ_PARAFAIL)            :
    ERR_TO_STR(T32_ERR_TRANSFERMEMOBJ_PARAFAIL)         :
    ERR_TO_STR(T32_ERR_TRANSFERMEMOBJ_TRANSFERFAIL)     :
    ERR_TO_STR(T32_ERR_READVAR_ALLOC)                   :
    ERR_TO_STR(T32_ERR_READVAR_ACCESS)                  :
    ERR_TO_STR(T32_ERR_READBPOBJ_PARAFAIL)              :
    ERR_TO_STR(T32_ERR_READBPOBJ_NOTFOUND)              :
    ERR_TO_STR(T32_ERR_WRITEBPOBJ_FAILED)               :
    ERR_TO_STR(T32_ERR_WRITEBPOBJ_ADDRESS)              :
    ERR_TO_STR(T32_ERR_WRITEBPOBJ_ACTION)               :
    ERR_TO_STR(T32_ERR_MMUTRANSLATION_FAIL)             :
    ERR_TO_STR(T32_ERR_EXECUTECOMMAND_FAIL)             :
    ERR_TO_STR(T32_ERR_EXECUTEFUNCTION_FAIL)            :
    "unknown error code"
  );
}

/*********************************************************************
*
*       SYS_Hexdump()
*
*/
void SYS_Hexdump(void *inbuf, unsigned inlen, bool ascii, bool addr) {
  unsigned char *cp = (unsigned char *)inbuf;
  unsigned char *ap = (unsigned char *)inbuf;
  int len = inlen;
  int clen, alen;
  char outbuf[96];
  char *outp = &outbuf[0];
  int  line = 0;

  Log_Print("====================================HEX DUMP START\n");
  while (len > 0) {
    if (addr)
      outp += sprintf(outp, "[0x%08X] ", (int)cp);

    clen = alen = MIN(HEX_BYTES_PER_LINE, len);

    // display data in hex
    for (int i = 0; i < HEX_BYTES_PER_LINE; i++) {
      unsigned char uc = *cp++;

      if (--clen >= 0) {
        *outp++ = hexchar[(uc >> 4) & 0x0f];
        *outp++ = hexchar[(uc) & 0x0f];
        *outp++ = ' ';
      }
      else if (line != 0) {
        *outp++ = ' ';
        *outp++ = ' ';
        *outp++ = ' ';
      }
    }

    if (ascii) {
      *outp++ = ' ';
      *outp++ = ' ';

      // display data in ascii
      while (--alen >= 0) {
        unsigned char uc = *ap++;
        *outp++ = ((uc >= 0x20) && (uc < 0x7f)) ? uc : '.';
      }
    }

    // output the line
    *outp++ = '\n';
    *outp++ = '\0';
    Log_Print("%s", outbuf);
    outp = &outbuf[0];
    len -= HEX_BYTES_PER_LINE;
    line++;
  }
  Log_Print("====================================HEX DUMP END\n\n");
}

/*********************************************************************
*
*       LRealPath()
*
*/
char * LRealPath (const char *filename) {
  // Method 1: The system has a compile time upper bound on a filename
  // path.  Use that and realpath() to canonicalize the name.  This is
  // the most common case.  Note that, if there isn't a compile time
  // upper bound, you want to avoid realpath() at all costs.
#if defined(__linux__)
  {
    char buf[PATH_MAX];
    const char *rp = realpath (filename, buf);
    if (rp == NULL)
      rp = filename;
    return strdup (rp);
  }
#endif

  // The MS Windows method.  If we don't have realpath, we assume we
  // don't have symlinks and just canonicalize to a Windows absolute
  // path.  GetFullPath converts ../ and ./ in relative paths to
  // absolute paths, filling in current drive if one is not given
  // or using the current directory of a specified drive (eg, "E:foo").
  // It also converts all forward slashes to back slashes.
#if defined (_WIN32)
  {
    char buf[MAX_PATH];
    char* basename;
    DWORD len = GetFullPathName (filename, MAX_PATH, buf, &basename);
    if (len == 0 || len > MAX_PATH - 1)
      return _strdup (filename);
    else
    {
      // The file system is case-preserving but case-insensitive,
      // Canonicalize to lowercase, using the codepage associated
      // with the process locale.
        CharLowerBuff (buf, len);
        return _strdup (buf);
    }
  }
#endif
}

#ifdef _WIN32

//
// epoch
//
static const unsigned __int64 epoch = 116444736000000000LL;

/*********************************************************************
*
*       gettimeofday()
*
*/
static int gettimeofday(struct timeval *tp, struct timezone *tzp) {
  FILETIME        file_time;
  SYSTEMTIME      system_time;
  ULARGE_INTEGER  ularge;

  GetSystemTime(&system_time);
  SystemTimeToFileTime(&system_time, &file_time);
  ularge.LowPart = file_time.dwLowDateTime;
  ularge.HighPart = file_time.dwHighDateTime;

  tp->tv_sec = (long) ((ularge.QuadPart - epoch) / 10000000L);
  tp->tv_usec = (long) (system_time.wMilliseconds * 1000);

  return 0;
}
#endif

/*********************************************************************
*
*       RTT_TelnetLogS()
*
*  Function description
*    Execute the current command line.
*
*/
static void RTT_TelnetLogS(char *logFile, char *outBuf, uint32_t outLen) {
  static FILE       *pFile    = NULL;
  static VT_STATE_T  vt_state = Normal;
  char              *sPath    = NULL;
  int                chr      = 0;

  if (pFile == NULL) {
    // Convert the script to an absolute path
    sPath = LRealPath(logFile);

    //
    // call output routine.
    //
    pFile = fopen(sPath, "a+");
    free(sPath);
  }

  if (pFile != NULL) {
    while (outLen--) {
      chr = *outBuf++ & 0xFF;
      if (vt_state == DropOne) {
        vt_state = Normal;
        continue;
      }
      // Handle normal ANSI escape mechanism
      // (Note that this terminates DCS strings!)
      if (vt_state == Esc && chr >= 0x40 && chr <= 0x5F) {
        vt_state = Normal;
        chr += 0x40;
      }
      switch (chr) {
      case CAN:
      case SUB:
        vt_state = Normal;
        break;
      case ESC:
        vt_state = Esc;
        break;
      case CSI:
        vt_state = Csi;
        break;
      case DCS:
      case OSC:    // VT320 commands
      case PM:
      case APC:
        vt_state = Dcs;
        break;
      default:
        if ((chr & 0x6F) < 0x20) { // Check controls
          switch (chr) {
          // VT oddity -- controls go through regardless of state.
          case BEL : break;  // Pass these through
          case BS  : break;
          case TAB : break;
          case LF  : fprintf(pFile,  "%c", chr); break;
          case VT  : break;
          case FF  : break;
          case CR  : break;
          }
          break;
        }
        switch (vt_state) {
        case Normal:
          fprintf(pFile,  "%c", chr);
          break;
        case Esc:
          vt_state = Normal;
          switch (chr) {
          case 'c': case '7': case '8':
          case '=': case '>': case '~':
          case 'n': case '\123': case 'o':
          case '|':
            break;
          case '#': case ' ': case '(':
          case ')': case '*': case '+':
            vt_state = DropOne;
            break;
          }
          break;
        case Csi:
        case Dcs:
          if (chr >= 0x40 && chr <= 0x7E)
            if (vt_state == Csi)
              vt_state = Normal;
            else
              vt_state = DcsString;
          break;
        case DcsString:
          // Just drop everything here
          break;
        } //switch (vt_state) {
      } // switch (chr) {
    } // while (outLen--) {
    fflush(pFile);
  } // if (pFile != NULL) {
}

/*********************************************************************
*
*       SEGGER_atoi()
*
*  Function description
*    Converts a string to an integer.
*
*  Parameters
*    s: String to parse.
*
*  Return value
*    Converted integer value.
*/
int SEGGER_atoi(const char* s) {
  int  Value;
  int  Digit;
  char c;

  Value = 0;
  c     = *s++;
  do {
    if ((c >= '0') && (c <= '9')) {
      Digit = (int)(c - '0');
    } else {
      break;  // Will break on '\0' as well.
    }
    Value = (Value * 10) + Digit;
    c     = *s++;
  } while (c != '\0');
  return Value;
}

/*********************************************************************
*
*       SYS_ExitHandler
*
*  Function description
*    This is signal Handler
*/
static void SYS_ExitHandler(int signum) {

  T32_DefaultState(0);
  //T32_Terminate(T32_OK);
  T32_Exit();

  switch (signum) {
    // interrupt
    case SIGINT  : Log_Print("Interrupt.\n"); break;
    // Software termination signal from kill
    case SIGTERM : Log_Print("Terminated.\n"); break;
    // illegal instruction - invalid function image
    case SIGILL : Log_Print("Terminated.\n"); break;
    // floating point exception
    case SIGFPE : Log_Print("Terminated.\n"); break;
    // segment violation
    case SIGSEGV : Log_Print("Terminated.\n"); break;
    //Ctrl-Break sequence
#ifdef _WIN32
    case SIGBREAK : Log_Print("Terminated.\n"); break;
#endif
    // abnormal termination triggered by abort call
    case SIGABRT : Log_Print("Terminated.\n"); break;
    default      : Log_Print("unknow.\n");  break;
  }

  exit(signum);
}


/*********************************************************************
*
*       trace32 rtt functions
*
**********************************************************************
*/


/*********************************************************************
*
*       T32_GetRTTCBAddr()
*
*/
unsigned int T32_GetRTTCBAddr(const char * symname) {
  unsigned int address, size, reserved;
  int Result;
  Result = T32_GetSymbol( symname, &address, &size, &reserved );
  if (Result != T32_OK) {
    Log_Print("T32_GetRTTCBAddr error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
  return address;
}

/*********************************************************************
*
*       T32_GetRTTCBSize()
*
*/
unsigned int T32_GetRTTCBSize(const char * symname) {
  unsigned int address, size, reserved;
  int Result;
  Result = T32_GetSymbol( symname, &address, &size, &reserved );
  if (Result != T32_OK) {
    Log_Print("T32_GetRTTCBSize error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
  return size;
}

/*********************************************************************
*
*      T32_GetBytes
*
*/
void T32_GetBytes(unsigned int address, unsigned int cnt, void *dest) {
  int Result;
  Result = T32_ReadMemory(address, 0x40 /* E:*/, (unsigned char*)(dest), cnt);
  if (Result != T32_OK) {
    Log_Print("T32_GetBytes error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
}

/*********************************************************************
*
*      T32_GetByte
*
*/
unsigned char T32_GetByte(unsigned int address) {
  char byte;
  T32_GetBytes(address, sizeof(char), &byte);
  return byte;
}

/*********************************************************************
*
*      T32_GetWord
*
*/
unsigned int T32_GetWord(unsigned int address) {
  int word;
  T32_GetBytes(address, sizeof(int), &word);
  return word;
}

/*********************************************************************
*
*      T32_SetBytes
*
*/
void T32_SetBytes(unsigned int address, unsigned int cnt, void const *src) {
  int Result;
  Result = T32_WriteMemory(address, 0x40 /* E:*/, (unsigned char*)(src), cnt);
  if (Result != T32_OK) {
    Log_Print("T32_SetBytes error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
}

/*********************************************************************
*
*      T32_SetByte
*
*/
void T32_SetByte(unsigned int address, unsigned char data) {
  T32_SetBytes(address, sizeof(char), &data);
}

/*********************************************************************
*
*      T32_SetWord
*
*/
void T32_SetWord(unsigned int address, unsigned int data) {
  T32_SetBytes(address, sizeof(int), &data);
}

/*********************************************************************
*
*       T32_strlen
*
*/
unsigned T32_strlen(const char * s) {
  unsigned Len;
  char   c = 0;
  int addr = 0;

   addr = (unsigned int)s;

  Len = 0;
  c = T32_GetByte(addr);
  while (c != '\0') {
    c = T32_GetByte(addr);
    addr = addr + 1;
    Len++;
  }
  return Len;
}

/*********************************************************************
*
*      T32_strcpy
*
*/
char * T32_strcpy(char * dst, char * src) {
   char * p = NULL;
   char   c = 0;
   int addr = 0;

   addr = (unsigned int)src;

   if (dst != NULL) {
     p = dst;
     c = T32_GetByte(addr);
     while(c != '\0') {
      c = T32_GetByte(addr);
      *dst++ = c;
      addr = addr + 1;
     }
     *dst = '\0';
   }
   return p;
}

/*********************************************************************
*
*       T32_memcpy2P()
*
*/
void T32_memcpy2P(void* pDest, void* pSrc, unsigned NumBytes) {
  int Result;
  Result = T32_ReadMemory((unsigned int)pSrc, 0x40 /* E:*/, (unsigned char *)pDest, NumBytes);
  if (Result != T32_OK) {
    Log_Print("T32 memcpy to pc error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
}

/*********************************************************************
*
*       T32_memcpy2C()
*
*/
void T32_memcpy2C(void* pDest, void* pSrc, unsigned NumBytes) {
  int Result;
  Result = T32_WriteMemory((unsigned int)pDest, 0x40 /* E:*/, (unsigned char *)pSrc, NumBytes);
  if (Result != T32_OK) {
    Log_Print("T32 memcpy to chip error, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
}

/*********************************************************************
*
*       T32_RetryGetState()
*
*/
static int T32_RetryGetState(int (*pGetState)(int *pState), int *pState, int nRetry) {
  int Result;

  while (nRetry > 0) {
    Result = pGetState(pState);
    if (Result == T32_ERR_COM_RECEIVE_FAIL || Result == T32_ERR_COM_TRANSMIT_FAIL) {
      nRetry--;
      SYS_Sleep(5);
    }
    else {
      return Result;
    }
  }
  return Result;
}

/*********************************************************************
*
*       T32_WaitPractise()
*
*/
static void T32_WaitPractise(int nRetry, int nDelay) {
  int Result;
  int pState;

  for (;;) {
    Result = T32_RetryGetState(&T32_GetPracticeState, &pState, nRetry);
    if (Result != T32_OK) {
      Log_Print("Failed to query Trace32 state (error code: %s)\n", T32_Err2Str(Result));
      SYS_ExitHandler(Result);
    }

    if (pState == 0) {
      Log_Print("Practise done.\n");
      break;
    }
    else if (pState == 1) {
      Log_Print("Practise running.\n");
      SYS_Sleep(nDelay);
    }
    else if (pState == 2) {
      Log_Print("ERROR Trace32 is in dialog mode. It waits for an input.\n");
      SYS_ExitHandler(Result);
    }
    else {
      Log_Print("ERROR Unknown status. What is going on?\n");
      SYS_ExitHandler(Result);
    } // end if (pState == 0)
  } // for (;;) {
}

/*********************************************************************
*
*       T32_RunScriptFile()
*
*  Function description
*    Makes Trace32 synchronouly execute a CMM script arg.
*    The script arg is systematically converted to an absolute path.
*
*/
static void T32_RunScriptFile(const char *pCmmFile) {
  int  Result;
  char *sPath = NULL;

  // Convert the script to an absolute path
  sPath = LRealPath(pCmmFile);

  // Ask T32 to run the script. If the function succeeds, T32 will be
  // loading the script asynchronously!
  Result = T32_Cmd_f("DO \"%s\"", sPath);
  if (Result != T32_OK) {
    Log_Print("Failed to execute CMM script '%s' (error code: %s)\n", sPath, T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }

  Log_Print("Remotely running script '%s'. This may take some time.\n", sPath);

  free(sPath);
  T32_WaitPractise(8, 2);
}

/*********************************************************************
*
*       T32_ConfigSet()
*
*/
static void T32_ConfigSet(const char *String1, const char *String2) {
  int Result;

  Log_Print("\"%s\", \"%s\"\n", String1, String2);
  Result = T32_Config(String1, String2);
  if (Result != T32_OK) {
    Log_Print("Failed to set config '%s-%s' (error code: %s)\n", String1, String2, T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
}

/*********************************************************************
*
*       T32_DefaultState()
*
*/
static void T32_DefaultState(int exit) {
  int Result;
  int pState;

  Result = T32_RetryGetState(&T32_GetPracticeState, &pState, 8);
  if (Result != T32_OK) {
    Log_Print("Failed to query Trace32 practice state (error code: %s)\n", T32_Err2Str(Result));
    if (exit == 1) {
      SYS_ExitHandler(Result);
    }
  }
  // Running
  if (pState == 1) {
    Log_Print("Practise running.\n");
    Result = T32_Stop();
    if (Result != T32_OK) {
      Log_Print("Failed to stop (error code: %s)\n", T32_Err2Str(Result));
      if (exit == 1) {
        SYS_ExitHandler(Result);
      }
    }
  }

  Result = T32_RetryGetState(&T32_GetState, &pState, 8);
  if (Result != T32_OK) {
    Log_Print("Failed to query Trace32 debugger state (error code: %s)\n", T32_Err2Str(Result));
    if (exit == 1) {
      SYS_ExitHandler(Result);
    }
  }
  // Running
  if (pState == 3) {
    Log_Print("Debugger running.\n");
    Result = T32_Break();
    if (Result != T32_OK) {
      Log_Print("Failed to break (error code: %s)\n", T32_Err2Str(Result));
      if (exit == 1) {
        SYS_ExitHandler(Result);
      }
    }
  }
}

/*********************************************************************
*
*       T32_IFStop2Run()
*
*/
static void T32_IFStop2Run(void) {
  int Result;
  int pState;

  Result = T32_RetryGetState(&T32_GetState, &pState, 8);
  if (Result != T32_OK) {
    Log_Print("Failed to query Trace32 debugger state (error code: %s)\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
  // Stopped
  if (pState == 2) {
    Log_Print("Debugger Stopped.\n");
    Result = T32_Go();
    if (Result != T32_OK) {
      Log_Print("Failed to break (error code: %s)\n", T32_Err2Str(Result));
      SYS_ExitHandler(Result);
    }
  }
}

/*********************************************************************
*
*       T32_IFRun2Stop()
*
*/
static void T32_IFRun2Stop(void) {
  int Result;
  int pState;

  Result = T32_RetryGetState(&T32_GetState, &pState, 8);
  if (Result != T32_OK) {
    Log_Print("Failed to query Trace32 debugger state (error code: %s)\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  }
  // Running
  if (pState == 3) {
    Log_Print("Debugger running.\n");
    Result = T32_Break();
    if (Result != T32_OK) {
      Log_Print("Failed to break (error code: %s)\n", T32_Err2Str(Result));
      SYS_ExitHandler(Result);
    }
  }
}

/*********************************************************************
*
*       T32_InitDEVICD()
*
*/
static void T32_InitDEVICD(char *Node, char *Port, char *PackLen, char *cmmFile ) {
  int Result;

  T32_ConfigSet("NODE="   , Node);
  T32_ConfigSet("PORT="   , Port);
  if (PackLen != NULL) {
    T32_ConfigSet("PACKLEN=", PackLen);
  }

  //
  // Trace32 Init
  //
  Result = T32_Init();
  if(Result != T32_OK) {
    Log_Print("Error initializing TRACE32, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  };

  //
  // Attach to T32. The parameter is the device identifier.  T32_DEV_ICD and
  // T32_DEV_ICE are identical and mean the same thing: the debugger
  //
  Result = T32_Attach(T32_DEV_ICD);
  if(Result != T32_OK) {
    Log_Print("Error no device, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  };

  if (cmmFile != NULL) {
    T32_IFRun2Stop();
  }
  else {
    T32_IFStop2Run();
  }

  Result = T32_Nop();
  if(Result != T32_OK) {
    Log_Print("Error nop, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  };

  Result = T32_Ping();
  if(Result != T32_OK) {
    Log_Print("Sends one PING message to the system fail, Result = %s.\n", T32_Err2Str(Result));
    SYS_ExitHandler(Result);
  };

  if ( cmmFile != NULL) {
    T32_RunScriptFile(cmmFile);
  }
}

/*********************************************************************
*
*       _WriteBlocking()
*
*  Function description
*    Stores a specified number of characters in SEGGER RTT ring buffer
*    and updates the associated write pointer which is periodically
*    read by the host.
*    The caller is responsible for managing the write chunk sizes as
*    _WriteBlocking() will block until all data has been posted successfully.
*
*  Parameters
*    pRing        Ring buffer to post to.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the SEGGER RTT control block.
*
*  Return value
*    >= 0 - Number of bytes written into buffer.
*/
static unsigned _WriteBlocking(unsigned Address, const char* pBuffer, unsigned NumBytes) {
  unsigned NumBytesToWrite;
  unsigned NumBytesWritten;
  unsigned RdOff;
  unsigned WrOff;
  char*    pDst;
  //Log_Print("Address = 0x%08X\n", Address);
  //
  // Write data to buffer and handle wrap-around if necessary
  //
  NumBytesWritten = 0u;
  WrOff = T32_GetWord(RTTBUFFER_OFFSET_WROFF(Address));
  do {
    RdOff = T32_GetWord(RTTBUFFER_OFFSET_RDOFF(Address));                         // May be changed by host (debug probe) in the meantime
    if (RdOff > WrOff) {
      NumBytesToWrite = RdOff - WrOff - 1u;
    } else {
      NumBytesToWrite = T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(Address)) - (WrOff - RdOff + 1u);
    }
    NumBytesToWrite = MIN(NumBytesToWrite, (T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(Address)) - WrOff));      // Number of bytes that can be written until buffer wrap-around
    NumBytesToWrite = MIN(NumBytesToWrite, NumBytes);
    pDst = (char *)(T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(Address)) + WrOff);
    T32_memcpy2C((void*)pDst, (void*)pBuffer, NumBytesToWrite);
    NumBytesWritten += NumBytesToWrite;
    pBuffer         += NumBytesToWrite;
    NumBytes        -= NumBytesToWrite;
    WrOff           += NumBytesToWrite;
    if (WrOff == T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(Address))) {
      WrOff = 0u;
    }
    T32_SetWord(RTTBUFFER_OFFSET_WROFF(Address), WrOff);
  } while (NumBytes);
  return NumBytesWritten;
}

/*********************************************************************
*
*       _WriteNoCheck()
*
*  Function description
*    Stores a specified number of characters in SEGGER RTT ring buffer
*    and updates the associated write pointer which is periodically
*    read by the host.
*    It is callers responsibility to make sure data actually fits in buffer.
*
*  Parameters
*    pRing        Ring buffer to post to.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the SEGGER RTT control block.
*
*  Notes
*    (1) If there might not be enough space in the "Up"-buffer, call _WriteBlocking
*/
static void _WriteNoCheck(unsigned Address, const char* pData, unsigned NumBytes) {
  unsigned NumBytesAtOnce;
  unsigned WrOff;
  unsigned Rem;
  char*    pDst;

  //Log_Print("Address = 0x%08X\n", Address);
  WrOff = T32_GetWord(RTTBUFFER_OFFSET_WROFF(Address));
  Rem = T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(Address)) - WrOff;
  if (Rem > NumBytes) {
    //
    // All data fits before wrap around
    //
    pDst = (char *)(T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(Address)) + WrOff);
    T32_memcpy2C((void*)pDst, (void*)pData, NumBytes);
    T32_SetWord(RTTBUFFER_OFFSET_WROFF(Address), WrOff + NumBytes);
  } else {
    //
    // We reach the end of the buffer, so need to wrap around
    //
    NumBytesAtOnce = Rem;
    pDst = (char *)(T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(Address)) + WrOff);
    T32_memcpy2C((void*)pDst, (void*)pData, NumBytesAtOnce);
    NumBytesAtOnce = NumBytes - Rem;
    pDst = (char *)T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(Address));
    T32_memcpy2C((void*)pDst, (void*)(pData + Rem), NumBytesAtOnce);
    T32_SetWord(RTTBUFFER_OFFSET_WROFF(Address), NumBytesAtOnce);
  }
}

/*********************************************************************
*
*       _GetAvailWriteSpace()
*
*  Function description
*    Returns the number of bytes that can be written to the ring
*    buffer without blocking.
*
*  Parameters
*    pRing        Ring buffer to check.
*
*  Return value
*    Number of bytes that are free in the buffer.
*/
static unsigned _GetAvailWriteSpace(unsigned Address) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned r;
  //Log_Print("Address = 0x%08X\n", Address);
  //
  // Avoid warnings regarding volatile access order.  It's not a problem
  // in this case, but dampen compiler enthusiasm.
  //
  RdOff = T32_GetWord(RTTBUFFER_OFFSET_RDOFF(Address));
  WrOff = T32_GetWord(RTTBUFFER_OFFSET_WROFF(Address));
  if (RdOff <= WrOff) {
    r = T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(Address)) - 1u - WrOff + RdOff;
  } else {
    r = RdOff - WrOff - 1u;
  }
  return r;
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/

/*********************************************************************
*
*       SEGGER_RTT_ReadUpBufferNoLock()
*
*  Function description
*    Reads characters from SEGGER real-time-terminal control block
*    which have been previously stored by the application.
*    Do not lock against interrupts and multiple access.
*    Used to do the same operation that J-Link does, to transfer
*    RTT data via other channels, such as TCP/IP or UART.
*
*  Parameters
*    BufferIndex  Index of Up-buffer to be used.
*    pBuffer      Pointer to buffer provided by target application, to copy characters from RTT-up-buffer to.
*    BufferSize   Size of the target application buffer.
*
*  Return value
*    Number of bytes that have been read.
*
*  Additional information
*    This function must not be called when J-Link might also do RTT.
*/
unsigned SEGGER_RTT_ReadUpBufferNoLock(unsigned Address, unsigned BufferIndex, void* pData, unsigned BufferSize) {
  unsigned                NumBytesRem;
  unsigned                NumBytesRead;
  unsigned                RdOff;
  unsigned                WrOff;
  unsigned                RingAddr;
  unsigned char*          pBuffer;
           char*          pSrc;

  RingAddr = RTTCB_OFFSET_AUP_INDEX(Address, BufferIndex);
  //Log_Print("Address = 0x%08X RingAddr = 0x%08X BufferIndex = 0x%08X\n", Address, RingAddr, BufferIndex);
  pBuffer = (unsigned char*)pData;
  RdOff = T32_GetWord(RTTBUFFER_OFFSET_RDOFF(RingAddr));
  WrOff = T32_GetWord(RTTBUFFER_OFFSET_WROFF(RingAddr));
  NumBytesRead = 0u;
  //
  // Read from current read position to wrap-around of buffer, first
  //
  if (RdOff > WrOff) {
    NumBytesRem = T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(RingAddr)) - RdOff;
    NumBytesRem = MIN(NumBytesRem, BufferSize);
    pSrc = (char *)(T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(RingAddr)) + RdOff);
    T32_memcpy2P(pBuffer, (void*)pSrc, NumBytesRem);
    NumBytesRead += NumBytesRem;
    pBuffer      += NumBytesRem;
    BufferSize   -= NumBytesRem;
    RdOff        += NumBytesRem;
    //
    // Handle wrap-around of buffer
    //
    if (RdOff == T32_GetWord(RTTBUFFER_OFFSET_SIZEOFBUFFER(RingAddr))) {
      RdOff = 0u;
    }
  }
  //
  // Read remaining items of buffer
  //
  NumBytesRem = WrOff - RdOff;
  NumBytesRem = MIN(NumBytesRem, BufferSize);
  if (NumBytesRem > 0u) {
    pSrc = (char *)(T32_GetWord(RTTBUFFER_OFFSET_PBUFFER(RingAddr)) + RdOff);
    T32_memcpy2P(pBuffer, (void*)pSrc, NumBytesRem);
    NumBytesRead += NumBytesRem;
    pBuffer      += NumBytesRem;
    BufferSize   -= NumBytesRem;
    RdOff        += NumBytesRem;
  }
  //
  // Update read offset of buffer
  //
  if (NumBytesRead) {
    T32_SetWord(RTTBUFFER_OFFSET_RDOFF(RingAddr), RdOff);
  }
  //
  return NumBytesRead;
}

/*********************************************************************
*
*       SEGGER_RTT_WriteDownBufferNoLock
*
*  Function description
*    Stores a specified number of characters in SEGGER RTT
*    control block inside a <Down> buffer.
*    SEGGER_RTT_WriteDownBufferNoLock does not lock the application.
*    Used to do the same operation that J-Link does, to transfer
*    RTT data from other channels, such as TCP/IP or UART.
*
*  Parameters
*    BufferIndex  Index of "Down"-buffer to be used.
*    pBuffer      Pointer to character array. Does not need to point to a \0 terminated string.
*    NumBytes     Number of bytes to be stored in the SEGGER RTT control block.
*
*  Return value
*    Number of bytes which have been stored in the "Down"-buffer.
*
*  Notes
*    (1) Data is stored according to buffer flags.
*    (2) For performance reasons this function does not call Init()
*        and may only be called after RTT has been initialized.
*        Either by calling SEGGER_RTT_Init() or calling another RTT API function first.
*
*  Additional information
*    This function must not be called when J-Link might also do RTT.
*/
unsigned SEGGER_RTT_WriteDownBufferNoLock(unsigned Address, unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  unsigned                Status;
  unsigned                Avail;
  const char*             pData;
  unsigned                RingAddr;
  unsigned                MaxUpNum;
  //
  // Get "to-target" ring buffer.
  // It is save to cast that to a "to-host" buffer. Up and Down buffer differ in volatility of offsets that might be modified by J-Link.
  //
  MaxUpNum = T32_GetWord(RTTCB_OFFSET_MAXNUMUPBUFFERS(Address));
  pData = (const char *)pBuffer;
  RingAddr = RTTCB_OFFSET_ADOWN_INDEX(Address, MaxUpNum, BufferIndex);
  //Log_Print("Address = 0x%08X RingAddr = 0x%08X BufferIndex = 0x%08X\n", Address, RingAddr, BufferIndex);
  //
  // How we output depends upon the mode...
  //
  switch (T32_GetWord(RTTBUFFER_OFFSET_FLAGS(RingAddr))) {
  case SEGGER_RTT_MODE_NO_BLOCK_SKIP:
    //
    // If we are in skip mode and there is no space for the whole
    // of this output, don't bother.
    //
    Avail = _GetAvailWriteSpace(RingAddr);
    if (Avail < NumBytes) {
      Status = 0u;
    } else {
      Status = NumBytes;
      _WriteNoCheck(RingAddr, pData, NumBytes);
    }
    break;
  case SEGGER_RTT_MODE_NO_BLOCK_TRIM:
    //
    // If we are in trim mode, trim to what we can output without blocking.
    //
    Avail = _GetAvailWriteSpace(RingAddr);
    Status = Avail < NumBytes ? Avail : NumBytes;
    _WriteNoCheck(RingAddr, pData, Status);
    break;
  case SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL:
    //
    // If we are in blocking mode, output everything.
    //
    Status = _WriteBlocking(RingAddr, pData, NumBytes);
    break;
  default:
    Status = 0u;
    break;
  }
  //
  // Finish up.
  //
  return Status;
}

/*********************************************************************
*
*       SEGGER_RTT_GetBytesInBuffer()
*
*  Function description
*    Returns the number of bytes currently used in the up buffer.
*
*  Parameters
*    BufferIndex  Index of the up buffer.
*
*  Return value
*    Number of bytes that are used in the buffer.
*/
unsigned SEGGER_RTT_GetBytesInBuffer(unsigned Address, unsigned BufferIndex) {
  unsigned RdOff;
  unsigned WrOff;
  unsigned r;
  //Log_Print("Address = 0x%08X BufferIndex = 0x%08X\n", Address, BufferIndex);
  //
  // Avoid warnings regarding volatile access order.  It's not a problem
  // in this case, but dampen compiler enthusiasm.
  //
  RdOff = T32_GetWord(RTTCB_OFFSET_AUP_RDOFF(Address, BufferIndex));
  WrOff = T32_GetWord(RTTCB_OFFSET_AUP_WROFF(Address, BufferIndex));
  if (RdOff <= WrOff) {
    r = WrOff - RdOff;
  } else {
    r = T32_GetWord(RTTCB_OFFSET_AUP_SIZEOFBUFFER(Address, BufferIndex)) - (WrOff - RdOff);
  }
  return r;
}

/*********************************************************************
*
*       SEGGER_Terminal_GetChannelID()
*
*  Function description
*    Returns the RTT <Up> / <Down> channel ID used by Terminal.
*/
int SEGGER_Terminal_GetChannelID(void) {
  return 0;
}

/*********************************************************************
*
*       _WaitPolling
*
*  Function description
*    Poll SystemView Buffer to reach threshold fill level.
*
*/
static void _WaitPolling(unsigned Address, int Timeout, int ChannelID) {
  int BytesInBuffer;
  do {
    BytesInBuffer = SEGGER_RTT_GetBytesInBuffer(Address, ChannelID);
    if (BytesInBuffer >= RTT_SEND_THRESHOLD) {
      break;
    }
    SYS_Sleep(RTT_COMM_POLL_INTERVAL);
    Timeout -= RTT_COMM_POLL_INTERVAL;
  } while (Timeout > 0);
}

/*********************************************************************
*
*       init functions
*
**********************************************************************
*/

/*********************************************************************
*
*       SIGNAL_HandlerInit()
*
*  Function description
*    This is NOP crash handler. Just to make things compiling if
*    you don't have a real crash handler on your OS.
*/
static int SIGNAL_HandlerInit(void) {
  signal(SIGINT  , SYS_ExitHandler);
  signal(SIGTERM , SYS_ExitHandler);
  signal(SIGILL  , SYS_ExitHandler);
  signal(SIGFPE  , SYS_ExitHandler);
  signal(SIGSEGV , SYS_ExitHandler);
#ifdef _WIN32
  signal(SIGBREAK, SYS_ExitHandler);
#endif
  signal(SIGABRT , SYS_ExitHandler);
  return 0;
}

#ifdef _WIN32
/*********************************************************************
*
*       _WSAStartup
*
*  Function description
*    Initializes Winsock API. Needs to be called once before using any socket API.
*    May be called multiple times.
*/
static void _WSAStartup(void) {
  WORD    wVersionRequested;
  WSADATA wsaData;
  //
  // Init Winsock API
  //
  wVersionRequested = MAKEWORD(2, 2);
  WSAStartup(wVersionRequested, &wsaData);
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _WSACleanup
*
*  Function description
*    Cleans up Winsock API. WSACleanup() needs to be called as many times
*    as WSAStartup() if socket API is no longer needed.
*/
static void _WSACleanup(void) {
  WSACleanup();
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_OpenUDP
*
*  Function description
*    Creates an IPv4 TCP socket.
*
*  Return value
*    Handle to socket
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_OpenUDP(void) {
  _SYS_SOCKET_HANDLE sock;
  //
  // Create socket
  //
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock != INVALID_SOCKET) {
    return (_SYS_SOCKET_HANDLE)sock;
  } else {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_IPV6_OpenUDP
*
*  Function description
*    Creates an IPv4 TCP socket.
*
*  Return value
*    Handle to socket
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_IPV6_OpenUDP(char DualStack) {
  _SYS_SOCKET_HANDLE sock;
  //
  // Create socket
  //
  sock = socket(AF_INET6, SOCK_DGRAM, 0);
  if (sock != INVALID_SOCKET && DualStack) {
    //
    // Disable Nagle's algorithm to speed things up
    // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
    //
    setsockopt(sock, IPPROTO_IPV6, TCP_CC_INFO, (char*)&_int1, sizeof(int));
  } else {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
  return (_SYS_SOCKET_HANDLE)sock;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_OpenTCP
*
*  Function description
*    Creates an IPv4 TCP socket.
*
*  Return value
*    Handle to socket
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_OpenTCP(void) {
  SOCKET sock;
  //
  // Init Winsock API as this is the first socket-related function being called
  // WSACleanup is called on SocketClose()
  //
  _WSAStartup();
  //
  // Create socket
  //
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock != INVALID_SOCKET) {
    //
    // Disable Nagle's algorithm to speed things up
    // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
    //
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
  } else {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
  return (_SYS_SOCKET_HANDLE)sock;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_OpenTCP
*
*  Function description
*    Creates an IPv4 TCP socket.
*
*  Return value
*    Handle to socket
*/
static _SYS_SOCKET_HANDLE SYS_SOCKET_IPV6_OpenTCP(char DualStack) {
  _SYS_SOCKET_HANDLE sock;
  //
  // Create socket
  //
  sock = socket(AF_INET6, SOCK_STREAM, 0);
  if (sock == INVALID_SOCKET) {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
  if (!DualStack) {
    return (_SYS_SOCKET_HANDLE)sock;
  }
  setsockopt(sock, IPPROTO_IPV6, TCP_CC_INFO, (char*)&_int1, sizeof(int));
  return (_SYS_SOCKET_HANDLE)sock;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_OpenTCP
*
*  Function description
*    Creates an IPv4 TCP socket.
*
*  Return value
*    Handle to socket
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_OpenTCP(void) {
  _SYS_SOCKET_HANDLE sock;
  //
  // Create socket
  //
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock != INVALID_SOCKET) {
    //
    // Disable Nagle's algorithm to speed things up
    // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
    //
    setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
  } else {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
  return (_SYS_SOCKET_HANDLE)sock;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_Close
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
static void _SYS_SOCKET_Close(_SYS_SOCKET_HANDLE hSocket) {
  SOCKET Sock;
  int OptVal;
  int OptLen;
  //
  // MSDN: A socket that is using the SO_EXCLUSIVEADDRUSE option must be shut down properly prior to closing it. Failure to do so can cause a denial of service attack if the associated service needs to restart.
  //
  Sock = (SOCKET)hSocket;
  OptLen = 4;
  OptVal = 0;  // Make sure it is zero initialized in case getsockopt does not fill it completely
  getsockopt(Sock, SOL_SOCKET, -5, (char*)&OptVal, &OptLen);  // SO_EXCLUSIVEADDRUSE (Define not known in the VC6 headers, we use...)
  if (OptVal) {
    shutdown(Sock, SD_BOTH);
  }
  //
  // Close socket
  //
  closesocket(Sock);
  //
  // De-init Winsock API. Needs to be called as often as WSAStartup() has been called
  // Has an internal reference counter
  // If the counter reaches 0, all sockets opened by the process, are forced closed
  //
  _WSACleanup();
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_Close
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
static void _SYS_SOCKET_Close(_SYS_SOCKET_HANDLE hSocket) {
  //
  // Close socket
  //
  close(hSocket);
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_Shutdown
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
static void _SYS_SOCKET_Shutdown (_SYS_SOCKET_HANDLE hSocket, int How) {
  _SYS_SOCKET_HANDLE Sock;
  int r;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  r    = How;

  switch (r) {
    case _SYS_SOCKET_SHUT_RD:
      shutdown(Sock, 0);
    case _SYS_SOCKET_SHUT_WR:
      shutdown(Sock, 1);
    case _SYS_SOCKET_SHUT_RDWR:
      shutdown(Sock, 2);
  }
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       SYS_SOCKET_SetTimeouts
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
int SYS_SOCKET_SetTimeouts(int hSocket, int TimeoutSend, int TimeoutReceive) {
  struct timeval timeout;

  timeout.tv_sec  = TimeoutSend / 1000;
  timeout.tv_usec = (TimeoutSend % 1000) * 1000;
  setsockopt(hSocket, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

  timeout.tv_sec  = TimeoutReceive / 1000;
  timeout.tv_usec = (TimeoutReceive % 1000) * 1000;
  return setsockopt(hSocket, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       SYS_SOCKET_DisableLinger
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
int SYS_SOCKET_DisableLinger(_SYS_SOCKET_HANDLE hSocket) {
  struct linger  so_linger;

  so_linger.l_onoff  = 0;
  so_linger.l_linger = 0;

  return setsockopt(hSocket, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       SYS_SOCKET_EnableLinger
*
*  Function description
*    Closes a socket. Resources allocated by this socket are freed.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*/
int SYS_SOCKET_EnableLinger(_SYS_SOCKET_HANDLE hSocket, int TimeoutLinger) {

  struct linger  so_linger;
  int timeout;

  timeout = 0;
  if (TimeoutLinger) {
    timeout = 1;
    if (TimeoutLinger > 999)
      timeout = (TimeoutLinger + 500) / 1000;
  }
  so_linger.l_onoff  = 1;
  so_linger.l_linger = timeout;
  setsockopt(hSocket, SOL_SOCKET, SO_LINGER, &so_linger, sizeof(so_linger));
  return 0;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_ListenAtTCPAddr
*
*  Function description
*    Puts IPv4 socket into listening state.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*    IPAddr   IPv4 address expected in little endian form, meaning 127.0.0.1 is expected as 0x7F000001
*             To accept connections from any IP address, pass _SYS_SOCKET_IP_ADDR_ANY
*    Port     Port to listen at
*
*  Return value
*    >= 0: O.K.
*     < 0: Error
*/
static int _SYS_SOCKET_ListenAtTCPAddr(_SYS_SOCKET_HANDLE hSocket, unsigned IPAddr, unsigned Port, unsigned NumConnectionsQueued) {
  struct sockaddr_in addr;
  SOCKET Sock;
  int r;
  //
  // Option SO_REUSEADDR:
  //
  // <IPAddr>: IP addresses of network adapters
  //
  // ==================================================
  // Original idea from BSD sockets
  // ==================================================
  // bind() without SO_REUSEADDR set (default):
  // bind(SockA, 0.0.0.0:21)
  // bind(SockB, 192.168.0.1:21)
  // SockB will fail because <IPAddr> of SockA is a wildcard that means "Any local address",
  // so it is not possible to bind to any other local address with the same port.
  // bind(SockA, 127.0.0.1:21)
  // bind(SockB, 192.168.0.1:21)
  // Both calls will succeed, as different <IPAddr>:<Port> combinations are used.
  //
  // bind() with SO_REUSEADDR set:
  // bind(SockA, 0.0.0.0:21)
  // bind(SockB, 192.168.0.1:21)
  // SockA and SockB will succeed.
  // The original idea includes that *each* of the sockets must have SO_REUSEADDR set before bind().
  // If only the second one calls it, bind() will fail as the first socket did not allow sharing at all.
  //
  // This was the original idea of SO_REUSEADDR.
  // NOTE: Not sure who really ever needed this, but that's the way it is...
  //
  // ==================================================
  // Second effect of SO_REUSEADDR (TIME_WAIT)
  // ==================================================
  // There is another case where this option has an effect on:
  // Calls to send() do not guarantee that data is sent when the function returns. It may be sent delayed.
  // Therefore, it is possible that when calling close() to close a socket, send data is still pending.
  // What the OS does is: preparing everything for closing the connection and return from close().
  // Now the socket changed it's state from ACTIVE to TIME_WAIT but still exists inside the OS (not accessible for the user anymore)
  // If now a new socket is opened and a bind() is performed on exactly the <IPAddr>:<Port> combination of the TIME_WAIT socket, the behavior depends on if the original socket had SO_REUSEADDR set.
  //
  // SO_REUSEADDR not set:
  // bind() will fail as TIME_WAIT is handled as if it is ACTIVE
  //
  // SO_REUSEADDR set:
  // bind() will succeed as TIME_WAIT is handled as if socket was not existing anymore.
  // NOTE: Under rare circumstances, it now can happen that if there is any receive data arriving late at the system, the new socket that did the bind(), will receive it.
  //
  // ==================================================
  // OS specifics
  // ==================================================
  //
  // Windows:
  // When specifying SO_REUSEADDR before bind(), Windows will report SUCCESS on bind(),
  // even if there is another ACTIVE socket that is bound to the same <IPAddr>:<Port> combination.
  // It does not matter if the process that did the first bind() did specify SO_REUSEADDR for its socket or not
  // This allows processes to steal data from other ones...pretty awful bug in Windows... (See MSDN: Using SO_REUSEADDR and SO_EXCLUSIVEADDRUSE)
  // Microsoft introduced SO_EXCLUSIVEADDRUSE for this.
  // This makes sure hijacking the socket data is not possible.
  // But it still allows the special behavior in case a bind() to a closed socket in TIME_WAIT state is possible. (See "Second effect of SO_REUSEADDR (TIME_WAIT)" above)
  //
  // MSDN: A socket that is using the SO_EXCLUSIVEADDRUSE option must be shut down properly prior to closing it. Failure to do so can cause a denial of service attack if the associated service needs to restart.
  //
  // Linux:
  // Listening socket:
  // SO_REUSEADDR does not have any effect for the "original idea" (see above). Linux is more restrictive than BSD sockets here.
  // But it has the desired effect on closed sockets in TIME_WAIT state (see above).
  //
  // Client socket:
  // Behaves like the original BSD idea and has the TIME_WAIT effect
  //
  // Kernel >= 3.9: To have "original idea" effect for listening sockets, since kernel 3.9 SO_REUSEPORT has been introduced.
  //
  // Normal TCP connection close:
  // Client1 (C1), Client2 (C2)
  // C1 -> C2  FIN
  // C1 <- C2  ACK
  // C1 <- C2  FIN
  // C1 -> C2  ACK
  // Socket of C1 (as the initiator of the close request) now is in TIME_WAIT state and can stay there several seconds/minutes
  // so the <IPAddr>:<Port> combination is blocked for some time, after the socket has been closed
  // As this is not acceptable for us as the DLL and other J-Link utilities must be able to be started / terminated multiple times in a row,
  // we make use of SO_REUSEADDR for all of our listener sockets which need to bind() to a specific port
  //
  Sock = (SOCKET)hSocket;
  r = setsockopt(Sock, SOL_SOCKET, -5, (char*)&_int1, sizeof(int));  // SO_EXCLUSIVEADDRUSE (Define not known in the VC6 headers, we use...)
  if (r == 0) {
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons((unsigned short)Port);
    addr.sin_addr.s_addr = htonl(IPAddr);
    r = bind(Sock, (struct sockaddr*)&addr, sizeof(addr));
    if (r == 0) {
      r = listen(Sock, NumConnectionsQueued);
    }
    if (r) {
      r = -1;
    }
  } else {
    r = -1;
  }
  return r;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_ListenAtTCPAddr
*
*  Function description
*    Puts IPv4 socket into listening state.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*    IPAddr   IPv4 address expected in little endian form, meaning 127.0.0.1 is expected as 0x7F000001
*             To accept connections from any IP address, pass _SYS_SOCKET_IP_ADDR_ANY
*    Port     Port to listen at
*
*  Return value
*    >= 0: O.K.
*     < 0: Error
*/
static int _SYS_SOCKET_ListenAtTCPAddr(_SYS_SOCKET_HANDLE hSocket, unsigned IPAddr, unsigned Port, unsigned NumConnectionsQueued) {
  _SYS_SOCKET_HANDLE Sock;
  struct sockaddr_in addr;
  int r;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  r = setsockopt(Sock, SOL_SOCKET, SO_REUSEADDR, (char*)&_int1, sizeof(int));
  if (r == 0) {
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons((unsigned short)Port);
    addr.sin_addr.s_addr = htonl(IPAddr);
    r = bind(Sock, (struct sockaddr*)&addr, sizeof(addr));
    if (r == 0) {
      r = listen(Sock, NumConnectionsQueued);
    }
    if (r) {
      r = -1;
    }
  } else {
    r = -1;
  }
  return r;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_IsReady
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is ready.
*    Mainly used on non-blocking sockets to check if they are ready to operate on.
*    The procedure (non-blocking connect, then trying FIONREAD) is recommended by MS (MSDN).
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket ready
*    == 0  O.K., socket not ready yet
*     < 0  Error
*/
static unsigned _SYS_SOCKET_IsReady(_SYS_SOCKET_HANDLE hSocket) {
  SOCKET Sock;
  int IsReady;
  unsigned long v;

  Sock = (SOCKET)hSocket;
  ioctlsocket(Sock, FIONREAD, &v);     // Check if socket is ready to read from
  IsReady = v ? 1 : 0;
  return IsReady;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_IsReady
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is ready.
*    Mainly used on non-blocking sockets to check if they are ready to operate on.
*    The procedure (non-blocking connect, then trying FIONREAD) is recommended by MS (MSDN).
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket ready
*    == 0  O.K., socket not ready yet
*     < 0  Error
*/
static unsigned _SYS_SOCKET_IsReady(_SYS_SOCKET_HANDLE hSocket) {
  _SYS_SOCKET_HANDLE Sock;
  int IsReady;
  unsigned long v;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  ioctl(Sock, FIONREAD, &v);     // Check if socket is ready to read from
  IsReady = v ? 1 : 0;
  return IsReady;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_IsReadable
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is readable.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket readable
*    == 0  O.K., socket not readable yet
*     < 0  Error
*/
static int _SYS_SOCKET_IsReadable(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  SOCKET Sock;
  struct timeval tv;
  fd_set rfds;
  fd_set efds;
  int v;
  //
  //
  // Two possible cases when opening a TCP connection:
  // 1) On the other side, there is a server listening on the destination port
  // 2) On the other side, there is NO server listening on the destination port
  //
  // Reg 1)
  //   In such cases, the socket is reported non-writable for a few [us] up to a few [s] depending on if this is a localhost, LAN or internet connection
  //   After that period, the socket is reported as writable
  //
  // Reg 2)
  //   In such cases, the following happens (S = Server):
  //   C -> S: SYN
  //   S -> C: RST ACK
  //   This means, the server has rejected the connection and closed it
  //   In such a case, we need to close the socket and try connecting again
  //
  // However, Windows is difficult regarding 2)...
  // Usually an RST means "it's over, you can close the socket. There is nobody listening"
  // Windows keeps the socket in connecting state, so IsReadable() IsWriteable() simply returns 0 instead of 1 (which would allow a following send/receive to return with error)
  // In the background, Windows performs the SYN sending another 3 times in intervals of 500ms before giving up and reporting an error state
  // Usually, the retransmissions are there in case we do not get an ACK from the other side and therefore must assume that the packet got lost
  // Then also the interval time is doubled for each retransmission
  // However, Microsoft decided to also retry it in case of RST ACK but without increasing the interval time
  // This is explained here: https://support.microsoft.com/en-in/help/175523/info-winsock-tcp-connection-performance-to-unused-ports
  //
  // Unfortunately, even after this 1.5 seconds, Windows not necessarily reports the socket as readable/writable but instead returns "exceptional state" on select()
  // Therefore, we also pass the "exceptional state" structure to select() to catch this case
  //
  Sock = (SOCKET)hSocket;
  FD_ZERO(&rfds);       // Zero init file descriptor list
  FD_SET(Sock, &rfds);  // Add socket to file descriptor list to be monitored by select()
  FD_ZERO(&efds);
  FD_SET(Sock, &efds);
  tv.tv_sec = (long)(TimeoutMs / 1000);
  tv.tv_usec = (TimeoutMs % 1000) * 1000;
  v = select(0, &rfds, NULL, &efds, &tv);   // > 0: in case of success, == 0: Timeout, < 0: Error
  return v;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_IsReadable
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is readable.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket readable
*    == 0  O.K., socket not readable yet
*     < 0  Error
*/
static int _SYS_SOCKET_IsReadable(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  _SYS_SOCKET_HANDLE Sock;
  struct timeval tv;
  fd_set rfds;
  int v;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  FD_ZERO(&rfds);       // Zero init file descriptor list
  FD_SET(Sock, &rfds);  // Add socket to file descriptor list to be monitored by select()
  tv.tv_sec = (long)(TimeoutMs / 1000);
  tv.tv_usec = (TimeoutMs % 1000) * 1000;
  v = select((hSocket + 1), &rfds, NULL, NULL, &tv);   // > 0: in case of success, == 0: Timeout, < 0: Error
  return v;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_AcceptEx
*
*  Function description
*    Waits for a connection (with timeout) on the given socket.
*
*  Parameters
*    hSocket   Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*    TimeoutMs Timeout in ms for waiting
*
*  Return value
*    >= 0  Handle to socket of new connection that has been established
*     < 0  Error   (_SYS_SOCKET_INVALID_HANDLE)
*      -2  Timeout
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_AcceptEx(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  SOCKET SockChild;
  int r;
  //
  // accept() itself does not allow using timeouts
  // Therefore we check readability first and then call accept() which should not block then
  //
  r = _SYS_SOCKET_IsReadable(hSocket, TimeoutMs);
  if (r < 0) {
    return _SYS_SOCKET_INVALID_HANDLE; // error
  } else if (r == 0) {
    return -2;                        // timeout
  } else {
    SockChild = accept((SOCKET)hSocket, NULL, NULL);
    if (SockChild != INVALID_SOCKET) {
      //
      // If connection is successfully established, handle it as an implicit open(), so also init Winsock API as WSACleanup() is called on SocketClose()
      // No matter if the socket has been opened via open() or accept()
      //
      _WSAStartup();
      //
      // Disable Nagle's algorithm to speed things up
      // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
      //
      setsockopt(SockChild, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
    } else {
      return _SYS_SOCKET_INVALID_HANDLE;
    }
  }
  return (_SYS_SOCKET_HANDLE)SockChild;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_Accept
*
*  Function description
*    Waits for a connection on the given socket.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    Handle to socket of new connection that has been established
*/
_SYS_SOCKET_HANDLE _SYS_SOCKET_Accept(_SYS_SOCKET_HANDLE hSocket) {
  _SYS_SOCKET_HANDLE SockChild;

  SockChild = accept((_SYS_SOCKET_HANDLE)hSocket, NULL, NULL);
  if (SockChild != INVALID_SOCKET) {
    //
    // Disable Nagle's algorithm to speed things up
    // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
    //
    setsockopt(SockChild, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
  } else {
    return _SYS_SOCKET_INVALID_HANDLE;
  }
  return (_SYS_SOCKET_HANDLE)SockChild;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_AcceptEx
*
*  Function description
*    Waits for a connection (with timeout) on the given socket.
*
*  Parameters
*    hSocket   Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*    TimeoutMs Timeout in ms for waiting
*
*  Return value
*    >= 0  Handle to socket of new connection that has been established
*     < 0  Error   (_SYS_SOCKET_INVALID_HANDLE)
*      -2  Timeout
*/
static _SYS_SOCKET_HANDLE _SYS_SOCKET_AcceptEx(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  _SYS_SOCKET_HANDLE SockChild;
  struct sockaddr_in  sockAddr;
  char ipStr[64];
  int r;
  int len;
  //
  // accept() itself does not allow using timeouts
  // Therefore we check readability first and then call accept() which should not block then
  //
  len = sizeof(struct sockaddr_in);
  r = _SYS_SOCKET_IsReadable(hSocket, TimeoutMs);
  if (r < 0) {
    return _SYS_SOCKET_INVALID_HANDLE; // error
  } else if (r == 0) {
    return -2;                        // timeout
  } else {
    SockChild = accept(hSocket, (struct sockaddr*)&sockAddr, (socklen_t *)&len);
    if (SockChild != INVALID_SOCKET) {
      //
      // Disable Nagle's algorithm to speed things up
      // Nagle's algorithm prevents small packets from being transmitted and collects some time until data is actually sent out
      //
      setsockopt(SockChild, IPPROTO_TCP, TCP_NODELAY, (char*)&_int1, sizeof(int));
    } else {
      return _SYS_SOCKET_INVALID_HANDLE;
    }
  }
  Log_Print("connect client ip: %s\t port: %d\n", inet_ntop(AF_INET, &sockAddr.sin_addr.s_addr, ipStr, sizeof(ipStr)), ntohs(sockAddr.sin_port));
  return (_SYS_SOCKET_HANDLE)SockChild;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_Receive
*
*  Function description
*    Receives data on the given socket.
*
*  Parameters
*    hSocket          Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    >= 0:  O.K., number of bytes received
*     < 0:  Error, see _SYS_SOCKET_ERR_*
*
*  Notes
*    (1) Returns as soon as something has been received (may be less than MaxNumBytes) or error happened
*/
static int _SYS_SOCKET_Receive(_SYS_SOCKET_HANDLE hSocket, void* pData, unsigned MaxNumBytes) {
  int r;
  int Err;
  SOCKET Sock;

  Sock = (SOCKET)hSocket;
  r = recv(Sock, (char*)pData, MaxNumBytes, 0);
  if (r < 0) {
    Err = WSAGetLastError();
    switch (Err) {
    case WSAEWOULDBLOCK:
      r = _SYS_SOCKET_ERR_WOULDBLOCK;
      break;
    case WSAECONNRESET:
      r = _SYS_SOCKET_ERR_CONNRESET;
      break;
    case WSAETIMEDOUT:
      r = _SYS_SOCKET_ERR_TIMEDOUT;
      break;
    default:
      r = _SYS_SOCKET_ERR_UNSPECIFIED;
    }
  }
  return r;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_Receive
*
*  Function description
*    Receives data on the given socket.
*
*  Parameters
*    hSocket          Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    >= 0:  O.K., number of bytes received
*     < 0:  Error, see _SYS_SOCKET_ERR_*
*
*  Notes
*    (1) Returns as soon as something has been received (may be less than MaxNumBytes) or error happened
*/
static int _SYS_SOCKET_Receive(_SYS_SOCKET_HANDLE hSocket, void* pData, unsigned MaxNumBytes) {
  int r;
  int Err;
  _SYS_SOCKET_HANDLE Sock;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  r = recv(Sock, (char*)pData, MaxNumBytes, 0);
  if (r < 0) {
    Err = errno;
    switch (Err) {
#if EAGAIN != EWOULDBLOCK
    case EAGAIN:
#endif
    case EWOULDBLOCK:
      r = _SYS_SOCKET_ERR_WOULDBLOCK;
      break;
    case ENETRESET:
    case ECONNRESET:
      r = _SYS_SOCKET_ERR_CONNRESET;
      break;
    case ETIMEDOUT:
      r = _SYS_SOCKET_ERR_TIMEDOUT;
      break;
    default:
      r = _SYS_SOCKET_ERR_UNSPECIFIED;
    }
  }
  return r;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_IsWriteable
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is writeable.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket writable
*    == 0  O.K., socket not writable yet
*/
static int _SYS_SOCKET_IsWriteable(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  SOCKET Sock;
  struct timeval tv;
  fd_set wfds;
  fd_set efds;
  int v;
  //
  //
  // Two possible cases when opening a TCP connection:
  // 1) On the other side, there is a server listening on the destination port
  // 2) On the other side, there is NO server listening on the destination port
  //
  // Reg 1)
  //   In such cases, the socket is reported non-writeable for a few [us] up to a few [s] depending on if this is a localhost, LAN or internet connection
  //   After that period, the socket is reported as writeable
  //
  // Reg 2)
  //   In such cases, the following happens (S = Server):
  //   C -> S: SYN
  //   S -> C: RST ACK
  //   This means, the server has rejected the connection and closed it
  //   In such a case, we need to close the socket and try connecting again
  //
  // However, Windows is difficult regarding 2)...
  // Usually an RST means "it's over, you can close the socket. There is nobody listening"
  // Windows keeps the socket in connecting state, so IsReadable() IsWriteable() simply returns 0 instead of 1 (which would allow a following send/receive to return with error)
  // In the background, Windows performs the SYN sending another 3 times in intervals of 500ms before giving up and reporting an error state
  // Usually, the retransmissions are there in case we do not get an ACK from the other side and therefore must assume that the packet got lost
  // Then also the interval time is doubled for each retransmission
  // However, Microsoft decided to also retry it in case of RST ACK but without increasing the interval time
  // This is explained here: https://support.microsoft.com/en-in/help/175523/info-winsock-tcp-connection-performance-to-unused-ports
  //
  // Unfortunately, even after this 1.5 seconds, Windows not necessarily reports the socket as readable/writable but instead returns "exceptional state" on select()
  // Therefore, we also pass the "exceptional state" structure to select() to catch this case
  //
  Sock = (SOCKET)hSocket;
  FD_ZERO(&wfds);       // Zero init file descriptor list
  FD_SET(Sock, &wfds);  // Add socket to file descriptor list to be monitored by select()
  FD_ZERO(&efds);
  FD_SET(Sock, &efds);
  tv.tv_sec = (long)(TimeoutMs / 1000);
  tv.tv_usec = (TimeoutMs % 1000) * 1000;
  v = select(0, NULL, &wfds, &efds, &tv);   // > 0: in case of success, == 0: Timeout, < 0: Error
  return v;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_IsWriteable
*
*  Function description
*    Checks if a socket that has been connected with _SYS_SOCKET_Connect() is writeable.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    == 1  O.K., socket writeable
*    == 0  O.K., socket not writeable yet
*/
static int _SYS_SOCKET_IsWriteable(_SYS_SOCKET_HANDLE hSocket, int TimeoutMs) {
  _SYS_SOCKET_HANDLE Sock;
  struct timeval tv;
  fd_set wfds;
  int v;
  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  FD_ZERO(&wfds);       // Zero init file descriptor list
  FD_SET(Sock, &wfds);  // Add socket to file descriptor list to be monitored by select()
  tv.tv_sec = (long)(TimeoutMs / 1000);
  tv.tv_usec = (TimeoutMs % 1000) * 1000;
  v = select((hSocket + 1), NULL, &wfds, NULL, &tv);   // > 0: in case of success, == 0: Timeout, < 0: Error
  return v;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       SYS_SOCKET_Connect
*
*  Function description
*    Connects a given IPv4 socket.
*
*  Parameters
*    hSocket  Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*    IPAddr   IPv4 address expected in little endian form, meaning 127.0.0.1 is expected as 0x7F000001
*    Port     Port to connect to
*
*  Notes
*    (1) Returns immediately in case the socket has been configured as non-blocking. In this case, SYS_SOCKET_IsReady() needs to be called afterwards to make sure that the socket is ready
*/
int _SYS_SOCKET_Connect(_SYS_SOCKET_HANDLE hSocket, unsigned IPAddr, unsigned Port) {
  _SYS_SOCKET_HANDLE Sock;
  struct sockaddr_in addr;
  int r;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  addr.sin_family                = AF_INET;
  addr.sin_port                  = htons((unsigned short)Port);
  *(unsigned int*)&addr.sin_addr = htonl(IPAddr);
  r = connect(Sock, (struct sockaddr *)&addr, sizeof(addr));
  return r;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_Send
*
*  Function description
*    Sends data on the specified socket
*
*  Parameters
*    hSocket          Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    >= 0:  O.K., number of bytes sent
*     < 0:  Error, see _SYS_SOCKET_ERR_*
*/
static int _SYS_SOCKET_Send(_SYS_SOCKET_HANDLE hSocket, const void* pData, unsigned NumBytes) {
  int r;
  int Err;
  SOCKET Sock;

  Sock = (SOCKET)hSocket;
  r = send(Sock, pData, NumBytes, 0);
  if (r == SOCKET_ERROR) {
    Err = WSAGetLastError();
    r = (Err == WSAEWOULDBLOCK) ? _SYS_SOCKET_ERR_WOULDBLOCK : _SYS_SOCKET_ERR_UNSPECIFIED;
  }
  return r;
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_Send
*
*  Function description
*    Sends data on the specified socket
*
*  Parameters
*    hSocket          Handle to socket that has been returned by _SYS_SOCKET_OpenTCP() / _SYS_SOCKET_OpenUDP()
*
*  Return value
*    >= 0:  O.K., number of bytes sent
*     < 0:  Error, see _SYS_SOCKET_ERR_*
*/
static int _SYS_SOCKET_Send(_SYS_SOCKET_HANDLE hSocket, const void* pData, unsigned NumBytes) {
  int r;
  int Err;
  _SYS_SOCKET_HANDLE Sock;

  Sock = (_SYS_SOCKET_HANDLE)hSocket;
  //
  // Sending without MSG_NOSIGNAL can cause a SIGPIPE to be sent if the
  // connection was closed by peer and our process would be killed.
  //
  r = send(Sock, pData, NumBytes, MSG_NOSIGNAL);
  if (r < 0) {
    Err = errno;
    r = (Err == EWOULDBLOCK) ? _SYS_SOCKET_ERR_WOULDBLOCK : _SYS_SOCKET_ERR_UNSPECIFIED;
  }
  return r;
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_SetNonBlocking
*
*/
void _SYS_SOCKET_SetNonBlocking(_SYS_SOCKET_HANDLE hSocket) {
  int on = 1;
  ioctlsocket(hSocket, FIONBIO, &on);
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_SetNonBlocking
*
*/
void _SYS_SOCKET_SetNonBlocking(_SYS_SOCKET_HANDLE hSocket) {
  int on = 1;
  ioctl(hSocket, FIONBIO, &on);
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_SetBlocking
*
*/
void _SYS_SOCKET_SetBlocking(_SYS_SOCKET_HANDLE hSocket) {
  int on = 0;
  ioctlsocket(hSocket, FIONBIO, &on);
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_SetBlocking
*
*/
void _SYS_SOCKET_SetBlocking(_SYS_SOCKET_HANDLE hSocket) {
  int on = 0;
  ioctl(hSocket, FIONBIO, &on);
}
#endif

#ifdef __linux__
/*********************************************************************
*
*       _SYS_SOCKET_EnableKeepalive
*
*/
void _SYS_SOCKET_EnableKeepalive(_SYS_SOCKET_HANDLE hSocket) {
  int on = 1;
  setsockopt(hSocket, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(int));
}
#endif

#ifdef _WIN32
/*********************************************************************
*
*       _SYS_SOCKET_EnableKeepalive
*
*/
void _SYS_SOCKET_EnableKeepalive(_SYS_SOCKET_HANDLE hSocket) {
  int on = 1;
  setsockopt(hSocket, SOL_SOCKET, SO_KEEPALIVE, (char*)&on, sizeof(int));
}
#endif

/*********************************************************************
*
*       T32_RTTCB_Dump()
*
*/
void T32_RTTCB_Dump(unsigned int address) {
  unsigned char _sName[16] = { 0 };

  Log_Print("\n====================================T32 RTTCB Dump Start\n");
  Log_Print("acID                  = 0x%08X\n", T32_GetWord(RTTCB_OFFSET_ACID(address)));
  T32_strcpy(_sName, (char *)RTTCB_OFFSET_ACID(address));
  Log_Print("acID                  = %s\n", _sName);

  int MaxNumUpBuffers = T32_GetWord(RTTCB_OFFSET_MAXNUMUPBUFFERS(address));
  Log_Print("MaxNumUpBuffers       = %d\n", MaxNumUpBuffers);

  int MaxNumDownBuffers = T32_GetWord(RTTCB_OFFSET_MAXNUMDOWNBUFFERS(address));
  Log_Print("MaxNumDownBuffers     = %d\n", MaxNumDownBuffers);

  for (size_t i = 0; i < MaxNumUpBuffers; i++) {
    unsigned int pnameaddr;
    pnameaddr = T32_GetWord(RTTCB_OFFSET_AUP_SNAME(address, i));
    if (pnameaddr != 0) {
      Log_Print("aUp[%d].sName          = 0x%08X\n", i, pnameaddr);
      T32_strcpy(_sName, (char *)pnameaddr);
      Log_Print("aUp[%d].sName          = %s\n", i, _sName);
    }
    Log_Print("aUp[%d].pBuffer        = 0x%08X\n", i, T32_GetWord(RTTCB_OFFSET_AUP_PBUFFER(address, i)));
    Log_Print("aUp[%d].SizeOfBuffer   = %d\n",     i, T32_GetWord(RTTCB_OFFSET_AUP_SIZEOFBUFFER(address, i)));
    Log_Print("aUp[%d].WrOff          = %d\n",     i, T32_GetWord(RTTCB_OFFSET_AUP_WROFF(address, i)));
    Log_Print("aUp[%d].RdOff          = %d\n",     i, T32_GetWord(RTTCB_OFFSET_AUP_RDOFF(address, i)));
    Log_Print("aUp[%d].Flags          = %d\n",     i, T32_GetWord(RTTCB_OFFSET_AUP_FLAGS(address, i)));

    pnameaddr = T32_GetWord(RTTCB_OFFSET_ADOWN_SNAME(address, MaxNumUpBuffers, i));
    if (pnameaddr != 0) {
      Log_Print("aDown[%d].sName        = 0x%08X\n", i, pnameaddr);
      T32_strcpy(_sName, (char *)pnameaddr);
      Log_Print("aDown[%d].sName        = %s\n", i, _sName);
    }
    Log_Print("aDown[%d].pBuffer      = 0x%08X\n", i, T32_GetWord(RTTCB_OFFSET_ADOWN_PBUFFER(address, MaxNumUpBuffers, i)));
    Log_Print("aDown[%d].SizeOfBuffer = %d\n",     i, T32_GetWord(RTTCB_OFFSET_ADOWN_SIZEOFBUFFER(address, MaxNumUpBuffers, i)));
    Log_Print("aDown[%d].WrOff        = %d\n",     i, T32_GetWord(RTTCB_OFFSET_ADOWN_WROFF(address, MaxNumUpBuffers, i)));
    Log_Print("aDown[%d].RdOff        = %d\n",     i, T32_GetWord(RTTCB_OFFSET_ADOWN_RDOFF(address, MaxNumUpBuffers, i)));
    Log_Print("aDown[%d].Flags        = %d\n",     i, T32_GetWord(RTTCB_OFFSET_ADOWN_FLAGS(address, MaxNumUpBuffers, i)));
  }
  Log_Print("====================================T32 RTTCB Dump End\n\n");
}

/*********************************************************************
*
*       public code
*
**********************************************************************
*/

static char * const _TELNET_RTT         = "TELNET-RTT";
static char * const _TELNET_RTT_VERSION = "0.0.1";

/*********************************************************************
*
*       usage
*
*/
static void usage(void) {
  printf("Usage:\n");
  printf("  telnet-rtt [OPTION] SUB-COMMAND [OPTION]\n");
  printf("\n");
  printf("Global options:\n");
  printf("  --help\n");
  printf("  --version\n");
  printf("\n");
  printf("Sub-commands:\n");
  printf("=============\n");
  printf("--node\n");
  printf("--------\n");
  printf("  telnet-rtt --node [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <node>\n");
  printf("      Defines on which host the TRACE32 display driver runs. Default is localhost.\n");
  printf("\n");
  printf("--packlen\n");
  printf("--------\n");
  printf("  telnet-rtt --packlen [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <package length>\n");
  printf("      Specifies the maximum data package length used for UDP. The value must not\n");
  printf("      be bigger than 1024 and must fit to the value defined in the config.t32 file.\n");
  printf("      No operation for TCP.\n");
  printf("\n");
  printf("--tport\n");
  printf("--------\n");
  printf("  telnet-rtt --tport [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <port number>\n");
  printf("      Defines the TCP/UDP port. Be sure that these settings fit to the RCL settings\n");
  printf("      in the config.t32 file.\n");
  printf("\n");
  printf("--lport\n");
  printf("--------\n");
  printf("  telnet-rtt --lport [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <port number>\n");
  printf("      Defines the TCP port. Be sure that these settings fit to the SecureCRT settings \n");
  printf("\n");
  printf("--cmm\n");
  printf("--------\n");
  printf("  telnet-rtt --cmm [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <file path>\n");
  printf("      trace32 script file path\n");
  printf("      This parameter is not required if running the CMM script manually\n");
  printf("\n");
  printf("--record\n");
  printf("--------\n");
  printf("  telnet-rtt --record [OPTION]\n");
  printf("\n");
  printf("  Options:\n");
  printf("    <file path>\n");
  printf("      log file path\n");
  printf("      This parameter is not required if not need log file\n");
  printf("\n");
  printf("telnet-rtt cmd author <wenshuaisong@gmail.com>\n");
  printf("\n");
}

static const struct option options[] = {
  {"help"   , no_argument      , NULL, 'h'},
  {"version", no_argument      , NULL, 'v'},
  {"node"   , required_argument, NULL, 'n'},
  {"packlen", required_argument, NULL, 'k'},
  {"tport"  , required_argument, NULL, 't'},
  {"lport"  , required_argument, NULL, 'l'},
  {"cmm"    , required_argument, NULL, 'c'},
  {"record" , required_argument, NULL, 'r'},
  {NULL     , 0                , NULL,  0 }
};

static char *optstring = "hvw:t:c:e:a:";

/*********************************************************************
*
*       main
*
*/
int main(int argc, char* argv[]) {
  int                opt         =  0;
  int                lindex      = -1;

  char              *Node        = NULL;
  char              *PackLen     = NULL;
  char              *tPort       = NULL;
  char              *lPort       = NULL;
  char              *cmmFile     = NULL;
  char              *logFile     = NULL;

  int                ChannelID   =  0;
  int                Result      =  0;
  int                NumBytes    =  0;
  unsigned int       Address     =  0;
  unsigned int       LocalPort   =  0;
  char               acBuf[2048] = {0};
  _SYS_SOCKET_HANDLE hSockListen = _SYS_SOCKET_INVALID_HANDLE;
  _SYS_SOCKET_HANDLE hSockSV     = _SYS_SOCKET_INVALID_HANDLE;

  if (argc <= 1) {
    printf("usage : telnet-rtt [OPTION] SUB-COMMAND [OPTION]. (argc <= 1)");
    return 0;
  }

  printf("/*********************************************************************\r\n");
  printf("*                                                                    *\r\n");
  printf("*                Copyright (C) 2022 SWS Inc.                         *\r\n");
  printf("*                      All rights reserved                           *\r\n");
  printf("*                                                                    *\r\n");
  printf("*           TELNET-RTT Compiled " __DATE__ " " __TIME__ "                 *\r\n");
  printf("*                                                                    *\r\n");
  printf("*        Author: songwenshuai <wenshuaisong@gmail.com>               *\r\n");
  printf("*                                                                    *\r\n");
  printf("*********************************************************************/\r\n");

  while((opt = getopt_long(argc, argv, optstring, options, &lindex)) != -1) {
    switch (opt) {
      case 'h':
        usage();
        goto Done1;
        break;
      case 'v':
        printf("%s %s", _TELNET_RTT, _TELNET_RTT_VERSION);
        goto Done1;
        break;
      case 'n':
        if(optarg == NULL) {
          printf("--node option requires an argument");
          goto Done1;
        }
        Node = optarg;
        break;
      case 'k':
        if(optarg == NULL) {
          printf("--packlen option requires an argument");
          goto Done1;
        }
        PackLen = optarg;
        break;
      case 't':
        if(optarg == NULL) {
          printf("--tport option requires an argument");
          goto Done1;
        }
        tPort = optarg;
        break;
      case 'l':
        if(optarg == NULL) {
          printf("--lport option requires an argument");
          goto Done1;
        }
        lPort = optarg;
        break;
      case 'c':
        if(optarg == NULL) {
          printf("--cmm option requires an argument");
          goto Done1;
        }
        cmmFile = optarg;
        break;
      case 'r':
        if(optarg == NULL) {
          printf("--cmm option requires an argument");
          goto Done1;
        }
        logFile = optarg;
        break;
      default:
        printf("not a valid option.");
        printf("usage : telnet-rtt [OPTION] SUB-COMMAND [OPTION].");
        goto Done1;
        break;
    }
  }

  if (Node == NULL || tPort == NULL || lPort == NULL) {
    printf("--node and --port parameters are required.");
    printf("usage : telnet-rtt [OPTION] SUB-COMMAND [OPTION].");
  }

  SIGNAL_HandlerInit();
  T32_InitDEVICD(Node, tPort, PackLen, cmmFile);

  Address   = T32_GetRTTCBAddr("_SEGGER_RTT");
  ChannelID = SEGGER_Terminal_GetChannelID();
  Log_Print("Address = 0x%08X ChannelID = %d\n", Address, ChannelID);

  //
  // Try and connect to SystemView instance
  //
  hSockListen = _SYS_SOCKET_OpenTCP();
  if (hSockListen == _SYS_SOCKET_INVALID_HANDLE) {  // Failed to open socket? => Done
    Log_Print("Failed to open socket\n");
    goto Done;
  }
  LocalPort = SEGGER_atoi(lPort);
  Result    = _SYS_SOCKET_ListenAtTCPAddr(hSockListen, _SYS_SOCKET_IP_ADDR_ANY, LocalPort, 1);
  if (Result < 0) {                                     // Failed to set socket to listening? => Done
    Log_Print("Failed to set socket to listening\n");
    goto Done;
  }
  //
  // After a succesful connection, poll RTT buffer for data
  //
  do {
    if (hSockSV > _SYS_SOCKET_INVALID_HANDLE) {
      Result = _SYS_SOCKET_IsWriteable(hSockSV, 10);
      if (Result == 0) {   // Timeout
        Log_Print("socket connection timeout .\n");
        continue;
      } else if (Result < 0) { // Error
        Log_Print("socket connection error .\n");
        _SYS_SOCKET_Close(hSockSV);
        hSockSV = _SYS_SOCKET_INVALID_HANDLE;
        continue;
      }
    } else {
      Log_Print("waiting a new socket connection.\n");
      hSockSV = _SYS_SOCKET_AcceptEx(hSockListen, RTT_IDLE_DELAY);
      if (hSockSV < 0 && hSockSV != -2 ) {
        Log_Print("Failed to waiting a new socket connection.\n");
        SYS_Sleep(1000);
        continue;
      }
      Result = _SYS_SOCKET_IsReady(hSockSV);
      if (Result != 1) {               // Failed to connect? => Try again later
        Log_Print("Failed to connect\n");
        SYS_Sleep(1000);
        continue;
      }
      _SYS_SOCKET_EnableKeepalive(hSockSV);
      _SYS_SOCKET_SetNonBlocking(hSockSV);

      _SYS_SOCKET_Send(hSockSV, telnetCmd, 9);
      _SYS_SOCKET_Receive(hSockSV, acBuf, 6);
    }

    _WaitPolling(Address, RTT_IDLE_DELAY, ChannelID);
    //
    // Connection established? => Handle communication
    // Check for data sent by SysView
    //
    Result = _SYS_SOCKET_IsReadable(hSockSV, 0);
    if (Result == 1) {                                 // Data to read from SysView available?
      Result = _SYS_SOCKET_Receive(hSockSV, acBuf, sizeof(acBuf));  // Receive all data
      if (Result <= 0) {                               // Failed to receive data? => Connection lost
        Log_Print("connect close: failed to receive data: %d\n", Result);
        _SYS_SOCKET_Close(hSockSV);
        hSockSV = _SYS_SOCKET_INVALID_HANDLE;
        continue;
      }
      NumBytes = SEGGER_RTT_WriteDownBufferNoLock(Address, ChannelID, &acBuf[0], Result);  // Write data into corresponding RTT buffer for application to read and handle accordingly
      if (logFile != NULL) {
        RTT_TelnetLogS(logFile, &acBuf[0], NumBytes);
      }

#ifdef _TELNET_RTT_DEBUG
      T32_RTTCB_Dump(Address);
      Log_Print("Result = %d, NumBytes = %d, _SYS_SOCKET_Receive (p=0x%08X)\n", Result, NumBytes, acBuf);
      SYS_Hexdump(acBuf, NumBytes, true, false);
#endif
    }
    //
    // Check for data to send to SysView
    //
    NumBytes = SEGGER_RTT_ReadUpBufferNoLock(Address, ChannelID, &acBuf[0], sizeof(acBuf));
    if (NumBytes > 0) {                               // Data to send available?
      Result = _SYS_SOCKET_Send(hSockSV, acBuf, NumBytes);  // Send data to SysView
      if (logFile != NULL) {
        RTT_TelnetLogS(logFile, &acBuf[0], Result);
      }
#ifdef _TELNET_RTT_DEBUG
      T32_RTTCB_Dump(Address);
      Log_Print("Result = %d, NumBytes = %d, SYS_SOCKET_Send (p=0x%08X)\n", Result, NumBytes, acBuf);
      SYS_Hexdump(acBuf, Result, true, false);
#endif
      if (NumBytes != Result) {                            // Failed to send data? => Connection lost
        Log_Print("connect close: failed to send data. err: %d\n", Result);
        _SYS_SOCKET_Close(hSockSV);
        hSockSV = _SYS_SOCKET_INVALID_HANDLE;
      }
    }
    SYS_Sleep(1);                                     // Sleep for some time before polling again
  } while (1);
Done:
  //
  // Clean up
  //
  if (hSockSV >= 0) {
    _SYS_SOCKET_Close(hSockSV);
  }
  if (hSockListen >= 0) {
    _SYS_SOCKET_Close(hSockListen);
  }

  SYS_ExitHandler(1);

Done1:
  Log_Print("telnet-rtt exit.\n");
  return 1;
}

/*************************** End of file ****************************/

/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 1995 - 2019 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
*       SEGGER RTT * Real Time Transfer for embedded targets         *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* SEGGER strongly recommends to not make any changes                 *
* to or modify the source code of this software in order to stay     *
* compatible with the RTT protocol and J-Link.                       *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* o Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************
---------------------------END-OF-HEADER------------------------------
File    : SEGGER_RTT.c
Purpose : Implementation of SEGGER real-time transfer (RTT) which
          allows real-time communication on targets which support
          debugger memory accesses while the CPU is running.
Revision: $Rev: 26642 $

Additional information:
          Type "int" is assumed to be 32-bits in size
          H->T    Host to target communication
          T->H    Target to host communication

          RTT channel 0 is always present and reserved for Terminal usage.
          Name is fixed to "Terminal"

          Effective buffer size: SizeOfBuffer - 1

          WrOff == RdOff:       Buffer is empty
          WrOff == (RdOff - 1): Buffer is full
          WrOff >  RdOff:       Free space includes wrap-around
          WrOff <  RdOff:       Used space includes wrap-around
          (WrOff == (SizeOfBuffer - 1)) && (RdOff == 0):  
                                Buffer full and wrap-around after next byte


----------------------------------------------------------------------
*/
