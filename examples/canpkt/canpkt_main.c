/****************************************************************************
 * net/recvfrom.c
 *
 *   Copyright (C) 2014 Gregory Nutt. All rights reserved.
 *   Author: Lazlo <dlsitzer@gmail.comg>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <netpacket/can.h>
#include <nuttx/can.h>


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: psock_create
 ****************************************************************************/

static int psock_create(void)
{
  int sd;
  struct sockaddr_can addr;
  int addrlen = sizeof(addr);

  sd = socket(AF_CAN, SOCK_RAW, CAN_RAW);
  if (sd < 0)
    {
      perror("ERROR: failed to create CAN socket");
      return -1;
    }

  /* Prepare sockaddr struct */

  addr.can_family = AF_CAN;
  addr.can_ifindex = 0;
  if (bind(sd, (const struct sockaddr *)&addr, addrlen) < 0)
    {
      perror("ERROR: binding socket failed");
      close(sd);
      return -1;
    }

  return sd;
}

/****************************************************************************
 * Name: print_buf
 ****************************************************************************/

static void print_buf(const uint8_t *buf, int len)
{
  int i;

  for (i = 0; i < len; i++)
    {
      printf("%02X", buf[i]);

      if ((i+1) % 16 == 0 || (i+1) == len)
        {
          printf("\n");
        }
      else if ((i+1) % 8 == 0)
        {
          printf("  ");
        }
      else
        {
          printf(" ");
        }
  }
}

/****************************************************************************
 * Name: canpkt_usage
 ****************************************************************************/

static void canpkt_usage(void)
{
  printf("usage: canpkt options\n");
  printf("\n");
  printf(" -a     transmit and receive\n");
  printf(" -r     receive\n");
  printf(" -t     transmit\n");
  printf(" -v     verbose\n");
  printf(" -f     test filters\n");
  printf("\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: canpkt_main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  int sd;
  int i;
  int txc;
  int rxc;
  int ret;
  uint8_t *buf;
  const int buflen = 128;
  const int canfd_on = 1;
  struct canfd_frame frame;

  const char data[8] = {0xDE, 0xAD, 0xBE, 0xAF, 0x45, 0x54};

  int opt;
  int verbose = 0;
  int filter = 0;
  int do_rx = 0;
  int do_rxtimes = 99999999;
  int do_tx = 0;
  int do_txtimes = 99999999;

  if (argc == 1)
    {
      canpkt_usage();
      return -1;
    }

  /* Parse arguments */

  while ((opt = getopt(argc, argv, "artvf")) != -1)
    {
      switch(opt)
        {
          case 'a':
            do_rx = 1;
            do_tx = 1;
            break;

          case 'r':
            do_rx = 1;
            break;

          case 't':
            do_tx = 1;
            break;

          case 'v':
            verbose = 1;
            break;

          case 'f':
            filter = 1;
            break;

          default:
            canpkt_usage();
            return -1;
      }
  }

  sd = psock_create();

  /* try to switch the socket into CAN FD mode */
  ret = setsockopt(sd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &canfd_on, sizeof(canfd_on));

  printf("setsockopt CAN_RAW_FD_FRAMES %i\r\n", ret);

  if (do_tx)
    {
      if (verbose)
        {
          printf("Testing CAN write() (%d times)\n", do_txtimes);
        }

      frame.can_id = 0x123;
      frame.len = 8;
      memcpy(frame.data, data, 12);

      for (i = 0; i < do_txtimes; i++)
        {
          #define PIN_PORTD               (3 << 8)
          #define PIN31                   (31 << 0)
          s32k1xx_gpiowrite(PIN_PORTD | PIN31, 1);
          if ((txc = write(sd, &frame, sizeof(struct can_frame))) < 0)
            {
              perror("ERROR: write failed");
              free(buf);
              close(sd);
              return -1;
            }
          else
            {
              if (verbose)
                {
                  printf("transmited %d octets\n", txc);
                  print_buf(frame.data, txc);
                }
            }
          usleep(1000);
        }

      free(buf);
    }

  if (do_rx)
    {
      if (verbose)
        {
          printf("Testing read() (%d times)\n", do_rxtimes);
        }

      if (filter)
        {
    	  struct can_filter rfilter[1];

    	  rfilter[0].can_id   = 0x0D6 | CAN_INV_FILTER;
    	  rfilter[0].can_mask = CAN_SFF_MASK;
    	  ret = setsockopt(sd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    	  printf("Enabled testing filter ret=%i\r\n", ret);
        }

      for (i = 0; i < do_rxtimes;  i++)
        {
          rxc = read(sd, &frame, sizeof(frame));
          s32k1xx_gpiowrite(PIN_PORTD | PIN31, 0);
          if (rxc < 0)
            {
              perror("ERROR: read failed");
              free(buf);
              close(sd);
              return -1;
            }
          else if (rxc == 0)
            {
              /* ignore silently */
            }
          else
            {
              if (verbose)
                {
                  printf("received %d octets\n", rxc);
                  print_buf(frame.data, rxc);
                }
            }
        }

      free(buf);
    }

  close(sd);
  return 0;
}
