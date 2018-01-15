/****************************************************************************
 * examples/am2320.c
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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

#include <nuttx/config.h>

#include <sys/ioctl.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <nuttx/fs/fs.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <sys/types.h>
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <strings.h>
#include <nuttx/i2c/i2c_master.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/
#define      CONFIG_Am2320_I2C_DEFFREQ		10*1000
#define      CONFIG_Am2320_I2C_ADDRESS		0xB8
static bool  g_am2320_started;

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static	int  fd_am2320;

/****************************************************************************
 * Name: am2320
 ****************************************************************************/
static int am2320(int argc, char *argv[])
{	
	uint8_t  cSendBuf[10] = {0};
	uint8_t  cRcvBuf[15]  = {0};
    struct  i2c_msg_s msg;
	struct  i2c_transfer_s xfer;
	int i = 0;
	
	//g_am2320_started = true;

  	//boardctl(AM2320_WAKEUP, 0);
  	//usleep(2*1000);
  	//boardctl(RECOVER_AM2320_I2C2_SET, 0);
	
	while(1)
	{
			boardctl(AM2320_WAKEUP, 0);

		  /////////////////////////////////////////////////////////////	
		  /*send read cmd */
		  cSendBuf[0] = 0x03;
		  cSendBuf[1] = 0x00;
		  cSendBuf[2] = 0x04;

		  msg.addr      = CONFIG_Am2320_I2C_ADDRESS;
		  msg.flags     = 0;
		  msg.buffer    = cSendBuf;
		  msg.length    = 3;

		  boardctl(RDWR_AM2320_DATA, (uintptr_t)&msg);
		  ////////////////////////////////////////////////////////////
		  usleep(2*1000);
		  /*send read timing */
		  msg.addr      = CONFIG_Am2320_I2C_ADDRESS;
		  msg.flags     = I2C_M_READ;
	      msg.buffer    = cRcvBuf;
	      msg.length    = 8;

		  boardctl(RDWR_AM2320_DATA, (uintptr_t)&msg);
		  
		  for(i=0;i<8;i++)
		  {
			  printf("buf[%d]:%2X  \n",i,cRcvBuf[i]);
		  }
		  
   		  ////////////////////////////////////////////////////////////
   		  /*
		  usleep(2100*1000);
		  //send read timing 
		  msg.addr      = CONFIG_Am2320_I2C_ADDRESS;
		  msg.flags     = I2C_M_READ;
	      msg.buffer    = cRcvBuf;
	      msg.length    = 8;

		  boardctl(RDWR_AM2320_DATA, (uintptr_t)&msg);
		  for(i=0;i<8;i++)
		  {
			  printf("buf[%d]:%2X  \n",i,cRcvBuf[i]);
		  }
		  */
		  usleep(5000*1000);
	}
	    
errout:
  g_am2320_started = false;

  printf("am2320: Terminating\n");
  return EXIT_FAILURE;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * am2320_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int am2320_main(int argc, FAR char *argv[])
#endif
{
  int ret;

  printf("am2320_main: Starting the am2320_main\n");
  if (g_am2320_started)
    {
      printf("am2320_main: am2320_main already running\n");
      return EXIT_SUCCESS;
    }

  ret = task_create("am2320i2c", CONFIG_EXAMPLES_AM2320_PRIORITY,
                    CONFIG_EXAMPLES_AM2320_STACKSIZE, am2320,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("am2320_main: ERROR: Failed to start am2320: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  printf("am2320_main: am2320 started\n");
  return EXIT_SUCCESS;
}
