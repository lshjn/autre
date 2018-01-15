/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
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
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <signal.h>
#include <pthread.h>

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int led_main(int argc, char *argv[])
#endif
{
	if(argc !=3)
	{
		printf("error: led 1/2/3/all on/off\n");
		return;
	}
	if(strcmp(argv[1],"1") == 0)
	{
		if(strcmp(argv[2],"on")== 0)
		{
			boardctl(BOARDIOC_LED1_ON, 0);
			printf("led1 on!!\n");
		}
		else if(strcmp(argv[2],"off")== 0)
		{
			boardctl(BOARDIOC_LED1_OFF, 0);
			printf("led1 off!!\n");
		}
	}
	else if(strcmp(argv[1],"2")== 0)
	{
		if(strcmp(argv[2],"on")== 0)
		{
			boardctl(BOARDIOC_LED2_ON, 0);
			printf("led2 on!!\n");
		}
		else if(strcmp(argv[2],"off")== 0)
		{
			boardctl(BOARDIOC_LED2_OFF, 0);
			printf("led2 off!!\n");
		}
	}
	else if(strcmp(argv[1],"3")== 0)
	{
		if(strcmp(argv[2],"on")== 0)
		{
			boardctl(BOARDIOC_LED3_ON, 0);
			printf("led3 on!!\n");
		}
		else if(strcmp(argv[2],"off")== 0)
		{
			boardctl(BOARDIOC_LED3_OFF, 0);
			printf("led3 off!!\n");
		}
	}
	else if(strcmp(argv[1],"all")== 0)
	{
		if(strcmp(argv[2],"on")== 0)
		{
			boardctl(BOARDIOC_LED1_ON, 0);
			boardctl(BOARDIOC_LED2_ON, 0);
			boardctl(BOARDIOC_LED3_ON, 0);
			printf("led all on!!\n");
		}
		else if(strcmp(argv[2],"off")== 0)
		{
			boardctl(BOARDIOC_LED1_OFF, 0);
			boardctl(BOARDIOC_LED2_OFF, 0);
			boardctl(BOARDIOC_LED3_OFF, 0);
			printf("led all off!!\n");
		}
	}
	
  return 0;
}
