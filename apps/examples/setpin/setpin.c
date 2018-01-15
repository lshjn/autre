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
int setpin_main(int argc, char *argv[])
#endif
{
	if(argc !=4)
	{
		printf("error: set a/b/c/b <0-15> on/off\n");
		return;
	}
	if(strcmp(argv[1],"a") == 0)
	{
		if(strcmp(argv[2],"12")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_A12_ON, 0);
				printf("a12 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_A12_OFF, 0);
				printf("a12 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"8")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_A8_ON, 0);
				printf("a8 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_A8_OFF, 0);
				printf("a8 on!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"11")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_A11_ON, 0);
				printf("a11 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_A11_OFF, 0);
				printf("a11 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else
		{
			printf("no support!!!\n");
		}
	}
	else if(strcmp(argv[1],"b")== 0)
	{
		if(strcmp(argv[2],"2")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B2_ON, 0);
				printf("b2 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B2_OFF, 0);
				printf("b2 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"10")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B10_ON, 0);
				printf("b10 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B10_OFF, 0);
				printf("b10 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"8")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B8_ON, 0);
				printf("b8 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B8_OFF, 0);
				printf("b8 on!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"9")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B9_ON, 0);
				printf("b9 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B9_OFF, 0);
				printf("b9 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"1")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B1_ON, 0);
				printf("b1 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B1_OFF, 0);
				printf("b1 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"5")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B1_ON, 0);
				printf("b1 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B1_OFF, 0);
				printf("b1 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"11")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_B1_ON, 0);
				printf("b1 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_B1_OFF, 0);
				printf("b1 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else
		{
			printf("no support!!!\n");
		}
	}
	else if(strcmp(argv[1],"c")== 0)
	{
		if(strcmp(argv[2],"5")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_C5_ON, 0);
				printf("c5 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_C5_OFF, 0);
				printf("c5 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"12")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_C12_ON, 0);
				printf("c12 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_C12_OFF, 0);
				printf("c12 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"8")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_C8_ON, 0);
				printf("c8 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_C8_OFF, 0);
				printf("c8 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"6")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_C6_ON, 0);
				printf("c6 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_C6_OFF, 0);
				printf("c6 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else if(strcmp(argv[2],"9")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_C9_ON, 0);
				printf("c9 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_C9_OFF, 0);
				printf("c9 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else
		{
			printf("no support!!!\n");
		}
	}
	else if(strcmp(argv[1],"d")== 0)
	{
		if(strcmp(argv[2],"2")== 0)
		{
			if(strcmp(argv[3],"on")== 0)
			{
				boardctl(BOARDIOC_D2_ON, 0);
				printf("d2 on!!\n");
			}
			else if(strcmp(argv[3],"off")== 0)
			{
				boardctl(BOARDIOC_D2_OFF, 0);
				printf("d2 off!!\n");
			}
			else
			{
				printf("no support!!!\n");
			}
		}
		else
		{
			printf("no support!!!\n");
		}
	}
	
  return 0;
}
