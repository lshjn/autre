/****************************************************************************
 * examples/485.c
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
#include <string.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <signal.h>
#include <pthread.h>

#include <sys/types.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>

#include "task_monitor.h"
#include "task_flash.h"
#include "task_adc.h"
#include "tmp431.h"

/****************************************************************************
 * lid_master_main
 * liushuhe
 * 2017.09.26
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int wave_record_main(int argc, FAR char *argv[])
#endif
{
  int ret;
  ret = task_create("master_flash", CONFIG_EXAMPLES_FLASH_PRIORITY,
                    CONFIG_EXAMPLES_FLASH_STACKSIZE, master_flash,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("flash: ERROR: Failed to start flash: %d\n",
             errcode);
      return EXIT_FAILURE;
    }


  ret = task_create("master_monitor", CONFIG_EXAMPLES_MONITOR_PRIORITY,
                    CONFIG_EXAMPLES_MONITOR_STACKSIZE, master_monitor,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_monitor: ERROR: Failed to start monitor: %d\n",errcode);
      return EXIT_FAILURE;
    }
  if (ret < 0)
    {
      int errcode = errno;
      printf("bluetooth_main: ERROR: Failed to start bluetooth: %d\n",errcode);
      return EXIT_FAILURE;
    }
  
  ret = task_create("master_adc", CONFIG_EXAMPLES_ADC_PRIORITY,
                    CONFIG_EXAMPLES_ADC_STACKSIZE, master_adc,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_adc: ERROR: Failed to start adc: %d\n",
             errcode);
      return EXIT_FAILURE;
    }

  ret = task_create("master_tmp431", CONFIG_EXAMPLES_TMP431_PRIORITY,
                    CONFIG_EXAMPLES_TMP431_STACKSIZE, master_tmp431,
                    NULL);
  if (ret < 0)
    {
      int errcode = errno;
      printf("master_tmp431: ERROR: Failed to start tmp431: %d\n",
             errcode);
      return EXIT_FAILURE;
    }


  //open
  boardctl(BOARDIOC_I2C_PWROFF, 0);
  boardctl(BOARDIOC_433_A3V_PWROFF, 0);

  //close
  boardctl(BOARDIOC_VREF_D3V_PWROFF, 0);
  //boardctl(BOARDIOC_BRG_CPU_PWRON, 0);



/*
  boardctl(qBOARDIOC_A12_ON, 0);
  boardctl(qBOARDIOC_A8_ON, 0);
  boardctl(qBOARDIOC_A11_ON, 0);
  boardctl(qBOARDIOC_B2_ON, 0);
  boardctl(qBOARDIOC_B10_ON, 0);
  boardctl(qBOARDIOC_B8_ON, 0);
  boardctl(qBOARDIOC_B9_ON, 0);
  boardctl(qBOARDIOC_B1_ON, 0);
  boardctl(qBOARDIOC_B5_ON, 0);
  boardctl(qBOARDIOC_B11_ON, 0);
  boardctl(qBOARDIOC_C5_ON, 0);
  boardctl(qBOARDIOC_C12_ON, 0);
  boardctl(qBOARDIOC_C8_ON, 0);
  boardctl(qBOARDIOC_C6_ON, 0);
  boardctl(qBOARDIOC_C9_ON, 0);
  boardctl(qBOARDIOC_D2_ON, 0);
  
  boardctl(qBOARDIOC_LED1_ON, 0);
  boardctl(qBOARDIOC_LED2_ON, 0);
  boardctl(qBOARDIOC_LED3_ON, 0);


*/

  while(1)	
  {
		//boardctl(BOARDIOC_LED1_ON, 0);
		//boardctl(BOARDIOC_LED2_ON, 0);
		//boardctl(BOARDIOC_LED3_ON, 0);
  		usleep(2000*1000);
		boardctl(BOARDIOC_LED1_OFF, 0);
		boardctl(BOARDIOC_LED2_OFF, 0);
		boardctl(BOARDIOC_LED3_OFF, 0);
  		usleep(2000*1000);
  }

  return EXIT_SUCCESS;
}
