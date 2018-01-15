/****************************************************************************
 * configs/nucleo-l476rg/src/stm32l4_appinit.c
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

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>
//add by liushuhe 2017.09.13
#include "nucleo-l476rg.h"
//add by liushuhe 2017.09.13
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mtd/mtd.h>
#include "stm32l4_i2c.h"
#include "stm32l476rg_gpioint.h"
#include <nuttx/drivers/drivers.h>

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

/****************************************************************************
 * Public Functions 
 ****************************************************************************/
//add by liushuhe 2017.09.01
#define LED_DRIVER_PATH  "/dev/userleds"


/****************************************************************************
 * Private Data
 ***************************************************************************/
//add by liushuhe 2017.09.15
#if defined(CONFIG_STM32L4_I2C1)
struct i2c_master_s* i2c1;
#endif
#if defined(CONFIG_STM32L4_I2C2)
struct i2c_master_s* i2c2;
#endif
#if defined(CONFIG_STM32L4_I2C3)
struct i2c_master_s* i2c3;
#endif


int stm32l476rg_at24c08_automount(struct i2c_master_s *i2c);

/****************************************************************************
 * Name: up_netinitialize
 *
 * Description:
 *   Dummy function expected to start-up logic.
 *
 ****************************************************************************/

#ifdef CONFIG_WL_CC3000
void up_netinitialize(void)
{
}
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initalization logic and the
 *         matching application logic.  The value cold be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *rtclower;
#endif
#ifdef CONFIG_SENSORS_QENCODER
  int index;
  char buf[9];
#endif
  int ret;

  (void)ret;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

//add by liushuhe 2017.11.30
#if defined(CONFIG_STM32L4_SPI1) || defined(CONFIG_STM32L4_SPI2) || defined(CONFIG_STM32L4_SPI3)
  stm32l4_spiinitialize();
#endif

//add by liushuhe 2017.10.09
//注册gpio设备，外部中断设备
	stm32l4_gpiodev_initialize();

  //add by liushuhe 2017.08.31
  /*
  ret = userled_lower_initialize(LED_DRIVER_PATH);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: userled_lower_initialize() failed: %d\n", ret);
    } 
  */

//add by liushuhe 2017.09.15

#if defined(CONFIG_I2C)
  /* Configure I2C */
				stm32l4_configgpio(GPIO_I2C_PWR);
				stm32l4_gpiowrite(GPIO_I2C_PWR,false);

  /* REVISIT: this is ugly! */

//现在用的是adc1
#if defined(CONFIG_STM32L4_I2C1)
  i2c1 = stm32l4_i2cbus_initialize(1);
#endif
#if defined(CONFIG_STM32L4_I2C2)
  i2c2 = stm32l4_i2cbus_initialize(2);
#endif

#if defined(CONFIG_STM32L4_I2C3)
  i2c3 = stm32l4_i2cbus_initialize(3);
#endif

#if defined(CONFIG_STM32L4_I2C1)
  i2c_register(i2c1, 1);
#endif
#if defined(CONFIG_STM32L4_I2C2)
  i2c_register(i2c2, 2);
#endif
#if defined(CONFIG_STM32L4_I2C3)
  i2c_register(i2c3, 3);
#endif
#endif /* CONFIG_I2C */

//add at24c08 driver ,add by liushuhe 2017.11.16
if(SUCCESS == stm32l476rg_at24c08_automount(i2c1))
{
	printf("at24c512 init ok\n");
}
else
{
	printf("at24c512 init fail\n");
}

//watch
//add by liushuhe 2017.10.25
#ifdef CONFIG_STM32L4_IWDG
  /* Initialize the watchdog timer */

  stm32l4_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_STM32L476_WDG
  /* Start WDG kicker thread */

  ret = STM32L476_watchdog_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR, "Failed to start watchdog thread: %d\n", ret);
      return ret;
    }
#endif


#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */
       
	  //add by liushue  2017.09.04
      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }

	}

#endif

#ifdef HAVE_MMCSD
  /* First, get an instance of the SDIO interface */

  g_sdio = sdio_initialize(CONFIG_NSH_MMCSDSLOTNO);
  if (!g_sdio)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize SDIO slot %d\n",
             CONFIG_NSH_MMCSDSLOTNO);
      return -ENODEV;
    }

  /* Now bind the SDIO interface to the MMC/SD driver */

  ret = mmcsd_slotinitialize(CONFIG_NSH_MMCSDMINOR, g_sdio);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to bind SDIO to the MMC/SD driver: %d\n",
             ret);
      return ret;
    }

  /* Then let's guess and say that there is a card in the slot. There is no
   * card detect GPIO.
   */

  sdio_mediachange(g_sdio, true);

  syslog(LOG_INFO, "[boot] Initialized SDIO\n");
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32l4_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_pwm_setup() failed: %d\n", ret);
    }
#endif

/***************************************************************************************/
  /*mtd  at24c512 add by liushuhe 2017.09.13*/

	//ret = progmem_initialize();
	if (ret < 0)
	{
		syslog(LOG_ERR, "ERROR: my_at24config() failed: %d\n", ret);
	}

	//ret = my_at24config();
	if (ret < 0)
	{
		syslog(LOG_ERR, "ERROR: my_at24config() failed: %d\n", ret);
	}



#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */
  /*
  //add by liushuhe 2017.09.08
  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
  */
#ifdef  CONFIG_STM32L4_ADC1
  //add by liushuhe 2017.09.08
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc1_setup failed: %d\n", ret);
    }  
#endif
#ifdef  CONFIG_STM32L4_ADC2
  ret = stm32_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_adc2_setup failed: %d\n", ret);
    }  
#endif

  
#endif


#ifdef CONFIG_AJOYSTICK
  /* Initialize and register the joystick driver */

  ret = board_ajoy_initialize();
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the joystick driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_TIMER
  /* Initialize and register the timer driver */

  ret = board_timer_driver_initialize("/dev/timer0", 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the timer driver: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER

  /* Initialize and register the qencoder driver */

  index = 0;

#ifdef CONFIG_STM32L4_TIM1_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 1);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM2_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 2);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM3_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 3);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM4_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 4);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM5_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 5);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#ifdef CONFIG_STM32L4_TIM8_QE
  sprintf(buf, "/dev/qe%d", index++);
  ret = stm32l4_qencoder_initialize(buf, 8);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

#endif



  UNUSED(ret);
  return OK;
}

//stm32l476rg_at24c08_automount
//add by liushuhe 2017.10.21
int stm32l476rg_at24c08_automount(struct i2c_master_s *i2c)
{
  FAR struct mtd_dev_s *mtd;
  static bool initialized = false;
  char blockdev[18];
  char chardev[12];
  int ret;

  /* Have we already initialized? */

  if (!initialized)
    {
      /* Now bind the I2C interface to the AT24 I2C EEPROM driver */

      mtd = at24c_initialize(i2c);
      if (!mtd)
        {
          syslog(LOG_ERR,
                 "ERROR: Failed to bind i2c%d to the AT24 EEPROM driver\n",AT24_BUS);
          return -ENODEV;
        }

	  
      /* And finally, use the FTL layer to wrap the MTD driver as a block driver */
      ret = ftl_initialize(AT24_MINOR, mtd);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to initialize the FTL layer: %d\n", ret);
          return ret;
        }
	  
      // Use the minor number to create device paths//

      snprintf(blockdev, 18, "/dev/mtdblock%d", AT24_MINOR);
      snprintf(chardev, 12, "/dev/mtd%d", AT24_MINOR);

      // Now create a character device on the block device //

      ret = bchdev_register(blockdev, chardev, false);
      if (ret < 0)
        {
          ferr("ERROR: bchdev_register %s failed: %d\n", chardev, ret);
          return ret;
        }
	 
      /* Now we are initialized */
      initialized = true;
    }

  return SUCCESS;
}


#ifdef CONFIG_BOARDCTL_IOCTL
int board_ioctl(unsigned int cmd, uintptr_t arg)
{
	switch(cmd)
	{
		/*********************************************************/
		
		//add by liushuhe  2017.11.16		
		//led		
		case BOARDIOC_LED1_ON:
				stm32l4_configgpio(GPIO_LED_1);
				stm32l4_gpiowrite(GPIO_LED_1,true);
				break;
		case BOARDIOC_LED1_OFF:
				stm32l4_configgpio(GPIO_LED_1);
				stm32l4_gpiowrite(GPIO_LED_1,false);
				break;
		case BOARDIOC_LED2_ON:
				stm32l4_configgpio(GPIO_LED_2);
				stm32l4_gpiowrite(GPIO_LED_2,true);
				break;
		case BOARDIOC_LED2_OFF:
				stm32l4_configgpio(GPIO_LED_2);
				stm32l4_gpiowrite(GPIO_LED_2,false);
				break;
		case BOARDIOC_LED3_ON:
				stm32l4_configgpio(GPIO_LED_3);
				stm32l4_gpiowrite(GPIO_LED_3,true);
				break;
		case BOARDIOC_LED3_OFF:
				stm32l4_configgpio(GPIO_LED_3);
				stm32l4_gpiowrite(GPIO_LED_3,false);
				break;	
		
		case BOARDIOC_I2C_PWRON:
				stm32l4_configgpio(GPIO_I2C_PWR);
				stm32l4_gpiowrite(GPIO_I2C_PWR,true);
				break;
		case BOARDIOC_I2C_PWROFF:
				stm32l4_configgpio(GPIO_I2C_PWR);
				stm32l4_gpiowrite(GPIO_I2C_PWR,false);
				break;
		case BOARDIOC_VREF_D3V_PWRON:
				stm32l4_configgpio(GPIO_VREF_D3V_PWR);
				stm32l4_gpiowrite(GPIO_VREF_D3V_PWR,true);
				break;
		case BOARDIOC_VREF_D3V_PWROFF:
				stm32l4_configgpio(GPIO_VREF_D3V_PWR);
				stm32l4_gpiowrite(GPIO_VREF_D3V_PWR,false);
				break;
		case BOARDIOC_433_A3V_PWRON:
				stm32l4_configgpio(GPIO_433_A3V_PWR);
				stm32l4_gpiowrite(GPIO_433_A3V_PWR,true);
				break;
		case BOARDIOC_433_A3V_PWROFF:
				stm32l4_configgpio(GPIO_433_A3V_PWR);
				stm32l4_gpiowrite(GPIO_433_A3V_PWR,false);
				break;	
		case BOARDIOC_BRG_CPU_PWRON:
				stm32l4_configgpio(GPIO_BRG_CPU_PWR);
				stm32l4_gpiowrite(GPIO_BRG_CPU_PWR,true);
				break;
		case BOARDIOC_BRG_CPU_PWROFF:
				stm32l4_configgpio(GPIO_BRG_CPU_PWR);
				stm32l4_gpiowrite(GPIO_BRG_CPU_PWR,false);
				break;
		
		//debug
		//add by liushuhe 2017.11.31
		case BOARDIOC_A12_ON:
				stm32l4_configgpio(BOARDIOC_A12);
				stm32l4_gpiowrite(BOARDIOC_A12,true);
				break;
		case BOARDIOC_A12_OFF:
				stm32l4_configgpio(BOARDIOC_A12);
				stm32l4_gpiowrite(BOARDIOC_A12,false);
				break;
		case BOARDIOC_A8_ON:
				stm32l4_configgpio(BOARDIOC_A8);
				stm32l4_gpiowrite(BOARDIOC_A8,true);
				break;
		case BOARDIOC_A8_OFF:
				stm32l4_configgpio(BOARDIOC_A8);
				stm32l4_gpiowrite(BOARDIOC_A8,false);
				break;
		case BOARDIOC_A11_ON:
				stm32l4_configgpio(BOARDIOC_A11);
				stm32l4_gpiowrite(BOARDIOC_A11,true);
				break;
		case BOARDIOC_A11_OFF:
				stm32l4_configgpio(BOARDIOC_A11);
				stm32l4_gpiowrite(BOARDIOC_A11,false);
				break;
		case BOARDIOC_B2_ON:
				stm32l4_configgpio(BOARDIOC_B2);
				stm32l4_gpiowrite(BOARDIOC_B2,true);
				break;
		case BOARDIOC_B2_OFF:
				stm32l4_configgpio(BOARDIOC_B2);
				stm32l4_gpiowrite(BOARDIOC_B2,false);
				break;
		case BOARDIOC_B10_ON:
				stm32l4_configgpio(BOARDIOC_B10);
				stm32l4_gpiowrite(BOARDIOC_B10,true);
				break;
		case BOARDIOC_B10_OFF:
				stm32l4_configgpio(BOARDIOC_B10);
				stm32l4_gpiowrite(BOARDIOC_B10,false);
				break;

		case BOARDIOC_B8_ON:
				stm32l4_configgpio(BOARDIOC_B8);
				stm32l4_gpiowrite(BOARDIOC_B8,true);
				break;
		case BOARDIOC_B8_OFF:
				stm32l4_configgpio(BOARDIOC_B8);
				stm32l4_gpiowrite(BOARDIOC_B8,false);
				break;

		case BOARDIOC_B9_ON:
				stm32l4_configgpio(BOARDIOC_B9);
				stm32l4_gpiowrite(BOARDIOC_B9,true);
				break;
		case BOARDIOC_B9_OFF:
				stm32l4_configgpio(BOARDIOC_B9);
				stm32l4_gpiowrite(BOARDIOC_B9,false);
				break;
		case BOARDIOC_B1_ON:
				stm32l4_configgpio(BOARDIOC_B1);
				stm32l4_gpiowrite(BOARDIOC_B1,true);
				break;
		case BOARDIOC_B1_OFF:
				stm32l4_configgpio(BOARDIOC_B1);
				stm32l4_gpiowrite(BOARDIOC_B1,false);
				break;
		case BOARDIOC_B5_ON:
				stm32l4_configgpio(BOARDIOC_B5);
				stm32l4_gpiowrite(BOARDIOC_B5,true);
				break;
		case BOARDIOC_B5_OFF:
				stm32l4_configgpio(BOARDIOC_B5);
				stm32l4_gpiowrite(BOARDIOC_B5,false);
				break;
		case BOARDIOC_B11_ON:
				stm32l4_configgpio(BOARDIOC_B11);
				stm32l4_gpiowrite(BOARDIOC_B11,true);
				break;
		case BOARDIOC_B11_OFF:
				stm32l4_configgpio(BOARDIOC_B11);
				stm32l4_gpiowrite(BOARDIOC_B11,false);
				break;

		case BOARDIOC_C5_ON:
				stm32l4_configgpio(BOARDIOC_C5);
				stm32l4_gpiowrite(BOARDIOC_C5,true);
				break;
		case BOARDIOC_C5_OFF:
				stm32l4_configgpio(BOARDIOC_C5);
				stm32l4_gpiowrite(BOARDIOC_C5,false);
				break;

		case BOARDIOC_C12_ON:
				stm32l4_configgpio(BOARDIOC_C12);
				stm32l4_gpiowrite(BOARDIOC_C12,true);
				break;
		case BOARDIOC_C12_OFF:
				stm32l4_configgpio(BOARDIOC_C12);
				stm32l4_gpiowrite(BOARDIOC_C12,false);
				break;
		case BOARDIOC_C8_ON:
				stm32l4_configgpio(BOARDIOC_C8);
				stm32l4_gpiowrite(BOARDIOC_C8,true);
				break;
		case BOARDIOC_C8_OFF:
				stm32l4_configgpio(BOARDIOC_C8);
				stm32l4_gpiowrite(BOARDIOC_C8,false);
				break;
		case BOARDIOC_C6_ON:
				stm32l4_configgpio(BOARDIOC_C6);
				stm32l4_gpiowrite(BOARDIOC_C6,true);
				break;
		case BOARDIOC_C6_OFF:
				stm32l4_configgpio(BOARDIOC_C6);
				stm32l4_gpiowrite(BOARDIOC_C6,false);
				break;
		case BOARDIOC_C9_ON:
				stm32l4_configgpio(BOARDIOC_C9);
				stm32l4_gpiowrite(BOARDIOC_C9,true);
				break;
		case BOARDIOC_C9_OFF:
				stm32l4_configgpio(BOARDIOC_C9);
				stm32l4_gpiowrite(BOARDIOC_C9,false);
				break;
		case BOARDIOC_D2_ON:
				stm32l4_configgpio(BOARDIOC_D2);
				stm32l4_gpiowrite(BOARDIOC_D2,true);
				break;
		case BOARDIOC_D2_OFF:
				stm32l4_configgpio(BOARDIOC_D2);
				stm32l4_gpiowrite(BOARDIOC_D2,false);
				break;
		

		
		//debug
		//add by liushuhe 2017.11.31
		case qBOARDIOC_A12_ON:
				stm32l4_configgpio(qBOARDIOC_A12);
				break;
		case qBOARDIOC_A8_ON:
				stm32l4_configgpio(qBOARDIOC_A8);
				break;
		case qBOARDIOC_A11_ON:
				stm32l4_configgpio(qBOARDIOC_A11);
				break;
		case qBOARDIOC_B2_ON:
				stm32l4_configgpio(qBOARDIOC_B2);
				break;
		case qBOARDIOC_B10_ON:
				stm32l4_configgpio(qBOARDIOC_B10);
				break;
		case qBOARDIOC_B8_ON:
				stm32l4_configgpio(qBOARDIOC_B8);
				break;
		case qBOARDIOC_B9_ON:
				stm32l4_configgpio(qBOARDIOC_B9);
				break;
		case qBOARDIOC_B1_ON:
				stm32l4_configgpio(qBOARDIOC_B1);
				break;
		case qBOARDIOC_B5_ON:
				stm32l4_configgpio(qBOARDIOC_B5);
				break;
		case qBOARDIOC_B11_ON:
				stm32l4_configgpio(qBOARDIOC_B11);
				break;
		case qBOARDIOC_C5_ON:
				stm32l4_configgpio(qBOARDIOC_C5);
				break;
		case qBOARDIOC_C12_ON:
				stm32l4_configgpio(qBOARDIOC_C12);
				break;
		case qBOARDIOC_C8_ON:
				stm32l4_configgpio(qBOARDIOC_C8);
				break;
		case qBOARDIOC_C6_ON:
				stm32l4_configgpio(qBOARDIOC_C6);
				break;
		case qBOARDIOC_C9_ON:
				stm32l4_configgpio(qBOARDIOC_C9);
				break;
		case qBOARDIOC_D2_ON:
				stm32l4_configgpio(qBOARDIOC_D2);
				break;	
		//add by liushuhe  2017.11.16		
		//led		
		case qBOARDIOC_LED1_ON:
				stm32l4_configgpio(qGPIO_LED_1);
				break;
		case qBOARDIOC_LED2_ON:
				stm32l4_configgpio(qGPIO_LED_2);
				break;
		case qBOARDIOC_LED3_ON:
				stm32l4_configgpio(qGPIO_LED_3);
				break;		
		/*********************************************************/
		default:
				return -ENOTTY;
				break;
	}
	return OK;
  
}
#endif

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif


