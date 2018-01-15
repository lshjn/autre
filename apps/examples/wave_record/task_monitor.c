#include <sys/ioctl.h>
#include <strings.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <sched.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>

#include <nuttx/analog/adc.h>
#include <nuttx/analog/ioctl.h>
#include <nuttx/leds/userled.h>
#include <nuttx/board.h>
#include <sys/boardctl.h>

#include <nuttx/board.h>
#include <sys/boardctl.h>
#include <errno.h>
#include <nuttx/ioexpander/gpio.h>

#include "task_monitor.h"
#include "task_adc.h"

/*****************************************************************************************************************************
 * Private Data
 ****************************************************************************************************************************/


struct  	TimeStruct  DisLocalTime;
struct		tm 			*tmp;
struct 		rtc_time	rtctime;
time_t		timelocal;

/****************************************************************************
 * getSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
void  getSystime(void)
{
	time(&timelocal);	
	tmp = localtime(&timelocal);  //��ȡ����ʱ��


	DisLocalTime.Year		=	1900+tmp->tm_year;
	DisLocalTime.Month		=	tmp->tm_mon;
	DisLocalTime.Day		=	tmp->tm_mday;
	DisLocalTime.Hour		=	tmp->tm_hour;
	DisLocalTime.Minute	=	tmp->tm_min;
	DisLocalTime.Second	=	tmp->tm_sec;
}
/****************************************************************************
 * setSystime
 * liushuhe
 * 2017.10.11
 ****************************************************************************/
int setSystime(struct rtc_time *rtc)
{
	//struct rtc_time rtc;  
	struct tm _tm;  
	struct timeval tv;  
	time_t timep;  

	_tm.tm_sec 	= rtc->tm_sec;  
	_tm.tm_min 	= rtc->tm_min;  
	_tm.tm_hour = rtc->tm_hour;  
	_tm.tm_mday = rtc->tm_mday;  
	_tm.tm_mon 	= rtc->tm_mon;  
	_tm.tm_year = rtc->tm_year - 1900;  

	timep = mktime(&_tm);  
	tv.tv_sec = timep;  
	tv.tv_usec = 0;  
	if(settimeofday (&tv, (struct timezone *) 0) < 0)  
	{  
		printf("Set system datatime error!/n");  
		return -1;  
	}  
	return 0;  

}
/****************************************************************************
 * master_monitor
 * liushuhe
 * 2017.09.28
 ****************************************************************************/
int master_monitor(int argc, char *argv[])
{	
	sleep(5);                                     //sleep 100ms
	while(1)
	{
		sleep(5);                                     //sleep 100ms
	/*************************************************************************/
		//����һ��adc�ɼ�
		//EnAdcSampl(&g_AdcConVar,&g_AdcMutex);
		if(1==SensorDate.sampleisok)
		{
			SensorDate.sampleisok	= 0;
			/*************************************************************************/

			printf("%s: value: %d \n", "sample_tempdata[0]", SensorDate.sample_tempdata[0].am_data);
			printf("%s: value: %d \n", "sample_tempdata[1]", SensorDate.sample_tempdata[1].am_data);
			printf("%s: value: %d \n", "sample_tempdata[2]", SensorDate.sample_tempdata[2].am_data);
			printf("%s: value: %d \n", "sample_tempdata[3]", SensorDate.sample_tempdata[3].am_data);
			printf("%s: value: %d \n", "sample_tempdata[4]", SensorDate.sample_tempdata[4].am_data);
			printf("%s: value: %d \n", "sample_tempdata[5]", SensorDate.sample_tempdata[5].am_data);
			printf("%s: value: %d \n", "sample_tempdata[6]", SensorDate.sample_tempdata[6].am_data);
			printf("%s: value: %d \n", "sample_tempdata[7]", SensorDate.sample_tempdata[7].am_data);
			printf("%s: value: %d \n", "sample_tempdata[8]", SensorDate.sample_tempdata[8].am_data);
			/*
			//VCC
			printf("%s: value: %d v\n", "SensorDate.adcmsg.VCC", SensorDate.adcmsg->VCC);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.O2", SensorDate.adcmsg->O2);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.NH3", SensorDate.adcmsg->NH3);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.H2S", SensorDate.adcmsg->H2S);
			printf("%s: value: %d v\n", "SensorDate.adcmsg.CO", SensorDate.adcmsg->CO);
			printf("%s: value: %d m\n", "SensorDate.adcmsg.Water_high", SensorDate.adcmsg->Water_high);
			*/
			/*************************************************************************/
			//�����ϱ�
			//��ʱ�ɼ� 8:00 | 18:00
		}
		
	}

 return EXIT_FAILURE;

}


