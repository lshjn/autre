#ifndef _TASK_MONITOR_H
#define _TASK_MONITOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include <nuttx/timers/rtc.h>


/****************************************************************************
 * Private function
 ****************************************************************************/

extern 		struct  	TimeStruct 		DisLocalTime;
extern		struct		tm 				*tmp;
extern		time_t		timelocal;
extern		struct 		rtc_time	rtctime;

extern		char		TimeInt_SampleFlag;

/****************************************************************************
 * Private struct
 ****************************************************************************/
struct  TimeStruct
{
    unsigned int   Year;          //��Rtc
    unsigned char  Month;         //��
    unsigned char  Day;           //��
    unsigned char  Hour;          //ʱ
    unsigned char  Minute;        //��
    unsigned char  Second;        //��
    unsigned long  NTPSecond;
};
/****************************************************************************
 * Private function
 ****************************************************************************/
int   master_monitor(int argc, char *argv[]);
void  getSystime(void);
int   setSystime(struct rtc_time *rtc);




int master_monitor(int argc, char *argv[]);




#ifdef __cplusplus
}
#endif

#endif 
