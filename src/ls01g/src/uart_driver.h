

#ifndef  __UART_DRIVER_H
#define  __UART_DRIVER_H

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <sstream>


#include <semaphore.h>
#include <pthread.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <errno.h>
#include <malloc.h>
#include <termios.h>
#include "math.h"
#include <stdbool.h>
#include <sys/time.h>


#define PACKSIZE 1812
#define PACKLEN (PACKSIZE/5-2)

#define LSLIDAR_CMD_BYTE               0xa5
#define LSLIDAR_CMD_START              0x2c
#define LSLIDAR_CMD_STOP               0x25
#define LSLIDAR_CMD_SCAN               0x20
#define LSLIDAR_CMD_END                0xd1

#define LSLIDAR_CMD_RESET              0x40
#define LSLIDAR_CMD_RESET_END          0xe5
#define LSLIDAR_CMD_STOPSCAN           0x21
#define LSLIDAR_CMD_STOPSCAN_END       0xc6

enum Command
{
	STOP_DATA = 1, STOP_MOTOR = 2, START_MOTOR_AND_SCAN = 4
};

struct ture_data{
	int ture;
	int curr;
	unsigned char data[1024];
};


struct wifides {
    int start;
    int end;
    int flag;
    int packsize;
    int packcurr;
    unsigned char buf[1024];
};

struct basedata {
   
     int flag;
     int start;
     int end;
     int curr;
     unsigned char data[PACKSIZE];
     struct basedata *next;
};

#pragma pack(1)
typedef struct _rplidar_response_measurement_node_t {
    unsigned char    sync_quality;      
    unsigned short   angle_q6_checkbit; //角度
    unsigned short   distance_q2;       //距离
}rplidar_response_measurement_node_t;

struct scanDot {
    unsigned char   quality;
    float angle;
    float dist;
};


class  io_driver
{

public:

    int OpenSerial(const char*, unsigned int baudrate);//it means fail if return -1
    int StartScan();
    int GetScanData( double *angle, double *distance, int len, double *speed);
    int Reset(void);
    int StopScan(Command cmd);
    void CloseSerial();
};

#endif
