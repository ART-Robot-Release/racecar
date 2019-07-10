#include "uart_driver.h"

#define NO_SCAN       0
#define START_SCAN    1
#define STOP_SCAN     2

static pthread_t id;
static int m_dFd;
static pthread_mutex_t g_tMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t g_tConVar = PTHREAD_COND_INITIALIZER;
static double g_angle[PACKLEN];
static double g_distance[PACKLEN];
static int creatPthread = 1;
static struct basedata *g_pcurr = NULL;
static double g_speed;
static int IS360;
static int g_start_scan = NO_SCAN;
//static struct wifides pack;

#define DEBUG  0

#if DEBUG
#define ALOGI(x...)     printf( x)
#else
#define ALOGI(x...)    
#endif

static int RestartGetData(void)
{
	int wRet;
	char wificmd0[] =
	{ 0xa5, 0x3A, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char wificmd1[] =
	{ 0xa5, 0x2C, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char wificmd2[] =
	{ 0xa5, 0x20, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	ALOGI("--------start scan----------\n");

	wRet = write(m_dFd, wificmd0, 7);
		if (wRet < 0)
			return wRet;
	usleep(5000000);
	
	wRet = write(m_dFd, wificmd1, 7);
	if (wRet < 0)
		return wRet;

	usleep(3000);
	wRet = write(m_dFd, wificmd2, 7);
	if (wRet < 0)
		return wRet;

	ALOGI("-------------end------------\n");

	return 0;
}

static int Uart_parameter(unsigned char *data, double *angle, double *dist, int len)
{
	int i, j;
	unsigned char *tmp;
	rplidar_response_measurement_node_t *curr;

	ALOGI("len = %d, data[] = %02x  %d\n", len, data[len-1], IS360);
	if (data[0] == 0xA5 && data[6] == 0x81 && data[len - 1] == 0xdd)
	{
		//pthread_mutex_lock(&g_tMutex);
		//pthread_cond_signal(&g_tConVar);
		tmp = data + 7;
		g_speed = data[1] / 15.0;
		curr = (rplidar_response_measurement_node_t *) tmp;

		for (i = 7, j = 0; i < len - 4 && j < IS360; curr++, i += 5, j++)
		{
			ALOGI("%d  ", curr->sync_quality);ALOGI("%d  ", curr->angle_q6_checkbit);
			if (IS360 == 720)
			{ //720
				if (curr->angle_q6_checkbit != (j + 1) * 5)
				{
					if (curr->angle_q6_checkbit == (j + 2) * 5)
					{
						angle[j] = (j + 1) * 5 / 10.0;
						dist[j] = 0;
						j++;
						angle[j] = curr->angle_q6_checkbit / 10.0;
						dist[j] = curr->distance_q2 / 1.0;
					}
					else
					{
						break;
					}
				}
				else
				{
					angle[j] = curr->angle_q6_checkbit / 10.0;
					ALOGI("%d  \n", curr->distance_q2);
					dist[j] = curr->distance_q2 / 1.0;
				}

			}
			else if (IS360 == 360)
			{
				angle[j] = curr->angle_q6_checkbit / 10.0;
				ALOGI("%d  \n", curr->distance_q2);
				dist[j] = curr->distance_q2 / 1.0;
			}
		}ALOGI("j= %d\n", j);
		if (j >= IS360)
		{
			pthread_mutex_lock(&g_tMutex);
			pthread_cond_signal(&g_tConVar);
			j = 0;
			pthread_mutex_unlock(&g_tMutex);
		}
		return j;
	}
	else
	{
		return 0;
	}

}

static struct basedata *creatlist(void)
{
	struct basedata *head;

	head = (struct basedata *) malloc(sizeof(struct basedata));
	if (NULL == head)
		return NULL;
	head->flag = 0;
	head->start = 0;
	head->end = 0;
	head->curr = 0;
	head->next = NULL;

	return head;
}

static struct basedata *initlist(void)
{
	struct basedata *head, *p;

	head = creatlist();
	if (NULL == head)
		return NULL;
	p = creatlist();
	if (NULL == p)
	{
		free(head);
		return NULL;
	}
	head->next = p;
	p->next = head;

	return head;
}

static int InitPackageSize()
{
	if (1812 == PACKSIZE)
	{
		IS360 = 360;
	}
	else if (3611 == PACKSIZE)
	{
		IS360 = 720;
	}
	else
	{
		return -1;
	}

	return 0;
}

static void analysis(unsigned char *buf, int nRet)
{
	unsigned char tempbuffer[2048];
	int i, j;
	int clen = 0;

	if (nRet > 0)
	{
		if (!g_pcurr->start && !g_pcurr->flag)
		{
			for (i = 0; i < nRet - 6; i++)
			{
				if (buf[i] == 0xa5 && buf[i + 6] == 0x81)
				{
					break;
				}
			}ALOGI("i0 = %d\n", i);
			if (i >= nRet - 6)
			{
				memcpy(g_pcurr->data, buf + nRet - 6, 6);
				g_pcurr->flag = 1;
				g_pcurr->curr = 6;
			}
			else
			{
				memcpy(g_pcurr->data, buf + i, nRet - i);
				g_pcurr->start = 1;
				g_pcurr->flag = 1;
				g_pcurr->curr = nRet - i;
			}
		}
		else if (!g_pcurr->start && g_pcurr->flag)
		{
			memset(tempbuffer, 0, sizeof(tempbuffer));
			memcpy(tempbuffer, g_pcurr->data, g_pcurr->curr);
			memcpy(tempbuffer + g_pcurr->curr, buf, nRet);
			clen = g_pcurr->curr + nRet;
			ALOGI("clen=%d,nRet=%d\n", clen,nRet);
			g_pcurr->start = 0;
			g_pcurr->end = 0;
			g_pcurr->flag = 0;
			g_pcurr->curr = 0;
			memset(g_pcurr->data, 0, PACKSIZE);
			for (i = 0; i < clen - 6; i++)
			{
				if (tempbuffer[i] == 0xa5 && tempbuffer[i + 6] == 0x81)
				{
					break;
				}
			}ALOGI("i1=%d\n", i);
			if (i >= clen - 6)
			{
				memcpy(g_pcurr->data, tempbuffer + clen - 6, 6);
				g_pcurr->flag = 1;
				g_pcurr->curr = 6;
			}
			else
			{
				if (clen - i < PACKSIZE)
				{
					memcpy(g_pcurr->data, tempbuffer + i, clen - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->curr = clen - i;
				}
				else if (clen - i == PACKSIZE)
				{
					memcpy(g_pcurr->data, tempbuffer + i, clen - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 1;
					g_pcurr->curr += clen - i;
				}
				else
				{
					if (tempbuffer[i + PACKSIZE] == 0xa5)
					{
						memcpy(g_pcurr->data, tempbuffer + i, PACKSIZE);
						g_pcurr->start = 1;
						g_pcurr->flag = 1;
						g_pcurr->end = 1;
						g_pcurr->curr = PACKSIZE;
						g_pcurr = g_pcurr->next;
						g_pcurr->start = 0;
						g_pcurr->flag = 0;
						g_pcurr->end = 0;
						g_pcurr->curr = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, tempbuffer + i + PACKSIZE, clen - i - PACKSIZE);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->end = 0;
						g_pcurr->curr = clen - i - PACKSIZE;
						g_pcurr = g_pcurr->next;
					}
					else
					{
						memcpy(g_pcurr->data, tempbuffer + i + 1, clen - i - 1);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->curr = clen - i - 1;
					}
				}
			}
		}
		else if (g_pcurr->start && !g_pcurr->end)
		{
			for (i = 0; i < nRet - 6; i++)
			{
				if (buf[i] == 0xa5 && buf[i + 6] == 0x81)
				{
					break;
				}
			}

			ALOGI("i2=%d,nRet=%d\n",i,nRet);
			if (i >= nRet - 6)
			{
				if (g_pcurr->curr + i < PACKSIZE)
				{
					if (g_pcurr->curr + nRet < PACKSIZE)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, nRet);
						g_pcurr->curr += nRet;
					}
					else if (g_pcurr->curr + nRet == PACKSIZE)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, nRet);
						g_pcurr->curr += nRet;
						g_pcurr->end = 1;
					}
					else
					{
						clen = PACKSIZE - g_pcurr->curr;
						if (buf[clen] == 0xa5)
						{
							memcpy(g_pcurr->data + g_pcurr->curr, buf, clen);
							g_pcurr->end = 1;
							g_pcurr->curr += clen;
							g_pcurr = g_pcurr->next;
							g_pcurr->start = 0;
							g_pcurr->end = 0;
							g_pcurr->flag = 0;
							memset(g_pcurr->data, 0, PACKSIZE);
							memcpy(g_pcurr->data, buf + clen, nRet - clen);
							g_pcurr->start = 0;
							g_pcurr->curr = nRet - clen;
							g_pcurr->end = 0;
							g_pcurr->flag = 1;
							g_pcurr = g_pcurr->next;
						}
						else
						{
							g_pcurr->start = 0;
							g_pcurr->end = 0;
							g_pcurr->flag = 0;
							memset(g_pcurr->data, 0, PACKSIZE);
							memcpy(g_pcurr->data, buf + nRet - 3, 3);
							g_pcurr->start = 0;
							g_pcurr->flag = 1;
							g_pcurr->curr = 3;
						}
					}
				} //
				else if (g_pcurr->curr + i == PACKSIZE)
				{
					if (buf[i] == 0xa5)
					{
						memcpy(g_pcurr->data + g_pcurr->curr, buf, i);
						g_pcurr->curr += i;
						g_pcurr->end = 1;
						g_pcurr = g_pcurr->next;
						g_pcurr->start = 0;
						g_pcurr->end = 0;
						g_pcurr->flag = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, buf + i, nRet - i);
						g_pcurr->start = 0; /* no start*/
						g_pcurr->flag = 1;
						g_pcurr->curr = nRet - i;
						g_pcurr = g_pcurr->next;
					}
					else
					{

						g_pcurr->start = 0;
						g_pcurr->end = 0;
						g_pcurr->flag = 0;
						memset(g_pcurr->data, 0, PACKSIZE);
						memcpy(g_pcurr->data, buf + nRet - 6, 6);
						g_pcurr->start = 0;
						g_pcurr->flag = 1;
						g_pcurr->curr = 6;
					}
				}
				else
				{             //(g_pcurr->curr+i > PACKSIZE)
					g_pcurr->start = 0;
					g_pcurr->end = 0;
					g_pcurr->flag = 0;
					memset(g_pcurr->data, 0, PACKSIZE);
					memcpy(g_pcurr->data, buf + nRet - 6, 6);
					g_pcurr->start = 0;
					g_pcurr->flag = 1;
					g_pcurr->curr = 6;

				}
			}
			else
			{
				if (g_pcurr->curr + i != PACKSIZE)
				{
					g_pcurr->start = 0;
					g_pcurr->end = 0;
					g_pcurr->flag = 0;
					memset(g_pcurr->data, 0, PACKSIZE);
					memcpy(g_pcurr->data, buf + i, nRet - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->curr = nRet - i;
				}
				else
				{
					memcpy(g_pcurr->data + g_pcurr->curr, buf, i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 1;
					g_pcurr->curr += i;
					g_pcurr = g_pcurr->next;
					memcpy(g_pcurr->data, buf + i, nRet - i);
					g_pcurr->start = 1;
					g_pcurr->flag = 1;
					g_pcurr->end = 0;
					g_pcurr->curr = nRet - i;
					g_pcurr = g_pcurr->next;
				}
			}
		}
		if (g_pcurr->start && g_pcurr->end)
		{
			//pthread_mutex_lock(&g_tMutex);
			//pthread_cond_signal(&g_tConVar);
			Uart_parameter(g_pcurr->data, g_angle, g_distance, g_pcurr->curr);
			g_pcurr->start = 0;
			g_pcurr->end = 0;
			g_pcurr->flag = 0;
			memset(g_pcurr->data, 0, PACKSIZE);
			g_pcurr = g_pcurr->next;
			//pthread_mutex_unlock(&g_tMutex);

		}
	}
}

void *Uart_creatPthread(void *data)
{

	unsigned char buf[1024];
	fd_set read_fds;
	struct timeval tm;
	int nRet;

	while (creatPthread)
	{
		FD_ZERO(&read_fds);
		FD_SET(m_dFd, &read_fds);
		tm.tv_sec = 1;
		tm.tv_usec = 0;
		nRet = select(m_dFd + 1, &read_fds, NULL, NULL, &tm);
		if (nRet < 0)
		{
			printf("select error!\n");
		}
		else if (nRet == 0)
		{
			ALOGI("select timeout!\n");
			if (START_SCAN == g_start_scan)
			{
				printf("timeout----\n");
				RestartGetData();
			}
		}
		else
		{
			if (FD_ISSET(m_dFd, &read_fds))
			{
				bzero(buf, 1024);
				nRet = read(m_dFd, buf, 1024);
				if (nRet > 0)
				{

					//ALOGI("nRet = %d\n", nRet);
					//printf("nRet = %d\n", nRet);
					analysis(buf, nRet);
					usleep(30000);

				}
			}
		}
		// usleep(30000);
	}
	return NULL;
}

int io_driver::OpenSerial(const char* port, unsigned int baudrate)
{
	int ret;
	struct termios m_stNew;
	struct termios m_stOld;

	const char* addr = port;
	const char* addr2 = port;

	m_dFd = open(addr, O_RDWR | O_NOCTTY | O_NDELAY);
	if (-1 == m_dFd)
	{
		//perror("Open Serial Port Error!\n");
		m_dFd = open(addr2, O_RDWR | O_NOCTTY | O_NDELAY);
		if (m_dFd < 0)
			return -1;
	}ALOGI("start init serial\n");
	if ((fcntl(m_dFd, F_SETFL, 0)) < 0)
	{
		perror("Fcntl F_SETFL Error!\n");
		return -1;
	}
	if (tcgetattr(m_dFd, &m_stOld) != 0)
	{
		perror("tcgetattr error!\n");
		return -1;
	}

	m_stNew = m_stOld;
	cfmakeraw(&m_stNew);		    //将终端设置为原始模式，该模式下所有的输入数据以字节为单位被处理

	//set speed
	cfsetispeed(&m_stNew, baudrate);		    //115200
	cfsetospeed(&m_stNew, baudrate);

	//set databits
	m_stNew.c_cflag |= (CLOCAL | CREAD);
	m_stNew.c_cflag &= ~CSIZE;
	m_stNew.c_cflag |= CS8;

	//set parity
	m_stNew.c_cflag &= ~PARENB;
	m_stNew.c_iflag &= ~INPCK;

	//set stopbits
	m_stNew.c_cflag &= ~CSTOPB;
	m_stNew.c_cc[VTIME] = 0;	//指定所要读取字符的最小数量
	m_stNew.c_cc[VMIN] = 1;	//指定读取第一个字符的等待时间，时间的单位为n*100ms
	//如果设置VTIME=0，则无字符输入时read（）操作无限期的阻塞
	tcflush(m_dFd, TCIFLUSH);	//清空终端未完成的输入/输出请求及数据。
	if (tcsetattr(m_dFd, TCSANOW, &m_stNew) != 0)
	{
		perror("tcsetattr Error!\n");
		return -1;
	}
	g_pcurr = initlist();
	if (NULL == g_pcurr)
		return -1;
	if (InitPackageSize())
		return -1;ALOGI("finish init seria!\n");
	return m_dFd;
}

int io_driver::StartScan(void)
{

	static int scanflags = 0;
	int wRet;
	char wificmd0[] =
	{ 0xa5, 0x3A, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char wificmd1[] =
	{ 0xa5, 0x2C, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char wificmd2[] =
	{ 0xa5, 0x20, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char wificmd3[] =
	{ 0xa5, 0x50, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	g_start_scan = START_SCAN;
	ALOGI("--------start scan----------\n");
//	if (scanflags == 0)
//	{
	wRet = write(m_dFd, wificmd0, 7);
		if (wRet < 0)
			return wRet;
	usleep(5000000);
	wRet = write(m_dFd, wificmd1, 7);
	if (wRet < 0)
		return wRet;
//	}
	usleep(30000);
	wRet = write(m_dFd, wificmd2, 7);
	if (wRet < 0)
		return wRet;

	usleep(30000);
	wRet = write(m_dFd, wificmd3, 7);
	if (wRet < 0)
		return wRet;
	ALOGI("-------------end------------\n");
	creatPthread = 1;
	if (scanflags == 0)
	{
		scanflags = 1;
		pthread_create(&id, NULL, Uart_creatPthread, NULL);
	}
	return wRet;

}

int io_driver::GetScanData(double *angle, double *distance, int len, double *speed)
{
	int min = 0;
	int i;
	unsigned char buffer[PACKSIZE];
	pthread_mutex_lock(&g_tMutex);
	pthread_cond_wait(&g_tConVar, &g_tMutex);
	min = len > PACKLEN ? PACKLEN : len;
	for (i = 0; i < min; i++)
	{
		angle[i] = g_angle[i];
		distance[i] = g_distance[i];
	}
	*speed = g_speed;
	pthread_mutex_unlock(&g_tMutex);

	return min;
}

int io_driver::Reset(void)
{
	char buf[] =
	{ 0xa5, 0x40, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	return write(m_dFd, buf, 7);
}

int io_driver::StopScan(Command cmd)
{
	//unsigned char buf[] = {LSLIDAR_CMD_BYTE, LSLIDAR_CMD_STOPSCAN, LSLIDAR_CMD_STOPSCAN_END};
	char stop_scan[] =
	{ 0xa5, 0x21, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };
	char stop_motor[] =
	{ 0xa5, 0x25, 0xe1, 0xaa, 0xbb, 0xcc, 0xdd };

	g_start_scan = STOP_SCAN;

	if (STOP_DATA == cmd)
	{
		usleep(50000);
		write(m_dFd, stop_scan, 7);
	}
	if (STOP_MOTOR == cmd)
	{
		usleep(50000);
		write(m_dFd, stop_motor, 7);
	}

	return 0;
}

void io_driver::CloseSerial(void)
{
	struct basedata *tmp;
	creatPthread = 0;
	g_start_scan = NO_SCAN;
	//sleep(2);
	pthread_join(id, NULL);

	tmp = g_pcurr;
	free(tmp);
	g_pcurr = g_pcurr->next;
	while (g_pcurr != tmp)
	{
		free(g_pcurr);
		g_pcurr = g_pcurr->next;
	}
	g_pcurr = tmp = NULL;
	close(m_dFd);
}

