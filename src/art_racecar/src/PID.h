
#ifndef PID_H
#define	PID_H

/***********************************/
typedef struct PID
{              //结构体定义
    double SetPoint;         //设定值
    double Proportion;         // Proportion 比例系数
    double  Integral;            // Integral   积分系数
    double  Derivative;          // Derivative  微分系数
    double  LastError;          // Error[-1]  前一拍误差
    double  PreError;           // Error[-2]  前两拍误差


}PID;


void PIDInit (struct PID *pp);
double PIDCal(struct PID *pp, double ThisError);




#endif


