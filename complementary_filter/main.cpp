#define VERSION 4

#include "complementary_filter.h"
#include <algorithm>
#include<stdio.h>
#include<wchar.h>
#include<stdlib.h>
#include<time.h>
#include<io.h>
#include <string> 
#include <iostream> 
#include <fstream>
#include <time.h> 
#include <math.h>
using namespace std;
using namespace imu_tools;

#define _EULER_GT 1 //data中是否有输出的角度

#define MAX_LINE 50000
#define IMU_RATE 100

#define GYROX_BIAS 0
#define GYROY_BIAS 0
#define GYROZ_BIAS 0


void quaternion2Euler(double q0, double q1, double q2, double q3, double* _pitch, double* _roll, double* _yaw)
{
    *_pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3;
    *_roll  = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3;
    *_yaw   = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;
}


void f(string inputfile,string outputfile)
{
 
	FILE *fp = NULL;
	char line[MAX_LINE];
 
	
	ofstream outedit(outputfile);
 
	if((fp = fopen(inputfile.c_str(),"r")) != NULL) //string 类为我们提供了一个转换函数 c_str()，
    //该函数能够将 string 字符串转换为C风格的字符串，并返回该字符串的 const 指针（const char*）
	{
		char delims[] = ",";
		string time;
		string device;
		double accX,accY,accZ,gyroX,gyroY,gyroZ,gyroXrad,gyroYrad,gyroZrad;
		double gyroXgt,gyroYgt,gyroZgt;
		double dlta_time = 1.0 / IMU_RATE;
		// double q0 = 0,q1 = 0,q2=0,q3=0;
		double pitch = 0,roll = 0,yaw = 0;
		double wx_bias = 0,wy_bias = 0,wz_bias = 0;
		double q0,q1,q2,q3;

		ComplementaryFilter filter;

		fgets(line, MAX_LINE, fp);//先把第一行读掉
		outedit<<"时间"<<','<<"pitch"<<','<<"roll"<<','<<"yaw"<<','<<"gyroX_bias"<<','<<"gyroY_bias"<<','<<"gyroZ_bias"<<','<<endl;//输出表格的第一行定义

		while (fgets(line, MAX_LINE, fp))
		{
			//strtok函数的作用是把字符串以规定的字符分割开
			//atoi()函数将数字格式的字符串转换为整数类型。
			time = strtok( line, delims );
			// device = strtok( NULL, delims );
			accX= atof(strtok( NULL, delims ))*9.81;		
			accY= atof(strtok( NULL, delims ))*9.81;
			accZ= atof(strtok( NULL, delims ))*9.81;
			gyroX = atof(strtok( NULL, delims )) - GYROX_BIAS;
			gyroY = atof(strtok( NULL, delims )) - GYROY_BIAS;
			gyroZ = atof(strtok( NULL, delims )) - GYROZ_BIAS;
			gyroXrad = (gyroX / 180)*3.1415926;
			gyroYrad = (gyroY / 180)*3.1415926;
			gyroZrad = (gyroZ / 180)*3.1415926;

			#if _EULER_GT
			gyroXgt = atof(strtok( NULL, delims ));
			gyroYgt = atof(strtok( NULL, delims ));
			gyroZgt = atof(strtok( NULL, delims ));
			#endif

			filter.update(accX,accY,accZ,gyroXrad,gyroYrad,gyroZrad,dlta_time);

			filter.getOrientation(q0,q1,q2,q3);

			quaternion2Euler(q0,q1,q2,q3,&pitch,&roll,&yaw);

            outedit<<time<<','<<pitch<<','<<roll<<','<<yaw<<','<<wx_bias<<','<<wy_bias<<','<<wz_bias;

			#if _EULER_GT
			outedit<<','<<gyroXgt<<','<<gyroYgt<<','<<gyroZgt;
			#endif
			
			outedit<<endl;

		}
		fclose(fp);
		fp = NULL;
	}
}

 
int main(int argc, char* argv[])
{
	string inputfile;
	string outputfile;
	inputfile="../data/WT931_100HZ_dynamic2.csv";
	outputfile="../result/CF/"+inputfile.substr(8,strlen(inputfile.c_str())-12)+to_string(VERSION)+".csv";
	f(inputfile,outputfile);
	system("pause");
	return 0;
}