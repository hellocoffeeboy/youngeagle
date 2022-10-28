#include "filter.h"

float angle = 0;      //最优估计值，同时也代表k-1时刻的最优估计值
float gyro_bias = 0;  //惯导偏差量，随机误差
float dt = 0.005; //卡尔曼滤波计算时间
float P[2][2] = { { 0 , 0 } , { 0 , 0 } }; //初始化为单位矩阵，之后会更新
float Q_angle = 0.002;    //陀螺仪角度自身误差
float R_angle = 0.03;     //陀螺仪测量角度误差
float Q_gyro  = 0.002;    //陀螺仪角加速度自身误差
float Measurements;       //测量值
short H = 1;              //测量系统参数
short Vk = 0;             //测量系统误差
float K0;                 //状态量1的卡尔曼增益
float K1;                 //状态量2的卡尔曼增益

float Kalman_Filter(float newangle,short newgyro)
{
	angle = angle + (newgyro - gyro_bias) * dt;       //第一步：预测当前状态 X(k|k-1) = A * X(k-1|k-1) + B * U(k-1) 
	
	P[0][0] = P[0][0] - (P[1][0] + P[0][1]) * dt + Q_angle;
	P[0][1] = P[0][1] - P[1][1] * dt;
	P[1][0] = P[0][1] - P[1][1] * dt;
	P[1][1] = P[1][1] + Q_gyro;                       //第二步：预测协方差矩阵 P(k|k-1) = A * P(k-1|k-1) * AT + Q;
	
	Measurements = H * newangle + Vk;                 //第三步：测量方程 Z(k) = H * X(k) + V(k);
	
	K0 = P[0][0] / (P[0][0] + R_angle);
	K1 = P[1][0] / (P[0][0] + R_angle);               //第四步：计算卡尔曼增益 Kg = (P(k|k-1) * HT) / (H * P(k|k-1) * HT + R)
	
	angle = angle + K0 * (newangle - angle);
	gyro_bias = gyro_bias + K1 * (newangle - angle);  //第五步：计算最优估计值 X(k|k) = X(k|k-1) + Kg * (Z(k) - H * X(k|k-1))
	
	P[0][0] = P[0][0] - K0 * P[0][0];
	P[0][1] = P[0][1] - K0 * P[0][1];
	P[1][0] = P[1][0] - K0 * P[1][0];
	P[1][1] = P[1][1] - K0 * P[1][1];                 //第六步：更新协方差矩阵 P(k|k) = (I - Kg * H) * P(k|k-1);
	
	return angle;
}
