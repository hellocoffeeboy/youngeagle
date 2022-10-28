#include "filter.h"

float angle = 0;      //���Ź���ֵ��ͬʱҲ����k-1ʱ�̵����Ź���ֵ
float gyro_bias = 0;  //�ߵ�ƫ������������
float dt = 0.005; //�������˲�����ʱ��
float P[2][2] = { { 0 , 0 } , { 0 , 0 } }; //��ʼ��Ϊ��λ����֮������
float Q_angle = 0.002;    //�����ǽǶ��������
float R_angle = 0.03;     //�����ǲ����Ƕ����
float Q_gyro  = 0.002;    //�����ǽǼ��ٶ��������
float Measurements;       //����ֵ
short H = 1;              //����ϵͳ����
short Vk = 0;             //����ϵͳ���
float K0;                 //״̬��1�Ŀ���������
float K1;                 //״̬��2�Ŀ���������

float Kalman_Filter(float newangle,short newgyro)
{
	angle = angle + (newgyro - gyro_bias) * dt;       //��һ����Ԥ�⵱ǰ״̬ X(k|k-1) = A * X(k-1|k-1) + B * U(k-1) 
	
	P[0][0] = P[0][0] - (P[1][0] + P[0][1]) * dt + Q_angle;
	P[0][1] = P[0][1] - P[1][1] * dt;
	P[1][0] = P[0][1] - P[1][1] * dt;
	P[1][1] = P[1][1] + Q_gyro;                       //�ڶ�����Ԥ��Э������� P(k|k-1) = A * P(k-1|k-1) * AT + Q;
	
	Measurements = H * newangle + Vk;                 //���������������� Z(k) = H * X(k) + V(k);
	
	K0 = P[0][0] / (P[0][0] + R_angle);
	K1 = P[1][0] / (P[0][0] + R_angle);               //���Ĳ������㿨�������� Kg = (P(k|k-1) * HT) / (H * P(k|k-1) * HT + R)
	
	angle = angle + K0 * (newangle - angle);
	gyro_bias = gyro_bias + K1 * (newangle - angle);  //���岽���������Ź���ֵ X(k|k) = X(k|k-1) + Kg * (Z(k) - H * X(k|k-1))
	
	P[0][0] = P[0][0] - K0 * P[0][0];
	P[0][1] = P[0][1] - K0 * P[0][1];
	P[1][0] = P[1][0] - K0 * P[1][0];
	P[1][1] = P[1][1] - K0 * P[1][1];                 //������������Э������� P(k|k) = (I - Kg * H) * P(k|k-1);
	
	return angle;
}
