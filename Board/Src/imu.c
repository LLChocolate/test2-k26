#include "imu.h"
#include "math.h"
#include "mymath.h"


#define  YAW_ERROR 0




float Kp =1.2f ;		/*��������*///0.4//200.0f
float Ki = 0.01f;		/*��������*///0.001f//1.0f
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;		/*��������ۼ�*/

static float q0 = 1.0f;	/*��Ԫ��*/
static float q1 = 0.0f;
static float q2 = 0.0f;
static float q3 = 0.0f;	
static float vecxZ, vecyZ, veczZ;	/*��������ϵ�µ�Z��������*/

//static float baseZacc = 1.0;		/*��̬Z����ٶ�*/


#define DEG2RAD		0.017453293f	/* ��ת���� ��/180 */
#define RAD2DEG		57.29578f		/* ����ת�� 180/�� */




//static float invSqrt(float x);	/*���ٿ�ƽ����*/

void imuUpdate(float gx, float gy, float gz, float ax, float ay, float az, float dt,float *roll,float *pitch,float *yaw)	/*�����ں� �����˲�*///gx gy gx��λΪ����ÿ�� ax ay azΪ��ÿ���η��� dtΪ���ڵ�λms
{
	float normalise;
	float ex, ey, ez;
	float q0s, q1s, q2s, q3s;	/*��Ԫ����ƽ��*/
	static float R11,R21;		/*����(1,1),(2,1)��*/

	float halfT =0.5f * dt;
	


	/* ĳһ��������ٶȲ�Ϊ0 */
	if((ax != 0.0f) || (ay != 0.0f) || (az != 0.0f))
	{
          
		/*��λ�����ټƲ���ֵ*/
		normalise = myInvSqrt(ax * ax + ay * ay + az * az);
		ax *= normalise;
		ay *= normalise;
		az *= normalise;

		/*���ټƶ�ȡ�ķ������������ټƷ���Ĳ�ֵ����������˼���*/
		ex = (ay * veczZ - az * vecyZ);
		ey = (az * vecxZ - ax * veczZ);
		ez = (ax * vecyZ - ay * vecxZ);
		
		/*����ۼƣ�����ֳ������*/
		exInt += Ki * ex * dt ;  
		eyInt += Ki * ey * dt ;
		ezInt += Ki * ez * dt ;
		
		/*�ò���������PI����������ƫ�����������ݶ����е�ƫ����*/
		gx += Kp * ex + exInt;
		gy += Kp * ey + eyInt;
		gz += Kp * ez + ezInt;
	}
	/* һ�׽����㷨����Ԫ���˶�ѧ���̵���ɢ����ʽ�ͻ��� */
	q0 += (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1 += (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2 += (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3 += (q0 * gz + q1 * gy - q2 * gx) * halfT;
	
	/*��λ����Ԫ��*/
	normalise = myInvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;
	/*��Ԫ����ƽ��*/
	q0s = q0 * q0;
	q1s = q1 * q1;
	q2s = q2 * q2;
	q3s = q3 * q3;
	
	R11 = q0s + q1s - q2s - q3s;	/*����(1,1)��*/
	R21 = 2 * (q1 * q2 + q0 * q3);	/*����(2,1)��*/

	/*��������ϵ�µ�Z��������*/
	vecxZ = 2 * (q1 * q3 - q0 * q2);/*����(3,1)��*/
	vecyZ = 2 * (q0 * q1 + q2 * q3);/*����(3,2)��*/
	veczZ = q0s - q1s - q2s + q3s;	/*����(3,3)��*/
	
	if (vecxZ>1) vecxZ=1;
	if (vecxZ<-1) vecxZ=-1;
	
	/*����roll pitch yaw ŷ����*/
	*pitch = -asinf(vecxZ) * RAD2DEG;//+PITCH_ERROR; 
	*roll = atan2f(vecyZ, veczZ) * RAD2DEG;//+ROLL_ERROR;
	*yaw = atan2f(R21, R11) * RAD2DEG+YAW_ERROR;
	
	//test_yaw+=	((Gyro_z-GyrooffsetX)*MPU6050G_s2000dps*0.0174)*RAD2DEG*dt;
	

  // az= ax*vecxZ + ay * vecyZ + az * veczZ - baseZacc;	/*Z����ٶ�(ȥ���������ٶ�)*/
	
	
}





//	Body_To_Earth(&earth_acc_x,&earth_acc_y,&earth_acc_z,mpu6050_Acc_x_ms2,mpu6050_Acc_y_ms2,mpu6050_Acc_z_ms2,test_yaw,test_pitch,test_roll);yaw��pitch����
void Body_To_Earth(float *Earth_x,float *Earth_y,float *Earth_z,float Body_x,float Body_y,float Body_z,float Yaw,float Roll,float Pitch)
{
	float COS_PSI;
	float COS_Theta;
	float COS_Phi;
	float SIN_PSI;	
	float SIN_Theta;
	float SIN_Phi;

	Yaw  *= DEG_TO_RAD;
	Roll *= DEG_TO_RAD;
	Pitch*= DEG_TO_RAD;
	
	COS_PSI   = cos(Yaw);
	COS_Theta = cos(Roll);
	COS_Phi   = cos(Pitch);
	SIN_PSI   = sin(Yaw);	
	SIN_Theta = sin(Roll);
	SIN_Phi	  = sin(Pitch);
	
	*Earth_x =  COS_PSI * COS_Theta * Body_x + (-COS_Phi * SIN_PSI + SIN_Phi * SIN_Theta * COS_PSI) * Body_y + ( SIN_PSI * SIN_Phi + COS_Phi * SIN_Theta * COS_PSI) * Body_z;
	*Earth_y =  COS_Theta * SIN_PSI * Body_x + ( COS_Phi * COS_PSI + SIN_Phi * SIN_Theta * SIN_PSI) * Body_y + (-COS_PSI * SIN_Phi + COS_Phi * SIN_Theta * SIN_PSI) * Body_z;
	*Earth_z = -SIN_Theta 					 * Body_x + 																 COS_Theta * SIN_Phi * Body_y + 																 COS_Theta * COS_Phi * Body_z;

};

float gravity=9.7;
float speed_change;
void acc_measure_upspeed(void)
{

//speed_change=(earth_acc_z-gravity)*0.002*1000;
//if((speed_change<3)&&(speed_change>-3))
//	speed_change=0;
//	
//up_speed=up_speed+speed_change;



}




///****��ѹ�����׻����˲����������ο���Դ�ɿ�APM****/
//#define TIME_CONTANST_ZER 2.5f
//#define K_ACC_ZER 	    (1.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER * TIME_CONTANST_ZER))
//#define K_VEL_ZER	        (3.0f / (TIME_CONTANST_ZER * TIME_CONTANST_ZER))														
//#define K_POS_ZER         (2.0f / TIME_CONTANST_ZER)
//#define Delay_Cnt 60
//void Altitude_Update(u16 Time_Ms,float ACC_Earth_Z,float ultr_get_cm)
//{
//	static float Origion_Acc_z;
//	static float acc_correction = 0;
//	static float vel_correction = 0;
//	static float pos_correction = 0;
//	static float Last_Acc_z = 0;
//	static float Origion_Speed_z = 0;
//	static float Origion_Position_z = 0;
//	static float SpeedDealt = 0;
//	static float Altitude_Dealt=0;
//	static float Altitude_Estimate=0;
//	static float Altitude_History[Delay_Cnt+1];
//	static float Delta_T;
//	u16 Cnt=0;
//	
//	static uint16_t Save_Cnt=0;
//	Save_Cnt++;//���ݴ洢����
//	
//	Delta_T = Time_Ms/1000.0;//msתs ��λ s
//	Altitude_Estimate = ultr_get_cm;//�߶ȹ۲��� mתcm ��λcm
//	Origion_Acc_z = ACC_Earth_Z;//���ٶ�  ��λcm/s2
//	
//	//�ɹ۲�������ѹ�ƣ��õ�״̬���
//	Altitude_Dealt = Altitude_Estimate - hight_now;//��ѹ��(������)��SINS�������Ĳ��λcm
//	//��·���ַ����������ߵ�
//	acc_correction +=Altitude_Dealt* K_ACC_ZER*Delta_T ;//���ٶȽ�����
//	vel_correction +=Altitude_Dealt* K_VEL_ZER*Delta_T ;//�ٶȽ�����
//	pos_correction +=Altitude_Dealt* K_POS_ZER*Delta_T ;//λ�ý�����
//	//���ٶȼƽ��������
//	Last_Acc_z = up_acc;//��һ�μ��ٶ���
//	up_acc = Origion_Acc_z + acc_correction;// ���ٶȵ�λcm/s2
//	//�ٶ�������������£����ڸ���λ��,���ڲ���h=0.005,��Խϳ���
//	//������ö����������������΢�ַ��̣��������ø��߽׶Σ���Ϊ���ٶ��źŷ�ƽ��
//	SpeedDealt = (Last_Acc_z + up_acc) * Delta_T/2.0;
//	//ԭʼλ�ø���
//	Origion_Position_z += (up_speed + 0.5*SpeedDealt) * Delta_T;
//	//λ�ý��������
//	hight_now = Origion_Position_z + pos_correction;    
//	//ԭʼ�ٶȸ���
//	Origion_Speed_z += SpeedDealt;
//	//�ٶȽ��������
//	up_speed = Origion_Speed_z + vel_correction;

//	if(Save_Cnt>=1)
//	{
//		for(Cnt = Delay_Cnt;Cnt > 0;Cnt--)//����
//		{
//			Altitude_History[Cnt]=Altitude_History[Cnt-1];
//		}
//		Altitude_History[0] = hight_now;
//		Save_Cnt=0;
//	}
//}





