#ifndef _DEFINE_H_
#define _DEFINE_H_

#include  "common.h"
/*************************************************************************
*  ģ�����ƣ�defineģ��
*  ����˵����Include �û��Զ���ĺ�
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-14
*************************************************************************/

//#define LCD_DISPLAY

//��·״̬��־
#define Speed_filter_Period       5//��ֵ�˲�����

typedef enum
{
  Nstart,//δ��ʼ״̬
  Straight,//��ֱ��
  Left_turn,//��ת
  Right_turn,//��ת
  Cross,//ʮ��
  Uphill,//�µ�
  Left_Island,//�󻷵�
  Right_Island//�һ���
  
}Road_Status;
 
typedef enum
{
  up   =1,
  left =2,
  right=4,
  down =8 
}Edge_Str_DIR;
typedef struct{
  int x;
  int y;
}Pixel;


typedef struct{
  int x;
  int y1;
  int y2;
}Edge_Str;

//������
#define MOTOR_1                         TPM0,TPM_CH0//����ӿ�
#define MOTOR_2                         TPM0,TPM_CH4//����ӿ�
//#define STEER_                          FTM1,CH0//����ӿ�
#define MOTOR1_DIR                       PTA6_OUT//����������
#define MOTOR2_DIR                       PTA7_OUT//����������
typedef struct{
  u8 Dir;
  s16 Speed;
}Motor_Status;          //���״̬�ṹ��




//ADC

#define MYADC_1                          ADC0_SE1
#define MYADC_2                          ADC0_DM1
#define MYADC_3                          ADC0_SE2
#define MYADC_4                          ADC0_DM2

//����
#define SW1              PTB9_IN
#define SW2              PTB10_IN
#define SW3              PTB11_IN
#define SW4              PTB16_IN
#define SW5              PTB17_IN
#define SW6              PTB18_IN
#define SW7              PTB19_IN
#define SW8              PTB20_IN

//����
#define KEY1             PTB23_IN
#define KEY1_PRES        1
#define KEY2             PTB22_IN
#define KEY2_PRES        2
#define KEY3             PTC0_IN
#define KEY3_PRES        3   

//������
#define BEEP PTE21_OUT

//��ˮ��
#define LED1             PTC5_OUT
#define LED2             PTC6_OUT
#define LED3             PTC7_OUT
#define LED4             PTC8_OUT

//PID�ṹ��
typedef struct                    //�ṹ�壬���PID��ر���
{
  float P;                        //����P
  float I;                        //����I
  float D;                        //����D
  float error[3];                 //���洢����
  float delat;                    //ÿ�εĵ��ڽ��
  float derr;                     //һ�����
  float dderr;                    //�������
  float result;                      //PID�����������������ʽ�����Գ�ֵ����Ϊ����ƽ��ʱ�����ֵ����Ҫ��������0Ҫ����ը��
  
  float target;                   //PID���ڵ�Ŀ��ֵ     
  float feedback;
  float UP_Limit;
  float LOW_Limit;
}PID_Struct;

typedef struct 
{
  float x_mid;
  float x_now;
  float p_mid ;
  float p_now;
  float kg;
  float ProcessNoise_Q;
  float MeasureNoise_R;
  float x_last1;
  float p_last1;
}Kalman_Date;

typedef struct
{
  float m_filter;
  float ResrcData_mem[2];
  float output_mem[2];
}Filter_1st_Str;


//typedef struct
//{
//  float Kp;		/*��������*///0.4//200.0f
//  float Ki;		/*��������*///0.001f//1.0f
//  float exInt;
//  float eyInt;
//  float ezInt;		/*��������ۼ�*/
//  float Q4[4];
//}


////ϵͳ״̬�ṹ��
//typedef struct
//{
//  long int Time_1ms;
//  
//}System_Status

#define     pit_delay_ms(PITn,ms)          pit_delay(PITn,ms * bus_clk_khz);        //PIT��ʱ ms
#define     pit_delay_us(PITn,us)          pit_delay(PITn,us * bus_clk_khz/1000);   //PIT��ʱ us

typedef enum{
  Unlock,
  Left_Lock,
  Right_Lock
}Black_View_Run_Status;

#endif