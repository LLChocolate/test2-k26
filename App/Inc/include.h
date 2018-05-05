#ifndef __INCLUDE_H__
#define __INCLUDE_H__

#include  "common.h"
#include  "define.h"
/*
 * Include �û��Զ����ͷ�ļ�
 */
#include  "MKL_BME.h"           //λ����
#include  "MKL_wdog.h"          //���Ź�
#include  "MKL_gpio.h"          //IO�ڲ���
#include  "MKL_uart.h"          //����
#include  "MKL_SysTick.h"       //�δ�ʱ��
#include  "MKL_lptmr.h"         //�͹��Ķ�ʱ��(��ʱ�������������ʱ����ʱ)
#include  "MKL_i2c.h"           //I2C
#include  "MKL_spi.h"           //SPI
#include  "MKL_tpm.h"           //TPM������K60�� FTM ��pwm�����������
#include  "MKL_pit.h"           //PIT
#include  "MKL_adc.h"           //ADC
#include  "MKL_dac.h"           //DAC
#include  "MKL_dma.h"           //DMA
#include  "MKL_FLASH.h"         //FLASH

#include  "init.h"
#include  "isr.h"
#include  "math.h"
#include  "mymath.h"
#include  "delay.h"
#include  "led.h"
#include  "mykey.h"
#include  "beep.h"
#include  "dial_switch.h"
#include  "myPWM.h"
#include  "getspeed.h"
#include  "lcd.h"
#include  "pid.h"
#include  "OV7725.h"
#include  "OV7725_REG.h"
#include  "SCCB.h"
#include  "image_process.h"
#include  "filter.h"
#include  "imu.h"
#include  "mpu6050.h"
#include  "mpuiic.h"

extern u16 Brush_Color; 
extern u16 Back_Color;

extern Motor_Status Motor1;//���״̬�ṹ��
extern Motor_Status Motor2;//���״̬�ṹ��
extern s32 Duty_Motor1;//���ռ�ձ�
extern s32 Duty_Motor2;//���ռ�ձ�
extern PID_Struct Motor1_PID;//pid�ṹ��
extern PID_Struct Motor2_PID;//pid�ṹ��
extern PID_Struct Diff_PID;//����PID
extern float Speed_goal1;//���ת��Ŀ��ֵ
extern float Speed_goal2;//���ת��Ŀ��ֵ
extern u16 Diff_goal;
extern int Diff_error;
extern u16 Master_Speed;
extern u16 ADC_Value;//ADCֵ
extern u16 Speed_stand;
extern int Weight_mean;
extern float stand_p;
extern float stand_d;
//***************************��ʱ����־λ*************************
extern u8 button_timeout;//����ʱ���־
extern long int Time_1ms;//ʱ����
//****************************************************************

extern u8 Road_Status_Flag;//��·��״�жϱ�־
extern u8 Island_Flag_Lock;
extern u16 Switch_Status;//����״̬
extern u8 Key_status;//����״̬
extern u8 LCD_DISPLAY_FLAG;
extern u8 Image_Flag;
extern u8 Black_Lock;
extern int DIFF_UP;
extern int DIFF_DOWN;
//***********************************����ͷ**********************************
extern u8 Memory_use_Flag;
extern u8 Image_fire_Memory1[CAMERA_H+1][CAMERA_DMA_NUM];
extern u8 Image_fire_Memory2[CAMERA_H+1][CAMERA_DMA_NUM];
extern u8 (*Image_fire)[CAMERA_DMA_NUM];//ָ��
//extern u8 Image_fire_extract[CAMERA_H+1][CAMERA_W];
extern volatile u8 img_flag;		//ͼ��״̬
extern int centre[ALL_LINE];
extern u8 halfwidth[ALL_LINE];
//extern int black_L[ALL_LINE];
//extern int black_R[ALL_LINE];
//extern u16 Right_count[ALL_LINE];
//extern u16 Left_count[ALL_LINE];
extern u8 getLeft_flag[ALL_LINE];
extern u8 getRight_flag[ALL_LINE];
extern u8 LED_timeout;
extern u16 hang;
extern u8 DIFF_PID_CHANGE_FLAG;
extern long int  temp_cnt;
extern u16 temp_CNT_WATCH;
extern u8 Three_lie_end[3];
extern u8 Speed_max_to_min_diff;
extern u8 Motor_enable_Flag;
extern u8 Slow_Flag;
extern u8 Reduct_Flag;
extern u8 Blue_Start_Flag;//��������
extern u8 Key_Start_Flag;
extern u8 Acc_Limit;
extern float Acc_K;
extern int Speed_L_sum;
extern int Speed_R_sum;
//extern u16 TPM2_Cnt;   
//extern u16 TPM1_Cnt;
extern u8 key_up;

extern float stand_half_k;
extern u16 Island_Delay_ms;
extern u8 Three_lie_end_jump_flag[3];
extern u8 Island_Stay_Flag;
extern long Servo_out;
extern u8 AD_double_flag;
extern u32 L_AD_Ave;
extern u32 R_AD_Ave;
//extern u8 Temp_L_DIR;
//extern u8 Temp_R_DIR;
extern u16 stand_AD;
extern u16 num;
extern u8 tanzhen;
extern PID_Struct Gyro_PID;//����PID
extern int Gyro_Up,Gyro_Down;
extern float Cur_error;
extern u16 stand_AD_L;
extern u16 stand_AD_R;
extern u16 stand_AD;
extern u16 Island_in_delay;
extern u16 Island_out_delay;
extern u8 Island_in_delay_flag;
extern u8 Island_out_flag;
extern u8 out_island_test_flag;
extern u8 out_Island_flag;
extern u8 Island_doublt_flag;
extern u8 island_flag_temp;
extern u8 road_filter_flag;
extern u8 Island_In_Flag;
extern u8 Far_correct_flag;
extern u8 Far_Diff;
extern u8 Cross_correct_flag;
extern int Cross_correct_value;
extern u8 Three_Lie[3];
extern u8 Angel_Find_Flag;
extern const u16 Next_Island_delay_const;
extern u16  Next_Island_delay;
extern u8   Next_Island_Flag;
extern int Diff_PID_ave;
#endif  //__INCLUDE_H__
