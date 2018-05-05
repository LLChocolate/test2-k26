/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外KL26 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-12-14
 */

#include "common.h"
#include "include.h"
#include "stdlib.h"

//math common
//signed char L_Diff_temp[70];
//signed char L_Second_Diff_temp[70];
//signed char R_Diff_temp[70];
//signed char R_Second_Diff_temp[70];

u16 Island_Delay_ms = 200;
u8 Three_lie_end_jump_flag[3] = {0};

//Pixel envelope_L[100];
//Pixel envelope_R[100];

u32 L_AD_Ave = 0;
u32 R_AD_Ave = 0;
//u16 TPM2_Cnt = 0;   
//u16 TPM1_Cnt = 0;
Motor_Status Motor1;//电机状态结构体
Motor_Status Motor2;//电机状态结构体
int Speed_L_sum=0;
int Speed_R_sum=0;
s32 Duty_Motor1;//电机占空比
s32 Duty_Motor2;//电机占空比
PID_Struct Motor1_PID;//pid结构体
PID_Struct Motor2_PID;//pid结构体
PID_Struct Diff_PID;//差速PID
PID_Struct Gyro_PID;//差速PID
PID_Struct Diff_Straight;//直线PID
float Speed_goal1=80;//电机转速目标值
float Speed_goal2=80;//电机转速目标值
u16 Diff_goal=0;
int Diff_error=0;
u16 Master_Speed=25;
u16 ADC_Value;//ADC值
u16 Speed_stand;
int Weight_mean=0;
float stand_half_k;
//***************************定时器标志位*************************
u8 button_timeout=255;//按键时间标志
u8 key_up = 0;
long int Time_1ms=0;//时间轴
//****************************************************************
u8 Road_Status_Flag=0;
u8 Island_Flag_Lock=0;
u16 Switch_Status;//拨码状态
u8 Key_status;//按键状态
u8 LCD_DISPLAY_FLAG=1;
u8 Motor_enable_Flag=1;
u8 Slow_Flag=0;
u8 Reduct_Flag=0;
u8 Blue_Start_Flag=0;//蓝牙开启
u8 Key_Start_Flag=0;
//***********************调试用临时全局变量**************************
float stand_p;
float stand_d;
float P_TEMP1=800;//201
float I_TEMP1=60;
float D_TEMP1=0;
float P_TEMP2=800;//90
float I_TEMP2=60;
float D_TEMP2=0;
signed int Speed_goal1_TEMP=80;//电机转速目标值
signed int Speed_goal2_TEMP=80;//电机转速目标值
u8 Image_Flag=1;
u8 LED_timeout=50;
u8 Dir_temp=1;
u16 hang=0;
int DIFF_UP=15000;
int DIFF_DOWN=-15000;
int Gyro_Up=20000,Gyro_Down=-20000;
long int  temp_cnt=0;
u8 DIFF_PID_CHANGE_FLAG=0;
u16 temp_CNT_WATCH=0;
u8 Speed_max_to_min_diff;
u8 Acc_Limit=40;
u16 stand_AD_L = 0xffff;
u16 stand_AD_R = 0xffff;
u16 stand_AD   = 0Xffff;
//u8 Temp_L_DIR = 0;
//u8 Temp_R_DIR = 0;
float Acc_K=1;

u8 DIR1 = 0;
u8 DIR2 = 0;
u16 num = 0;
//float error_diff_2[6];
//******************************************************************
u8 tanzhen = 0;
void main()
{
//  u8 Str_temp[40];//
  u8 Key;
//  while(1);
  System_Init();
  Brush_Color=Black;
//  Motor1_PID.P=P_TEMP1;
//  Motor1_PID.I=I_TEMP1;
//  Motor1_PID.D=D_TEMP1;
//  Motor2_PID.P=P_TEMP2;
//  Motor2_PID.I=I_TEMP2;
//  Motor2_PID.D=D_TEMP2;
  Motor1_PID.target=Speed_goal1;
  Motor2_PID.target=Speed_goal2;
  Duty_Motor1=32767;
  Duty_Motor2=32767;
//  while(1)
//  {
////    MOTOR1_DIR=0;
//    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
////    MOTOR2_DIR=0;
//    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
//  }
while(1)
{
//  Motor1_PID.P=P_TEMP1;
//  Motor1_PID.I=I_TEMP1;
//  Motor1_PID.D=D_TEMP1;
//  Motor2_PID.P=P_TEMP2;
//  Motor2_PID.I=I_TEMP2;
//  Motor2_PID.D=D_TEMP2;

//摄像头采集一次
//图像处理 
//  DIR1 = gpio_get(PTA13);
//  DIR2 = gpio_get(PTA12);
//  LED1 = LED2 = LED3 = LED4 = 0;
    Key = KEY_Scan();
    if(Key == KEY1_PRES)
    {
      Reduct_Flag=1;
//      Key_status=KEY1_PRES;
//        BEEP=1;
//      LED1 = 0;
    }
    else if(Key == KEY2_PRES)
    {
      LED2 = 0;
    }
    else if(Key == KEY3_PRES)
    {
      Key_status=KEY3_PRES;
      Blue_Start_Flag = 1;
      LED3 = !LED3;
    }
//    delayms(1000);
    if(Image_Flag==1)
    {
      tanzhen = 1;
      Image_Flag=0;
//      StoreDate();//转存数据
      ov7725_get_img();//转存结束后立刻允许接收场中断
      if(LCD_DISPLAY_FLAG==1)  
      Send_Image_to_LCD(Image_fire);
      image_process();
      if(LCD_DISPLAY_FLAG==1)
      {
        LCD_Draw_Line(Three_Lie[0],Three_lie_end[0],Three_Lie[0],160);  
        LCD_Draw_Line(Three_Lie[1],Three_lie_end[1],Three_Lie[1],160);  
        LCD_Draw_Line(Three_Lie[2],Three_lie_end[2],Three_Lie[2],160);
        
//        LCD_Draw_Line(0,90,319,90);
//        LCD_Draw_Line(0,80,319,80);
//        LCD_Draw_Line(0,70,319,70);
//        LCD_Draw_Line(0,60,319,60);
//        LCD_Draw_Line(37,0,37,240);  
//        LCD_Draw_Line(0,73,319,73);  
//        LCD_Draw_Line(0,90,319,90);  
//        LCD_Draw_Line(0,140,319,140);
//        LCD_Draw_Line(0,105,319,105);
//        LCD_Draw_Line(0,Start_Point,319,Start_Point);
//        LCD_Draw_Line(0,Far_Point,319,Far_Point);
        LCD_Draw_Line(0,AD_Near_hang,319,AD_Near_hang);
        LCD_Draw_Line(0,AD_Near_hang-15,319,AD_Near_hang-15);
//        LCD_Draw_Line(0,Three_lie_end[1]+26,319,Three_lie_end[1]+26);
//        LCD_Draw_Line(0,Three_lie_end[1]+59,319,Three_lie_end[1]+59);
        
//        LCD_Draw_Line(0,Near_Point,319,Near_Point);
      }
      temp_cnt++;
    }
    if(LCD_DISPLAY_FLAG==1)
    {
      LCD_Put_Int(210,10,"",Diff_PID.result,Red,White);
      LCD_Put_Int(240,10,"",Weight_mean,Red,White);
      LCD_Put_Int(180,10,"",Black_Lock,Red,White);
//      LCD_DrawPoint(160-90,Start_Point);
////      LCD_DrawPoint(160-60 ,Start_Point);
////      LCD_DrawPoint(160+60 ,Start_Point);
//      LCD_DrawPoint(160+90,Start_Point);
//      LCD_DrawPoint(160-65,Start_Point-20);
//      LCD_DrawPoint(160+65,Start_Point-20);
//
//      LCD_DrawPoint(160-40,Start_Point-40);
//      LCD_DrawPoint(160+40,Start_Point-40);
//      LCD_DrawPoint(160-120,Start_Point);
    }
//  Speed_goal1=Speed_goal2=Speed_stand;

//  Motor2_PID.target=Speed_goal2;
//  SCISend_to_PIDDebug(UART5);
//  SCI_Send_Datas(UART1);
//  hang++;
  if(Motor_enable_Flag==0)
  {
    Speed_goal1=0;
    Speed_goal2=0;
    if(Motor1.Speed<1&&Motor2.Speed<1)
    {
      disable_irq(PIT_IRQn); 
      MOTOR1_DIR=0;
      tpm_pwm_duty(MOTOR_1,0);
      MOTOR2_DIR=0;
      tpm_pwm_duty(MOTOR_2,0);

    }
  }
  
//  if(Blue_Start_Flag==0)
//  {
//    MOTOR1_DIR=1;
//    tpm_pwm_duty(MOTOR_1,0);
//    MOTOR2_DIR=1;
//    tpm_pwm_duty(MOTOR_2,0);
//  }
  
//  if(fangcha_cnt == 1000)
//  {
//    error_diff_2[0] = calculate_fangcha(accx,1000);
//    error_diff_2[1] = calculate_fangcha(accy,1000);
//    error_diff_2[2] = calculate_fangcha(accz,1000);
//    error_diff_2[3] = calculate_fangcha(gyrox,1000);
//    error_diff_2[4] = calculate_fangcha(gyroy,1000);
//    error_diff_2[5] = calculate_fangcha(gyroz,1000);
//    fangcha_cnt++;
//  }
}
}



