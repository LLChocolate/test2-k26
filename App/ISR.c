/*
* �жϴ������
*/
#include "common.h"
#include "include.h"
#include "define.h"
#include "ISR_FUN.h"
#include "ISR.h"
extern signed int Speed_goal1_TEMP;//���ת��Ŀ��ֵ
extern signed int Speed_goal2_TEMP;//���ת��Ŀ��ֵ
unsigned long int  fangcha_cnt = 0;
u8 TimeCnt_Start_Reduct_Flag = 1 ;


/*************************************************************************
*                             Ұ��Ƕ��ʽ����������
*
*  �������ƣ�HardFault_Handler
*  ����˵����Ӳ���Ϸ��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-2-4    �Ѳ���
*  ��    ע��������LED��˸��ָʾ������Ӳ���Ϸ�
*************************************************************************/

/*************************************************************************
*  �������ƣ�VSYNC_IRQ
*  ����˵����PORTD�˿��жϷ�����
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�Լ���ʼ�������
*************************************************************************/
void VSYNC_IRQ(void)
{
    static u32 flag;
    //Clear Interrupt flag
    flag = PORTC_ISFR;
    PORTC_ISFR = flag;
    
    if(flag & (1<<13))
    {
      Image_Flag=0;
      if(img_flag == IMG_START)	//��Ҫ��ʼ�ɼ�ͼ��
      {
              //����洢��ѭ��ʹ��
          //���֮ǰͼ����ʹ�õĴ洢����2����DMA�����λ����1�����ʱ�Ѵ���λ�ø�Ϊ2
          if(Memory_use_Flag==2)
          {
            Memory_use_Flag=1;
            dma_repeat(CAMERA_DMA_CH, (void *)&PTD_B0_IN, (void *)Image_fire_Memory2,CAMERA_SIZE);
          }
          //���֮ǰͼ����ʹ�õĴ洢����1����DMA�����λ����2�����ʱ�Ѵ���λ�ø�Ϊ1
          else if(Memory_use_Flag==1)
          {
            Memory_use_Flag=2;
            dma_repeat(CAMERA_DMA_CH, (void *)&PTD_B0_IN, (void *)Image_fire_Memory1,CAMERA_SIZE);
          }
//        
          DMA_EN(CAMERA_DMA_CH);            		//ʹ��ͨ��CHn Ӳ������
          disable_irq(PORTC_PORTD_IRQn);  
          img_flag=IMG_GATHER;
      }

      else					//ͼ��ɼ�����
      {
          img_flag = IMG_START;	//��ʼ�ɼ�ͼ��
          PORTC_ISFR = flag;		//д1���жϱ�־λ(����ģ���Ȼ�ص���һ���жϾ����ϴ����ж�)
          enable_irq(PORTC_PORTD_IRQn); 
      }
    }
//    if(flag & (1<<0))
//    {
//      button_timeout=15;
//    }
}

/*************************************************************************
*  �������ƣ�DMA0_IRQHandler
*  ����˵����DMA0
*  ����˵������
*  �������أ���
*  �޸�ʱ�䣺2012-1-25    �Ѳ���
*  ��    ע�����ź���Ҫ�����Լ���ʼ�����޸�
*************************************************************************/
void DMA0_IRQHandler()
{
    if(Memory_use_Flag==1)
    {
      Image_fire=&Image_fire_Memory2[0];
    }
    else if(Memory_use_Flag==2)
    {
      Image_fire=&Image_fire_Memory1[0];
    }
    DMA_DIS(CAMERA_DMA_CH);            	//�ر�ͨ��CHn Ӳ������
    DMA_IRQ_CLEAN(CAMERA_DMA_CH);           //���ͨ�������жϱ�־λ
    img_flag = IMG_FINISH ; 
    Image_Flag=1;
}

void PendSV_Handler(void)
{
  
}
void HardFault_Handler(void)
{
  MOTOR1_DIR=0;
  tpm_pwm_duty(MOTOR_1,0);
  MOTOR2_DIR=0;
  tpm_pwm_duty(MOTOR_2,0);
}
void SysTick_Handler(void)
{
  
}
void USART5_IRQHandler()
{
  
}

void USART2_IRQHandler()
{

  
}
  

void USART3_IRQHandler()
{

}
void USART1_IRQHandler()
{
  static u8 b_cnt=0;
  static u8 d_cnt = 0;
  const u8 i=0;
  
  u8 j;
  int Sum=0;
  char c;
  char Res_Temp[10]={0};
  static char Res[5];
  DisableInterrupts;
  if(UART_S1_REG(UARTN[UART1]) & UART_S1_RDRF_MASK)
  {
  uart_querystr(UART1,Res_Temp,1);
//  uart_sendStr(UART5,Res_Temp);
  c=Res_Temp[0];
  switch(c)
  {
//  case 'n':
//    {
//      for(j=0;j<i;j++)
//      {
//        Res[j]=0;
//      }
//      i=0;              //�������
//      break;
//    }
  case 'm':
    {
//     Debug_Mode=Motor;//�����PID
      LED2=!LED2;
      break;
    }
  case 's':
    {
//      Debug_Mode=Diff;//������PD
      break;
    }
  case 'p':
    {
      for(j=0;j<i;j++)//��P
      {
        Sum+=Res[j]*power_s16(10,i-j-1);
      }
//      if(Debug_Mode==Motor)
//      {
//        Motor1_PID.P=Sum;
//        Motor2_PID.P=Sum;
//      }
//      else if(Debug_Mode==Diff)
//      {
//        Diff_PID.P=Sum;
//      }
      break;
    }
  case 'i':
    {
      for(j=0;j<i;j++)//��I
      {
        Sum+=Res[j]*power_s16(10,i-j-1);
      }
//      if(Debug_Mode==Motor)
//      {
//        Motor1_PID.I=Sum;
//        Motor2_PID.I=Sum;
//      }
//      else if(Debug_Mode==Diff)
//      {
//        Diff_PID.I=Sum;
//      }
      break;
    }
  case 'd':
    {
      if(d_cnt == 0)
      {
        Gyro_PID.target = 30;
        d_cnt++;
      }
      else if(d_cnt == 1)
      {
        Gyro_PID.target = -30;
        d_cnt--;
      }
      break;
    }
  case 'b':
    {
      if(b_cnt==0)
      {
        Blue_Start_Flag=1;
//        Speed_Start_Flag=1;
        b_cnt++;
      }
      else
      {
        Speed_goal1=0;
        Speed_goal2=0;
        Speed_stand=0;
        stand_p=0;
        stand_d=0;
        Motor_enable_Flag=0;
      }
      break;
    }
  case 'a'://ͣ��
    {
      Speed_goal1_TEMP=Speed_goal1;
      Speed_goal2_TEMP=Speed_goal2;
      Speed_goal1=0;
      Speed_goal2=0;
      Speed_stand=0;
      stand_p=0;
      stand_d=0;
      Motor_enable_Flag=0;
      
//      disable()
      break;
//      tpm_pwm_duty(MOTOR_1,0);
//      tpm_pwm_duty(MOTOR_2,0);
    }
  case '1':
    {
      Speed_goal1+=100;
      break;
    }
  case '2':
    {
      Speed_goal1-=100;
      break;
    }
  case '3':
    {
      Speed_goal2+=100;
      break;
    }
  case '4':
    {
      Speed_goal2-=100;
      break;
    }
    
  default:
  {
    if(c>=48&&c<=57)//��������֣�����Ч
    {
//      i=strlen(Res_Temp);
      for(j=0;j<i;j++)
      {
        Res[j]=Res_Temp[j]-48;
      }
//      printf("\r\n%d\r\n",i);
    }
    
    break;
  }
  }
//  if(Debug_Mode==Motor)
//  {
//    printf("\r\nMode:Motor\r\n");
//  }
//  else if(Debug_Mode==Diff)
//  {
//    printf("\r\nMode:Diff\r\n");
//  }
  printf("\r\nRes:");
  
  for(j=0;j<i;j++)
  {
    uart_putchar(UART1,Res[j]+48);
  }
  printf("\r\n");

  }
  EnableInterrupts;
}


void PIT_IRQHandler()
{
  
  static u8 temp_dir = 0;
  static unsigned char TimeCnt_20ms = 0,TimeCnt_5ms=0;	  //5ms,20msʱ�������    
  static unsigned int  TimeCnt_2000ms = 0;
  static unsigned int  TimeCnt_Key_Start_ms = 0;
  static unsigned int  TimeCnt_Key__ms = 0;
  static unsigned int  TimeCnt_Start_Reduct_ms = 0;


  //ʱ���߸��� 
  Time_1ms++; 
  TimeCnt_5ms++;
  TimeCnt_20ms++;
  if(Reduct_Flag==1&&Blue_Start_Flag==1)
    TimeCnt_2000ms++;
  if(Key_Start_Flag==1)
    TimeCnt_Key_Start_ms++;
  if(Blue_Start_Flag==1&&TimeCnt_Start_Reduct_ms<1002)
  {
    TimeCnt_Start_Reduct_ms++;
    if(TimeCnt_Start_Reduct_ms==1000)
    TimeCnt_Start_Reduct_Flag=0;
  }
//  if((Time_1ms>2000)&&(fangcha_cnt<1000))
//    fangcha_cnt++;
  if(TimeCnt_5ms >= 5)
    TimeCnt_5ms = 0;
  if(TimeCnt_20ms >= 20)
    TimeCnt_20ms = 0;
  if(TimeCnt_2000ms >= 2000)
  {
    TimeCnt_2000ms = 0;
    Speed_stand--;
  }
//  if(Island_in_delay<=1000)
//  {
//    Island_in_delay--; 
//  }
  if(Island_in_delay_flag==1)
  {
    Island_in_delay--;
  }
  if(Island_in_delay_flag==1&&Island_in_delay == 0)
  {
//    Island_doublt_flag |= 8;
    Island_in_delay_flag = 0;
    Island_in_delay--;
    out_island_test_flag = 1;
    LED4 = 0;
  }
  if(Next_Island_Flag==1)
  {
    Next_Island_delay--;
  }
  if(Next_Island_Flag==1&&Next_Island_delay==0)
  {
    Next_Island_Flag=0;
    island_flag_temp = 0;
    Island_Flag_Lock = 0;
    Island_out_flag = 0;
    Island_Stay_Flag = 0;
    Island_doublt_flag = 0;
    out_island_test_flag = 0;
    Island_In_Flag = 0;
    Angel_Find_Flag = 0;
  }
  
  if(TimeCnt_Key_Start_ms >= 3000)
  {
    Blue_Start_Flag=1;
    Key_Start_Flag=0;
  }
  if(Island_out_flag==1)
  {
    Island_out_delay--;
  }
  if(Island_out_flag==1&&Island_out_delay==0)
  {
    island_flag_temp = 0;
    Island_Flag_Lock = 0;
    Island_out_flag = 0;
    Island_Stay_Flag = 0;
    Island_doublt_flag = 0;
    out_island_test_flag = 0;
    Island_In_Flag = 0;
    Angel_Find_Flag = 0;
  }
if(Time_1ms==2000)
{
  temp_CNT_WATCH=temp_cnt;
}
//PIT0������
//��������
    if(button_timeout<16)
    {
      button_timeout--;
    }
  if(button_timeout==0)//��ʱ10ms��ⰴ��״̬
  {
//    key_up = 0;
      if(KEY1==0)
      {

      }
      else if(KEY2==0)
      {
//        Key_Start_Flag=1;
//        Key_status=KEY2_PRES;
//        LED2=0;
      }
      else if(KEY3==0)
      {
        Key_status=KEY3_PRES;
        Blue_Start_Flag = 1;
        LED3 = !LED3;
//        if(temp_dir == 0)
//        {
//          Duty_Motor1 = 0;
//          Duty_Motor2 = 32767;
//          temp_dir++;
//        }
//        else if(temp_dir == 1)
//        {
//          Duty_Motor1 = 32767;
//          Duty_Motor2 = 0;
//          temp_dir--;
//        }
        
      }
  }

//����������������
//**********************************************����ͷͼ����
//  if
  
  
//***************************************************���ٺ���
  if(TimeCnt_5ms==1)
  {
    
//    if(DIFF_PID_CHANGE_FLAG==0)
//    {
//      Diff_PID.P=stand_p;
//      Diff_PID.D=stand_d;
//      Master_Speed=Speed_stand+Speed_max_to_min_diff;
//    }
//    else 
//    {
//        Diff_PID.P=stand_p*1.05;
//        Diff_PID.D=stand_d*1.05;
//      if(Slow_Flag==1)
//      {
//        Master_Speed=Speed_stand;
//      }
//      else 
//      {
//        Master_Speed=Speed_stand;
//      }
//    }

//    Speed_goal1=Master_Speed-Diff_PID.result;
//    Speed_goal2=Master_Speed+Diff_PID.result;
  }
  
//****************************************************���ٺ���
  if(TimeCnt_5ms==2)//&&Blue_Start_Flag==1)//20msִ��һ��
  {
//    MPU6050_Data_Prepare();
//    if((Time_1ms>2000)&&(fangcha_cnt<1000))
//    {
//      accx[fangcha_cnt] = mpu6050_Acc_x;
//      accy[fangcha_cnt] = mpu6050_Acc_y;
//      accz[fangcha_cnt] = mpu6050_Acc_z;
//      gyrox[fangcha_cnt]= mpu6050_Gyro_x;
//      gyroy[fangcha_cnt]= mpu6050_Gyro_x;
//      gyroz[fangcha_cnt]= mpu6050_Gyro_x;
//      fangcha_cnt++;
//    }
    Get_speed1(&Motor1);
    Get_speed2(&Motor2);
    Speed_Control();
    Speed_output();
    AD_new();
  }
    //*****************************************************adc���

  if(Time_1ms == 1000)
  {
    stand_AD_L = L_AD_Ave;
    stand_AD_R = R_AD_Ave;
    if(stand_AD_L>10000||stand_AD_R>10000)
    {
//      BEEP = 1;
    }
    stand_AD = (stand_AD_L+stand_AD_R)/2;
  }
//  if((Time_1ms%15)==0)
//  {
//    if(tanzhen == 1)
//    {
//      tanzhen = 0;
//      BEEP = 0;
//    }
//    else
//    {
//      BEEP = 1;
//    }
//  }
  
  //***************************MPU6050*****************

//���ٺ�������
//  LED1 = ~LED1;
  
//  LED1 = ~LED1;
//  LED2 = !LED2;
  PIT_Flag_Clear(PIT0);
}



void PORTC_IRQHandler()
{

}
void PORTA_IRQHandler()
{
  u8 n=0;
  n=17;
  if(PORTA_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTA_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
  n=15;
  if(PORTA_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTA_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
}

void PORTB_IRQHandler()
{
  u8 n=0;
  n=1;
  if(PORTB_ISFR & (1<<n))
  {
//�����ж�1���жϷ���
    button_timeout=15;//��־��15
    PORTB_ISFR |=(1<<n);//д��1����жϱ�־λ
  }
}

void PORTE_IRQHandler()
{

}
void FTM0_IRQHandler()
{
  
}
void FTM1_IRQHandler()
{
  
}

//void TPM2_IRQHandler()
//{
////  u8 s;
////  s = TPM2_STATUS;
//  TPM2_STATUS = ~0;
//  TPM2_STATUS = ~0;
////  if(s & (1 << 0))
////  {
//    TPM2_Cnt++;
////  }
//}


//void TPM1_IRQHandler()
//{
////  u8 s;
////  s = TPM1_STATUS;
//  TPM1_STATUS = ~0;
//  TPM1_STATUS = ~0;
////  if(s & (1 << 0))
////  {
//    TPM1_Cnt++;
////  }
//}



