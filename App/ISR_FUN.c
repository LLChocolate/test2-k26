#include "ISR_FUN.h"
#include "include.h"
int Diff_PID_ave = 0;

void Speed_Control(void)
{
  static unsigned char speed_Period=0;//�ٶȿ������ڱ���
  static int L_ALL[Speed_filter_Period]={0},
              R_ALL[Speed_filter_Period]={0};//100ms�ڵ��ٶ�ֵ
//  static float speed_Delta=0;
//  static float Tmp_Speed_P =0; 
//  float SpeedRate = 0;
  //float SpeedRate;  
  //�����ٶȣ�ǰ20ms���ۼ�ֵ��
  if(speed_Period >= (Speed_filter_Period))   
  speed_Period = 0;    
  Speed_L_sum-=L_ALL[speed_Period];
  Speed_R_sum-=R_ALL[speed_Period];
  L_ALL[speed_Period]=Motor1.Speed;
  R_ALL[speed_Period]=Motor2.Speed;
  Speed_L_sum+=L_ALL[speed_Period];
  Speed_R_sum+=R_ALL[speed_Period];
  speed_Period++;
}

//void Speed_output(void)
//{

//  static float PWM_result = 0;
//  //���ٶȲ��ٻ�
//
//  //�ٶȷ�������ֵ�˲�
//  if(TimeCnt_Start_Reduct_Flag==1)
//  {
//    Motor1_PID.feedback = (Motor1.Speed+Motor2.Speed)/2;
////    Motor2_PID.feedback = Motor2.Speed;
//  }
//  else 
//  {
//    Motor1_PID.feedback = (Motor1.Speed+Motor2.Speed)/2;//(Speed_L_sum+Speed_R_sum)*1.0/(Speed_filter_Period*2);
////    Motor2_PID.feedback = Motor2.Speed;
//  }
//  Motor1_PID.target = Speed_goal1;
//  PID_process(&Motor1_PID);//�ٶ�PID
////  PID_process(&Motor2_PID);
//  
//  Diff_PID.feedback = Diff_error;
//  Diff_PID.UP_Limit  =  Motor1_PID.result;
//  Diff_PID.LOW_Limit = -Motor1_PID.result;
////  Gyro_PID.feedback  = mpu6050_Gyro_x_ave;
////  Gyro_PID.target    = Cur_error;
////  Gyro_PID.UP_Limit  = Motor1_PID.result;
////  Gyro_PID.LOW_Limit = -Motor1_PID.result;
//  if(DIFF_PID_CHANGE_FLAG==0)
//  {
//    Diff_PID.P=stand_p;
//    Diff_PID.D=stand_d;
//    Master_Speed=Speed_stand;//+Speed_max_to_min_diff;
//  }
//  else 
//  {
//    Diff_PID.P=stand_p*1.1;
//    Diff_PID.D=stand_d*1.1;
//    if(Slow_Flag==1)
//    {
//      Master_Speed=Speed_stand;
//    }
//    else 
//    {
//      Master_Speed=Speed_stand;
//    }
//  }
//
//  Diff_PID_Process(&Diff_PID);
////  gyro_diff_pid_process(&Diff_PID);
////  Gyro_PID_Process(&Gyro_PID);
////  if(Black_Lock==Left_Lock)
////  {
////    Diff_PID.result=DIFF_UP;
////  }
////  else if(Black_Lock==Right_Lock)
////  {
////    Diff_PID.result=DIFF_DOWN;
////  }
////  Diff_PID.result = 0;
////  Speed_goal1=Master_Speed-Diff_PID.result;
////  Speed_goal2=Master_Speed+Diff_PID.result;
//  //+1;
////  Motor2_PID.target = Speed_goal2;
////    Gyro_PID.feedback = mpu6050_Gyro_x_rs;
////    Gyro_PID_Process(&Gyro_PID);
//  
//
////  if(DIFF_PID_CHANGE_FLAG==0)
////  {
//    Duty_Motor1 = Motor1_PID.result - Diff_PID.result;
//    Duty_Motor2 = Motor1_PID.result + Diff_PID.result;
////  }
////  else if(DIFF_PID_CHANGE_FLAG==1)
////  {
////    Duty_Motor1 = Motor1_PID.result - Gyro_PID.result;
////    Duty_Motor2 = Motor1_PID.result + Gyro_PID.result;
////  }
////  Duty_Motor1 = Motor1_PID.result - Diff_PID.result;
////  Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;
//  if(Duty_Motor1>=65000)Duty_Motor1 = 65000;
//  else if(Duty_Motor1<=-65000)Duty_Motor1 = -65000;
//  if(Duty_Motor2>=65000)Duty_Motor2 = 65000;
//  else if(Duty_Motor2<=-65000)Duty_Motor2 = -65000;
//  if(Duty_Motor1<0)
//  {
//    MOTOR1_DIR = 1;//���result<0���������
//    Duty_Motor1 = 0xffff+Duty_Motor1;
//    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
//  }
//  else
//  {
//    MOTOR1_DIR = 0;
//    Duty_Motor1 = Duty_Motor1;
//    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
//  }
//  if(Duty_Motor2<0)
//  {
//    MOTOR2_DIR = 1;//���result<0���������
//    Duty_Motor2 = 0xffff+Duty_Motor2;
//    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
//  }
//  else
//  {
//    MOTOR2_DIR = 0;
//    Duty_Motor2 = Duty_Motor2;
//    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
//  }
//}


void Speed_output(void)
{
  static u8 Diff_PID_period = 0;
  static int Diff_PID_old[Diff_error_Period_Constant] = {0};
  static float PWM_result = 0;
  //���ٶȲ��ٻ�
  if(Far_correct_flag == 1&&Island_Flag_Lock==0)//Զ�㲹��
  {
    Diff_PID.feedback = Diff_error*0.9 + Far_Diff*0.1;
  }
  else if(Cross_correct_flag == 1&&Island_Flag_Lock==0)//ʮ�ֲ���
  {
    Diff_PID.feedback = Diff_error*0.2 + (160-Cross_correct_value)*0.8;
  }
  else
  {
    Diff_PID.feedback = Diff_error;
  }
  
  if(Diff_PID_period > (Diff_error_Period_Constant - 1))
  Diff_PID_period = 0;
  
  Diff_PID_ave -= Diff_PID_old[Diff_PID_period];
  Diff_PID_old[Diff_PID_period] = Diff_PID.feedback;
  Diff_PID_ave += Diff_PID_old[Diff_PID_period];
  Diff_PID_period++;  
  
  Diff_PID.UP_Limit  =  Speed_stand;
  Diff_PID.LOW_Limit = -Speed_stand;
//  Gyro_PID.feedback  = mpu6050_Gyro_x_ave;
//  Gyro_PID.target    = Cur_error;
//  Gyro_PID.UP_Limit  = Motor1_PID.result;
//  Gyro_PID.LOW_Limit = -Motor1_PID.result;
  if(road_filter_flag == 1&&Island_Flag_Lock == 0)
  {
    Diff_PID.P=stand_p*0.5;
    Diff_PID.D=stand_d*0.5;
    Master_Speed=Speed_stand;
  }
  else if(DIFF_PID_CHANGE_FLAG==1&&Island_Flag_Lock == 0)
  {
    Diff_PID.P=stand_p*1.3;
    Diff_PID.D=stand_d*1.3;
    if(Slow_Flag==1)
    {
      Master_Speed=Speed_stand-Speed_max_to_min_diff;
    }
    else 
    {
      Master_Speed=Speed_stand-Speed_max_to_min_diff;
    }
  }
  else
  {
    Diff_PID.P=stand_p;
    Diff_PID.D=stand_d;
    Master_Speed=Speed_stand;//+Speed_max_to_min_diff;
  }

  Diff_PID_Process(&Diff_PID);
  //�ٶȷ�������ֵ�˲�
  if(TimeCnt_Start_Reduct_Flag==1)
  {
    Motor1_PID.feedback = Motor1.Speed*Speed_filter_Period;
    Motor2_PID.feedback = Motor2.Speed*Speed_filter_Period;
  }
  else 
  {
    Motor1_PID.feedback = Speed_L_sum;//Motor1.Speed;//+Motor2.Speed)/2;//(Speed_L_sum+Speed_R_sum)*1.0/(Speed_filter_Period*2);
    Motor2_PID.feedback = Speed_R_sum;//Motor2.Speed;
  }
  Speed_goal1=Master_Speed-Diff_PID.result;
  Speed_goal2=Master_Speed+Diff_PID.result;
  Motor1_PID.target = Speed_goal1;
  Motor2_PID.target = Speed_goal2;
  PID_process(&Motor1_PID);//�ٶ�PID
  PID_process(&Motor2_PID);
  

//  gyro_diff_pid_process(&Diff_PID);
//  Gyro_PID_Process(&Gyro_PID);
//  if(Black_Lock==Left_Lock)
//  {
//    Diff_PID.result=DIFF_UP;
//  }
//  else if(Black_Lock==Right_Lock)
//  {
//    Diff_PID.result=DIFF_DOWN;
//  }
//  Diff_PID.result = 0;

  //+1;
//  Motor2_PID.target = Speed_goal2;
//    Gyro_PID.feedback = mpu6050_Gyro_x_rs;
//    Gyro_PID_Process(&Gyro_PID);
  

//  if(DIFF_PID_CHANGE_FLAG==0)
//  {
    Duty_Motor1 = Motor1_PID.result;// - Diff_PID.result;
    Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;
//  }
//  else if(DIFF_PID_CHANGE_FLAG==1)9
//  {
//    Duty_Motor1 = Motor1_PID.result - Gyro_PID.result;
//    Duty_Motor2 = Motor1_PID.result + Gyro_PID.result;
//  }
//  Duty_Motor1 = Motor1_PID.result - Diff_PID.result;
//  Duty_Motor2 = Motor2_PID.result;// + Diff_PID.result;
  if(Duty_Motor1>=65000)Duty_Motor1 = 65000;
  else if(Duty_Motor1<=-65000)Duty_Motor1 = -65000;
  if(Duty_Motor2>=65000)Duty_Motor2 = 65000;
  else if(Duty_Motor2<=-65000)Duty_Motor2 = -65000;
  if(Duty_Motor1<0)
  {
    MOTOR1_DIR = 1;//���result<0���������
    Duty_Motor1 = 0xffff+Duty_Motor1;
    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
  }
  else
  {
    MOTOR1_DIR = 0;
    Duty_Motor1 = Duty_Motor1;
    tpm_pwm_duty(MOTOR_1,Duty_Motor1);
  }
  if(Duty_Motor2<0)
  {
    MOTOR2_DIR = 1;//���result<0���������
    Duty_Motor2 = 0xffff+Duty_Motor2;
    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
  }
  else
  {
    MOTOR2_DIR = 0;
    Duty_Motor2 = Duty_Motor2;
    tpm_pwm_duty(MOTOR_2,Duty_Motor2);
  }
}
void AD_new(void)
{
  static u8  AD_Period = 0;
  static u16 L_AD[speed_Period_Constant],
              R_AD[speed_Period_Constant];
  if(AD_Period > (speed_Period_Constant - 1))
    AD_Period = 0;
  
  L_AD_Ave -= L_AD[AD_Period];
  R_AD_Ave -= R_AD[AD_Period];
  L_AD[AD_Period] = adc_once(MYADC_2,ADC_12bit);
  R_AD[AD_Period] = adc_once(MYADC_3,ADC_12bit);
  L_AD_Ave += L_AD[AD_Period];
  R_AD_Ave += R_AD[AD_Period];
  AD_Period++;  
}


#define Dir_Gyro_OFFSET         -23

//void Direction_control()
//{
//  Direction.Gyro = (float)(Gyro_ADC.x - Dir_Gyro_OFFSET); 
////  if(Block.flag==0)//û�к�שͷ�ϰ����
//  {
//    Direction.PWM_Per = Direction.P * (black_centre - CENTER )	//���
//      - Direction.D * Direction.Gyro
//        - Direction.PWM;
//  }
////  else//�к�שͷ�ϰ����
////  {
////    if(Time_1ms < Block.time + Block.delaytime1)
////      Direction.PWM_Per = Direction.P * (black_centre + Block.Offset - CENTER )	//���
////        + Direction.D * Direction.Gyro
////          - Direction.PWM;
////    else
////      Block.flag = 0;
//  }
//  Direction.PWM += Direction.PWM_Per;
//  //PWM�޷�
//  if(Direction.PWM>=Direction.PWM_Limit_max) Direction.PWM=Direction.PWM_Limit_max;
//  if(Direction.PWM<=Direction.PWM_Limit_min) Direction.PWM=Direction.PWM_Limit_min;
//}
