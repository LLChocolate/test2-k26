#include "pid.h" 

////PID�ṹ��
//typedef struct                    //�ṹ�壬���PID��ر���
//{
//  int P;                        //����P
//  int I;                        //����I
//  int D;                        //����D
//  float error[3];                 //���洢����
//  float delat;                    //ÿ�εĵ��ڽ��
//  float derr;                     //һ�����
//  float dderr;                    //�������
//  signed long result;                      //PID�����������������ʽ�����Գ�ֵ����Ϊ����ƽ��ʱ�����ֵ����Ҫ��������0Ҫ����ը��
//  
//  float target;                   //PID���ڵ�Ŀ��ֵ     
//  float feedback;
//  float UP_Limit;
//  float LOW_Limit;
//}PID_Struct;

void PID_process(PID_Struct* pid)                    //PID����
{
    pid->error[0] = pid->target   - pid->feedback ;  //����λ��ƫ�Ԥ��ֵ-����ֵ���������С���������˿��Է���������      
//    pid->error[0] = pid->feedback   - pid->target ;
    pid->derr     = pid->error[0] - pid->error[1];   //����һ�����
    pid->dderr    = pid->error[0] - 2*pid->error[1] + pid->error[2]; //����������
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];  //���������������
    pid->delat    = pid->I*pid->error[0] + pid->P*pid->derr + pid->D*pid->dderr ;  //����PID���ڵĽ��
    pid->result  += pid->delat;                                          //�ô˽����Ϊ��������
    if(pid->result>pid->UP_Limit)
    {
      pid->result=pid->UP_Limit;
    }
    else if(pid->result<pid->LOW_Limit)
    {
      pid->result=pid->LOW_Limit;
    }
    
}

void Diff_PID_Process(PID_Struct* pid)
{
  pid->error[2]=pid->error[1];//�������
  pid->error[1]=pid->error[0];
  pid->error[0]=pid->feedback;
  pid->result=pid->P*pid->error[0]+pid->D*(pid->error[0]-pid->error[1]);
  if(pid->result>pid->UP_Limit)
  {
    pid->result=pid->UP_Limit;
  }
  else if(pid->result<pid->LOW_Limit)
  {
    pid->result=pid->LOW_Limit;
  }
}

void gyro_diff_pid_process(PID_Struct* pid)
{
  pid->error[2]=pid->error[1];//�������
  pid->error[1]=pid->error[0];
  pid->error[0]=pid->feedback;
  pid->result=pid->P*pid->feedback-pid->D*(mpu6050_Gyro_x_ave-3);
  if(pid->result>pid->UP_Limit)
  {
    pid->result=pid->UP_Limit;
  }
  else if(pid->result<pid->LOW_Limit)
  {
    pid->result=pid->LOW_Limit;
  }
}

void Gyro_PID_Process(PID_Struct* pid)
{
  pid->error[2]=pid->error[1];//�������
  pid->error[1]=pid->error[0];
  pid->error[0]=pid->feedback-3-pid->target;  //3Ϊ��̬���
  pid->result=pid->P*pid->error[0]+pid->D*(pid->error[0]-pid->error[1]);
  if(pid->result>pid->UP_Limit)
  {
    pid->result=pid->UP_Limit;
  }
  else if(pid->result<pid->LOW_Limit)
  {
    pid->result=pid->LOW_Limit;
  }
}
    
void PID_Init(PID_Struct* pid,float P_,float I_,float D_,signed long Result,float Target,float up_limit,float low_limit)
{
  pid->P=P_;
  pid->I=I_;
  pid->D=D_;
  pid->error[0]=pid->error[1]=pid->error[2]=0;
  pid->delat=0;
  pid->derr=0;
  pid->dderr=0;
  pid->result=Result;
  pid->target=Target;
  pid->feedback=0;
  pid->UP_Limit=up_limit;
  pid->LOW_Limit=low_limit;
}
