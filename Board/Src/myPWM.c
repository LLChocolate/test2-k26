#include "myPWM.h"
void Motot1_Init(void)
{
  gpio_init(PTA6,GPO,LOW);//DIR
  tpm_pwm_init(MOTOR_1,16000,0);
}
void Motot2_Init(void)
{
  gpio_init(PTA7,GPO,LOW);//DIR
  tpm_pwm_init(MOTOR_2,16000,0);//����Ƶ��Ϊ16k
}

//void Steer_Init(void)
//{
//  FTM_PWM_init(STEER_,50,(u16)(1.5/20*65536));//��ʼ0.5ms�ߵ�ƽ��ת0��
// // Time_Steer=1.5;
//}

void PWM_DISENABLE(TPMn_e tpmn, TPM_CHn_e ch)
{
    tpm_pwm_duty(tpmn,ch,0);//�ر�PWM
}

