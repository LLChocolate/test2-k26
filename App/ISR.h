#ifndef __ISR_H
#define __ISR_H 1

#include  "include.h"
extern unsigned long int  fangcha_cnt;
extern u8 TimeCnt_Start_Reduct_Flag;
extern void PendSV_Handler(void);         //�������ϵͳ�����жϷ�����
extern void HardFault_Handler(void);      //����Ӳ���Ϸã������ܷɣ�
extern void SysTick_Handler(void);        //�δ�ʱ��(os �õ�)
extern void USART1_IRQHandler();          //����1 �жϽ��պ���
extern void USART2_IRQHandler();
extern void USART3_IRQHandler();
extern void USART5_IRQHandler();
extern void PIT_IRQHandler();            //PIT0 ��ʱ�жϷ�����
extern void PIT3_IRQHandler();            //PIT0 ��ʱ�жϷ�����
extern void PIT2_IRQHandler();
extern void PORTA_IRQHandler();           //PORTA�жϷ�����
extern void PORTB_IRQHandler();           //PORTB�жϷ�����
extern void PORTC_IRQHandler();
//PORTC�жϷ�����
extern void PORTE_IRQHandler();
     
extern void FTM0_IRQHandler();            //FTM0���벶׽�ж�
extern void FTM1_IRQHandler();            //FTM0���벶׽�ж�
extern void VSYNC_IRQ();
extern void DMA0_IRQHandler();
extern void TPM2_IRQHandler();
extern void TPM1_IRQHandler();


#endif
