#ifndef __IMAGE_PROCESS_H__
#define __IMAGE_PROCESS_H__

#include "common.h"
//寻找黑线并寻找中心值
#define CENTER_             160
#define HALF_WIDTH          98
#define HALF_NEAR           115
#define VN_HALF             170
#define Island_HALF         108
#define Far_Point           72            //1m处  76半宽
#define Far_Half            80
#define Start_Point         105         //45cm处
#define Near_Point          150            //近处
#define VN_Point            180
#define Island_Point        125
#define ALL_LINE            240
#define Image_Point(x,y)    (Image_fire[x][y/8]>>(7-(y%8))&1)
#define AD_Near_hang        120
#define Turn_Point          100

#define Center_correct(hang)   ((hang-Start_Point)*(3-2)*1.0/(Far_Point-Start_Point)+2)


#define _1m_Left        108
#define _1m_Right       198
#define Near_Left       76
#define Near_Right      230


#define  Centerdiff_const    10
#define  Center_Filter_Period 5

typedef enum
{
  turn_left  = 1,
  turn_right = 2,
  go_str     = 3
}_1m_Road_Status;
void image_process(void);
void get_black_line(unsigned char *ImageData_in,int hang,int half_width);//捕捉黑线 
u8 CenterlineToDiff(int center);
void get_three_lie(void);
void StoreDate(void);
u8 Cross_Test(void);
u8 Straight_test(void);
u8 Fine_turn_point(void);
u8  Straight_line_test(void);
u8  abnormal_width(u8* hang,int* ave_center);
u8  abnormal_width_AD(int* ave_center,u8 *dir);
u8 New_abnormal_width_AD(int* ave_center,u8 *dir);
u8 out_island(u8 Lock_island);
u8 double_AD(void);
float chabihe(void);
float Curvature(int x1, int y1, int x2, int y2, int x3, int y3);
void get_top_board(unsigned char *ImageData_in,u8 top_hang);
void get_near_board(unsigned char *ImageData_in,u8 near_hang);
void get_island_center(unsigned char *ImageData_in,int hang,u8 half_width);
u8 Find_Start_line(void);
void Find_edge_flag(u8 *ImageData_in,u8 hang,u8 dir);
u8 little_line(u8 hang,u8 dir);
u8 far_near_lie(void);
void far_cen2diff(int center);
int In_Island(u8 hang,int *Hang_);
u8 In_Island_black_line(unsigned char *ImageData_in,int hang);
int Cross_correct(void);
void get_angel_edge(unsigned char *ImageData_in,int hang,int *left,int *right);

#endif