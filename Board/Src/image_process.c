////环岛判断：1.前方直道
////          2.半宽异常（判断环岛左右后锁定）
////          3.判断不为直道后判断进入环岛时前方碰到的角度的形状
////          4.出环岛时判断前方竖直三线（三线尽头的位置连续跳变三次）
////          5.跳变结束后DElay 200ms后解除环岛锁定
//
//
//
#include "image_process.h"
#include "include.h"
#include "stdlib.h"
int centre[ALL_LINE];
u8 ImageData[320];
u8 halfwidth[ALL_LINE];
int black_L[ALL_LINE];
int black_R[ALL_LINE];
//u16 Right_count[ALL_LINE];
//u16 Left_count[ALL_LINE];
u8 getLeft_flag[ALL_LINE];
u8 getRight_flag[ALL_LINE];
u8 Black_Lock=0;
u8 Image_Weight[Available_Num]={1};
u8 Three_Lie[3]={86,156,226};
u8 Three_lie_end[3]={0};
u8 Straight_lie_end[3]={0};
u8 Three_lie_end_old[3]={0};
u8 Three_lie_end_err[3]={0};
u8 Island_Stay_Flag;
u8 AD_double_flag = 0;
Edge_Str Island_mark_edge[50];
float island_addline_k = 0;
float Cur_error = 0;
u8 out_Island_flag = 0;
u8 Island_doublt_flag = 0;
u8 diff_done_flag = 0;
float CHABIHE = 0;
u8 Island_in_delay_flag  = 0;
u8 Island_out_flag = 0;
u16 Island_in_delay = 300;
u16 Island_out_delay = 300;
u8 out_island_test_flag = 0;
int top_angel_board[10][4];
u8  top_angel_board_flag[10][4];
int near_angel_board[20][2];
u8  near_angel_board_flag[20][2];
const u8 island_in_plus = 80;
const u16 island_delay_time = 1500;
u8 island_flag_temp = 0;
u8 Island_In_Flag = 0;
u16 Start_line_delay = 0xffff;
u8  Start_line_flag  = 0;
u8  road_filter_flag = 0;
u8  Far_correct_flag = 0;
u8  Far_Diff = 0;
u8  Cross_correct_flag = 0;
int  Cross_correct_value = 0;
u8  Island_Center_Lock_flag = 0;
u8  Angel_Find_Flag = 0;
Filter_1st_Str Center_Filter = {0.5,{0,0},{0,0}};
//Kalman_Date Center_Filter={0,0,0,0,0,0.1,40,0,0};
s16 fangcha_test[200]={0};
u8 str_temp[20];
const u16 Next_Island_delay_const = 5000;
u16  Next_Island_delay = 5000;
u8   Next_Island_Flag  = 0;
void image_process(void)
{
//  u8 Lock_center_right = 155;
//  u8 Lock_center_left  = 165;
//  u16 Delay_const_time = 300;
  const u8 Left_Turn_Point = 86;
  const u8 Right_Turn_Point= 86;//60:90 ,100:75
  const u8 Left_Turn_Half  = 0;
  const u8 Right_Turn_Half = 40;
  const u8 Diff_ = 8;
  static u8 hang_for_island;
  static u8 Straight_1m_flag = 0;
  
  u8  Island_out_hang_cnt = 0;
  int i,j;
  int center_temp;
  int hang_temp;
  u8 max_temp;
  u8 min_temp;
//  static int centre_temp=CENTER_;
  diff_done_flag = 0;//每次循环的开头将diff传值标志置零
  get_three_lie();
//  Straight_1m_flag = Straight_line_test();
  min_temp=min_u8(Three_lie_end,3);
  max_temp=max_u8(Three_lie_end,3);
  
//  get_black_line(Image_fire[Start_Point],Start_Point,HALF_NEAR);
//  get_black_line(Image_fire[Far_Point],Far_Point,HALF_NEAR);
  
  if(Three_lie_end[1]<65)
  {
    road_filter_flag = 1;
  }
  else
    road_filter_flag = 0;
  
  if((Island_doublt_flag&1)!=1)
    Island_doublt_flag |= double_AD();
  if((Island_doublt_flag&1)==1&&((Island_doublt_flag>>1)&1)!=1)
  {
//    Island_doublt_flag |= New_abnormal_width_AD(&center_temp,&island_flag_temp);
//    if(((Island_doublt_flag>>1)&1)!=1)
//    {
      Island_doublt_flag |= New_abnormal_width_AD(&center_temp,&island_flag_temp);
//    }
  }
  if(Island_doublt_flag == 3)//进环岛
  {
    if(island_flag_temp==Right_Island)
    {
      Find_edge_flag(Image_fire[Right_Turn_Point-Diff_],Right_Turn_Point-Diff_,Right_Island);
      Find_edge_flag(Image_fire[Right_Turn_Point+Diff_],Right_Turn_Point+Diff_,Right_Island);
      center_temp = In_Island(AD_Near_hang+20,&hang_temp);
      if(getRight_flag[Right_Turn_Point-Diff_]==1&&/*black_R[Right_Turn_Point-Diff_]<250&&*/getRight_flag[Right_Turn_Point+Diff_]==0)//||hang_temp>Right_Turn_Point)
      {
        Island_doublt_flag |=4;
        Island_Flag_Lock = island_flag_temp;
        Island_Stay_Flag = 1;
      }
    }
    else if(island_flag_temp==Left_Island)
    {
      Find_edge_flag(Image_fire[Left_Turn_Point-Diff_],Left_Turn_Point-Diff_,Left_Island);
      Find_edge_flag(Image_fire[Left_Turn_Point+Diff_],Left_Turn_Point+Diff_,Left_Island);
      center_temp = In_Island(AD_Near_hang+20,&hang_temp);
      if(getLeft_flag[Left_Turn_Point-Diff_]==1&&/*black_L[Left_Turn_Point-Diff_]>80&&*/getLeft_flag[Left_Turn_Point+Diff_]==0)//||hang_temp<Left_Turn_Point)
      {
        Island_doublt_flag |=4;
        Island_Flag_Lock = island_flag_temp;
        Island_Stay_Flag = 1;
      }
    }
  }
  
  if(out_island_test_flag==1)
  {
    if(double_AD()==1)
    {
      island_flag_temp = 0;
      Island_Stay_Flag = 0;
      Island_Flag_Lock = 0;
      Island_out_flag = 1;
      Island_out_delay = island_delay_time;
      Island_In_Flag = 0;
      Angel_Find_Flag = 0;
    }
  }
  
    if(LCD_DISPLAY_FLAG==1)
    {
      LCD_Put_Int(100,100,"",L_AD_Ave,Red,White);
      LCD_Put_Int(100,120,"",R_AD_Ave,Red,White);
    }
  if(Cross_Test()==1)
  {
    Road_Status_Flag = Cross;
  }
  
  if(Island_Flag_Lock == Left_Island)
  {
    LED1 = 0;
  }
  else
    LED1 = 1;
  if(Island_Flag_Lock == Right_Island)
  {
    LED2 = 0;
  }
  else
    LED2 = 1;
  
  
  if(Island_Stay_Flag == 1)//手动加数强行拐
  {
    get_black_line(Image_fire[Near_Point],Near_Point,HALF_NEAR);
    if(Island_In_Flag == 0)
    center_temp = In_Island(AD_Near_hang+20,&hang_temp);
    for(i=Three_lie_end[1]+10;i<Three_lie_end[1]+51;i+=10)
    {
      if(sum_point(Image_fire[i],40)<=5)
      {
        Island_out_hang_cnt++;
      }
    }
    if(Island_out_hang_cnt>=4&&abs_s16(Three_lie_end[0]-Three_lie_end[2])<10)
    {
      Island_Center_Lock_flag = 1;
    }
    else
    {
      Island_Center_Lock_flag = 0;
    }
    if(Island_Flag_Lock == Right_Island)
    {
//      if(Island_In_Flag == 0&&(getLeft_flag[Near_Point]==0||Three_lie_end[1]<70))
//      if(centre[Right_Turn_Point]<200)
      if(Island_In_Flag == 0)//||getLeft_flag[Near_Point]==0||Three_lie_end[1]<70))
      {
        if(Next_Island_Flag!=1)
        {
          Next_Island_Flag = 1;
          Next_Island_delay = Next_Island_delay_const;
        }
        
        LED3 = 0;
        if(center_temp!=-1)
        {
          Angel_Find_Flag=1;
        }
        else if(center_temp==-1&&Angel_Find_Flag==1&&Three_lie_end[1]>90&&getLeft_flag[Near_Point]==0)
        {
          Island_In_Flag = 1;
        }
        
        if(center_temp!=-1&&center_temp<300&&center_temp>190)
        {
          centre[Right_Turn_Point+Diff_]=center_temp;//60:230 100:250
        }
        else
        {
//        Find_edge_flag(Image_fire[Right_Turn_Point-Diff_],Right_Turn_Point-Diff_,Right_Island);
          centre[Right_Turn_Point+Diff_]=210;//60:230 100:250
        }
        CenterlineToDiff(centre[Right_Turn_Point+Diff_]);
      }
      else if(Island_Center_Lock_flag==1)
      {
//        CenterlineToDiff(280);
        CenterlineToDiff(Three_lie_end[1]+120 - 2*Min_2_num(abs_s16(Three_lie_end[1]-Three_lie_end[0]),abs_s16(Three_lie_end[1]-Three_lie_end[2])));
      }
      else if(min_temp<=100)
      {
        Island_In_Flag = 1;
        LED3 = 1;
        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
        {
          Island_in_delay_flag = 1;
          Island_in_delay=island_delay_time;
        }
        get_black_line(Image_fire[Start_Point],Start_Point,HALF_WIDTH);
        CenterlineToDiff(centre[Start_Point]);
      }
//      else
//      {
//        Island_In_Flag = 1;
//        LED3 = 1;
//        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
//        {
//          Island_in_delay_flag = 1;
//          Island_in_delay=island_delay_time;
//        }
//        get_black_line(Image_fire[Island_Point],Island_Point,Island_HALF);
//        CenterlineToDiff(centre[Island_Point]);
//      }
      else
      {
        Island_In_Flag = 1;
        LED3 = 1;
        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
        {
          Island_in_delay_flag = 1;
          Island_in_delay=island_delay_time;
        }
        get_black_line(Image_fire[Island_Point],Island_Point,Island_HALF);
        CenterlineToDiff(centre[Island_Point]);
      }
    }
    else if(Island_Flag_Lock == Left_Island)
    {
//      get_black_line(Image_fire[Left_Turn_Point],Left_Turn_Point,HALF_WIDTH);
//      if(centre[Left_Turn_Point]>150)
//      if(Island_In_Flag == 0&&(getRight_flag[Near_Point]==0||Three_lie_end[1]<70))
      if(Island_In_Flag == 0)//||getRight_flag[Near_Point]==0||Three_lie_end[1]<70))
      {
        LED3 = 0;
        if(center_temp!=-1)
        {
          Angel_Find_Flag=1;
        }
        else if(center_temp==-1&&Angel_Find_Flag==1&&Three_lie_end[1]>90&&getRight_flag[Near_Point]==0)
        {
          Island_In_Flag = 1;
        }
        
        if(center_temp!=-1&&center_temp<130&&center_temp>20)
        {
          centre[Right_Turn_Point+Diff_]=center_temp;//60:230 100:250
        }
        else
        {
//        Find_edge_flag(Image_fire[Right_Turn_Point-Diff_],Right_Turn_Point-Diff_,Right_Island);
          centre[Right_Turn_Point+Diff_]=115;//60:230 100:250
        }
        CenterlineToDiff(centre[Left_Turn_Point+Diff_]);
      }
      else if(Island_Center_Lock_flag==1)
      {
        CenterlineToDiff(Three_lie_end[1] - 120 + 2*Min_2_num(abs_s16(Three_lie_end[1]-Three_lie_end[0]),abs_s16(Three_lie_end[1]-Three_lie_end[2])));
//        CenterlineToDiff(40);
      }
//      else if(min_temp<=100)
//      {
//        Island_In_Flag = 1;
//        LED3 = 1;
//        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
//        {
//          Island_in_delay_flag = 1;
//          Island_in_delay=island_delay_time;
//        }
//        get_black_line(Image_fire[Start_Point],Start_Point,HALF_WIDTH);//45cm处中心点
//        CenterlineToDiff(centre[Start_Point]);
//      }
//      else 
//      {
//        Island_In_Flag = 1;
//        LED3 = 1;
//        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
//        {
//          Island_in_delay_flag = 1;
//          Island_in_delay=island_delay_time;
//        }
////        get_black_line(Image_fire[Near_Point],Near_Point,HALF_NEAR);
//        CenterlineToDiff(centre[Near_Point]);
//      }
      else
      {
        Island_In_Flag = 1;
        LED3 = 1;
        if(Island_in_delay_flag==0)//在本环岛内还没有置进环岛flag
        {
          Island_in_delay_flag = 1;
          Island_in_delay=island_delay_time;
        }
        get_black_line(Image_fire[Island_Point],Island_Point,Island_HALF);
        CenterlineToDiff(centre[Island_Point]);
      }
    }
  }
  
//  if(out_island_test_flag == 1&&out_Island_flag == 0)
//  {
//      out_Island_flag = out_island(Island_Flag_Lock);
//  }
    
  
  if(Three_lie_end[1]<60)
  {
    far_cen2diff(far_near_lie());
    Far_correct_flag = 1;
  }
  else
    Far_correct_flag = 0;
  
  if(min_temp<=100)
  {
//    BEEP=0;
    Slow_Flag=0;
    
    Cross_correct_value = Cross_correct();
    if(Road_Status_Flag==Cross||Island_Flag_Lock != 0)
    {
      get_black_line(Image_fire[Far_Point],Far_Point,Far_Half);//45cm处中心点
      CenterlineToDiff(centre[Far_Point]);
    }
    else
    {
      Slow_Flag=1;
      get_black_line(Image_fire[Start_Point],Start_Point,HALF_WIDTH);//45cm处中心点
      CenterlineToDiff(centre[Start_Point]);
    }
    DIFF_PID_CHANGE_FLAG=0;//使用直道PID
    Road_Status_Flag=Straight;
  }
  else if(Three_lie_end[1]<145)
  {
//      Cur_error = Curvature(90,Three_lie_end[0],160,Three_lie_end[1],230,Three_lie_end[2]);
    get_black_line(Image_fire[Near_Point],Near_Point,HALF_NEAR);
    CenterlineToDiff(centre[Near_Point]);

    DIFF_PID_CHANGE_FLAG=1;//使用弯道PID
  }
  else
  {
    if(Three_lie_end[0]>Three_lie_end[2])
    {
//    get_black_line(Image_fire[Three_lie_end[]],VN_Point,VN_HALF);
      CenterlineToDiff(220);
    }
    else if(Three_lie_end[0]<Three_lie_end[2])
    {
      CenterlineToDiff(100);
    }
    DIFF_PID_CHANGE_FLAG=1;//使用弯道PID
  }
  if(Three_lie_end[1]>=125)//直道
  {
//    BEEP=1;
    Speed_max_to_min_diff=(Three_lie_end[1] - 125);
    DIFF_PID_CHANGE_FLAG=0;//使用直道PID
  }
  else 
  {
    BEEP=0;
    Speed_max_to_min_diff=0;
    DIFF_PID_CHANGE_FLAG=1;//使用直道PID
  }
  if(Speed_max_to_min_diff>Acc_Limit)
    Speed_max_to_min_diff=Acc_Limit;
}

//
void get_black_line(unsigned char *ImageData_in,int hang,int half_width)//捕捉黑线  
{
  static int Middle=160;  //黑线中间默认为CENNTER
  static u8  dir_flag = 0;
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int getleft_flag=0,getright_flag=0;//找到左右标志0
  int _black_R,_black_L;//黑线左右端
  static int _halfwidth = 160;//黑线一半宽度默认80
  static unsigned char first_run=0;//开跑点0
  int i=0;
  if(hang<0)hang=0;
  else if(hang>239)hang=239;
//  ASSERT( (hang>=0) && (hang<=239) ); //检查行数是否正确；
  if(first_run==0)  //开跑点是0处
  {
    first_run++;//开跑点加1
  }
  else//如果first_run!=0
  {
//    if(max_u8_index(Three_lie_end,3)==1)
//      Middle = Three_Lie;//centre[hang];//centre[hang];//中间为centre[hang]
//    else if(max_u8_index(Three_lie_end,3)==0)
//      Middle = 76;
//    else if(max_u8_index(Three_lie_end,3)==2)
//      Middle = 216;
    Middle = Three_Lie[min_u8_index(Three_lie_end,3)];
    _halfwidth = half_width;//halfwidth[hang];//一半为halfwidth[hang]
  }
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Right_Count = Middle;//把黑线中间值赋给右计数起点
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
    Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
  
  if(Right_Count<ccd_end)//如果在有效范围内
  {
    _black_R = Right_Count;
    getRight_flag[hang]=1;
    getright_flag=1;
  }
  else
  {
    getright_flag=0;
    getRight_flag[hang]=0;
  }
  Left_Count = Middle;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count > ccd_start)
  {
    _black_L = Left_Count; 
    getLeft_flag[hang]=1;
    getleft_flag=1;
  } 
  else
  {
    getleft_flag=0;
    getLeft_flag[hang]=0;
  }
  if(Left_Count==Middle||Right_Count==Middle)//中间部分为黑色
  {
//    Center_Flag[hang]=0;
    getright_flag=0;
    getRight_flag[hang]=0;
    getleft_flag=0;
    getLeft_flag[hang]=0;//flag先清零
    
    if(dir_flag == Left_turn)
    {
      Right_Count = Middle;//
      while(!(ImageData[Right_Count-3]==0 
              && ImageData[Right_Count-2]==0
                && ImageData[Right_Count-1]==0)
            && Right_Count > ccd_start)	  
        Right_Count--;
      if(Right_Count > ccd_start)
      {
        _black_R = Right_Count; 
        getRight_flag[hang]=1;
        getright_flag=1;
      } 
      else
      {
        Left_Count = Middle;//把黑线中间值赋给右计数起点
        while(!(ImageData[Left_Count+3]==0 
                && ImageData[Left_Count+2]==0
                  && ImageData[Left_Count+1]==0)
              && Left_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
          Left_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停

        if(Left_Count<ccd_end)//如果在有效范围内
        {
          _black_L = Left_Count;
          getLeft_flag[hang]=1;
          getleft_flag=1;
        }
      }
    }
    else if(dir_flag == Right_turn)
    {
      Left_Count = Middle;//把黑线中间值赋给右计数起点
      while(!(ImageData[Left_Count+3]==0 
              && ImageData[Left_Count+2]==0
                && ImageData[Left_Count+1]==0)
            && Left_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Left_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停

      if(Left_Count<ccd_end)//如果在有效范围内
      {
        _black_L = Left_Count;
        getLeft_flag[hang]=1;
        getleft_flag=1;
      }
      else
      {
        Right_Count = Middle;//
        while(!(ImageData[Right_Count-3]==0 
                && ImageData[Right_Count-2]==0
                  && ImageData[Right_Count-1]==0)
              && Right_Count > ccd_start)	  
          Right_Count--;
        if(Right_Count > ccd_start)
        {
          _black_R = Right_Count; 
          getRight_flag[hang]=1;
          getright_flag=1;
        } 
      }
    }
  }
//  else Center_Flag[hang]=1;
  

  if(getleft_flag==0 && getright_flag==0)//左右边界都没有找到
  {
    Middle = Three_Lie[1];
    if(sum_point(ImageData_in,40)>310)//视野全黑
    {
      if(centre[hang]<160&&Black_Lock==Unlock)
      {
        Black_Lock=Left_Lock;
      }
      else if(centre[hang]>160&&Black_Lock==Unlock)
      {
        Black_Lock=Right_Lock;
      }
    }//不做任何动作
    else 
    {
      Black_Lock=Unlock;
    }
    
  }
  else if(getleft_flag!=1 && getright_flag==1)//找到右边界
  {
    Black_Lock=Unlock;
    Middle = _black_R-_halfwidth;//黑线中间位置为右边界-黑线宽的一半
    _black_L = _black_R - _halfwidth*2;//黑线左边位置为右边界-黑线宽
//    Middle = _black_R-((hang - Far_Point)*stand_half_k + Far_Half);//黑线中间位置为右边界-黑线宽的一半
//    _black_L = 0;//_black_R - _halfwidth*2;//黑线左边位置为右边界-黑线宽
//    _halfwidth = 160;
  }
  else if(getleft_flag==1 && getright_flag!=1)//找到左边界
  {
    Black_Lock=Unlock;
    Middle = _black_L+_halfwidth;
    _black_R = _black_L + _halfwidth*2;
//    Middle = _black_L+((hang - Far_Point)*stand_half_k + Far_Half);
//    _black_R = 319;//_black_L + _halfwidth*2;
//    _halfwidth = 160;
  }
  else if(getleft_flag==1 && getright_flag==1) //左右边界都找到
  {
    Black_Lock=Unlock;
      _halfwidth=(int)((_black_R - _black_L)/2.0); //如果检测到的左右差值超出160，取中间位置
    if(_halfwidth < 80)//宽度限幅 
      _halfwidth = 80;
    else if(_halfwidth >120)
      _halfwidth = 120; 
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  if(Middle<10) //中心点限幅 
    Middle=10;
  else if(Middle>310)
    Middle=310;
  
  //data record 记录参数到数组中
  if(Middle>=160)
  {
    dir_flag = Right_turn;
  }
  else if(Middle<160)
  {
    dir_flag = Left_turn;
  }
  centre[hang] = Middle + Center_correct(hang);
  if(_black_L>319)_black_L=319;
  else if(_black_L<0)_black_L=0;
  black_L[hang] = _black_L;
  if(_black_R>319)_black_R=319;
  else if(_black_R<0)_black_R=0;
  black_R[hang] = _black_R;
  halfwidth[hang] = _halfwidth;
  
// Right_count[hang]=Right_Count;
//  Left_count[hang]=Left_Count;

}


void get_island_center(unsigned char *ImageData_in,int hang,u8 half_width)
{
  u8 i;
  const int ccd_start=100,ccd_end=230;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  for(i=0;i<40;i++)
  for(int k=0;k<8;k++)
    ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  if(Island_Flag_Lock == Left_Island)
  {
    Left_Count = 20;//把黑线中间值赋给右计数起点
    while(!(ImageData[Left_Count+3]==1 
      && ImageData[Left_Count+2]==1
        && ImageData[Left_Count+1]==1)
        && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Left_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    centre[hang]=Left_Count-half_width + Center_correct(hang);
//      centre[hang]=Left_Count/2;
  }
  else if(Island_Flag_Lock == Right_Island)
  {
    Right_Count = 300;//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count-3]==1 
      && ImageData[Right_Count-2]==1
        && ImageData[Right_Count-1]==1)
        && Right_Count > ccd_start)//如果在有效区内没有找到连续三个黑点
      Right_Count--;//从中间位置开始，往右数，发现往右三点都是黑点停
    centre[hang]=Right_Count+half_width + Center_correct(hang);
//      centre[hang]=(Right_Count+319)/2;
  }
}

//void New_get_island_center(void)
//{
//  if(Island_Flag_Lock == Left_Island)
//  {
//    
//  }
//  else if(Island_Flag_Lock == Right_Island)
//  {
//    
//  }
//}

void get_three_lie(void)
{
  u16 left_lie=Three_Lie[0],middle_lie=Three_Lie[1],right_lie=Three_Lie[2];
  u8 Left_flag=0,Right_flag=0,middle_flag=0;
  u8 Left_point,Middle_point,Right_point;
  Left_point=239;
  while(!(Image_Point(Left_point,left_lie)==1
          &&Image_Point(Left_point-1,left_lie)==1
            &&Image_Point(Left_point-2,left_lie)==1)&&Left_point>=2)
    Left_point--;
  if(Left_point!=239)//左线初始不为黑
  {
    Left_flag=1;
  }
  Three_lie_end[0]=Left_point;
  Middle_point=239;
  while(!(Image_Point(Middle_point,160)==1
          &&Image_Point(Middle_point-1,160)==1
            &&Image_Point(Middle_point-2,160)==1)&&Middle_point>=2)
    Middle_point--;
  if(Middle_point!=239)//左线初始不为黑
  {
    middle_flag=1;
  }
  Three_lie_end[1]=Middle_point;
  Right_point=239;
  while(!(Image_Point(Right_point,right_lie)==1
          &&Image_Point(Right_point-1,right_lie)==1
            &&Image_Point(Right_point-2,right_lie)==1)&&Right_point>=2)
    Right_point--;
  if(Right_point!=239)//左线初始不为黑
  {
    Right_flag=1;
  }
  Three_lie_end[2]=Right_point;
  if(Left_flag==0)
    Road_Status_Flag=Right_turn;
  else if(Right_flag==0)
    Road_Status_Flag=Left_turn;
  else 
  {
    if(Left_point<Middle_point&&Middle_point<Right_point)
    {
      Road_Status_Flag=Left_turn;
    }
    else if(Left_point>Middle_point&&Middle_point>Right_point)
    {
      Road_Status_Flag=Right_turn;
    }
  }
}

u8 far_near_lie(void)
{
  u8 i;
  u16 lie[10]={144,148,142,150,140,152,138,154,136,156};
  u8 flag[10]={0};
  u8 point[10];
  u8 center_;
  for(i=0;i<10;i++)
  {
    point[i] = Three_lie_end[1];
    while(!(Image_Point(point[i],lie[i])==1
            &&Image_Point(point[i]-1,lie[i])==1
              &&Image_Point(point[i]-2,lie[i])==1)&&point[i]>=2)
      point[i]--;
    if(point[i]!=239)//左线初始不为黑
    {
      flag[i]=1;
    }
  }
  center_ = lie[min_u8_index(point,10)] + Center_correct(hang);
  return center_;
}


void far_cen2diff(int center)
{
  Far_Diff = 160 - center;
}


u8 CenterlineToDiff(int center)
{
  float FangCha=0;
  static u8 i=0;
  static u8  center_period = 0;
  static u16 center_old[Center_Filter_Period];
  static u16 center_ave = 0;
  if(diff_done_flag == 1)return 0;

  
  if(Island_Flag_Lock==0&&island_flag_temp == Left_Island && center>150)
    center = 150;
  else if(Island_Flag_Lock==0&&island_flag_temp == Right_Island && center<170)
    center = 170;
  else if(Island_Flag_Lock==0&&Three_lie_end[1]<65)
  {
    if(center>170)
      center = 170;
    else if(center<150)
      center = 150;
  }
    
  
  if(Island_Flag_Lock == Left_Island)
  {
    if(center>150&&Island_In_Flag == 1)
    {
//      center -= (island_in_plus);
      if((center_ave/Center_Filter_Period)<130)//中间值突变点
//      if(abs_s16(center_ave/10-center)>25)
      {
        center = center_ave/Center_Filter_Period;
//        out_island_test_flag = 1;
//        LED4 = 0;
      }
    }
//    center -= island_in_plus;
//    if(out_island_test_flag==1)
//    {
//      if(Island_out_flag == 0)//延时标志
//      {
//        Island_out_flag = 1;
//        Island_out_delay = island_delay_time;
//      }
//    }
  }
  else if(Island_Flag_Lock == Right_Island)
  {
    if(center<170&&Island_In_Flag == 1)
    {
//      center += (island_in_plus);
      if((center_ave/Center_Filter_Period)>220)
//      if(abs_s16(center_ave/10-center)>25)
      {
        center = center_ave/Center_Filter_Period;
//        out_island_test_flag = 1;
//        LED4 = 0;
      }
    }
    else if(center<240)
    {
//      center += island_in_plus;
    }
//    center += island_in_plus;
//    if(out_island_test_flag==1)
//    {
//      if(Island_out_flag == 0)//延时标志
//      {
//        Island_out_flag = 1;
//        Island_out_delay = island_delay_time;
//      }
//    }
  }

//***************************************************使用加权平均值的转弯*************************************  
  if(center-CAMERA_W/2>=0)//右转判定
  {
//      Road_Status_Flag=Right_turn; 
      if(LCD_DISPLAY_FLAG==1)
      LCD_PutChar(300,10,'R',Red,White);
  }
  else                       //左转判定   
  {
//      Road_Status_Flag=Left_turn;
      if(LCD_DISPLAY_FLAG==1)
      LCD_PutChar(300,10,'L',Red,White);
  }
  if(center_period > (Center_Filter_Period - 1))
  center_period = 0;
  
  center_ave -= center_old[center_period];
  center_old[center_period] = center;
  center_ave += center_old[center_period];
  center_period++;  
  
  diff_done_flag = 1;
  if(Island_In_Flag == 1)
  {
//    Diff_error = 160 - center_ave/Center_Filter_Period;
//    Diff_error = 
//    if(i<200)
//    {
//      fangcha_test[i++]=center;
//    }
//    else
//    {
//      FangCha = calculate_fangcha(fangcha_test,200);
//      sprintf(str_temp,"%d      ",(int)FangCha);
//      uart_putstr(UART1,str_temp);
////      printf("fangcha is %f",FangCha);
//    }
    Diff_error = 160 - center;
    Diff_error = filter_1st(Diff_error,&Center_Filter);
  }
  else
  {
    Diff_error = 160 - center;
  }
//  Servo_PID(center);
}

//u8 CenterlineToDiff(int center)
//{
//  u8 i;
//  int cnt = 0;//防止一直卡在寻找中心点突变的循环里
//  static unsigned char Center_period = 0;//速度控制周期变量
//  static u8 diff_period = 0;
//  static int center_diff[Centerdiff_const];
//  static int center_diff_sum = 0;
//  static float center_diff_ave = 0;
//  static int center_queue[255]={0};//中心点队列
//  const float Q = 0.5;
////  ASSERT()
//  center_queue[Center_period] = center;
//  static int diff_err = 0;
//  CHABIHE = chabihe();
//  if(diff_done_flag == 1)return 0;//如果该操作之前已经被执行了，那么之后将不再执行
//  
////***************************************************使用加权平均值的转弯*************************************  
//  if(center-CAMERA_W/2>=0)//右转判定
//  {
////      Road_Status_Flag=Right_turn; 
//      if(LCD_DISPLAY_FLAG==1)
//      LCD_PutChar(300,10,'R',Red,White);
//  }
//  else                       //左转判定   
//  {
////      Road_Status_Flag=Left_turn;
//      if(LCD_DISPLAY_FLAG==1)
//      LCD_PutChar(300,10,'L',Red,White);
//  }
//  diff_err   =  CAMERA_W/2-center;
////  Diff_error = diff_err;
//   //差比和 和 中心点的乘积大于0（方向相反），不在环岛里，且AD差值大于误差允许范围
//  if(abs_s16(diff_err>30)&&(abs_s16(R_AD_Ave-L_AD_Ave))>(stand_AD/10)&&(Island_Flag_Lock == 0)&&(chabihe()*diff_err)>0)
//  {
//    i = Center_period;
//    while((cnt++)<20)//最多回溯到20*6 =120ms之前
//    {
//      if(abs_s16( center_queue[(u8)i]-center_queue[(u8)(i-1)] > center_diff_ave*3))//找中心点突变点
//      {
//        break;
//      }
//      i--;
//    }
//    CHABIHE = (CAMERA_W/2 - center_queue[(u8)(i-1)])*Q-chabihe()*(1-Q);
//    Diff_error = CHABIHE;
//    diff_done_flag = 1;
//  }
//  else
//  {
//    if(diff_period >= (Centerdiff_const))   
//    diff_period = 0; 
//    center_diff_sum -= center_diff[diff_period];
//    center_diff[diff_period] = abs_s16(center_queue[(u8)Center_period] - center_queue[(u8)(Center_period-1)]);
//    center_diff_sum += center_diff[diff_period];//中心差值队列更新
//    center_diff_ave = center_diff_sum*1.0/Centerdiff_const;
//    
//    Diff_error = diff_err;
//    diff_done_flag = 1;
//  }
//  Center_period++;//队列标志更新
//  
//  
////  Diff_error = diff_err;
////  if(test_flag == go_str)
////  {
//////    if(diff_err>10)diff_err = 10;//直道限幅
//////    else if(diff_err<-10)diff_err = -10;
////    old_center = center;
////    Diff_error = diff_err;
////    diff_done_flag = 1;
////    return 1;
////  }
////  else if(test_flag == turn_left)//左转
////  {
////    if(diff_err>0)//中心点在左侧
////    {
////      old_center = center;
////      Diff_error = diff_err;
////      diff_done_flag = 1;
////      return 1;
////    }
////    else 
////    {
////      Diff_error = CAMERA_W/2-old_center;
////      diff_done_flag = 1;
//////      hang = Straight_lie_end[1];
//////      get_black_line(Image_fire[hang],hang,((hang - Far_Point)*stand_half_k + Far_Half));
//////      diff_err = CAMERA_W/2-centre[hang];
////      return 0;
////    }
////  }
////  else if(test_flag == turn_right)//右转
////  {
////    if(diff_err<0)//中心点在右侧
////    {
////      old_center = center;
////      Diff_error = diff_err;
////      diff_done_flag = 1;
////      return 1;
////    }
////    else 
////    {
////      Diff_error = CAMERA_W/2-old_center;
//////      hang = Straight_lie_end[1];
//////      get_black_line(Image_fire[hang],hang,((hang - Far_Point)*stand_half_k + Far_Half));
////      diff_done_flag = 1;
////      return 0;
////    }
////  }
//  
////  Diff_error = 
//  
////  Servo_PID(center);
////  return 1;
//}



u8 Cross_Test(void)
{
  u8 i,cnt=0;
  for(i=0;i<5;i++)
  {
    if(sum_point(Image_fire[Start_Point+i],40)<=5)
      cnt++;
  }
  if(cnt>3)
  {//BEEP=1;
  return 1;}
  else 
  {//BEEP=0;
  return 0;}
}


//
///*************************************************************************
//*  函数名称：Straight_line_test
//*  功能说明：判断前方是否为直道
//*  参数说明：无
//*  函数返回：道路弯曲情况
//*  修改时间：2018-4-19    未测试
//*  备    注：//按照矫正后的图像从近往远找
//              //每一行计算出对应的在车前直线的点
//              //
//*************************************************************************/
//

u8  Straight_line_test(void)
{
  u8 Left_Straight_Flag = 0,Right_Straight_Flag = 0;//左右直道flag
  u8 Left_end_flag = 0,Middle_end_flag = 0,Right_end_flag = 0;//找到三线尽头flag
  u8 Left_end_cnt = 0, Middle_end_cnt = 0, Right_end_cnt = 0;//连续三个黑点则找到了尽头
  u8 Left_cnt = 0,Right_cnt = 0;//用来检测前方一米处是否为直道
  u8 i;
  get_three_lie();
  float Left_stand_k,Right_stand_k;
  int Left_calculate_point,Right_calculate_point;
  
  Left_stand_k = (Near_Left-_1m_Left)*1.0/(Start_Point-Far_Point);
  Right_stand_k = (Near_Right-_1m_Right)*1.0/(Start_Point-Far_Point);
  for(i=239;i>0;i--)
  {
    Left_calculate_point = (i - Far_Point)*Left_stand_k + _1m_Left;
    Right_calculate_point = (i - Far_Point)*Right_stand_k + _1m_Right;
    if(Image_Point(i,Left_calculate_point)==1)Left_end_cnt++;
    else Left_end_cnt = 0;
    if(Image_Point(i,160)==1)Middle_end_cnt++;
    else Middle_end_cnt = 0;
    if(Image_Point(i,Right_calculate_point)==1)Right_end_cnt++;
    else Right_end_cnt = 0;
      
    if(i>=Far_Point&&i<Start_Point)//测量前方一米
    {
      if(Image_Point(i,Left_calculate_point) == 0)Left_cnt++;
      if(Image_Point(i,Right_calculate_point) == 0)Right_cnt++;
    }
    
    if(Left_end_cnt==3)
    {
//      Three_lie_end_old[0] = Three_lie_end[0];
      Straight_lie_end[0] = i-2;
//      Three_lie_end_err[0] = Three_lie_end[0] - Three_lie_end_old[0];  //用来检测是否出现突变
      Left_end_flag = 1;
    }
    if(Middle_end_cnt==3)
    {
//      Three_lie_end_old[1] = Three_lie_end[1];
      Straight_lie_end[1] = i-2;
//      Three_lie_end_err[1] = Three_lie_end[1] - Three_lie_end_old[1];
      Middle_end_flag = 1;
    }
    if(Right_end_cnt==3)
    {
//      Three_lie_end_old[2] = Three_lie_end[2];
      Straight_lie_end[2] = i-2;
//      Three_lie_end_err[2] = Three_lie_end[2] - Three_lie_end_old[2];
      Right_end_flag = 1;
    }
    if(Left_end_flag&&Middle_end_flag&&Right_end_flag)break;
  }
  if(Left_cnt>(Start_Point-Far_Point)*0.95)Left_Straight_Flag = 1;
  if(Right_cnt>(Start_Point-Far_Point)*0.95)Right_Straight_Flag = 1;
  
//  if(Left_Straight_Flag == 0 && Right_Straight_Flag == 0)return 0;//左右都不直，不太现实……
//  else if(Left_Straight_Flag == 1 && Right_Straight_Flag == 0)return 1;//左直右不直，右边有弯道
//  else if(Left_Straight_Flag == 0 && Right_Straight_Flag == 1)return 2;//左不直右直，左边有弯道
  if(Left_Straight_Flag == 1 && Right_Straight_Flag == 1)     return go_str;//非常直
  else if(Three_lie_end[0] < Three_lie_end[2])          return turn_left;//左拐
  else if(Three_lie_end[0] > Three_lie_end[2])          return turn_right;//右拐
  else return 0;
}


//u8 abnormal_width_AD(int* ave_center,u8 *dir)
//{
//  
//}

u8 abnormal_width_AD(int* ave_center,u8 *dir)
{
  u16 sum_calculate_half = 0, sum_real_half = 0, sum_center = 0;
  const u8 all_hang = 15;
  u8 i;
  u8 left_cnt = 0,right_cnt = 0;
  float calculate_half;
//  get_black_line(Image_fire[AD_Near_hang+1],AD_Near_hang+1,HALF_NEAR);
  for(i=AD_Near_hang;i>AD_Near_hang-all_hang;i-=3)
  {
    calculate_half = (i - Far_Point)*stand_half_k + Far_Half;
    get_black_line(Image_fire[i],i,calculate_half);
    sum_calculate_half += calculate_half;
    sum_center += centre[i];
    sum_real_half += halfwidth[i];
//    if(black_L[i]<black_L[i+1]||(getLeft_flag[i]==0&&getLeft_flag[i+1]==1))
//    {
//      left_cnt++;
//    }
//    else if(black_R[i]>black_R[i+1]||(getRight_flag[i]==0&&getRight_flag[i+1]==1))
//    {
//      right_cnt++;
//    }
//    
//    if(left_cnt == 3)
//    {
//      *dir = Left_Island;
//      return 2;
//    }
//    else if(right_cnt == 3)
//    {
//      *dir = Right_Island;
//      return 2;
//    }
//    if(getLeft_flag[i]==1)left_cnt++;
//    if(getRight_flag[i]==1)right_cnt++;
  }
  if(abs_s16(sum_center/all_hang-Three_Lie[1])>0)//&&(abs_s16(sum_calculate_half-sum_real_half)/all_hang > 5))//半宽异常
  {
    *ave_center = sum_center/all_hang;
    if(*ave_center>Three_Lie[1])
    {
      *dir = Right_Island;
      return 2;
    }
    else if(*ave_center<Three_Lie[1])
    {
      *dir = Left_Island;
      return 2;
    }
  }
//  for(i=AD_Near_hang;i>AD_Near_hang-all_hang;i--)
//  {
//    Find_edge_flag(Image_fire[i],i,Right_Island);
//    Find_edge_flag(Image_fire[i],i,Left_Island);
//    if(getLeft_flag[i]==1)left_cnt++;
//    if(getRight_flag[i]==1)right_cnt++;
//  }
//  if(left_cnt>right_cnt)//右边边界较少
//  {
//    *dir = Right_Island;
//    return 2;
//  }
//  else
//  {
//    *dir = Left_Island;
//    return 2;
//  }
  return 0;
}

////流程：先检测左边黑色地带（尖角）
////向上依次寻找黑色地带每行寻找白色边缘
////向上找不到黑色地带后向下找黑色边缘
////


u8 double_AD(void)
{
//  if((L_AD_Ave>stand_AD_L*1.8)&&(R_AD_Ave>stand_AD_R*1.8))
  if((L_AD_Ave>2200)||(R_AD_Ave>5800))
  {
//    BEEP = 1;
    return 1;
  }
  else 
  {
//    BEEP = 0;
    return 0;
  }
}
//
////float Process_to_real(u8 hang, int length)
////{
////  
////}
//
//float Curvature(int x1, int y1, int x2, int y2, int x3, int y3)
//{
//  float S;
//  float K;
//  float L1, L2, L3;     //三角形三边的倒数
//  S  = ((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2;
//  S  = fabs(S);
//  L1 = myInvSqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
//  L2 = myInvSqrt((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3));
//  L3 = myInvSqrt((x2-x3)*(x2-x3) + (y2-y3)*(y2-y3));
//  K  = 30000*S*L1*L2*L3;
//  return K;
//}
//
float chabihe(void)
{
  return (R_AD_Ave - L_AD_Ave*stand_AD_R*1.0/stand_AD_L)*1.0/(R_AD_Ave + L_AD_Ave*stand_AD_R*1.0/stand_AD_L)*200;
}

void get_top_board(unsigned char *ImageData_in,u8 top_hang)//捕捉黑线  
{
  const int Middle=160;  //黑线中间默认为CENNTER
//  static u8  dir_flag = 0;
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
//  int getleft_flag=0,getright_flag=0;//找到左右标志0
//  int _black_R1,_black_R2,_black_L1,_black_L2;//图像边界
//  static int _halfwidth = 160;//黑线一半宽度默认80
//  static u8 first_run=0;//开跑点0
  int i=0;
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  if(ImageData[160]==0)//中间点为白色
  {
    Right_Count = Middle;//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count+3]==1 
            && ImageData[Right_Count+2]==1
              && ImageData[Right_Count+1]==1)
          && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    
    if(Right_Count<ccd_end)//如果在有效范围内
    {
      top_angel_board[top_hang][2] = Right_Count;
      top_angel_board_flag[top_hang][2] = 1;
      while(!(ImageData[Right_Count+3]==0 
        && ImageData[Right_Count+2]==0
          && ImageData[Right_Count+1]==0)
      && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
      if(Right_Count<ccd_end)//如果在有效范围内
      {
        top_angel_board[top_hang][3] = Right_Count;
        top_angel_board_flag[top_hang][3] = 1;
      }
      else
      {
        top_angel_board_flag[top_hang][3] = 0;
      }
    }
    else
    {
      top_angel_board_flag[top_hang][2] = 0;
      top_angel_board_flag[top_hang][3] = 0;
    }
    Left_Count = Middle;
    while(!(ImageData[Left_Count-3]==1 
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      top_angel_board[top_hang][1] = Left_Count;
      top_angel_board_flag[top_hang][1] = 1;
      while(!(ImageData[Left_Count-3]==0 
            && ImageData[Left_Count-2]==0
              && ImageData[Left_Count-1]==0)
          && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count>ccd_start)//如果在有效范围内
      {
        top_angel_board[top_hang][0] = Left_Count;
        top_angel_board_flag[top_hang][0] = 1;
      }
      else
      {
        top_angel_board_flag[top_hang][0] = 0;
      }
    }
    else
    {
      top_angel_board_flag[top_hang][1] = 0;
      top_angel_board_flag[top_hang][0] = 0;
    }
  }
  else
  {
    Right_Count = Middle;//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count+3]==0
            && ImageData[Right_Count+2]==0
              && ImageData[Right_Count+1]==0)
          && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    
    if(Right_Count<ccd_end)//如果在有效范围内
    {
      top_angel_board[top_hang][2] = Right_Count;
      top_angel_board_flag[top_hang][2] = 1;
      while(!(ImageData[Right_Count+3]==1 
        && ImageData[Right_Count+2]==1
          && ImageData[Right_Count+1]==1)
      && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
        Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
      if(Right_Count<ccd_end)//如果在有效范围内
      {
        top_angel_board[top_hang][3] = Right_Count;
        top_angel_board_flag[top_hang][3] = 1;
      }
      else
      {
        top_angel_board_flag[top_hang][3] = 0;
      }
    }
    else
    {
      top_angel_board_flag[top_hang][2] = 0;
      top_angel_board_flag[top_hang][3] = 0;
    }
    Left_Count = Middle;
    while(!(ImageData[Left_Count-3]==0
            && ImageData[Left_Count-2]==0
              && ImageData[Left_Count-1]==0)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      top_angel_board[top_hang][1] = Left_Count;
      top_angel_board_flag[top_hang][1] = 1;
      while(!(ImageData[Left_Count-3]==1
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
        Left_Count--;
      if(Left_Count>ccd_start)//如果在有效范围内
      {
        top_angel_board[top_hang][0] = Left_Count;
        top_angel_board_flag[top_hang][0] = 1;
      }
      else
      {
        top_angel_board_flag[top_hang][0] = 0;
      }
    }
    else
    {
      top_angel_board_flag[top_hang][1] = 0;
      top_angel_board_flag[top_hang][0] = 0;
    }
  }
}

void get_near_board(unsigned char *ImageData_in,u8 near_hang)//捕捉黑线  
{
  const int Middle=160;  //黑线中间默认为CENNTER
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int i=0;
  Right_Count = Middle;//把黑线中间值赋给右计数起点
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
    Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
  if(Right_Count<ccd_end)//如果在有效范围内
  {
    near_angel_board[near_hang][1] = Right_Count;
    near_angel_board_flag[near_hang][1] = 1;
  }
  else
  {
    near_angel_board_flag[near_hang][1] = 0;
  }
  Left_Count = Middle;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count>ccd_start)
  {
    near_angel_board[near_hang][0] = Left_Count;
    near_angel_board_flag[near_hang][0] = 1;
  }
  else
  {
    near_angel_board_flag[near_hang][0] = 0;
  }
}

//
///*************************************************************************
//*  函数名称：xierushizi（斜入十字）
//*  功能说明：判断前方是否斜入十字，以及做出的判断
//*  参数说明：无
//*  函数返回：推测的中心点
//*  修改时间：2018-4-24    未测试
//*  备    注：//
//*************************************************************************/
//
int xierushizi(void)
{
  int i,j,k,cnt;
  u8 find_L_flag = 0,find_R_flag = 0;
  const u8 left = 1,right = 2;
  Pixel top,near;
  u8 near_dir = 0;
  static u8 real_top[10][2];
  static u8 real_near[20];
  for(i=0;i<10;i++)
  {
    get_top_board(Image_fire[Three_lie_end[1]-4+i],i);//从远向近找
    if(i==0)
    {
      if(top_angel_board_flag[i][1])
        real_top[i][0] = top_angel_board[i][1];
      else 
        real_top[i][0] = 0;
      if(top_angel_board_flag[i][2])
        real_top[i][1] = top_angel_board[i][2];
      else 
        real_top[i][1] = 319;
    }
    if(i>0)
    {
      find_L_flag = find_R_flag = 0;
      for(j=0;j<4;j++)
      {
        if(top_angel_board_flag[i][j]==0)continue;
        else
        {
          if(find_L_flag==0&&abs_s16(top_angel_board[i][j]-real_top[i-1][0])<10)
          {
            real_top[i][0] = top_angel_board[i][j];
            find_L_flag = 1;
          }
          if(find_R_flag==0&&abs_s16(top_angel_board[i][j]-real_top[i-1][1])<10)
          {
            real_top[i][1] = top_angel_board[i][j];
            find_R_flag = 1;
          }
        }
      }
      if(real_top[i][0]==real_top[i][1])
      {
        top.x = Three_lie_end[1]-4+i;
        top.y = real_top[i][0];
        break;
      }
      if(find_L_flag == 0)real_top[i][0] = 0;
      if(find_R_flag == 0)real_top[i][1] = 319;
    }
  }
  if(i==11)
  {
    top.x = Three_lie_end[1]-4+i;
    top.y = (real_top[i][0]+real_top[i][1])/2;
  }
  for(i=0;i<20;i++)
  {
    get_near_board(Image_fire[Near_Point-i*3],i);//从近向远找
    if(near_dir==0)
    {
      if(near_angel_board_flag[i][0]==1)//找到左边界
      {
        near_dir = left;
        real_near[i]=near_angel_board[i][0];
      }
      else if(near_angel_board_flag[i][1]==1)
      {
        near_dir = right;
        real_near[i]=near_angel_board[i][1];
      }
    }
    else
    {
      if(near_dir==left)
      {
        if(near_angel_board_flag[i][0]==0||near_angel_board[i][0]<real_near[i-1])
        {
          near.x = Near_Point-i*3;
          near.y = real_near[i-1];
          break;
        }
        else 
        {
          real_near[i]=near_angel_board[i][0];
        }
      }
      else if(near_dir==right)
      {
        if(near_angel_board_flag[i][1]==0||near_angel_board[i][1]>real_near[i-1])
        {
          near.x = Near_Point-i*3;
          near.y = real_near[i-1];
          break;
        }
        else 
        {
          real_near[i]=near_angel_board[i][1];
        }
      }
    }
  }
  if(i==21)
  {
    if(near_dir==left)
      return 319;
    else if(near_dir==right)
      return 0;
    else 
      return 1000;//非法数据
  }
  if(near_dir==left)
    return top.y+(top.y-near.y);
  else if(near_dir==right)
    return top.y-(near.y-top.y);

}


u8 Find_Start_line(void)
{
  const u8 Start_line = 150;
  int cnt = 0;
  u8 line_cnt = 0;
  u8 dir = 0;
  if(Image_Point(Start_line,0)==0)//第一个点是白点
  {
    dir = 1;//找黑点
  }
  else//第一个点是黑点
  {
    dir = 0;//找白点
  }
  while(cnt<320)
  {
    if((Image_Point(Start_line,cnt+3)==dir)&&(Image_Point(Start_line,cnt+2)==dir)&&(Image_Point(Start_line,cnt+1)==dir))
    {
      line_cnt++;
      dir =! dir;
    }
    cnt++;
  }
  return line_cnt;
}

void Find_edge_flag(u8 *ImageData_in,u8 hang,u8 dir)
{
  int i;
  for(i=0;i<40;i++)
  for(int k=0;k<8;k++)
    ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  if(dir == Left_Island)
  {
    getLeft_flag[hang]=0;
    for(i=160;i>10;i--)
    {
      if((ImageData[i-3]==1)&&(ImageData[i-2]==1)&&(ImageData[i-1]==1))
      {
        black_L[hang] = i;
        getLeft_flag[hang]=1;
        break;
      }
    }
  }
  else if(dir == Right_Island)
  {
    getRight_flag[hang]=0;
    for(i=160;i<310;i++)
    {
      if((ImageData[i+3]==1)&&(ImageData[i+2]==1)&&(ImageData[i+1]==1))
      {
        black_R[hang] = i;
        getRight_flag[hang]=1;
        break;
      }
    }
  }
}

//u8 road_filter(void)
//{
//  u8 far_flag  = 0;
//  u8 near_flag = 0;
//  Find_edge_flag(Image_fire[Far_Point]  ,Far_Point  ,Left_Island);
//  Find_edge_flag(Image_fire[Far_Point]  ,Far_Point  ,Right_Island);
//  Find_edge_flag(Image_fire[Start_Point],Start_Point,Left_Island);
//  Find_edge_flag(Image_fire[Start_Point],Start_Point,Right_Island);
//  if(getRight_flag[])
//  
//}

u8 little_line(u8 hang,u8 dir)
{
  const u16 left_lie=40,right_lie=280;
  u8 Left_flag=0,Right_flag=0,middle_flag=0;
  u8 Left_point,Right_point;
  if(dir == Left_Island)
  {
    Left_point=hang+10;
    while(!(Image_Point(Left_point,left_lie)==1
            &&Image_Point(Left_point-1,left_lie)==1
              &&Image_Point(Left_point-2,left_lie)==1)&&Left_point>=hang-15)
      Left_point--;
    if(Left_point>(hang - 15))
    {
      return 1;
    }
  }
  else if(dir == Right_Island)
  {
    Right_point=hang+10;
    while(!(Image_Point(Right_point,right_lie)==1
            &&Image_Point(Right_point-1,right_lie)==1
              &&Image_Point(Right_point-2,right_lie)==1)&&Right_point>=hang-15)
      Right_point--;
    if(Right_point>(hang - 15))
    {
      return 1;
    }
  }
  return 0;
}

u8 little_black_line(unsigned char *ImageData_in,int hang,const int doublt_center)
{
  static int Middle=160;  //黑线中间默认为CENNTER
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int getleft_flag=0,getright_flag=0;//找到左右标志0
  int _black_R,_black_L;//黑线左右端
//  static int _halfwidth = 160;//黑线一半宽度默认80
  int i=0;

  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Right_Count = doublt_center;//把黑线中间值赋给右计数起点
  while(!(ImageData[Right_Count+3]==1 
          && ImageData[Right_Count+2]==1
            && ImageData[Right_Count+1]==1)
        && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
    Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
  
  if(Right_Count<ccd_end)//如果在有效范围内
  {
    _black_R = Right_Count;
    getRight_flag[hang]=1;
    getright_flag=1;
  }
  else
  {
    getright_flag=0;
    getRight_flag[hang]=0;
  }
  
  
  Left_Count = doublt_center;
  while(!(ImageData[Left_Count-3]==1 
          && ImageData[Left_Count-2]==1
            && ImageData[Left_Count-1]==1)
        && Left_Count > ccd_start)	  
    Left_Count--;
  if(Left_Count > ccd_start)
  {
    _black_L = Left_Count; 
    getLeft_flag[hang]=1;
    getleft_flag=1;
  } 
  else
  {
    getleft_flag=0;
    getLeft_flag[hang]=0;
  }

  if(getleft_flag==1 && getright_flag==1) //左右边界都找到
  {
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  else 
    return 0;
  if(Middle<10) //中心点限幅 
    Middle=10;
  else if(Middle>310)
    Middle=310;
  
  centre[hang] = Middle + Center_correct(hang);;
  return 1;
}

u8 In_Island_black_line(unsigned char *ImageData_in,int hang)
{
  static int Middle=160;  //黑线中间默认为CENNTER
  int ccd_start=10,ccd_end=310;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  int getleft_flag=0,getright_flag=0;//找到左右标志0
  int _black_R,_black_L;//黑线左右端
//  static int _halfwidth = 160;//黑线一半宽度默认80
  int i=0;
  
  for(i=0;i<40;i++)
    for(int k=0;k<8;k++)
      ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;
  
  Middle = Three_Lie[min_u8_index(Three_lie_end,3)];

    Right_Count = Middle;//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count+3]==1 
            && ImageData[Right_Count+2]==1
              && ImageData[Right_Count+1]==1)
          && Right_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Right_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    
    if(Right_Count<ccd_end)//如果在有效范围内
    {
      _black_R = Right_Count;
      getRight_flag[hang]=1;
      getright_flag=1;
    }
    else
    {
      getright_flag=0;
      getRight_flag[hang]=0;
    }
    Left_Count = Middle;
    while(!(ImageData[Left_Count-3]==1 
            && ImageData[Left_Count-2]==1
              && ImageData[Left_Count-1]==1)
          && Left_Count > ccd_start)	  
      Left_Count--;
    if(Left_Count > ccd_start)
    {
      _black_L = Left_Count; 
      getLeft_flag[hang]=1;
      getleft_flag=1;
    } 
    else
    {
      getleft_flag=0;
      getLeft_flag[hang]=0;
    }


  if(getleft_flag==1 && getright_flag==1) //左右边界都找到
  {
    Middle = (int)((_black_R + _black_L)/2.0);
  }
  else 
    return 0;
  if(Middle<10) //中心点限幅 
    Middle=10;
  else if(Middle>310)
    Middle=310;
  
  centre[hang] = Middle + Center_correct(hang);
  
  if(_black_L>319)_black_L=319;
  else if(_black_L<0)_black_L=0;
  black_L[hang] = _black_L;
  if(_black_R>319)_black_R=319;
  else if(_black_R<0)_black_R=0;
  black_R[hang] = _black_R;
  
  return 1;
}

int In_Island(u8 hang,int *Hang_)
{
  const u8 Stand_hang = 150;
  int i;
  int center = 0;
  int center_temp = 0;
  u8 cnt = 0;
  u8 edge=0,nor_edge=0;
  u8 start_edge = 239;
  for(i=AD_Near_hang;i>AD_Near_hang-91;i-=3)
  {
    In_Island_black_line(Image_fire[i],i);
    if(island_flag_temp == Left_Island)
    {
      if(getLeft_flag[i]==1)
      {
        if(nor_edge>0)
        {
          edge++;
          if(cnt<=5)
          {
            cnt++;
            start_edge = i;
            *Hang_ = i;
          }
        }
      }
      else
      {
  //      cnt[0]++;//在检测到黑边的情况下连续三次检测到白边
        nor_edge++;
      }
    }
    else if(island_flag_temp == Right_Island)
    {
      if(getRight_flag[i]==1)
      {
        if(nor_edge>0)
        {
          edge++;
          if(cnt<=5)
          {
            cnt++;
            start_edge = i;
            *Hang_ = i;
          }
        }
      }
      else
      {
  //      cnt[1]++;//在检测到黑边的情况下连续三次检测到白边
        nor_edge++;
      }
    }
  }
  
  if(start_edge!=239)
  {
    if(island_flag_temp == Left_Island)
    {
      center_temp = ave_s16(&black_L[start_edge],cnt);
      if(center_temp!=-1)
      {
        center = (center_temp*3 - (center_temp*3 - Three_Lie[1]*2)*(Island_Point - Stand_hang)/(*Hang_ - Stand_hang))/2;
      }
    }
    else if(island_flag_temp == Right_Island)
    {
      center_temp = ave_s16(&black_R[start_edge],cnt);
      if(center_temp!=-1)
      {
        center = (center_temp*3 - center_temp*3*(Island_Point - Stand_hang)/(*Hang_ - Stand_hang)+Three_Lie[1]*2)/2;
      }
    }
  }
  
  
  if(edge!=0&&nor_edge!=0)//&&shunxu == 2)
  {
    return center;
  }
  else 
  {
    *Hang_ = -1;
    return -1;
  }
}
u8 New_abnormal_width_AD(int* ave_center,u8 *dir)
{
  int i;
  int center[2] = {0,312};
  int center_temp = 0;
  int start_edge[2]={239,239};
//  u8 error_flag = 0;
  u8 cnt[2] = {0};
  u8 edge[2]={0},nor_edge[2]={0};
  for(i=AD_Near_hang;i>AD_Near_hang-91;i-=3)
  {
    In_Island_black_line(Image_fire[i],i);
    if(getLeft_flag[i]==1)
    {
      if(nor_edge[0]>0)
      {
        edge[0]++;
        if(cnt[0]<=5)
        {
          cnt[0]++;
          start_edge[0] = i;
        }
      }
    }
    else
    {
//      cnt[0]++;//在检测到黑边的情况下连续三次检测到白边
      nor_edge[0]++;
    }
    if(getRight_flag[i]==1)
    {
      if(nor_edge[1]>0)
      {
        edge[1]++;
        if(cnt[1]<=5)
        {
          cnt[1]++;
          start_edge[1] = i;
        }
      }
    }
    else
    {
//      cnt[1]++;//在检测到黑边的情况下连续三次检测到白边
      nor_edge[1]++;
    }
  }
  if(start_edge[0]!=239)
  {
    center_temp = ave_s16(&black_L[start_edge[0]],cnt[0]);
    if(center_temp!=-1)
    {
      center[0] = center_temp*3;
    }
  }
  if(start_edge[1]!=239)
  {
    center_temp = ave_s16(&black_R[start_edge[1]],cnt[1]);
    if(center_temp!=-1)
    {
//      *ave_center = 160;
      center[1] = center_temp*3;
    }
  }
  
  
  if(edge[0]!=0&&nor_edge[0]!=0&&abs_s16(center[0]-Three_Lie[1])<abs_s16(center[1]-Three_Lie[1]))//&&shunxu == 2)
  {
    *dir = Left_Island;
    return 2;
  }
  else if(edge[1]!=0&&nor_edge[1]!=0&&abs_s16(center[1]-Three_Lie[1])<abs_s16(center[0]-Three_Lie[1]))
  {
    *dir = Right_Island;
    
    return 2;
  }
  return 0;
}

int Cross_correct(void)
{
  int center;
  u16 left_lie=41 ,right_lie=261;
  u8 Left_flag=0,Right_flag=0;
  u8 Left_point,Right_point;
  int left_edge=0,right_edge=319;
  u8 index = 0;
  u8 Near[2];
  u8 edge_cnt = 0;
  u8 white_line_flag = 0;
  u8 white_cnt = 0;
  u8 i;
  int Sum_Point = 0;
  Left_point=239;
  while(!(Image_Point(Left_point,left_lie)==1
          &&Image_Point(Left_point-1,left_lie)==1
            &&Image_Point(Left_point-2,left_lie)==1)&&Left_point>=2)
    Left_point--;
  if(Left_point!=239)//左线初始不为黑
  {
    Left_flag=1;
  }
  Near[0] = Left_point;
  Right_point=239;
  while(!(Image_Point(Right_point,right_lie)==1
          &&Image_Point(Right_point-1,right_lie)==1
            &&Image_Point(Right_point-2,right_lie)==1)&&Right_point>=2)
    Right_point--;
  if(Right_point!=239)//左线初始不为黑
  {
    Right_flag=1;
  }
  Near[1] = Right_point;
  
  for(i=140;i>61;i-=4)
  {
    Sum_Point = sum_point(Image_fire[i],40);
    if(Sum_Point<5)
    white_cnt++;
    if(white_cnt>0&&Sum_Point>10&&Image_Point(i,Three_Lie[1])==1)
    {
      white_line_flag = 1;
      get_angel_edge(Image_fire[i],i,&left_edge,&right_edge);
      break;
    }
  }
  
  
  if(white_line_flag==1)
  {
    Cross_correct_flag = 1;//十字修正
    if(Diff_PID_ave>0)
    {
      center = left_edge - HALF_WIDTH;
    }
    else
    {
      center = right_edge + HALF_WIDTH;
    }
    return center;
  }
  else 
    if(max_u8(Near,2)>150)
  {
    Cross_correct_flag = 1;//十字修正
    index = max_u8_index(Near,2);
    if(index==0)//左角
    {
      center = left_lie + Near[index] + Center_correct(Near[index]);
    }
    else if(index==1)//右角
    {
      center = right_lie - Near[index] + Center_correct(Near[index]);
    }
    return center;
  }
  else 
  {
    Cross_correct_flag = 0;
    return -1;
  }
}

void get_angel_edge(unsigned char *ImageData_in,int hang,int *left,int *right)
{
  u8 i;
  const int ccd_start=100,ccd_end=230;  //ccd扫描起点10，终点310   
  int Left_Count=0,Right_Count=0;//左右计数为0
  for(i=0;i<40;i++)
  for(int k=0;k<8;k++)
    ImageData[i*8+k] = (ImageData_in[i]>>(7-k))&0x01;

    Left_Count = 20;//把黑线中间值赋给右计数起点
    while(!(ImageData[Left_Count+3]==1 
      && ImageData[Left_Count+2]==1
        && ImageData[Left_Count+1]==1)
        && Left_Count < ccd_end)//如果在有效区内没有找到连续三个黑点
      Left_Count++;//从中间位置开始，往右数，发现往右三点都是黑点停
    *left = Left_Count+Center_correct(hang);


    Right_Count = 300;//把黑线中间值赋给右计数起点
    while(!(ImageData[Right_Count-3]==1 
      && ImageData[Right_Count-2]==1
        && ImageData[Right_Count-1]==1)
        && Right_Count > ccd_start)//如果在有效区内没有找到连续三个黑点
      Right_Count--;//从中间位置开始，往右数，发现往右三点都是黑点停
    *right = Right_Count+Center_correct(hang);

}

