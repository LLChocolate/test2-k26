

///*************************************************************************
//*  �������ƣ�out_island
//*  ����˵�����ڳ�������ʱ�����Ǳ�־
//*  ����˵������������
//*  �������أ��Ƿ���Ҫ����
//*  �޸�ʱ�䣺2018-4-16    �Ѳ���
//*  ��    ע��//���̣��ȼ����ߺ�ɫ�ش�����ǣ�
//              //��������Ѱ�Һ�ɫ�ش�ÿ��Ѱ�Ұ�ɫ��Ե
//              //�����Ҳ�����ɫ�ش��������Һ�ɫ��Ե
//*************************************************************************/

#include "image2.h"
u8 out_island(u8 Lock_island)
{
  u8 over_flag = 0;
  u8 find_dir = 0;
  u8 edge_j = 0;
  int px = 239,py;
//  u8 finish_flag = 0;
  if(Lock_island == Right_Island)
  {
    py = 0;
    if(Image_Point(239,0)==0)
    {
      find_dir = right;//�����½��Ǻ�ɫ�������Ұ�ɫ
      py++;
    }
    else 
    {
      while(!(Image_Point(px,py)==1
        &&Image_Point(px-1,py)==1
          &&Image_Point(px-2,py)==1)&&px>70)//�ҵ���ɫ�ش�
      px--;
      if(px>70)
        Island_mark_edge[edge_j++].x = px;
      else return 0;//���
    }
  }
  else if(Lock_island == Left_Island)
  {
    py = 319;
    if(Image_Point(239,0)==0)
    {
      find_dir = left;//�����½��Ǻ�ɫ�������Ұ�ɫ
      py--;
    }
    else 
    {
      while(!(Image_Point(px,py)==1
        &&Image_Point(px-1,py)==1
          &&Image_Point(px-2,py)==1)&&px>70)//�ҵ���ɫ�ش�
      px--;
      if(px>70)
        Island_mark_edge[edge_j++].x = px;
      else return 0;
    }
  }
  while(px>70)
  {
    if(Image_Point(px,py)==1)//��ǰΪ��ɫ����
    {
    switch(find_dir)
    {
    case left:
      while(!(Image_Point(px,py)==0
        &&Image_Point(px,py-1)==0
          &&Image_Point(px,py-2)==0)&&py>160)
      py--;
      if(py>160)
      Island_mark_edge[edge_j++].y1 = py;//��Ե��y����
      else 
//      Island_mark_edge[edge_j].dir |= left;//��Ե�����
      break;
    case right:
      while(!(Image_Point(px,py)==0
      &&Image_Point(px,py+1)==0
        &&Image_Point(px,py+2)==0)&&py<160)
      py--;
      if(py<160)
      Island_mark_edge[edge_j++].y1 = py;//��Ե��y����
//      Island_mark_edge[edge_j].dir |= left;//��Ե�����
      break;
    default:
      break;
    }
      px--;
      Island_mark_edge[edge_j].x = px;
    }
    else
    {
      over_flag = 1;
      island_addline_k = (Island_mark_edge[edge_j].y1-Island_mark_edge[0].y1)/(Island_mark_edge[edge_j].x-Island_mark_edge[0].x);
      break;
    }
  }
  return over_flag;
}

