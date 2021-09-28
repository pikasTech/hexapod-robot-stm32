#ifndef __genetic_TASK_H
#define __genetic_TASK_H
#include "sys.h"
#define SUM 10                //�ܹ���Ⱦɫ������
#define MAXloop 12000         //���ѭ������
#define genetic_error 0.001f  //����������ֵ֮��С�ڴ�������Ϊ���û�иı�
#define crossp 0.7            //�������
#define mp 0.01               //�������
void gengetic_main(void);
/**************��������******************/
void initiate(void);  //��ʼ����������Ҫ���������ʼ����Ⱥ
void evaluation(int flag, int flag_move);  //������Ⱥ�и�Ⱦɫ�����Ӧ�ȣ����ݴ˽�������
void cross(void);                          //���溯��
void selection(void);                      //ѡ����
int record(void);  //��¼ÿ��ѭ�����������ŽⲢ�ж��Ƿ���ֹѭ��
void mutation(int flag_move);  //���캯��
void showresult(int);          //��ʾ���
int randsign(float p);  //���ո���p���������0��1����ֵΪ1�ĸ���Ϊp
int randbit(int i, int j);  //����һ����i��j������֮����������
int randnum(void);  //�������һ����14��������ɵ�Ⱦɫ��
int convertionD2B(float x,
                  float min,
                  float max);  //����ʵ��ռ�Ŀ��ܽ�x���ж����Ʊ��루Ⱦɫ����ʽ��
float convertionB2D(int x, float min, float max);  //�������Ʊ���xת��Ϊ��ʵ��ռ��ֵ
int createmask(int a);  //���ڽ������

#endif
