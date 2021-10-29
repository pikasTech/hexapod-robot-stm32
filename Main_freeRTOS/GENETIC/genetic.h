#ifndef __genetic_TASK_H
#define __genetic_TASK_H
#include "sys.h"
#define SUM 10            //总共的染色体数量
#define MAXloop 12000       //最大循环次数
#define genetic_error 0.001f        //若两次最优值之差小于此数则认为结果没有改变
#define crossp 0.7        //交叉概率
#define mp 0.01           //变异概率
void gengetic_main(void);
/**************函数声明******************/
void initiate(void);            //初始化函数，主要负责产生初始化种群
void evaluation(int flag,int flag_move);    //评估种群中各染色体的适应度，并据此进行排序
void cross(void);               //交叉函数
void selection(void);           //选择函数
int  record(void);              //记录每次循环产生的最优解并判断是否终止循环
void mutation(int flag_move);           //变异函数
void showresult(int);       //显示结果
int   randsign(float p);    //按照概率p产生随机数0、1，其值为1的概率为p
int   randbit(int i,int j); //产生一个在i，j两个数之间的随机整数
int   randnum(void);            //随机产生一个由14个基因组成的染色体
int   convertionD2B(float x,float min,float max);//对现实解空间的可能解x进行二进制编码（染色体形式）
float convertionB2D(int x,float min,float max); //将二进制编码x转化为现实解空间的值
int   createmask(int a);    //用于交叉操作


#endif
