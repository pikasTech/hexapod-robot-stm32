#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include "genetic.h"
#include "action_task_723.h"
#include "FreeRTOS.h"
#include "task.h"
//用于求解函数y=x^6-10x^5-26x^4+344x^3+193x^2-1846x-1680在（-8，8）之间的最小值

struct gen                //定义染色体结构
{
    int info[4];                //染色体结构，用一整型数的后14位作为染色体编码
    float suitability;      //次染色体所对应的适应度函数值，在本题中为表达式的值
};
struct gen gen_group[SUM];//定义一个含有20个染色体的组
struct gen gen_new[SUM];
struct gen gen_family[2*SUM];

struct gen gen_result;    //记录最优的染色体
int result_unchange_time; //记录在genetic_error前提下最优值为改变的循环次数

struct log                //形成链表，记录每次循环所产生的最优的适应度
{
    float suitability;
    struct log *next;
}llog,*head,*end;
int log_num;              //链表长度

/**************函数声明******************/
void initiate(void);            //初始化函数，主要负责产生初始化种群
void evaluation(int flag,int flag_move);    //评估种群中各染色体的适应度，并据此进行排序
void cross(void);               //交叉函数
void selection(void);           //选择函数
int  record(void);              //记录每次循环产生的最优解并判断是否终止循环
void mutation(int flag_move);           //变异函数
void showresult(int);       //显示结果
//-----------------------以上函数由主函数直接调用
int   randsign(float p);    //按照概率p产生随机数0、1，其值为1的概率为p
int   randbit(int i,int j); //产生一个在i，j两个数之间的随机整数
int   randnum(void);            //随机产生一个由14个基因组成的染色体
int   convertionD2B(float x,float min,float max);//对现实解空间的可能解x进行二进制编码（染色体形式）
float convertionB2D(int x,float min,float max); //将二进制编码x转化为现实解空间的值
int   createmask(int a);    //用于交叉操作

void gengetic_main(void)
{
    int i;
    initiate();             //产生初始化种群
    evaluation( 0 ,0);      //对初始化种群进行评估、排序
    for( i = 0 ; i < MAXloop ; i++ )
    {
        cross();            //进行交叉操作
        evaluation( 1 ,0);  //对子种群进行评估、排序
        selection();        //对父子种群中选择最优的NUM个作为新的父种群
        if( record() == 1 ) //满足终止规则1，则flag=1并停止循环
        {
            break;
        }
        mutation(1);            //变异操作
    }
//  showresult( flag );     //按照flag显示寻优结果
}

void initiate(void)
{
    int i , stime,n;
    long ltime;
    ltime=systime+13547;
    stime=(unsigned)ltime/2;
    srand(stime);
    for( i = 0 ; i < SUM ; i++ )
    {
        for(n=0;n<4;n++)gen_group[i].info[n] = randnum();       //调用randnum()函数建立初始种群
    }
    for(n=0;n<4;n++)gen_result.suitability=1000;
    result_unchange_time=0;
//  head=(struct log *)malloc(sizeof(llog));//初始化链表
//  end=(struct log *)malloc(sizeof(llog));//初始化链表
//  if(head==NULL)
//  {
//      //printf("\n内存不够！\n");
//      exit(0);
//  }
//  end->next = NULL;
    log_num = 1;
}
float sum_adc=0;

float fitiniss_stand(int info[4])
{
    int k;

            sum_adc=0;
            H_init = convertionB2D( info[0], -18, -8);
            L_init = convertionB2D( info[1], 9, 11);
            flag_action_run_single=1;
            vTaskDelay(1500);
            for (k=0;k<30;k++)
            {
                vTaskDelay(50);
                sum_adc+=RF_s32.adc2+RF_s32.adc3    \
                                +RM_s32.adc2+RM_s32.adc3    \
                                +RB_s32.adc2+RB_s32.adc3    \
                                +LF_s32.adc2+LF_s32.adc3    \
                                +LM_s32.adc2+LM_s32.adc3    \
                                +LB_s32.adc2+LB_s32.adc3    \
                                ;
            }
            return sum_adc;
}

float fitiniss_move(int info[4])
{
    int i/*, k*/;
//  int test_time;
    int test_times=0;
//  int test_step;
    int STEP_number_last;
            sum_adc=0;
            test_times=0;
            Action_T_per_V = convertionB2D( info[2], 8000, 13000);
            Rf = convertionB2D( info[3], 0, 1);
            Rz = convertionB2D( info[3], 0, 1);
            flag_action_run_continiue=1;

            for(i=0;i<2;i++)
            {
                STEP_number_last=STEP_number;
                if(i==0)
                {
                //前进的运动命令
                Order[0]=0;
                Order[1]=speed;
                Order[2]=0;
                }

                else if(i==1)
                {
                Order[0]=0;
                Order[1]=-speed;
                Order[2]=0;
                }
                while (STEP_number<STEP_number_last+2)
                {
                    while(STA_STOP==1)vTaskDelay(30);
                    vTaskDelay(50);
                    sum_adc+=RF_s32.adc1+RF_s32.adc2+RF_s32.adc3    \
                                    +RM_s32.adc1+RM_s32.adc2+RM_s32.adc3    \
                                    +/*RB_s32.adc1*/+RB_s32.adc2+RB_s32.adc3    \
                                    +LF_s32.adc1+LF_s32.adc2+LF_s32.adc3    \
                                    +LM_s32.adc1+LM_s32.adc2+LM_s32.adc3    \
                                    +LB_s32.adc1+LB_s32.adc2+LB_s32.adc3    \
                                    ;
                    test_times++;
                }

                Order[0]=0;
                Order[1]=0;
                Order[2]=0;
                vTaskDelay(1000);
            }
            return sum_adc/test_times;

}


void evaluation(int flag,int flag_move)
{
    int i,j,n;
    struct gen *genp;
    int gentinfo[4];
    float gentsuitability;

    if( flag == 0 )         // flag=0的时候对父种群进行操作
        genp = gen_group;
    else genp = gen_new;
    for(i = 0 ; i < SUM ; i++)//计算各染色体对应的表达式值
    {
        if(flag_move==0)
        {
            genp[i].suitability=fitiniss_stand(genp[i].info);
        }
        else if(flag_move==1)
        {
            genp[i].suitability=fitiniss_move(genp[i].info);
        }

    }
    for(i = 0 ; i < SUM - 1 ; i++)//按表达式的值进行排序，
    {
        for(j = i + 1 ; j < SUM ; j++)
        {
            if( genp[i].suitability > genp[j].suitability )
            {
                for(n=0;n<4;n++)
                {
                    gentinfo[n] = genp[i].info[n];
                    genp[i].info[n] = genp[j].info[n];
                    genp[j].info[n] = gentinfo[n];
                }
                gentsuitability = genp[i].suitability;
                genp[i].suitability = genp[j].suitability;
                genp[j].suitability = gentsuitability;
            }
        }
    }
}

void cross(void)
{
    int i , j , k , n;
    int mask1 , mask2;
    int a[SUM];
    for(i = 0 ; i < SUM ; i++)  a[i] = 0;
    k = 0;
    for(i = 0 ; i < SUM ; i++)
    {
        if( a[i] == 0)
        {
            for( ; ; )//随机找到一组未进行过交叉的染色体与a[i]交叉
            {
                j = randbit(i + 1 , SUM - 1);
                if( a[j] == 0)  break;
            }
            if(randsign(crossp) == 1)       //按照crossp的概率对选择的染色体进行交叉操作
            {
                mask1 = createmask(randbit(0 , 14));        //由ranbit选择交叉位
                mask2 = ~mask1;             //形成一个类似 111000 000111之类的二进制码编码
                for(n=0;n<4;n++)
                {
                    gen_new[k].info[n] = ((gen_group[i].info[n]) & mask1 )+((gen_group[j].info[n]) & mask2);
                    gen_new[k+1].info[n]=((gen_group[i].info[n]) & mask2 )+((gen_group[j].info[n]) & mask1);
                }
                k = k + 2;
            }
            else        //不进行交叉
            {
                for(n=0;n<4;n++)
                {
                    gen_new[k].info[n]=gen_group[i].info[n];
                    gen_new[k+1].info[n]=gen_group[j].info[n];
                }
                k = k + 2;
            }
            a[i] = a[j] = 1;
        }
    }
}

void selection(void)
{
    int i , j , n;
    int gentinfo[4];
    float gentsuitability;
    for(i=0;i<SUM;i++)
    {
        for(n=0;n<4;n++)gen_family[i].info[n]=gen_group[i].info[n];
        gen_family[i].suitability=gen_group[i].suitability;

        for(n=0;n<4;n++)gen_family[i+SUM].info[n]=gen_new[i].info[n];
        gen_family[i+SUM].suitability=gen_new[i].suitability;
    }

        for(i = 0 ; i < 2*SUM - 1 ; i++)//按表达式的值进行排序，
    {
        for(j = i + 1 ; j < 2*SUM ; j++)
        {
            if( gen_family[i].suitability > gen_family[j].suitability )
            {
                for(n=0;n<4;n++)
                {
                    gentinfo[n] = gen_family[i].info[n];
                    gen_family[i].info[n] = gen_family[j].info[n];
                    gen_family[j].info[n] = gentinfo[n];
                }
                gentsuitability = gen_family[i].suitability;
                gen_family[i].suitability = gen_family[j].suitability;
                gen_family[j].suitability = gentsuitability;
            }
        }
    }

        for(i=0;i<SUM;i++)
    {
        for(n=0;n<4;n++)gen_group[i].info[n]=gen_family[i].info[n];
        gen_group[i].suitability=gen_family[i].suitability;
    }
        L_init_best=convertionB2D( gen_group[0].info[0],-18,-8);
        H_init_best=convertionB2D( gen_group[0].info[1], 9, 11);

}

int record(void)    //记录最优解和判断是否满足条件
{
    float x;
    int n;
//  struct log *r;
//  r=(struct log *)malloc(sizeof(llog));
//  if(r==NULL)
//  {
//      //printf("\n内存不够！\n");
//      exit(0);
//  }
//  r->next = NULL;
//  end->suitability = gen_group[0].suitability;
//  end->next = r;
//  end = r;
//  log_num++;

    x = gen_result.suitability - gen_group[0].suitability;
    if(x < 0)x = -x;
    if(x < genetic_error)
    {
        result_unchange_time++;
        if(result_unchange_time >= 50)return 1;
    }
    else
    {
        for(n=0;n<4;n++)gen_result.info[n] = gen_group[0].info[n];
        gen_result.suitability = gen_group[0].suitability;
        result_unchange_time=0;
    }
    return 0;
}

void mutation(int flag_move)
{
    int i , j , n;
    float gmp;
    int gentinfo[4];
    float gentsuitability;
    gmp = 1 - pow(1 - mp , 11);//在基因变异概率为mp时整条染色体的变异概率
    for(i = 3 ; i < SUM ; i++)
    {
        if(randsign(gmp) == 1)
        {
            if(flag_move==0)
            {
                gen_group[i].suitability=fitiniss_stand(gen_group[i].info);
            }
            else if(flag_move==1)
            {
                gen_group[i].suitability=fitiniss_move(gen_group[i].info);
            }
        }
    }
    for(i = 0 ; i < SUM - 1 ; i++)
    {
        for(j = i + 1 ; j < SUM ; j++)
        {
            if(gen_group[i].suitability > gen_group[j].suitability)
            {
                for(n=0;n<4;n++)
                {
                    gentinfo[n] = gen_group[i].info[n];
                    gen_group[i].info[n] = gen_group[j].info[n];
                    gen_group[j].info[n] = gentinfo[n];
                }
                gentsuitability = gen_group[i].suitability;
                gen_group[i].suitability = gen_group[j].suitability;
                gen_group[j].suitability = gentsuitability;
            }
        }
    }
    /*
    *为了提高执行速度，在进行变异操作的时候并没有直接确定需要进行变异的位
    *而是先以cmp概率确定将要发生变异的染色体，再从染色体中随进选择一个基因进行变异
    *由于进行选择和变异后父代种群的次序已被打乱，因此，在变异前后对种群进行一次排序
    */
}

//void showresult(int flag)//显示搜索结果并释放内存
//{
//  int i , j;
//  struct log *logprint,*logfree;
//  FILE *logf;
//  if(flag == 0)
//      //printf("已到最大搜索次数，搜索失败！");
//  else
//  {
//      //printf("当取值%f时表达式达到最小值为%f\n",convertionB2D(gen_result.info),gen_result.suitability);
//      //printf("收敛过程记录于文件log.txt");
//      if((logf = fopen("log.txt" , "w+")) == NULL)
//      {
//          //printf("Cannot create/open file");
//          exit(1);
//      }
//      logprint=head;
//      for(i = 0 ; i < log_num ; i = i + 5)//对收敛过程进行显示
//      {
//          for(j = 0 ; (j < 5) & ((i + j) < log_num-1) ; j++)
//          {
//              fprintf(logf , "%20f" , logprint->suitability);
//              logprint=logprint->next;
//          }
//          fprintf(logf,"\n\n");
//      }
//  }
//  for(i = 0 ; i< log_num ; i++)//释放内存
//  {
//      logfree=head;
//      head=head->next;
//      free(logfree);
//      fclose(logf);
//  }
//  getchar();
//}
//
int randsign(float p)//按概率p返回1
{
    if(rand() > (p * 32768))
        return 0;
    else return 1;
}
int randbit(int i, int j)//产生在i与j之间的一个随机数
{
    int a , l;
    l = j - i + 1;
    a = i + rand() * l / 32768;
    return a;
}
int randnum()
{
    int x;
    x = rand() / 2;
    return x;
}
float convertionB2D(int x,float min,float max)
{
    float y;
    y = x;
    y = y/16384.0f;//归一化，得到[0,1]的随机数
    y = y*(max-min);
    y = y+min;
    return y;

}
int convertionD2B(float x,float min,float max)
{
    int g;
    g = (x * 1000) + 8192;
    return g;
}
int createmask(int a)
{
    int mask;
    mask=(1 << (a + 1)) - 1;
    return mask;
}
