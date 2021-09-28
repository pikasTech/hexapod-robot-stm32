#include "genetic.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "FreeRTOS.h"
#include "action_task_723.h"
#include "task.h"
//������⺯��y=x^6-10x^5-26x^4+344x^3+193x^2-1846x-1680�ڣ�-8��8��֮�����Сֵ

struct gen  //����Ⱦɫ��ṹ
{
    int info[4];  //Ⱦɫ��ṹ����һ�������ĺ�14λ��ΪȾɫ�����
    float suitability;  //��Ⱦɫ������Ӧ����Ӧ�Ⱥ���ֵ���ڱ�����Ϊ����ʽ��ֵ
};
struct gen gen_group[SUM];  //����һ������20��Ⱦɫ�����
struct gen gen_new[SUM];
struct gen gen_family[2 * SUM];

struct gen gen_result;     //��¼���ŵ�Ⱦɫ��
int result_unchange_time;  //��¼��genetic_errorǰ��������ֵΪ�ı��ѭ������

struct log  //�γ���������¼ÿ��ѭ�������������ŵ���Ӧ��
{
    float suitability;
    struct log* next;
} llog, *head, *end;
int log_num;  //��������

/**************��������******************/
void initiate(void);  //��ʼ����������Ҫ���������ʼ����Ⱥ
void evaluation(int flag, int flag_move);  //������Ⱥ�и�Ⱦɫ�����Ӧ�ȣ����ݴ˽�������
void cross(void);                          //���溯��
void selection(void);                      //ѡ����
int record(void);  //��¼ÿ��ѭ�����������ŽⲢ�ж��Ƿ���ֹѭ��
void mutation(int flag_move);  //���캯��
void showresult(int);          //��ʾ���
//-----------------------���Ϻ�����������ֱ�ӵ���
int randsign(float p);  //���ո���p���������0��1����ֵΪ1�ĸ���Ϊp
int randbit(int i, int j);  //����һ����i��j������֮����������
int randnum(void);  //�������һ����14��������ɵ�Ⱦɫ��
int convertionD2B(float x,
                  float min,
                  float max);  //����ʵ��ռ�Ŀ��ܽ�x���ж����Ʊ��루Ⱦɫ����ʽ��
float convertionB2D(int x, float min, float max);  //�������Ʊ���xת��Ϊ��ʵ��ռ��ֵ
int createmask(int a);  //���ڽ������

void gengetic_main(void) {
    int i;
    initiate();  //������ʼ����Ⱥ
    evaluation(0, 0);  //�Գ�ʼ����Ⱥ��������������
    for (i = 0; i < MAXloop; i++) {
        cross();  //���н������
        evaluation(1, 0);  //������Ⱥ��������������
        selection();  //�Ը�����Ⱥ��ѡ�����ŵ�NUM����Ϊ�µĸ���Ⱥ
        if (record() == 1)  //������ֹ����1����flag=1��ֹͣѭ��
        {
            break;
        }
        mutation(1);  //�������
    }
    //	showresult( flag );		//����flag��ʾѰ�Ž��
}

void initiate(void) {
    int i, stime, n;
    long ltime;
    ltime = systime + 13547;
    stime = (unsigned)ltime / 2;
    srand(stime);
    for (i = 0; i < SUM; i++) {
        for (n = 0; n < 4; n++)
            gen_group[i].info[n] = randnum();  //����randnum()����������ʼ��Ⱥ
    }
    for (n = 0; n < 4; n++)
        gen_result.suitability = 1000;
    result_unchange_time = 0;
    //	head=(struct log *)malloc(sizeof(llog));//��ʼ������
    //	end=(struct log *)malloc(sizeof(llog));//��ʼ������
    //	if(head==NULL)
    //	{
    //		//printf("\n�ڴ治����\n");
    //		exit(0);
    //	}
    //	end->next = NULL;
    log_num = 1;
}
float sum_adc = 0;

float fitiniss_stand(int info[4]) {
    int k;

    sum_adc = 0;
    H_init = convertionB2D(info[0], -18, -8);
    L_init = convertionB2D(info[1], 9, 11);
    flag_action_run_single = 1;
    vTaskDelay(1500);
    for (k = 0; k < 30; k++) {
        vTaskDelay(50);
        sum_adc += RF_s32.adc2 + RF_s32.adc3 + RM_s32.adc2 + RM_s32.adc3 +
                   RB_s32.adc2 + RB_s32.adc3 + LF_s32.adc2 + LF_s32.adc3 +
                   LM_s32.adc2 + LM_s32.adc3 + LB_s32.adc2 + LB_s32.adc3;
    }
    return sum_adc;
}

float fitiniss_move(int info[4]) {
    int i /*, k*/;
    //	int test_time;
    int test_times = 0;
    //	int test_step;
    int STEP_number_last;
    sum_adc = 0;
    test_times = 0;
    Action_T_per_V = convertionB2D(info[2], 8000, 13000);
    Rf = convertionB2D(info[3], 0, 1);
    Rz = convertionB2D(info[3], 0, 1);
    flag_action_run_continiue = 1;

    for (i = 0; i < 2; i++) {
        STEP_number_last = STEP_number;
        if (i == 0) {
            //ǰ�����˶�����
            Order[0] = 0;
            Order[1] = speed;
            Order[2] = 0;
        }

        else if (i == 1) {
            Order[0] = 0;
            Order[1] = -speed;
            Order[2] = 0;
        }
        while (STEP_number < STEP_number_last + 2) {
            while (STA_STOP == 1)
                vTaskDelay(30);
            vTaskDelay(50);
            sum_adc += RF_s32.adc1 + RF_s32.adc2 + RF_s32.adc3 + RM_s32.adc1 +
                       RM_s32.adc2 + RM_s32.adc3 +
                       /*RB_s32.adc1*/ +RB_s32.adc2 + RB_s32.adc3 +
                       LF_s32.adc1 + LF_s32.adc2 + LF_s32.adc3 + LM_s32.adc1 +
                       LM_s32.adc2 + LM_s32.adc3 + LB_s32.adc1 + LB_s32.adc2 +
                       LB_s32.adc3;
            test_times++;
        }

        Order[0] = 0;
        Order[1] = 0;
        Order[2] = 0;
        vTaskDelay(1000);
    }
    return sum_adc / test_times;
}

void evaluation(int flag, int flag_move) {
    int i, j, n;
    struct gen* genp;
    int gentinfo[4];
    float gentsuitability;

    if (flag == 0)  // flag=0��ʱ��Ը���Ⱥ���в���
        genp = gen_group;
    else
        genp = gen_new;
    for (i = 0; i < SUM; i++)  //�����Ⱦɫ���Ӧ�ı���ʽֵ
    {
        if (flag_move == 0) {
            genp[i].suitability = fitiniss_stand(genp[i].info);
        } else if (flag_move == 1) {
            genp[i].suitability = fitiniss_move(genp[i].info);
        }
    }
    for (i = 0; i < SUM - 1; i++)  //������ʽ��ֵ��������
    {
        for (j = i + 1; j < SUM; j++) {
            if (genp[i].suitability > genp[j].suitability) {
                for (n = 0; n < 4; n++) {
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

void cross(void) {
    int i, j, k, n;
    int mask1, mask2;
    int a[SUM];
    for (i = 0; i < SUM; i++)
        a[i] = 0;
    k = 0;
    for (i = 0; i < SUM; i++) {
        if (a[i] == 0) {
            for (;;)  //����ҵ�һ��δ���й������Ⱦɫ����a[i]����
            {
                j = randbit(i + 1, SUM - 1);
                if (a[j] == 0)
                    break;
            }
            if (randsign(crossp) == 1)  //����crossp�ĸ��ʶ�ѡ���Ⱦɫ����н������
            {
                mask1 =
                    createmask(randbit(0, 14));  //��ranbitѡ�񽻲�λ
                mask2 = ~mask1;  //�γ�һ������ 111000 000111֮��Ķ����������
                for (n = 0; n < 4; n++) {
                    gen_new[k].info[n] = ((gen_group[i].info[n]) & mask1) +
                                         ((gen_group[j].info[n]) & mask2);
                    gen_new[k + 1].info[n] = ((gen_group[i].info[n]) & mask2) +
                                             ((gen_group[j].info[n]) & mask1);
                }
                k = k + 2;
            } else  //�����н���
            {
                for (n = 0; n < 4; n++) {
                    gen_new[k].info[n] = gen_group[i].info[n];
                    gen_new[k + 1].info[n] = gen_group[j].info[n];
                }
                k = k + 2;
            }
            a[i] = a[j] = 1;
        }
    }
}

void selection(void) {
    int i, j, n;
    int gentinfo[4];
    float gentsuitability;
    for (i = 0; i < SUM; i++) {
        for (n = 0; n < 4; n++)
            gen_family[i].info[n] = gen_group[i].info[n];
        gen_family[i].suitability = gen_group[i].suitability;

        for (n = 0; n < 4; n++)
            gen_family[i + SUM].info[n] = gen_new[i].info[n];
        gen_family[i + SUM].suitability = gen_new[i].suitability;
    }

    for (i = 0; i < 2 * SUM - 1; i++)  //������ʽ��ֵ��������
    {
        for (j = i + 1; j < 2 * SUM; j++) {
            if (gen_family[i].suitability > gen_family[j].suitability) {
                for (n = 0; n < 4; n++) {
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

    for (i = 0; i < SUM; i++) {
        for (n = 0; n < 4; n++)
            gen_group[i].info[n] = gen_family[i].info[n];
        gen_group[i].suitability = gen_family[i].suitability;
    }
    L_init_best = convertionB2D(gen_group[0].info[0], -18, -8);
    H_init_best = convertionB2D(gen_group[0].info[1], 9, 11);
}

int record(void)  //��¼���Ž���ж��Ƿ���������
{
    float x;
    int n;
    //	struct log *r;
    //	r=(struct log *)malloc(sizeof(llog));
    //	if(r==NULL)
    //	{
    //		//printf("\n�ڴ治����\n");
    //		exit(0);
    //	}
    //	r->next = NULL;
    //	end->suitability = gen_group[0].suitability;
    //	end->next = r;
    //	end = r;
    //	log_num++;

    x = gen_result.suitability - gen_group[0].suitability;
    if (x < 0)
        x = -x;
    if (x < genetic_error) {
        result_unchange_time++;
        if (result_unchange_time >= 50)
            return 1;
    } else {
        for (n = 0; n < 4; n++)
            gen_result.info[n] = gen_group[0].info[n];
        gen_result.suitability = gen_group[0].suitability;
        result_unchange_time = 0;
    }
    return 0;
}

void mutation(int flag_move) {
    int i, j, n;
    float gmp;
    int gentinfo[4];
    float gentsuitability;
    gmp = 1 - pow(1 - mp, 11);  //�ڻ���������Ϊmpʱ����Ⱦɫ��ı������
    for (i = 3; i < SUM; i++) {
        if (randsign(gmp) == 1) {
            if (flag_move == 0) {
                gen_group[i].suitability = fitiniss_stand(gen_group[i].info);
            } else if (flag_move == 1) {
                gen_group[i].suitability = fitiniss_move(gen_group[i].info);
            }
        }
    }
    for (i = 0; i < SUM - 1; i++) {
        for (j = i + 1; j < SUM; j++) {
            if (gen_group[i].suitability > gen_group[j].suitability) {
                for (n = 0; n < 4; n++) {
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
     *Ϊ�����ִ���ٶȣ��ڽ��б��������ʱ��û��ֱ��ȷ����Ҫ���б����λ
     *��������cmp����ȷ����Ҫ���������Ⱦɫ�壬�ٴ�Ⱦɫ�������ѡ��һ��������б���
     *���ڽ���ѡ��ͱ���󸸴���Ⱥ�Ĵ����ѱ����ң���ˣ��ڱ���ǰ�����Ⱥ����һ������
     */
}

// void showresult(int flag)//��ʾ����������ͷ��ڴ�
//{
//	int i , j;
//	struct log *logprint,*logfree;
//	FILE *logf;
//	if(flag == 0)
//		//printf("�ѵ������������������ʧ�ܣ�");
//	else
//	{
//		//printf("��ȡֵ%fʱ����ʽ�ﵽ��СֵΪ%f\n",convertionB2D(gen_result.info),gen_result.suitability);
//		//printf("�������̼�¼���ļ�log.txt");
//		if((logf = fopen("log.txt" , "w+")) == NULL)
//		{
//			//printf("Cannot create/open file");
//			exit(1);
//		}
//		logprint=head;
//		for(i = 0 ; i < log_num ; i = i + 5)//���������̽�����ʾ
//		{
//			for(j = 0 ; (j < 5) & ((i + j) < log_num-1) ; j++)
//			{
//				fprintf(logf , "%20f" , logprint->suitability);
//				logprint=logprint->next;
//			}
//			fprintf(logf,"\n\n");
//		}
//	}
//	for(i = 0 ; i< log_num ; i++)//�ͷ��ڴ�
//	{
//		logfree=head;
//		head=head->next;
//		free(logfree);
//		fclose(logf);
//	}
//	getchar();
//}
//
int randsign(float p)  //������p����1
{
    if (rand() > (p * 32768))
        return 0;
    else
        return 1;
}
int randbit(int i, int j)  //������i��j֮���һ�������
{
    int a, l;
    l = j - i + 1;
    a = i + rand() * l / 32768;
    return a;
}
int randnum() {
    int x;
    x = rand() / 2;
    return x;
}
float convertionB2D(int x, float min, float max) {
    float y;
    y = x;
    y = y / 16384.0f;  //��һ�����õ�[0,1]�������
    y = y * (max - min);
    y = y + min;
    return y;
}
int convertionD2B(float x, float min, float max) {
    int g;
    g = (x * 1000) + 8192;
    return g;
}
int createmask(int a) {
    int mask;
    mask = (1 << (a + 1)) - 1;
    return mask;
}
