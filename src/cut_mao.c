/*
 * ccc.c
 *
 *  Created on: 2026年3月5日
 *      Author: L.YF
 */


//test1：2021年10月4日，进行GPIO0,2,4进行测试，控制高低电平，为后续进行测试；
//test2: 2021年10月4日，利用定时器Timer0，来进行翻转IO，确定当前的翻转时间，确保时钟正常。
//        请注意，该部分需要充分利用CpuTimer的使用；外部时钟和内部时钟均使用，初步正常；
//test3: 2021年10月4日，使用PWM测试；EPWM6，EPWM7，EPWM8，EPWM2；互补使用，200MHz的主频；
//test4: 2021年10月6日，测试AD相关程序，EPWM触发AD，设置AD中断,达到ADCA和ADCB的同时采样
//test5: 2021年10月6日，添加8301SPI
//2021年10月14日：添加SVPWM
//test6：增加电流开环，但是PWM配置有问题，无法闭环
//test7：电流闭环
//20211026:20K,位置闭环
//test8:改成40K
//test9:速度闭环
//20211123完成28377的程序调试，共包含电压环，电流闭环 位置闭环 转速闭环
//20211214 氢泵转速闭环完成。
//20220110 三电阻采样，电流环升速150kpm
//20220111 调试转速环
// 48V/1200w

#include "F28x_Project.h"
#include "C28x_FPU_FastRTS.h"
#include "ePWMs.h"
#include "math.h"
#include "SVPWM_2.h"
#include "clark.h"
#include "park.h"
#include "ipark.h"
#include "pi_reg.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_can.h"
#include "driverlib/can.h"
#include "VOLT3TO2.h"
#include "SMOTRACK.h"
#include "wave.h"

#define PWMBASE_50KHZ        4010  //20K   20220126//更改adc采样频率
#define PWMBASE_50KHZ_Half   2005  //20k  20220126
#define PI  3.141592654
#define PI2 6.283185307
#define MOTOR_NTC
volatile Uint16 EnDrive=0,EnSystem=0,phase_flag=3,RunMode=1,MotorMode=0,MOS_test=0;


float K1=(0.998);      //Offset filter coefficient K1: 0.05/(T+0.05);
float K2=(0.001999);
volatile float offsetA=0,offsetB=0,offsetC=0,offsetD=0,offsetE=0;
volatile Uint16 IsrTicker_adc1=0, IsrTicker_adc2=0,IsrTicker_ecap=0;
volatile float ElecTheta=0,dElecTheta_pu=0,ElecTheta_pu=0,ElecTheta_pu_comp=0,ElecTheta_pu_tmp=0;
volatile double Speed_acc_krpmp200us=0,SpeedSet_krpm=0,SpeedRef_krpm=0.9,Speed_acc_krpmps=0.5;
volatile float dElecTheta_pu_100us=0, SpeedLoop_flg = 0;
volatile unsigned int speed_cnt=0,speed_cnt2=0,speed_cnt3=0;
volatile Uint32  MotorMode_cnt=0,LockTime = 200000;
volatile float Valpha=0, Vbeta=0;

//-------------------ADC采样---------------------------------
volatile float Ia=0,Ib=0,Ic=0,I_main=0,I_heating, a=0,b=0;
volatile float V_96=0,V_48=0,V_15=0,V_5=0,V_33=0,V_96_stop=98;
volatile float T_shell=0,T_motor=0,T_mos=0,T_out=0,T_AMB=0;
//-------------------ADC采样---------------------------------
//-------------------IO控制---------------------------------
volatile unsigned char Brake_IOC=0,Heating_IOC=0,BUS_IOC=0,Phase_IOC=0;
//-------------------IO控制---------------------------------
volatile float T_shell_set=25,T_pwmduty=0;//温度控制
volatile Uint16 T_ctr=0,IOC_cnt=0,SpL=1;//温度、过流、速度环控制
volatile unsigned int Fault=0;
volatile unsigned char PWMTZ_cnt=0;

volatile float Id_init=0.04,Iq_init=0.16;   // 0.16
volatile float UdTest=0,UdTest_1=0.1,UqTest=0,UqTest_1=0.1;

volatile float Valpha_fo_pre_1=0, Vbeta_fo_pre_1=0;
volatile float Motor_Rs=0.12, Motor_Ls=0.000273;//Motor_Rs=0.07, Motor_Ls=0.000147 4500
volatile float smo_F=0, smo_G=0;
volatile float smo_Kslide=0.4, smo_Theta=0, smo_Theta_comp=0.0, smo_Theta_comp_extra=-0.08, LPF_filter = 30;
volatile float dsmo_Theta=0, dsmo_Theta_pre=0, smo_Theta_pre=0;
volatile float dEstErr=0,Iq_ref=0, count_delay_2 = 0;
//-------------------速度控制----------------------------------
volatile float smo_speed_hz_fo_2=0, Speed_filter_K2=0.0014, smo_speed_hz_sl_tmp[4]={0,0,0,0};
volatile float smo_speed_hz_sl_tmp1[4]={0,0,0,0}, smo_speed_hz_fo_3=0;
volatile float smo_Speed_krpm=0, smo_speed_hz = 0, smo_speed_hz_fo=0;
volatile Uint16 speed_cal_cnt=0,Shortcircuit_flag=0,auto_acc_flag=0;
volatile float spd_ki1=0.00002,spd_ki2=0.00011,spd_kacc=0.005;
volatile float ecap_speed_hz=0,ecap_speed_hz_fo=0,ecap_speed_krpm=0,ecap_speed_hz_pre=0,speed_acc=0,speed_acc_f=0;
volatile Uint32 EHTime_c=0,Brake_pwm=0;


EPWMS   Epwm_modules = EPWMS_DEFAULTS;          //ePWM模块对应的结构体变量；
SVPWM_2 Svpwm = SVPWM_2_DEFAULTS;            //两电平矢量SVPWM算法对应的结构变量；

CLARK   Iabc_to_Ialphabeta = CLARK_DEFAULTS;    //CLARK
//CLARK   Iabc_to_Ialphabeta_smo = CLARK_DEFAULTS; //CLARK
PARK    Ialphabeta_to_Idq = PARK_DEFAULTS;      //Park
//PARK    Ualphabeta_to_Udq = PARK_DEFAULTS;      //Park
IPARK   Udq_to_Ualphabeta = IPARK_DEFAULTS;     //

PIREG   A_Isd_R = PIREG_ID_DEFAULTS;            //d
PIREG   A_Isq_R = PIREG_IQ_DEFAULTS;            //q
PIREG   A_Spd_R = PIREG_SPD_DEFAULTS;
PIREG   A_Temp_R = PIREG_TEMP_DEFAULTS;

VOLT3TO2 volt = VOLT3TO2_DEFAULTS;
SMOTRACK smo = SMOTRACK_DEFAULTS;

interrupt void adc_offset_isr(void);   //使用ADCA1进行处理采样；
interrupt void adc_main_isr(void);
interrupt void ECAP3_ISR_1(void);
interrupt void eCANINTA_isr(void);

void GpioSetup(void);
void SetAdc(void);
void InitAdc(void);
void InitECap(void);
void SetCapMode(void);
void Init_cana_tr(void);

//-------------------低通滤波器---------------------------------
float LPF_out[13];

float one_order_LPF(float cutoff_fre,float input,Uint16 i)
{
    float A = 2*PI*(cutoff_fre)*0.00005/(1+2*PI*(cutoff_fre)*0.00004);//25k采样频率//更改adc采样频率
    LPF_out[i] = A*(input)+(1-A)*(LPF_out[i]);
    return LPF_out[i];
}
//-------------------低通滤波器---------------------------------

void main(void)
{
    InitSysCtrl();
    GpioSetup();
    InitSpiaGpio();

    CANInit(CANB_BASE);
    CANClkSourceSelect(CANB_BASE, 0);
    CANBitRateSet(CANB_BASE, 200000000, 1000000);
    CANIntEnable(CANB_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;
    InitPieVectTable();
    //DRV8301_SPI_Init(&SpiaRegs);
    SetAdc();
    InitECap();
    EALLOW;
    InputXbarRegs.INPUT1SELECT = 90;       //
    GpioCtrlRegs.GPCDIR.bit.GPIO90 = 0;    // input
    GpioCtrlRegs.GPCPUD.bit.GPIO90 = 0;    // Enable pull-up on GPIO90 (CAP3)
    EDIS;

    EALLOW;
    PieVectTable.ADCA1_INT = &adc_offset_isr;     //修改中断向量表中中断入口地址
    EDIS;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;  // Enable INT 1.1 in the PIE    使能ADCA1中断ADC INT1
    PieCtrlRegs.PIEIER4.bit.INTx3 = 1;  //ecap3
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;  //can
    IER |= M_INT1;
    EALLOW;

    InitAdc();

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    EPwm7Regs.ETSEL.bit.SOCAEN = 1;                //使能SOCA
    EPwm7Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;      //计数到最大值产生SOCA   触发采样
    EPwm7Regs.ETPS.bit.SOCAPRD = ET_1ST;           //每计数到最大值一次产生一次中断
    EPwm7Regs.ETCLR.bit.SOCA = 1;
    Epwm_modules.PeriodMax = PWMBASE_50KHZ;        //设定计数器的最大计数值
    Epwm_modules.init(&Epwm_modules);              //调用ePWM模块的初始化函数，开始初始化EPWM；
    EPwm2Regs.TBPRD = PWMBASE_50KHZ+20;

    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;//配置完成后重新使能TBCLK时钟信号。

    EDIS;
    //强制EPWM完全关闭；
    EALLOW;
    EPwm7Regs.TZFRC.bit.OST=1;
    EPwm8Regs.TZFRC.bit.OST=1;
    EPwm9Regs.TZFRC.bit.OST=1;
    EDIS;

    Init_cana_tr();

    EnDrive=0;
    ElecTheta=0;
    ElecTheta_pu=0;
    dElecTheta_pu=0;
    SpeedRef_krpm=1.8;           //电流闭环时设置1.2               /////150V设置
    Speed_acc_krpmps=0.5;

    A_Isq_R.Ref = 0;
    A_Isq_R.Fdb = 0;
    A_Isq_R.Kp = 0.35;//0.6;//1.1(19.5匝~22.5匝);0.6(14.5匝~16.5匝);
    A_Isq_R.Ki = 0.0075;//0.01(19.5匝~22.5匝);0.003(14.5匝~16.5匝);
    A_Isq_R.UiMAX = 1;    //积分限幅
    A_Isq_R.UiMIN = -0.3;
    A_Isq_R.OutMax = 1;   //输出限幅
    A_Isq_R.OutMin = -0.3;

    A_Isd_R.Ref = 0;
    A_Isd_R.Fdb = 0;
    A_Isd_R.Kp = 0.6;//0.6;//1.1(19.5匝~22.5匝);0.6(14.5匝~16.5匝);
    A_Isd_R.Ki = 0.0045;//0.01(19.5匝~22.5匝);0.003(14.5匝~16.5匝);
    A_Isd_R.UiMAX = 0.3;
    A_Isd_R.UiMIN = -0.5;
    A_Isd_R.OutMax = 0.3;
    A_Isd_R.OutMin = -0.5;

    A_Spd_R.Ref = 0;
    A_Spd_R.Fdb = 0;
    A_Spd_R.Kp = 0.5;//0.02;
    A_Spd_R.Ki = 0.0001;
    A_Spd_R.Kd = 1;
    A_Spd_R.UiMAX = 0.5;
    A_Spd_R.UiMIN = -0.4;
    A_Spd_R.OutMax = 0.5;
    A_Spd_R.OutMin = -0.4;

    // 20211025
    smo_F = exp((-Motor_Rs/Motor_Ls)*(0.001/25));                //更改adc采样频率
//    smo_G = (28.8/50)*(1/Motor_Rs)*(1-smo_F);
    smo_G = (126/40)*(1/Motor_Rs)*(1-smo_F);   //126对应4096对应的电压除根号三，40对应电流最大幅值
    //

    EINT;               //开启CPU的中断，系统开始执行
    ERTM;

    for(;;)  //死循环
    {
          ;
    }

}

////////////////
interrupt void adc_main_isr(void)
{
    if(EnSystem==1)
    {
        GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;    //驱动使能
        GpioDataRegs.GPASET.bit.GPIO11 = 1;    //96v输出
    }
    else
    {
        EnDrive=0;
        GpioDataRegs.GPASET.bit.GPIO18 = 1;    //驱动关闭
        GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;    //96v输出
        EALLOW;
        EPwm7Regs.TZFRC.bit.OST=1;
        EPwm8Regs.TZFRC.bit.OST=1;
        EPwm9Regs.TZFRC.bit.OST=1;
        EDIS;
    }

    IsrTicker_adc2++;

#ifndef MOTOR_REVERSAL
    Ic = ((AdcaResultRegs.ADCRESULT1)*0.000244140625 - offsetC) * 2; // Phase C curr.C相是反的
    Ib = ((AdcaResultRegs.ADCRESULT2)*0.000244140625 - offsetB) * 2; //电流采样反向，加负号
    Ia = ((AdcaResultRegs.ADCRESULT3)*0.000244140625 - offsetA) * 2;
#else
    Ic = ((AdcaResultRegs.ADCRESULT1)*0.000244140625 - offsetC) * 2; // Phase C curr.C相是反的
    Ia = ((AdcaResultRegs.ADCRESULT2)*0.000244140625 - offsetB) * 2; //电流采样反向，加负号
    Ib = ((AdcaResultRegs.ADCRESULT3)*0.000244140625 - offsetA) * 2;
#endif
    I_main=0.01996*(one_order_LPF(4,AdcaResultRegs.ADCRESULT4,0)-offsetD);//滤波0
    I_heating=0.01682*(one_order_LPF(4,AdcaResultRegs.ADCRESULT0,1)-offsetE);//滤波1
    V_96=0.0536*one_order_LPF(6,AdcbResultRegs.ADCRESULT5,2)+0.281;//滤波2
    //V_48=0.0188*AdcaResultRegs.ADCRESULT5-1.6734;//滤波3
    V_15=AdcaResultRegs.ADCRESULT6;//滤波4
    V_5=one_order_LPF(4,AdcbResultRegs.ADCRESULT6*0.00161133,5);//滤波5
    V_33=AdcbResultRegs.ADCRESULT0;//滤波6


#endif
    T_shell=one_order_LPF(10,T_pt1000(AdcbResultRegs.ADCRESULT4),10);//滤波10
    T_AMB=one_order_LPF(1,T_pt1000(AdcbResultRegs.ADCRESULT1),7);

    ecap_speed_hz_fo=one_order_LPF(2,ecap_speed_hz,11);//滤波11
    ecap_speed_krpm=ecap_speed_hz_fo*0.06;
    speed_acc=ecap_speed_hz-ecap_speed_hz_pre;
    speed_acc_f=one_order_LPF(1,speed_acc,12);//滤波11


    Brake_IOC=GpioDataRegs.GPADAT.bit.GPIO10;
    Heating_IOC=GpioDataRegs.GPCDAT.bit.GPIO92;
    BUS_IOC=GpioDataRegs.GPCDAT.bit.GPIO91;
    //Phase_IOC=GpioDataRegs.GPCDAT.bit.GPIO90;

    Iabc_to_Ialphabeta.As  = Ia; // Phase A curr.
    Iabc_to_Ialphabeta.Bs = Ib; // Phase B curr.
    Iabc_to_Ialphabeta.calc(&Iabc_to_Ialphabeta);     //clark 变换

    Fault=Brake_IOC+2*Heating_IOC+4*BUS_IOC+8*Phase_IOC;  //故障位

    if(EnDrive==1)
    {
        if(MotorMode==1)       //模式 motormode=1 电机预定位
        {
            A_Isd_R.Ref = Id_init;
            A_Isq_R.Ref = 0;

            MotorMode_cnt++;          //2次转子定位 将磁极N极拉到与d轴重合进行初始定位20201130
            if(MotorMode_cnt <= 40000)//25000//50000//更改adc采样频率
            {
                ElecTheta_pu = 0.25;
            }

            else if(MotorMode_cnt<=100000)
            {
                ElecTheta_pu = 0;
            }
            else if(MotorMode_cnt==LockTime)
            {
                A_Isd_R.Ref = 0;
                A_Isq_R.Ref = Iq_init;
                MotorMode=2;
                Speed_acc_krpmps = 0.05;
                MotorMode_cnt=LockTime+1;
                SpeedRef_krpm = 1.8;//0.8
             }
        }
        else if(MotorMode==2)   //位置开环
        {
            speed_cnt++;
            if(speed_cnt==5)   // 速度频率5k//更改adc采样频率
            {
                Speed_acc_krpmp200us = Speed_acc_krpmps * 0.0002;    //加速斜线
                if(fabs(SpeedRef_krpm-SpeedSet_krpm)>Speed_acc_krpmp200us)
                {
                    speed_cnt2=1;
                    if(SpeedRef_krpm>SpeedSet_krpm)
                    {
                        SpeedSet_krpm = SpeedSet_krpm + Speed_acc_krpmp200us; //每隔200us增加一次速度  SpeedRef_krpm窗口更改
                        speed_cnt3=1;
                    }
                    else
                    {
                        SpeedSet_krpm = SpeedSet_krpm - Speed_acc_krpmp200us;
                    }
                }
                else
                {
                    SpeedSet_krpm = SpeedRef_krpm;
                }
                speed_cnt = 0;
            }

            dElecTheta_pu_100us = SpeedSet_krpm * (0.000666667);//角度增量 按25kHz：SpeedSet_krpm*1k/60/25k krpm→Hz //更改adc采样频率
            if(SpeedSet_krpm>0)
            {
                ElecTheta_pu += (dElecTheta_pu_100us); //角度递增
                if(ElecTheta_pu>=1)
                {
                    ElecTheta_pu =0;  //ElecTheta_pu-1;//20210312与小电机保持一致并去除反转部分
                }
            }

            dEstErr = smo_Theta - ElecTheta_pu;
            if((SpeedSet_krpm==SpeedRef_krpm))
            {
                dEstErr = smo_Theta - ElecTheta_pu;
                if((dEstErr<0.1) && (dEstErr>-0.1))  //位置误差在0.1以内切换到模式3
                {
                    MotorMode=3;
                    SpL=1;
                    Iq_ref =  0.8*A_Isq_R.Ref; //位置闭环后乘以系数0.4，降低参考电流
                    speed_cnt=0;
                    smo_Theta_comp_extra=-0.03;
                    Speed_acc_krpmps=0.018;
                    SpeedRef_krpm=1;
                    SpeedSet_krpm=1;
                }
            }
        }
        else if(MotorMode==3)
        {
            ElecTheta_pu = smo_Theta;
            if(ecap_speed_hz_fo==0)
            {
                EnSystem=0;
                EnDrive=0;
                b=1;
             }
            if(ecap_speed_hz_fo>30&&SpL==1)
            {
                SpeedLoop_flg=1;
                A_Spd_R.Ui=Iq_ref;
                SpeedSet_krpm=ecap_speed_krpm;
                SpeedRef_krpm=4.8;
                Speed_acc_krpmps=0.035;
                SpL=0;
            }

            if(SpeedLoop_flg==1)
            {
                if(auto_acc_flag==1&&ecap_speed_hz_fo<200)
                {
                    Speed_acc_krpmps=0.05;
                }
                else if(auto_acc_flag==1&&ecap_speed_hz_fo<325)
                {
                    Speed_acc_krpmps=-ecap_speed_hz_fo*0.00024+0.098;
                }
                speed_cnt++;
                if(speed_cnt==5)   //50K,5KHz//更改adc采样频率
                {
                   Speed_acc_krpmp200us = Speed_acc_krpmps * 0.0002;   //
                   if(fabs(SpeedRef_krpm-SpeedSet_krpm)>Speed_acc_krpmp200us)
                   {
                       if(SpeedRef_krpm>SpeedSet_krpm)
                       {
                           SpeedSet_krpm = SpeedSet_krpm + Speed_acc_krpmp200us;
                       }
                       else
                       {
                           SpeedSet_krpm = SpeedSet_krpm - Speed_acc_krpmp200us;
                       }

                   }
                   else
                   {
                       SpeedSet_krpm = SpeedRef_krpm;
                   }
                   speed_cnt = 0;
                   A_Spd_R.Ref = SpeedSet_krpm;
                   A_Spd_R.Fdb = ecap_speed_krpm;
                   if(SpeedRef_krpm==SpeedSet_krpm)
                   {
                       if(fabsf(A_Spd_R.Err)<0.001)// 速度环滞环，只在稳速时启用，当误差小于0.01，ki减小，但误差处于0.01与0.05之间但误差微分>0,ki增大
                       {
                           A_Spd_R.Ki=spd_ki1;
                       }
                       else if(fabsf(A_Spd_R.Err)<0.05&&A_Spd_R.Ud<0)
                       {
                           A_Spd_R.Ki=spd_ki1;
                       }
                       else
                       {
                           A_Spd_R.Ki=spd_ki2;
                       }

                   }
                   else if(fabs(SpeedRef_krpm-ecap_speed_krpm)<0.1&&fabs(SpeedRef_krpm-ecap_speed_krpm)>0.01)
                   {
                       A_Spd_R.Ui-=speed_acc_f*spd_kacc;
                   }

                   A_Spd_R.calc(&A_Spd_R);
                   A_Isq_R.Ref = A_Spd_R.Out;
                }
            }
            else
            {
                A_Isq_R.Ref = Iq_ref;
                A_Spd_R.OutPreSat=0;
                A_Spd_R.Ui=0;
                A_Spd_R.Up=0;
            }

        }


        ElecTheta = ElecTheta_pu * PI2;
        Ialphabeta_to_Idq.Alpha = Iabc_to_Ialphabeta.Alpha;
        Ialphabeta_to_Idq.Beta  = Iabc_to_Ialphabeta.Beta;     //电流环闭环计算20201202

        Ialphabeta_to_Idq.Cos = cos (ElecTheta);
        Ialphabeta_to_Idq.Sin = sin (ElecTheta);
        Ialphabeta_to_Idq.calc(&Ialphabeta_to_Idq);            //反馈电流计算Id Iq

        A_Isq_R.Fdb = Ialphabeta_to_Idq.Qs;
        A_Isq_R.calc(&A_Isq_R);
        A_Isd_R.Fdb = Ialphabeta_to_Idq.Ds;                   //经过PI控制器输出Ud Uq
        A_Isd_R.calc(&A_Isd_R);

            //------------------------------------
         volt.Va = 0.44 * (Svpwm.Ta);
         volt.Vb = 0.44 * (Svpwm.Tb);
         volt.Vc = 0.44 * (Svpwm.Tc);  //
         volt.calc(&volt);
         Valpha = volt.alpha;
         Vbeta  = volt.beta;
         //-SMO观测器
         smo.Valpha = Valpha_fo_pre_1;
         smo.Vbeta  = Vbeta_fo_pre_1;
         Valpha_fo_pre_1 = Valpha;
         Vbeta_fo_pre_1 = Vbeta;
         smo.Ialpha = Iabc_to_Ialphabeta.Alpha;
         smo.Ibeta  = Iabc_to_Ialphabeta.Beta;
         smo.Kslide = smo_Kslide;
         smo.Theta_comp_extra = smo_Theta_comp_extra;

         smo.Fsmopos = smo_F;
         smo.Gsmopos = smo_G;
         if(smo_speed_hz_fo>30)
         {
           smo.LPF = smo_speed_hz_fo;
         }
         else
         {
           smo.LPF = LPF_filter;
         }
         smo.speed_hz_fo = smo_speed_hz_fo;
         smo.calc(&smo);
         smo_Theta = smo.Theta;


        //--------------------------------------------------------------------
        dsmo_Theta = (smo_Theta - smo_Theta_pre);
        if((dsmo_Theta>0.1) || (dsmo_Theta<-0.1))
            dsmo_Theta = dsmo_Theta_pre;

        smo_speed_hz = dsmo_Theta * 24937;  //50KHz//更改adc采样频率
        //-------------------------------------------------------------------------------
        smo_speed_hz_fo_2 = smo_speed_hz_fo_2 + Speed_filter_K2 * (smo_speed_hz - smo_speed_hz_fo_2);

        smo_speed_hz_sl_tmp[speed_cal_cnt] = smo_speed_hz_fo_2;
        smo_speed_hz_fo_3 = (smo_speed_hz_sl_tmp[0] + smo_speed_hz_sl_tmp[1] + smo_speed_hz_sl_tmp[2] + smo_speed_hz_sl_tmp[3])*0.25;

        smo_speed_hz_sl_tmp1[speed_cal_cnt] = smo_speed_hz_fo_3;
        smo_speed_hz_fo = (smo_speed_hz_sl_tmp1[0] + smo_speed_hz_sl_tmp1[1] + smo_speed_hz_sl_tmp1[2] + smo_speed_hz_sl_tmp1[3])*0.25;
        speed_cal_cnt++;
        if((speed_cal_cnt)==4)
        {
            speed_cal_cnt=0;
        }
        smo_Speed_krpm = smo_speed_hz_fo * 0.06;

        dsmo_Theta_pre = dsmo_Theta;    //
        smo_Theta_pre = smo_Theta;   //


    }
    else
    {
        ElecTheta_pu=0;
        dElecTheta_pu=0;
        ElecTheta = 0;

        A_Isq_R.Out = 0;
        A_Isd_R.Out = 0;
        A_Isq_R.Ui=0;
        A_Isd_R.Ui=0;
        A_Isd_R.OutPreSat=0;
        A_Isq_R.OutPreSat=0;
        SpeedSet_krpm=0;
        SpeedSet_krpm=0;
        MotorMode_cnt=0;
        smo_Speed_krpm=0;
        smo_speed_hz_fo=0;
        SpeedLoop_flg=0;

    }

    if(RunMode==0)
    {
        Udq_to_Ualphabeta.Qs = UqTest; //
        Udq_to_Ualphabeta.Ds = UdTest; //
    }
    else
    {
        Udq_to_Ualphabeta.Qs = A_Isq_R.Out; //
        Udq_to_Ualphabeta.Ds = A_Isd_R.Out; //UdTest;
    }

    Udq_to_Ualphabeta.Cos = Ialphabeta_to_Idq.Cos;
    Udq_to_Ualphabeta.Sin = Ialphabeta_to_Idq.Sin;
    Udq_to_Ualphabeta.calc(&Udq_to_Ualphabeta);


    Svpwm.Ualpha = Udq_to_Ualphabeta.Alpha;
    Svpwm.Ubeta =  Udq_to_Ualphabeta.Beta;
    Svpwm.calc(&Svpwm);

//    EPwm6Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Tc + PWMBASE_50KHZ_Half);
    if(Shortcircuit_flag==1)      //电路短路处理
    {
        EnDrive = 0;
        EPwm7Regs.CMPA.bit.CMPA = (unsigned int)(0);
        EPwm8Regs.CMPA.bit.CMPA = (unsigned int)(0);
        EPwm9Regs.CMPA.bit.CMPA = (unsigned int)(0);
    }
    else
    {
    #ifndef MOTOR_REVERSAL
        EPwm9Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Ta + PWMBASE_50KHZ_Half);
        EPwm8Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Tb + PWMBASE_50KHZ_Half);
        EPwm7Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Tc + PWMBASE_50KHZ_Half);
    #else
        EPwm9Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Tb + PWMBASE_50KHZ_Half);
        EPwm8Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Ta + PWMBASE_50KHZ_Half);
        EPwm7Regs.CMPA.bit.CMPA = (unsigned int)(PWMBASE_50KHZ_Half * Svpwm.Tc + PWMBASE_50KHZ_Half);
    #endif
    }


    GpioDataRegs.GPCDAT.bit.GPIO87 = 1;    //初始化输出为0
    GpioDataRegs.GPCDAT.bit.GPIO86 = 1;    //初始化输出为0

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       //Clear ADCAINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
    return;
}

interrupt void adc_offset_isr(void)
{
    //先根据AD寄存器数值得到电压，再根据电压转换为电流；注意检测相关的零偏数值，观察系统的零偏；
    IsrTicker_adc1++;
    if (IsrTicker_adc1>=5000)    //等待 ADC 硬件稳定，避免初始采样值波动影响零偏计算
    {
        offsetA= K1*offsetA + K2*(AdcaResultRegs.ADCRESULT3)*0.000244140625;             //Phase A offset 将电流范围设置为-1~1最大为-1~1；
        offsetB= K1*offsetB + K2*(AdcaResultRegs.ADCRESULT2)*0.000244140625;             //Phase B offset   请注意相关问题；中间的偏置并不会达到相关范围；由偏置offsetA决定；
        offsetC= K1*offsetC + K2*(AdcaResultRegs.ADCRESULT1)*0.000244140625;             //Phase C
        offsetD= K1*offsetD + K2*(AdcaResultRegs.ADCRESULT4);             //总母线电流
        offsetE= K1*offsetE + K2*(AdcaResultRegs.ADCRESULT0);             //加热电流零偏
        V_96=K1*V_96+K2*(0.0524*AdcbResultRegs.ADCRESULT5+1.281);         //96V母线电压零偏校准
    }
    if (IsrTicker_adc1 > 20000)
    {
        //V_96_stop=V_96+1;
        EALLOW;
        PieVectTable.ADCA1_INT = &adc_main_isr;
        EDIS;
    }

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;       //Clear ADCAINT1 flag reinitialize for next SOC
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
    return;
}

void GpioSetup(void)
{
    EALLOW;

      GpioCtrlRegs.GPAGMUX1.bit.GPIO2 = 0;   //mux同时选择决定GPIO的复用20211005
      GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;    //配置为ePWM2A
      GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;     ////关闭上拉；

      GpioCtrlRegs.GPAGMUX1.bit.GPIO3 = 0;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;    //配置为ePWM2B
      GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    ////关闭上拉；

      GpioCtrlRegs.GPAGMUX1.bit.GPIO12 = 0;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;    //配置为ePWM7A
      GpioCtrlRegs.GPAPUD.bit.GPIO12 = 1;     ////关闭上拉；

      GpioCtrlRegs.GPAGMUX1.bit.GPIO13 = 0;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;    //配置为ePWM7B
      GpioCtrlRegs.GPAPUD.bit.GPIO13 = 1;    ////关闭上拉；

      GpioCtrlRegs.GPAGMUX1.bit.GPIO14 = 0;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;    //配置为ePWM8A
      GpioCtrlRegs.GPAPUD.bit.GPIO14 = 1;    ////关闭上拉；

      GpioCtrlRegs.GPAGMUX1.bit.GPIO15 = 0;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;    //配置为ePWM8B
      GpioCtrlRegs.GPAPUD.bit.GPIO15 = 1;    ////关闭上拉；

      GpioCtrlRegs.GPAGMUX2.bit.GPIO16 = 1;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;    //配置为ePWM9A
      GpioCtrlRegs.GPAPUD.bit.GPIO16 = 1;    ////关闭上拉；

      GpioCtrlRegs.GPAGMUX2.bit.GPIO17 = 1;   //mux同时选择决定GPIO的复用
      GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;    //配置为ePWM9B
      GpioCtrlRegs.GPAPUD.bit.GPIO17 = 1;    ////关闭上拉；
/*******************************CAN**************************/
      GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 0;   //TX
      GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;
      GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;    //方向为输出
      GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;    ////GPIO上拉使能（OC

      GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 0;  //RX
      GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;
      GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3;
      GpioCtrlRegs.GPADIR.bit.GPIO21 = 0;    //方向为输入
/******************************CAN***************************/

/*******************************IO控制***************************/
      GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;  //使能上拉；FAULT
      GpioCtrlRegs.GPADIR.bit.GPIO10 = 0;  //配置为输入；

      GpioCtrlRegs.GPCPUD.bit.GPIO90 = 0;  //使能上拉；IOC
      GpioCtrlRegs.GPCDIR.bit.GPIO90 = 0;  //配置为输入；

      GpioCtrlRegs.GPCPUD.bit.GPIO91 = 0;  //使能上拉；HEATING IOC
      GpioCtrlRegs.GPCDIR.bit.GPIO91 = 0;  //配置为输入；

      GpioCtrlRegs.GPCPUD.bit.GPIO92 = 0;  //使能上拉；BRAKE IOC
      GpioCtrlRegs.GPCDIR.bit.GPIO92 = 0;  //配置为输入；

      GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;    //方向为输出  DRIVE96
      GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    ////GPIO上拉使能（OC
      GpioDataRegs.GPACLEAR.bit.GPIO11 = 1;    //初始化输出为0

      GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;    //方向为输出  PWM_OE
      GpioCtrlRegs.GPAPUD.bit.GPIO18 = 1;    ////GPIO上拉使能（OC
      GpioDataRegs.GPASET.bit.GPIO18 = 1;    //初始化输出为0

      GpioCtrlRegs.GPCDIR.bit.GPIO86 = 1;    //方向为输出  LED_RED
      GpioCtrlRegs.GPCPUD.bit.GPIO86 = 1;    ////GPIO上拉使能（OC
      GpioDataRegs.GPCCLEAR.bit.GPIO86 = 1;    //初始化输出为0

      GpioCtrlRegs.GPCDIR.bit.GPIO87 = 1;    //方向为输出  LED_GREEN
      GpioCtrlRegs.GPCPUD.bit.GPIO87 = 1;    ////GPIO上拉使能（OC
      GpioDataRegs.GPCCLEAR.bit.GPIO87 = 1;    //初始化输出为0
/*******************************IO控制***************************/

    EDIS;
}
void SetAdc(void)
{
    EALLOW;
    //write configurations
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4  时钟最高50M
    AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //

    AdcSetMode(ADC_ADCA, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);  //设置ADCA的分辨率12位   单端模式
    AdcSetMode(ADC_ADCB, ADC_RESOLUTION_12BIT, ADC_SIGNALMODE_SINGLE);

    //Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;  //设置中断脉冲产生时刻   由于中断在ADCA产生所以只需设置一个
    // ADCC不产生中断，20201103；

    //power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  //上电ADC

    //delay for 1ms to allow ADC time to power up
    DELAY_US(1000);
    EDIS;
}

void InitAdc(void)
{
    EALLOW;

    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //使能ADCAINT1的中断；
    AdcaRegs.ADCINTSEL1N2.bit.INT1CONT = 0;  //关闭连续中断模式；
    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 6;   //持续采样3个； EOC3 is trigger for ADCINT1

    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0x00;    //ADCINA0;   加热电流采样
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 17;     //触发源选择  EPwm7 SOCA
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 40;      //20201113 从25改为35
    // ADCC的SCO进行配置，20201103；
    AdcbRegs.ADCSOC0CTL.bit.CHSEL = 0x00;    //ADCINB0; 20201103  3.3v采样
    AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC0CTL.bit.ACQPS = 40;      //12位模式必须10.5个时钟周期以上，40 SYSCLK cycles

    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0x01;    //ADCINA1   IC
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 17;
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 40;      //20201113 从25改为40
    // ADCC的SC1进行配置，20201103；
    AdcbRegs.ADCSOC1CTL.bit.CHSEL = 0x01;    //ADCINB1; 20201103 机壳温度
    AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 17;
    AdcbRegs.ADCSOC1CTL.bit.ACQPS = 40;

    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 0x02;    //ADCINA2   IB
    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 17;     //同步采样方式ADCA ADCB
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 40;
    // ADCC的SC2进行配置，20201103；
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 0x02;    //ADCINB2; 20201103 电机定子温度
    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 40;

    AdcaRegs.ADCSOC3CTL.bit.CHSEL = 0x03;    //ADCINA3   IA
    AdcaRegs.ADCSOC3CTL.bit.TRIGSEL = 17;     //同步采样方式ADCA ADCB
    AdcaRegs.ADCSOC3CTL.bit.ACQPS = 40;
    // ADCC的SC3进行配置，20201103；
    AdcbRegs.ADCSOC3CTL.bit.CHSEL = 0x03;    //ADCINB3; 20201103 驱动板温度
    AdcbRegs.ADCSOC3CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC3CTL.bit.ACQPS = 40;

    AdcaRegs.ADCSOC4CTL.bit.CHSEL = 0x04;    //ADCINA4   总母线电流采样
    AdcaRegs.ADCSOC4CTL.bit.TRIGSEL = 17;     //同步采样方式ADCA ADCB
    AdcaRegs.ADCSOC4CTL.bit.ACQPS = 40;
    // ADCC的SC2进行配置，20201103；
    AdcbRegs.ADCSOC4CTL.bit.CHSEL = 0x04;    //ADCINB4; 20201103 外部温度
    AdcbRegs.ADCSOC4CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC4CTL.bit.ACQPS = 40;

    AdcaRegs.ADCSOC5CTL.bit.CHSEL = 0x05;    //ADCINA5   48V采样
    AdcaRegs.ADCSOC5CTL.bit.TRIGSEL = 17;     //同步采样方式ADCA ADCB
    AdcaRegs.ADCSOC5CTL.bit.ACQPS = 40;
    // ADCC的SC2进行配置，20201103；
    AdcbRegs.ADCSOC5CTL.bit.CHSEL = 0x05;    //ADCINB5; 20201103 96v电压采样
    AdcbRegs.ADCSOC5CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC5CTL.bit.ACQPS = 40;


    AdcaRegs.ADCSOC6CTL.bit.CHSEL = 0x0E;    //ADCIN14    12v
    AdcaRegs.ADCSOC6CTL.bit.TRIGSEL = 17;     //同步采样方式ADCA ADCB
    AdcaRegs.ADCSOC6CTL.bit.ACQPS = 40;

    AdcbRegs.ADCSOC6CTL.bit.CHSEL = 0x0F;    //ADCIN15; 20201103   5v
    AdcbRegs.ADCSOC6CTL.bit.TRIGSEL = 17;     //EPwm6 SOCA
    AdcbRegs.ADCSOC6CTL.bit.ACQPS = 40;

    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //ADCINT1中断标志位清除

    EDIS;

}
