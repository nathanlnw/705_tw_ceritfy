/*
     App_gsm.h
*/
#ifndef  _RT_GSM
#define _RT_GSM

#include <rthw.h>
#include <rtthread.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>


//#define      MG323_GSM
#define      M66_GSM      0
#define     M50_GSM       1

      #define    APN_initSTR_LEN     19


#define flash const    

//-----  Dial Step-------
typedef enum DialStage {Dial_DialInit0, Dial_DialInit1, Dial_DialInit2, Dial_DialInit3,Dial_DialInit4,Dial_DialInit5,Dial_DialInit6,Dial_DNSR1,Dial_DNSR2,Dial_DialInit7,Dial_MainLnk,Dial_AuxLnk,Dial_ISP,Dial_Idle} T_Dial_Stage;




#define MULTI_LINK



#define GPRS_GSM_Power    GPIO_Pin_13
#define GPRS_GSM_PWKEY    GPIO_Pin_12  
#define GPRS_GSM_RST      GPIO_Pin_11


#define  Dial_Timeout			   40  //100		// 3 seconds        500
#define  Dial_Dial_Retry_Time	   40 //100		// 6 seconds
#define  Dial_max_Timeout	   100	//300		// 30 seconds        1000
#define  Dial_Step_MAXRetries			4

//
#define  Max_DIALSTEP            20         //  最大拨号次数 


 //-----------  GSM_RX  buffer size -------------
#define GSMRX_SIZE		2500
#define GSM_AsciiTX_SIZE   2500 



typedef   struct
{
  u8   GSM_PowerEnable;      // 使能开始上电 ，初始化为 1    
  u16  GSM_powerCounter;   //  上电计数器
  u8   GSM_power_over;     //   1: module on  2:   IMSI get        
} GSM_POWER;

typedef struct
{
u8   check_simcard;     //  检查SIM卡
u8   Get_state;        //  0: idle    1: Get IMSI_CODE successfully    2: No SIM card
u8   imsi_error_count;
u8   Checkcounter;   
}IMSI_GET;

typedef struct      //  获取到IMSI 后 登网前 初始化设置命令
{
u8     Total_initial; 
u8     Initial_step;
u8     Execute_couter; 
u8     Execute_enable; 

}COMM_AT; 

typedef  struct
{
	//---- Dial  Datalink----------
	u8     Dial_ON;                       //  拨号和数据模式下的状态信息，掉线清空
	u8     Pre_Dial_flag;              //  准备拨号使能
	u8     Pre_dial_counter;        //  准备拨号计时器
	u8     start_dial_stateFLAG;  //从开始拨号时写为1 
	u16     start_dial_counter;           //处于拨号状态标志位
       u8     Dial_GPRS_Flag;           // 开始登录GPRS 
	u8     Dial_step;                         //拨号步骤计数器 

        //----Single Setp  Operate  related  -----
        u16   Dial_step_RetryTimer;         //  单步骤内延时计数器
        u8     Dial_step_Retry;                  //  单步骤内重试次数
        //-----  close  related   -----------
	u8      DataLink_EndFlag;    
	u8      DataLink_end_counter; 
	u8      Connect_counter;         //  在没有登网前重拨次数限制计数器    11-3-4 补加的 ERROR 20 
 
} DATA_DIAL;



//  TTS   相关
#define   TTS_BUSY        1
#define   TTS_OK            0
#define   Speak_Ctrl             GPIO_Pin_9 


#define  Speak_OFF         GPIO_SetBits(GPIOD,Speak_Ctrl)
#define  Speak_ON           GPIO_ResetBits(GPIOD,Speak_Ctrl)       


typedef   struct  
{
     u8  HEX_BUF[400];     // GBK hex
     u16  HEX_len;
     u8   Save;    //  如果当前播放信息忙那么存储到HEX_BUF做缓存 ，缓存有信息1 ，没有 0	 
     u8  ASCII_BUF[800];  // ASCII GBK  
     u16  ASCII_Len;
     u8  NeedtoPlay;  	 
     u8  Playing;                 //   1: pLaying    0:  idle     根据状态开关功放
     u16  TimeOut_limt;               //  播放超时 单位:s      正常语速180 字每分，3个字每秒计算
     u16  TimeCounter;               // 播报异常超时计时器
     
}TTS;


//   Voice Recrod  
 #define   VOICEREC_IDLE                  0      //  空闲
 #define   VOICEREC_DeleteOld             1     //   删除旧的
 #define   VOICEREC_RECORD_START          2       // 录音开始
 #define   VOICEREC_RECORDING             3       //  录音中
 #define   VOICEREC_REC_STOP              4       //  停止录音
 #define   VOICEREC_GET_FILESIZE          5       //  获取 文件大小 
 #define   VOICEREC_FileOpen			  6		 //  打开文件
 #define   VOICEREC_Sd_getcmd             7      //  发送获取消息命令
 #define   VOICEREC_Rdy_Rxing             8      //  准备接收
 #define   VOICEREC_DataRXing             9       //  数据接收中   
 #define   VOICEREC_Close                 10      //  关闭文件

typedef struct
{
     u8    Sate;   // Operate  step
     u8   ExcuteFlag;  // 执行标志
     u8    file_name[20]; //时分秒ASCII.AMR 
     u8    handle_name[30];
     u32  filesize;      //  录音文件大小
     u32  file_write;    //  文件写入
     u16   packet;
     u8    excption_counter;  // 异常情况下定时恢复
     u8    running;        //   工作状态。 1 忙 0 空闲     
     u16   recording_timer;  // 录音时间
     

		//at  &   save  related  
     u8   SendString[50];  //  AT 发送命令      
     u16   HEX_len;
     u8    HEX[2100];        //   HEX 原始信息    
     u16  curren_PageCNT;  // 存储空间计数器
     u8   rdytoTrans;     //   0 时 不存储 1时存储
           //  808
     u16  JT808_Duration;   //  国标要求录音时间
     u8   JT808_SaveOrNot; //  存储标志 0: 实时上传  1: 保存
     u8   JT808_SampleRate;//  采样速率  
     
} VOC_REC;

#define  GSM_TYPEBUF_SIZE  1500
typedef __packed struct
{
	u16	       gsm_wr;
	u8		gsm_content[GSM_TYPEBUF_SIZE];  
}GSM_typeBUF;

//  Voice  Record 
extern VOC_REC       VocREC;    // 录音上传相关  
//   TTS  
extern   TTS              TTS_Var;  //  TTS 类型变量  

extern COMM_AT       CommAT; 
extern DATA_DIAL     DataDial;  


//-------- TCP2 send ---------
extern u8       TCP2_ready_dial;  
extern u16     Ready_dial_counter2;
extern u16     TCP2_notconnect_counter;
extern u8       TCP2_Connect;
extern u8	      TCP2_IC_sdFlag;		//定时发送GPS位置信息标志
extern u16     TCP2_sdDuration;
extern u8       TCP2_Coutner;  // 定时器计数
extern u8       TCP2_login;       // TCP 建立好连接后的标志位   
  //----- handle jianquan flag
extern u8       Hand_Login; // 手动鉴权标志位
extern u8     Enable_IClink;  //  手动使能连接IC 卡中心 

extern u8   Get_voicedata_delay;       
extern u8   GEt_voice_Counter;    


ALIGN(RT_ALIGN_SIZE)
extern  u8     GSM_rx[GSMRX_SIZE];       
extern  u8  GSM_AsciiTx[GSM_AsciiTX_SIZE];

extern void GSM_CSQ_timeout(void);
extern void GSM_CSQ_Query(void);
extern void  DataLink_MainSocket_set(u8 *IP, u16  PORT, u8 DebugOUT);
extern void  DataLink_AuxSocket_set(u8 *IP, u16  PORT,u8 DebugOUT) ;
extern void  DataLink_IC_Socket_set(u8 *IP, u16  PORT,u8 DebugOUT); 
extern void  DataLink_APN_Set(u8* apn_str,u8 DebugOUT);
extern void  DataLink_DNSR_Set(u8* Dns_str,u8 DebugOUT); 
extern void  DataLink_DNSR2_Set(u8* Dns_str,u8 DebugOUT);
extern  void  gsm_init(void);
extern  void  GSM_RxHandler(u8 data);     
extern  void  GSM_Buffer_Read_Process(void);


extern u8    GPRS_GSM_PowerON(void);   
extern void  Data_Send(u8* DataStr, u16  Datalen,u8  Link_Num);
extern void  End_Datalink(void);
extern void  ISP_Timer(void); 
extern void Redial_Init(void);
extern void rt_hw_gsm_output(const char *str);
extern void rt_hw_gsm_output_Data(u8 *Instr,u16 len);  
extern  u16  GSM_AsciitoHEX_Convert(u8 *Src_str,u16 Src_infolen,u8* Out_Str); 
extern void GSM_Module_TotalInitial(void);
extern void DataLink_Process(void);
extern void   Dial_step_Single_10ms_timer(void);
extern void  IMSIcode_Get(void);
extern void  rt_hw_gsm_init(void);


extern u8    TTS_Data_Play(void);
extern u8    TTS_Get_Data(u8* Instr,u16 LEN) ; 
extern void TTS_Exception_TimeLimt(void);     //  单位: s 

//   VOC REC 
extern void    VOC_REC_Stop(u8 *instr)  ;
extern  void   VOC_REC_Start(u8 value);     // 录音开始 
extern void    VOC_REC_process(void); 
extern void    VOC_REC_timer(void);
extern u8  HexValue (u8 inchar); 

extern void    VOC_REC_Init(void);    

#endif 

