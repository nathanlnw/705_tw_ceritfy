/*
    App Gsm uart
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//数字转换成字符串
#include  <stdio.h>
#include  <string.h>
#include  <ctype.h>
#include  <stdlib.h>
#include  <stdarg.h>
#include "App_moduleConfig.h" 


#define GSM_GPIO			GPIOC
#define GSM_TX_PIN			GPIO_Pin_10
#define GSM_TX_PIN_SOURCE	GPIO_PinSource10

#define GSM_RX_PIN			GPIO_Pin_11
#define GSM_RX_PIN_SOURCE	GPIO_PinSource11



#ifdef  M50_GSM



//----------  EM310 自带协议栈相关  -----------------------------
//   1.  通过命令和状态 初始化命令   
flash char  SIM_Check_Str[]="AT+QCCID\r\n";   //  检查SIM卡的存在
flash char  IMSI_Check_str[]="AT+CIMI\r\n";   
flash char  Signal_Intensity_str[]="AT+CSQ\r\n";  // 信号强度 ，常用
flash char  CommAT_Str1[]="ATV1\r\n";
flash char  CommAT_Str2[]="ATE0\r\n";
flash char  CommAT_Str3[]="AT+QSPN?\r\n";  
flash char  CommAT_Str4[]="AT+QAUDCH=2\r\n";           // 设置音频通道  主默认    2   免提
flash char  CommAT_Str5[] ="AT+QMIC=0,10\r\n";  // 设置音频mic 输入通道 选择第一路      
flash char  CommAT_Str6[] ="AT+QIMUX=1\r\n";   //启用多链路模式
flash char  CommAT_Str7[]="AT+CLVL=80\r\n";  // 扬声器设置  --没有音频功放 没用
flash char  CommAT_Str8[]="AT+CRSL=80\r\n";//;来电音量
flash char  CommAT_Str9[]="ATL2\r\n";// 设置监听扬声器音量
flash char  CommAT_Str10[]="AT+CSCA?\r\n";    // 
flash char  CommAT_Str11[]="AT+CGMR\r\n";//"AT%RECFDEL\r\n";//"AT\r\n"; //   
flash char  CommAT_Str12[]="AT+CMGF=1\r\n"; 
flash char  CommAT_Str13[]="AT+CPMS=\"SM\",\"SM\",\"SM\"\r\n";   
flash char  CommAT_Str14[]="AT+CNMI=2,1\r\n"; 
flash char  CommAT_Str15[]="AT+CMGD=1,4\r\n"; 



//    2.   通信模块登陆数据链路相关命令
flash char       DialInit1[]                          = "AT+CGATT=1\r\n"; 
flash char	DialInit2[]				=	"AT\r\n";//"AT+CGACT=1,1\r\n";	
flash char	DialInit3[]				=	"AT+CGPADDR=1\r\n";	  
flash char	DialInit4[] 			       =	"AT\r\n";  // 查看当前IP   
char	Dialinit_DNSR[50] 			=	"AT+QIDNSGIP=\"";     //  域名 up.gps960.com
char	Dialinit_DNSR2[50] 			=	"AT+QIDNSGIP=\"";     //  域名

flash char	DialInit6[] 			       =	"AT+CREG?\r\n";  
flash char	DialInit7[] 			       =	"AT\r\n";	 
flash char	DialInit8[] 			       =	"AT\r\n";//"AT+CGPADDR\r\n";     

static u8   Mainlink_send_over=0; // 主链路已经发送信息
flash u8	Send_TCP[20]="AT+QISEND=1,%d\r\n";
flash u8	Send_ISP[20]="AT+QISEND=2,%d\r\n";	
static u8	DialStr_Link1_demo[90]="AT+QIOPEN=%d,\"%s\",\"%d.%d.%d.%d\",\"%d\"\r\n";   //"ATD*99***1#\r\n"; AT%IPOPENX=1,"UDP","117.11.126.248",7106
static u8   DialStr_LinkAux_demo[90]="AT+QIOPEN=%d,\"%s\",\"%d.%d.%d.%d\",\"%d\"\r\n"; //"ATD*99***1#\r\n"; AT%IPOPENX=1,"UDP","117.11.126.248",7106
static u8   DialStr_IC_card_demo[90]="AT+QIOPEN=%d,\"%s\",\"%d.%d.%d.%d\",\"%d\"\r\n"; //"ATD*99***1#\r\n"; AT%IPOPENX=1,"UDP","117.11.126.248",7106

static u8	DialStr_Link1[90];  
static u8   DialStr_LinkAux[90]; 
static u8   DialStr_IC_card[90];


static u8	Dialinit_APN[40]="AT+CGDCONT=1,\"IP\",\"";		//CMNET\"\r\n"; 	 // oranges Access Point Name
static char DialStr_ISP[50]="AT+QIOPEN=2,\"TCP\",\"60.28.50.210\",\"6071\"\r\n"; //  远程升级            
 


flash char  CutDataLnk_str2[]="AT+QICLOSE=1\r\n"; 
flash char  CutDataLnk_str3[]="AT\r\n";
flash char  CutDataLnk_str4[]="AT+QIDEACT\r\n";  
flash char  CutDataLnk_str5[]="ATH\r\n";     

#endif

//  M66  Rec voice 
flash char VoiceRec_config[]="ATI\r\n";  // 设置录音格式和录音质量  
                                                                                                 //  0C: 28 长度 18组 504
flash char VoiceRec_start[]="AT+QAUDRD=1,\"voice.amr\",3\r\n";   // 1: start rec    3: amr format
flash char VoiceRec_stop[]="AT+QAUDRD=0,\"voice.amr\",3\r\n";	
flash char VoiceRec_file_size[]="AT+QFLST\r\n";	 
flash char VoiceRec_delete[]="AT+QFDEL=\"voice.amr\"\r\n";	
flash char VoiceRec_fileopen[]="AT+QFOPEN=\"voice.amr\"\r\n";	

static u8  VoiceRec_sd_getAT[50]="AT+QFREAD=%s,%d\r\n"; 
static u8 VoiceRec_fileclose[]="AT+QFCLOSE=%s\r\n";	

u16  Voice_Rec_Duartion=17; 
u8   AT_waitOK=0;
u8   Voice_data_come=0; 
u16  Voice_rx_len=0; // 语音数据接收长度
u8   Get_voicedata_delay=0; 
u8   GEt_voice_Counter=0;    
u8	AT_str[50]; 	
u8  Rx_Error_counter=0;  // 连续错误计数器，连续错误3次以上认为有问题，挂断重播链路

                                                                                                 
//-------  struct  variables -------------
static GSM_POWER   GSM_PWR;   
static IMSI_GET       IMSIGet; 
COMM_AT               CommAT; 
DATA_DIAL             DataDial;  



ALIGN(RT_ALIGN_SIZE)
 u8     GSM_rx[GSMRX_SIZE];       
u16     GSM_rx_Wr=0;
u16     gsm_rx_infolen=0;  
u8      gsm_rx_linknum=0; 
u8      gsm_rdy_2_rxEnable=0;   // 等待接收


static GSM_typeBUF GSM_INT_BUFF;
static GSM_typeBUF GSM_RX_BUFF;

u8  GSM_AsciiTx[GSM_AsciiTX_SIZE];
u16   GSM_AsciiTx_len=0; 

//-----  1 s timer  related below   ------
u16   one_second_couner=0;  
u8     Enable_UDP_sdFlag=0;      

 //----- handle jianquan flag
 u8   Hand_Login=0; // 手动鉴权标志位
 u8   Enable_IClink=0;  //  手动使能连接IC 卡中心

u8    Dial_jump_State=0;  // 拨号过程中  状态跳转标志， DialInit5  DialInit6 到 DialInit7 的跳转标记
u8    Dnsr_state=0;  //  DNSR 状态   1: 表示在域名解析成功的前提下


//------- GPPRS 功能相关 ----------------
//u8 Datalink_close=0;  //挂断后不再登陆



//-------- TCP2 send ---------
u8     TCP2_ready_dial=0;
u16    Ready_dial_counter2=0;
u16    TCP2_notconnect_counter=0;
u8     TCP2_Connect=0;
u8	  TCP2_IC_sdFlag=0;		//定时发送GPS位置信息标志
u16    TCP2_sdDuration=50; 
u8      TCP2_Coutner=0;  // 定时器计数
u8      TCP2_login=0;       // TCP 建立好连接后的标志位


//    TTS   相关
 TTS              TTS_Var;  //  TTS 类型变量
 ALIGN(RT_ALIGN_SIZE)
 u8                AT_TTS[1024];
 u16               TTS_Len=0;	 


//  Voice  Record 
VOC_REC       VocREC;    // 录音上传相关      

 

static void GSM_Process(u8 *instr, u16 len);
u32 GSM_HextoAscii_Convert(u8*SourceHex,u16 SouceHexlen,u8 *Dest);   

//   VOICE  RECORD   
void  VOC_REC_Init(void)
{
            VocREC.running=0;  								
			VocREC.Sate=VOICEREC_IDLE; 
			VocREC.ExcuteFlag=0;   
	        VocREC.rdytoTrans=0;  
			VocREC.file_write=0; 	 
			VocREC.recording_timer=0;
			VocREC.packet=0;
			AT_waitOK=0;

}
FINSH_FUNCTION_EXPORT(VOC_REC_Init, TTS VOC_REC_Init); 

void VOC_REC_timer(void)
{ 
     //  1.  录音时间计数器
     if(VOICEREC_RECORDING==VocREC.Sate)
     {
         VocREC.recording_timer++;
		 if(VocREC.recording_timer>=Voice_Rec_Duartion)//20 
		 {
		    VocREC.recording_timer=0;
			
			VocREC.Sate=VOICEREC_REC_STOP;
			VocREC.ExcuteFlag=1; // enable  once 
		 }

     }
     //  2.   voice   trans  timer       数据传输定时器
     if(VocREC.rdytoTrans)	
     	{
               VocREC.rdytoTrans++;
	        if(VocREC.rdytoTrans>=3)
		   {
                       Sound_send_start(); //开始上传
		        VocREC.rdytoTrans	=0;	
	           }

     	}

}

void VOC_REC_FileEndProcess(void)
{
     u16  namelen=0,i;	
     u8    pic_buf[50]; 
	
    		 {    //   最后一包
                       //--------------------------
                       VocREC.curren_PageCNT=SoundStart_offdet;
			  memset(pic_buf,0,sizeof(pic_buf)); 
                       memcpy(pic_buf,(u8*)&VocREC.file_write,4);	  // filesize  
                       namelen=strlen((const char*)VocREC.file_name); // get filename len
			  memcpy(pic_buf+4,VocREC.file_name,namelen);			  
			  DF_WriteFlashDirect(SoundStart_offdet,0,pic_buf, 25);   //  4+20  1 (null)
			  DF_delay_ms(10); 			   
                      //--------------------------------------------------------------------------                  
	               rt_kprintf("\r\n        VoiceFileSize: %d Bytes\r\n   \r\n",VocREC.file_write); 
                      //-------------------------------------------------------------------------- 
                      VocREC.Sate=VOICEREC_IDLE;  
			 VocREC.file_write=0;   // clear  
                        DF_LOCK=disable; 

			
			 //   开始准备上传 录音
			 VocREC.rdytoTrans	=1;					 
			 //--------------------------------------------------------------------------
		  }
}


void VOC_REC_process(void)
{
    u16 str_wr=0, send_len=0;
	  u16  i=0;//,j=0;
	  u8   pic_buf[550];  
	  
  if(VocREC.running==1)
  {
        
      switch(VocREC.Sate)
      {
		   case	VOICEREC_DeleteOld:   // 1.  删除old  文件
									 if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
									 {	
										rt_kprintf(VoiceRec_delete); 
										rt_hw_gsm_output((const char*)VoiceRec_delete);   //给模块发送停止录音命令	   
										AT_waitOK=1;
									  }
								 break; 				
           case VOICEREC_RECORD_START:   //  2.  开始录音
                                       if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           {  
							             //  1 .   get name ,  clear df  area  , write multimedia index 
							             DF_LOCK=enable; 
										memset(VocREC.file_name,0,sizeof(VocREC.file_name)); 	    									
										sprintf((char*)VocREC.file_name,"%d%d%d%d.amr",time_now.day,time_now.hour,time_now.min,time_now.sec);    
										Api_DFdirectory_Delete(voice);   //clear  voice  DF  area 
										rt_kprintf("\r\n			录音文件名称: %s \r\n",VocREC.file_name);	
										Save_MediaIndex(1,VocREC.file_name,0,0);     
										memset(VocREC.SendString,0,sizeof((const char*)VocREC.SendString));
										str_wr=strlen((const char*)VoiceRec_start);
										memcpy(VocREC.SendString,VoiceRec_start,str_wr); 
										// debug
										rt_kprintf("\r\n %s",VocREC.SendString); 
										rt_hw_gsm_output((const char*)VocREC.SendString);   //给模块发送录音命令										
                                        AT_waitOK=1;  
							           }	
                           			   
								  break;
			case  VOICEREC_REC_STOP:    // 3.  停止录音
				                     if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           { 
										  rt_kprintf("\r\n %s",VoiceRec_stop); 
										  rt_hw_gsm_output((const char*)VoiceRec_stop);   //给模块发送停止录音命令	 
										 AT_waitOK=1;
				                       }
									   break;
		    case  VOICEREC_GET_FILESIZE: //  4.  获取已录制文件的 大小
				                      if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           { 
										  rt_kprintf("\r\n %s",VoiceRec_file_size); 
										  rt_hw_gsm_output((const char*)VoiceRec_file_size);   //给模块发送停止录音命令	 
										 AT_waitOK=1; 
				                       }
	   	                               break;
			 case  VOICEREC_FileOpen:  //   5.   打开文件 获取句柄
				                       if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           { 
										  rt_kprintf("\r\n %s",VoiceRec_fileopen); 
										  rt_hw_gsm_output((const char*)VoiceRec_fileopen);   //给模块发送停止录音命令	 
										  AT_waitOK=1;
				                       }
	   	                               break;						   

			case  VOICEREC_Sd_getcmd:	   	 //6.  发送命令 获取 录音 数据                             
									  if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           { 
							              memset(AT_str,0,sizeof(AT_str)); 
                                          send_len=VocREC.filesize-VocREC.file_write;
										  if(send_len>1024)
										  	   send_len=1024;
										 if(strlen(VocREC.handle_name)	)   
										 {
										  sprintf((char*)AT_str,VoiceRec_sd_getAT,VocREC.handle_name,send_len);
										  rt_kprintf("\r\n %s",AT_str); 
										  rt_hw_gsm_output((const char*)AT_str);   //给模块发送停止录音命令	 
										  VocREC.Sate=VOICEREC_Rdy_Rxing;
										  VocREC.ExcuteFlag=0;
										 } 
									   }
									   break;	 
			case  VOICEREC_DataRXing:		//  7. 接收存储 录音数据
				                       if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
							           { 
                                            OutPrint_HEX("voice 数据",VocREC.HEX, VocREC.HEX_len); 
										  // 3.  write and	check
										  if(VocREC.HEX_len>512)
										    {
										       DF_WriteFlashDirect( VocREC.curren_PageCNT,0,VocREC.HEX, 512); 
											   			   //---  read compare 
											  memset(pic_buf,0,sizeof(pic_buf)); 
											  delay_ms(200);
											  DF_ReadFlash(VocREC.curren_PageCNT,0,pic_buf, 512);	  
											  for(i=0;i<512;i++)
											 {		  if(pic_buf[i]!=VocREC.HEX[i])
														 {
															   rt_kprintf(" \r\n 1----read not equal write  where i=%d  \r\n",i);  
													  break;	   
														  }
											  } 
                                               VocREC.curren_PageCNT++;
											   //------------------------------------
											   DF_WriteFlashDirect( VocREC.curren_PageCNT,0,VocREC.HEX+512, VocREC.HEX_len-512); 
											   		   //---  read compare 
											  memset(pic_buf,0,sizeof(pic_buf)); 
											  delay_ms(200);
											  DF_ReadFlash(VocREC.curren_PageCNT,0,pic_buf, VocREC.HEX_len-512);	  
											  for(i=0;i<(VocREC.HEX_len-512);i++)
											 {		  if(pic_buf[i]!=VocREC.HEX[512+i])
														 {
															   rt_kprintf(" \r\n 2----read not equal write  where i=%d  \r\n",i);  
													  break;	   
														  }
											  } 
											   VocREC.curren_PageCNT++;
										  	}   
										  else
										  	{  
										  	  DF_WriteFlashDirect( VocREC.curren_PageCNT,0,VocREC.HEX, VocREC.HEX_len); 
										  
													   //---  read compare 
											  memset(pic_buf,0,sizeof(pic_buf)); 
											  DF_ReadFlash(VocREC.curren_PageCNT,0,pic_buf, VocREC.HEX_len);	  
											  for(i=0;i<VocREC.HEX_len;i++)
											 {		  if(pic_buf[i]!=VocREC.HEX[i])
														 {
															   rt_kprintf(" \r\n 3----read not equal write  where i=%d  \r\n",i);  
													  break;	   
														  }
											  } 
											  VocREC.curren_PageCNT++; 
										  	} 
											 // 4.  write update  
											 send_len=VocREC.filesize-VocREC.file_write;
											 VocREC.file_write+=VocREC.HEX_len;	   //  add len
											 VocREC.packet++;
											 rt_kprintf(" \r\n VocREC.file_write=%d   packetnum: %d \r\n",VocREC.file_write,VocREC.packet); 		
										  
										  
											//	 5.  judge end
											  if(send_len<1024)  //  可以认为是最后一包
											  {												  
												   VocREC.ExcuteFlag=1;
												   VocREC.Sate=VOICEREC_Close;
												   break; 
											  }								
									      VocREC.ExcuteFlag=0;   
				                       }   
				                       break; 
			case  VOICEREC_Close:	   if(( 1==VocREC.ExcuteFlag)&&(AT_waitOK==0))
				                       {  //VoiceRec_fileclose
				                           memset(AT_str,0,sizeof(AT_str)); 
										   sprintf((char*)AT_str,VoiceRec_fileclose,VocREC.handle_name);
										  rt_kprintf("\r\n %s",AT_str); 
										  rt_hw_gsm_output((const char*)AT_str);   //给模块发送停止录音命令	 
                                          VOC_REC_FileEndProcess();
										  VocREC.ExcuteFlag=0;    
										  VocREC.Sate=VOICEREC_IDLE;
										  VocREC.running=0;  
				                       }  
				                       break;
    	    case  VOICEREC_IDLE:
	   	                               break;   
											 
	   

      	} 
  }
}


void VOC_REC_Stop(u8 *instr)     //  录音结束
{                                                           //%RECSTOP: 10,7000
    u16  i;	

               for(i=0;i<20;i++)   //  从前20个字节中找第一个,
		{
			if(instr[i]==',')
				break;
		} 
	      sscanf((const char*)instr+i+1, "%u", (u32*)&VocREC.filesize);

             // 1. 对比
	      rt_kprintf("\r\n   VoiceFile Size: %d    read =%d Bytes",VocREC.filesize,VocREC.file_write);
            //  2. end process
	      VOC_REC_FileEndProcess();

}
void VOC_REC_dataGet(u8 *instr )
{
    u16  i=0,commer=0;//,j=0;
    u16  namelen=0,len=0;	
    u8   pic_buf[550];  
	
   	if(VocREC.Sate==VOICEREC_DataRXing)  //%RECDATA:18,"0C49CA45E3F24DF500BE1
    	{
              for(i=0;i<20;i++)   //  从前20个字节中找第一个:
		{
		      if(instr[i]==':')
			  	commer=i;
			if(instr[i]=='"')
				break;
		} 
		 //  1. 获取长度	  
		  sscanf(instr+commer+1, "%d", (u32*)&len); 
		  rt_kprintf("\r\n  Rxlen=%d   RxinfoLen=%d bytes\r\n     ",len,len*28); //  
		  len=len*28;
                //  for(j=0;j<len;j++)
		  //          rt_kprintf("%c",instr[i+1+j]);   

		   // 2.  转换		
		   VocREC.HEX_len= GSM_AsciitoHEX_Convert(instr+i+1,len,VocREC.HEX); 	 
		   if(VocREC.HEX_len!=(len>>1))
		 	    rt_kprintf("\r\n   VocREC.HEX_len=%d  len=%d  voice  ASCII to HEX error!\r\n",VocREC.HEX_len,(len>>1));
                   
 
		    // 3.  write and  check
	                 DF_WriteFlashDirect( VocREC.curren_PageCNT,0,VocREC.HEX, VocREC.HEX_len); 
                         //---  read compare 
			    memset(pic_buf,0,sizeof(pic_buf)); 
			    DF_ReadFlash(VocREC.curren_PageCNT,0,pic_buf, VocREC.HEX_len);      
			    for(i=0;i<VocREC.HEX_len;i++)
			   {		if(pic_buf[i]!=VocREC.HEX[i])
			    	       {
			    	             rt_kprintf(" \r\n ----read not equal write  where i=%d  \r\n",i);  
						break;		 
			    	        }
			    }	    
                    // 4.  write update 
			  VocREC.file_write+=VocREC.HEX_len;	 //  add len
			   rt_kprintf(" \r\n VocREC.file_write=%d  \r\n",VocREC.file_write);      


		      //   5.  judge end
		        if(len<560)  //  可以认为是最后一包
		        {
		             	      VOC_REC_FileEndProcess();
		        }
		
    	} 
       else
	   	 rt_kprintf("\r\n Other VocREC.Sate=%d\r\n",VocREC.Sate);    


}


 void VOC_REC_Start(u8 value)    // 录音开始
 { 
     if(value==1)
	 	VocREC.running=0;
	 
     if(GSM_PWR.GSM_power_over)     //  必须模块ON 起来才能操作
     {
         if(VocREC.running==0)      //  若正在处理中，不处理    
		{											   
			VocREC.running=1;  								
			VocREC.JT808_Duration=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
			VocREC.JT808_SaveOrNot=UDP_HEX_Rx[16];
			VocREC.Sate=VOICEREC_DeleteOld; 
			VocREC.ExcuteFlag=1;   
			VocREC.curren_PageCNT=SoundStart_offdet+1;  // clear    // 第一个page 留着最后写语音信息用，空着先
	        VocREC.rdytoTrans=0;  
			VocREC.file_write=0; 	 
			VocREC.recording_timer=0;
			VocREC.packet=0;
		}
     }			 
 }
//FINSH_FUNCTION_EXPORT(VOC_REC_Start, voice recrod); 

void record_voice(u16 dur)
{
   rt_kprintf("\r\n 手动录音: %d s\r\n",dur);      
   Voice_Rec_Duartion=dur;
   VOC_REC_Start(1);
}
FINSH_FUNCTION_EXPORT(record_voice, voice record_voice);

//=========================================================================
//    TTS  realated
u8    TTS_Get_Data(u8 *Instr,u16 LEN)     //  return   0   : OK     return   1 : busy
{
     // 1. check status 
   if(TTS_Var.Playing)
   	{
           memset(TTS_Var.HEX_BUF,0,sizeof((const char*)TTS_Var.HEX_BUF));
          TTS_Var.HEX_len=LEN;
	    memcpy(TTS_Var.HEX_BUF,Instr,LEN);		
           TTS_Var.Save=1;  
	    rt_kprintf("\r\n   播报中,来信息存储\r\n"); 	   
   	    return  TTS_BUSY;
   	}  	
    //  2.   HEX to  ASCII convert 
     memset(TTS_Var.ASCII_BUF,0,sizeof((const char*)TTS_Var.ASCII_BUF));
	#ifdef M50_GSM
	    memcpy(TTS_Var.ASCII_BUF,Instr,LEN);
	    TTS_Var.ASCII_Len=LEN;  
    #else       
		TTS_Var.ASCII_Len=GSM_HextoAscii_Convert(Instr,LEN,TTS_Var.ASCII_BUF);
		TTS_Var.ASCII_Len=(LEN<<1);// 长度乘以 2	
	#endif
     //  3. cacu  timeout value
     TTS_Var.TimeOut_limt=LEN/3+3;  // 3个字每秒+ 3  秒保护时间
     TTS_Var.TimeCounter=0;
     //  4.  ready to play
	TTS_Var.NeedtoPlay=1;	 
    return  TTS_OK;
}
u8    TTS_Data_Play(void) 
{
       u16   i=0;
	//  1.  check status  
      if(TTS_Var.Playing)
              return  TTS_BUSY;
      else
	if(TTS_Var.Save)  //  2.  check  save  status
	{      
	      TTS_Var.Save=0;
	     //  3. HEX to  ASCII	  
	     memset(TTS_Var.ASCII_BUF,0,sizeof((const char*)TTS_Var.ASCII_BUF));
	     TTS_Var.ASCII_Len=GSM_HextoAscii_Convert(TTS_Var.HEX_BUF,TTS_Var.HEX_len,TTS_Var.ASCII_BUF);
           TTS_Var.ASCII_Len=(TTS_Var.HEX_len<<1);// 长度乘以 2	 
	     //  4. cacu  timeout value
	     TTS_Var.TimeOut_limt=TTS_Var.HEX_len/3+3;  // 3个字每秒+ 3  秒保护时间
	     TTS_Var.TimeCounter=0;
	     //  5.  ready to play
		TTS_Var.NeedtoPlay=1;        
	}
       	
      //   6.   play process
      if( TTS_Var.NeedtoPlay	==1)
      	{
      	  Speak_ON;   // 开功放
      	   TTS_Var.Playing=1;
           //  head
           memset(AT_TTS,0,sizeof(AT_TTS));  
	    strcat(AT_TTS,"AT+QTTS=2,\""); 
	    TTS_Len=strlen(AT_TTS)	;	      
	   //  info		 
	     memcpy(AT_TTS+TTS_Len,TTS_Var.ASCII_BUF,TTS_Var.ASCII_Len);	
	     TTS_Len+=TTS_Var.ASCII_Len; 
	   //  tail	 
	      memcpy((char*)AT_TTS+TTS_Len,"\",4\r\n",5);    // tail 	  ,4  是M50 技术支持说的
            TTS_Len+=5;      	 
	        
	    for(i=0;i<TTS_Len;i++)
			 rt_kprintf("%c",AT_TTS[i]);   

	   rt_hw_gsm_output_Data(AT_TTS,TTS_Len);	

	  WatchDog_Feed();
         delay_ms(30); // rt_thread_delay(RT_TICK_PER_SECOND/8);	
 
           //---------------------
           TTS_Var.NeedtoPlay=0; 
      	}
	  return TTS_OK;
}

void   TTS_Play_End(void)
{
             TTS_Var.Playing=0;
	      TTS_Var.TimeCounter=0;
		 
	   Speak_OFF;	   // 关闭功放
}

void TTS_Exception_TimeLimt(void)     //  单位: s 
{    //  TTS  播报异常 超时计数器
      if(TTS_Var.Playing)
      	{
	         TTS_Var.TimeCounter++;
		  if(TTS_Var.TimeCounter>TTS_Var.TimeOut_limt)
		  	{
		           TTS_Var.Playing=0;
		           TTS_Var.TimeCounter=0;
		  	}
      	}
}

u8  TTS_ACK_Error_Process(void)
{   // 发送TTS  语音播报命令后返回Error 异常
      if(TTS_Var.Playing)
        {      
	           TTS_Var.Playing=0;
	           TTS_Var.TimeCounter=0; 
		     return true;	   
	 }	
      else
	  	   return false;
}

void TTS_play(u8 * instr)
{
      WatchDog_Feed();
      TTS_Get_Data(instr,strlen(instr));
      rt_kprintf("\r\n    手动语音播报: %s\r\n",instr);
}
FINSH_FUNCTION_EXPORT(TTS_play, TTS play);


void GSM_CSQ_timeout(void)
{
            CSQ_counter++;
          if(CSQ_counter>=CSQ_Duration)
          {
              CSQ_counter=0;  
              CSQ_flag=1;    
          }	 

}

void GSM_CSQ_Query(void)
{    
    	    if((CSQ_flag==1)&&(MediaObj.Media_transmittingFlag==0)&&(GSM_PWR.GSM_power_over)&&(stopNormal==0)&&(VocREC.running==0))   
	   { 
		  CSQ_flag=0; 
		  rt_hw_gsm_output("AT+CSQ\r\n");    //检查信号强度
		    if(DispContent)	
		        rt_kprintf("AT+CSQ\r\n");  
	   } 	
}




//-------------------------------------------------------------------------------
void  DataLink_MainSocket_set(u8 *IP, u16  PORT, u8 DebugOUT)
{
	  memset((char *)DialStr_Link1,0,sizeof(DialStr_Link1)); // clear	  
	  sprintf((char*)DialStr_Link1,DialStr_Link1_demo,1,"TCP",IP[0],IP[1],IP[2],IP[3],PORT);	

	     if(DebugOUT)
	     	{
		  rt_kprintf("		   Main Initial  Str:");  
		  rt_kprintf((char*)DialStr_Link1);
	     	}  
}

void  DataLink_AuxSocket_set(u8 *IP, u16  PORT,u8 DebugOUT) 
{
  

	  memset((char *)DialStr_LinkAux,0,sizeof(DialStr_LinkAux)); // clear	  
	  sprintf((char*)DialStr_LinkAux,DialStr_LinkAux_demo,1,"TCP",IP[0],IP[1],IP[2],IP[3],PORT);
	  
         if(DebugOUT)
	 {
	   rt_kprintf("\r\n  辅IP   DialString : "); 
	   rt_kprintf((char*)DialStr_LinkAux);          
	 } 
}

void  DataLink_IC_Socket_set(u8 *IP, u16  PORT,u8 DebugOUT) 
{

      memset((char *)DialStr_IC_card,0,sizeof(DialStr_IC_card)); // clear	  
	  sprintf((char*)DialStr_IC_card,DialStr_IC_card_demo,2,"TCP",IP[0],IP[1],IP[2],IP[3],PORT); 
     if(DebugOUT)
	 {
	     rt_kprintf("\r\n IC  DialString : "); 
	     rt_kprintf((char*)DialStr_IC_card);         
	 } 	 
		 
}

void  DataLink_APN_Set(u8* apn_str,u8 DebugOUT)
{
         memset(Dialinit_APN+APN_initSTR_LEN,0,sizeof(Dialinit_APN)-APN_initSTR_LEN);
	  memcpy(Dialinit_APN+APN_initSTR_LEN,apn_str,strlen((char const*)apn_str));
	  strcat( (char *)Dialinit_APN,"\"\r\n" );

	  if(DebugOUT)
	  {
	     rt_kprintf("\r\n APN 设置 :  ");    
	     rt_kprintf((const char*)Dialinit_APN);      
	 }
}


void  DataLink_DNSR_Set(u8* Dns_str,u8 DebugOUT)
{
         memset(Dialinit_DNSR+9,0,sizeof(Dialinit_DNSR)-9); 
	  memcpy(Dialinit_DNSR+9,Dns_str,strlen((char const*)Dns_str)); 
	  strcat( Dialinit_DNSR,"\"\r\n" );  
	  	
	if(DebugOUT)
	{
		  rt_kprintf("\r\n		 域名1 设置 :	 "); 
		  rt_kprintf(Dialinit_DNSR); 
	}

}

void  DataLink_DNSR2_Set(u8* Dns_str,u8 DebugOUT)
{
         memset(Dialinit_DNSR2+9,0,sizeof(Dialinit_DNSR2)-9); 
	  memcpy(Dialinit_DNSR2+9,Dns_str,strlen((char const*)Dns_str)); 
	  strcat( Dialinit_DNSR2,"\"\r\n" ); 
	  	
	if(DebugOUT)
	{
		  rt_kprintf("\r\n		Aux  域名设置 :	 "); 
		  rt_kprintf(Dialinit_DNSR2); 
	}

}

 void Gsm_RegisterInit(void)
{  
     //--------   Power  Related   ---------
     GSM_PWR.GSM_PowerEnable=1;  // 开始使能
     GSM_PWR.GSM_powerCounter=0;
     GSM_PWR.GSM_power_over=0;  

    // --------  IMSI  -------------------
    memset((u8*)&IMSIGet,0,sizeof(IMSIGet));       

    //---------- COMM AT ----------------
    memset((u8*)&CommAT,0,sizeof(CommAT)); 
   

   //----------- Data Dial ----------------
   memset((u8*)&DataDial,0,sizeof(DataDial)); 
   DataDial.start_dial_stateFLAG=1; //important 

  //---  Network Setting  Default ----
  DataLink_APN_Set(APN_String,1);   // apn 
  DataLink_DNSR_Set(DomainNameStr,0);    // DNSR  MG323  没有 
  DataLink_DNSR2_Set(DomainNameStr_aux,0);  
  DataLink_MainSocket_set(RemoteIP_main, RemotePort_main,1); 
  DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_aux,1);

 
  #if 0
   GsmRxBuf_Wr=0;
   GsmRxBuf_Rd=0;
  
   GSM_rx_Wr=-1;
  
   GsmTxLen=0;
   GsmTxIndex=0;
   pGsmTx=NULL;
   GsmTxActive=OS_FALSE;
  
  
  //------- GPPRS 功能相关 ----------------
  GSM_PWR.GSM_powerCounter=0;
  quick_flag=0;
  GSM_PWR.GSM_power_over=0;  
  Datalink_close=0;	//挂断后不再登陆
  
  //-------  蜂鸣器状体 ------------------------
  buz_on_Flag=0;
  buz_on_Counter=0;  
  
  DataLink_Online=0;    // GPRS 在线标志
  
  EM310_StartFLAG=0;// EM310 模块开启标志位
  COPS_Couter=0;    // COPS  返回次数
  
  CSQ_counter=0;
  CSQ_Duration=32;	//查询CSQ 的定时间隔
  CSQ_flag=1;
  ModuleSQ=0;       //GSM 模块信号强度数值
  ModuleStatus=0;   //网络状态  
  
  
  
  
  //-----  1 s timer  related below   ------
  one_second_couner=0;
  Enable_UDP_sdFlag=0;
  Timercounter=0;
    
  //------ WatchDog --
   WatchDog_Feed();  //  Task Idle Hook 相关
      
  //-----  AT -----------
  memset(&AT,0,sizeof(AT));
  gsm_rx_infolen=0;
  DataLink_EndFlag=0;  
  DataLink_end_counter=0;
  DataConnect_counter=0;		 //  在没有登网前重拨次数限制计数器    11-3-4 补加的 ERROR 20 
     
    
  //----- UDP ------------
  UDP_sdFlag=0;		  //定时发送GPS位置信息标志
  UDP_sdDuration=10; 
  UDP_Connect=0; 	  
#endif
}

 void  Total_Satus_Initial(void)
{
     Gsm_RegisterInit();   
} 


/* write one character to serial, must not trigger interrupt */
 void rt_hw_gsm_putc(const char c)
{
	/*
		to be polite with serial console add a line feed
		to the carriage return character
	*/
	//if (c=='\n')rt_hw_gps_putc('\r');	  
	//USART_SendData(UART4,  c);  
	//while (!(UART4->SR & USART_FLAG_TXE));   
	//UART4->DR = (c & 0x1FF);   
       	USART_SendData( UART4, c );
		while( USART_GetFlagStatus( UART4, USART_FLAG_TC ) == RESET )  
		{
		}
	
}

void rt_hw_gsm_output(const char *str)
{
     u16  len=0;
	/* empty console output */
	//--------  add by  nathanlnw ---------
	
     while (*str) 
	{
		rt_hw_gsm_putc (*str++);
	}
	
	/* len=strlen(str);
       while( len )
	{
		USART_SendData( UART4, *str++ );	
		while (!(UART4->SR & USART_FLAG_TXE));  
		UART4->DR = (*str++ & 0x1FF);   
		len--;
	}  */
       //--------  add by  nathanlnw  --------	
}

void rt_hw_gsm_output_Data(u8 *Instr,u16 len)  
{
      unsigned int  infolen=0;

	  infolen=len;

	  //--------  add by  nathanlnw ---------
       while (infolen)
	{
		rt_hw_gsm_putc (*Instr++);
		infolen--;
	}
       //--------  add by  nathanlnw  --------	

}
void Dial_Stage(T_Dial_Stage  Stage)
{	// set the AT modem stage
	//if (DataDial.Dial_step == Stage) return;	// no change    
									//
	DataDial.Dial_step = Stage;				//
	DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
	DataDial.Dial_step_Retry=0;
}
void  GSM_RxHandler(u8 data)
{
    static u8  former_byte=0;
	static  u16  i=0;

		if(gsm_rx_infolen)
		{
		   if(gsm_rdy_2_rxEnable==1)   // first  rx 
		   	{ 
		   	  GSM_INT_BUFF.gsm_wr = 0; // clear first
		   	  gsm_rdy_2_rxEnable=0;
		   	}
		   gsm_rx_infolen--;
           
		   GSM_INT_BUFF.gsm_content[GSM_INT_BUFF.gsm_wr++] = data;
		   
		   if(gsm_rx_infolen==0)    // rx end 
		   	{     
        
	                switch(gsm_rx_linknum)
	                	{
	                	   case  1:  // mainlink								
								 memcpy(GSM_HEX, GSM_INT_BUFF.gsm_content,GSM_INT_BUFF.gsm_wr); 
								 GSM_HEX_len=GSM_INT_BUFF.gsm_wr; 
								 App_rxGsmData_SemRelease(GSM_HEX, GSM_HEX_len,gsm_rx_linknum-1); 
								 gsm_rx_linknum=0;
								 break;
						   case  2:  // aux link
						          memcpy(GSM_HEX, GSM_INT_BUFF.gsm_content,GSM_INT_BUFF.gsm_wr); 
								 GSM_HEX_len=GSM_INT_BUFF.gsm_wr; 
								 App_rxGsmData_SemRelease(GSM_HEX, GSM_HEX_len,gsm_rx_linknum-1); 
								 gsm_rx_linknum=0; 
							     break; 
	                	}			  					
				GSM_INT_BUFF.gsm_wr = 0; 
		   	}
		   else   //  rx  not  end  
		   	{
		   	  //---------------------------------------------------------
	             if( GSM_INT_BUFF.gsm_wr == GSM_TYPEBUF_SIZE )
				{
					GSM_INT_BUFF.gsm_wr = 0;
				}
				GSM_INT_BUFF.gsm_content[GSM_INT_BUFF.gsm_wr ]=0;  
			  //-------------------------------------------------------- 	
		   	}
             
		}
		else
   		if( ( data == 0x0a ) && ( former_byte == 0x0d ) ) /*遇到0d 0a 表明结束*/
		{
			GSM_INT_BUFF.gsm_content[GSM_INT_BUFF.gsm_wr++] = data;
			if( GSM_INT_BUFF.gsm_wr < 1400 )
			{
				
				  /* 在中断里判断并处理*/  
                #ifdef M50_GSM
				 if( strncmp( GSM_INT_BUFF.gsm_content, "+RECEIVE:", 9 ) == 0 ) //+RECEIVE: 1, 29 
				 {
					 /*解析出净信息,编译器会优化掉pdst*/
					 i = sscanf(GSM_INT_BUFF.gsm_content, "+RECEIVE: %d, %d", &gsm_rx_linknum, &gsm_rx_infolen );
					 if( i != 2 )
					 {
						 gsm_rx_linknum  = 0;  
						 gsm_rx_infolen= 0;
					 }
					 gsm_rdy_2_rxEnable=1;	 

					// rt_kprintf("\r\n  Gsmrx_linknum=%d \r\n",gsm_rx_linknum);
				 }
				else
               #endif
				  rt_mq_send( &mq_GSM, (void*)&GSM_INT_BUFF, GSM_INT_BUFF.gsm_wr+ 2 );
			}
			GSM_INT_BUFF.gsm_wr = 0;
		}
		else
		{
			GSM_INT_BUFF.gsm_content[GSM_INT_BUFF.gsm_wr++] = data;    
			if( GSM_INT_BUFF.gsm_wr == GSM_TYPEBUF_SIZE )
			{
				GSM_INT_BUFF.gsm_wr = 0;
			}
			GSM_INT_BUFF.gsm_content[GSM_INT_BUFF.gsm_wr ]=0;  
		}
		former_byte = data;

  
	  
}

void  GSM_Buffer_Read_Process(void)
{
      char ch;  
//-----------------------------------------------------
	rt_err_t	res;

	{
		res = rt_mq_recv( &mq_GSM,(void*)&GSM_RX_BUFF, 1400, RT_TICK_PER_SECOND/50 ); //等待100ms,实际上就是变长的延时,最长100ms
		if( res == RT_EOK )                                                     //收到一包数据
		{
				GSM_Process(GSM_RX_BUFF.gsm_content, GSM_RX_BUFF.gsm_wr); 
		}
	}

}
//------------------------  ASCII    HEX  convert   ---------------------
u8  HexValue (u8 inchar) 
{
     switch(inchar)
     	{     	    
		   case '0':
                       return  0;
		   case '1': 
		   	           return  1;
		   case '2':
		   	           return  2;
	         case '3':
		   	           return  3;
		   case '4':
		   	           return  4;
		   case '5':    
		   	           return 5;
		   case '6':   
		   	           return 6;
		   case '7':
		   	            return 7;
		   case '8':     
		   	            return 8;
		   case '9':   
		   	           return 9;
		   case 'A':
		   	            return 0x0A;
		   case 'B':   

		   	            return 0x0B;
		   case 'C':
		   	            return 0x0C;
		   case 'D':    
		   	             return 0x0D;
		   case 'E': 
		   	            return 0x0E;
		   case 'F':    
		   	             return 0x0F;
		   default :
		   	          // rt_kprintf("\r\n 转义有错误:%c \r\n",inchar);    
				    return  0xFF;
     	}
}
//------------------------------- HEX to ASCII --------------------------------------
u32 GSM_HextoAscii_Convert(u8*SourceHex,u16 SouceHexlen,u8 *Dest)   
{
  u16 len_counter=0;
  u16 dest_counter=0;  
  u8  c=0;

  for(len_counter=0;len_counter<SouceHexlen;len_counter++)   
  {
     c=SourceHex[len_counter];

	//---------------- High -------------------------------- 
    if((c>>4)>=10)
       Dest[dest_counter++]='A'+(c>>4)-10;
	else
	   Dest[dest_counter++]='0'+(c>>4);
	
    Dest[dest_counter]=0x00;

	//----------------- Low --------------------------------
	c=c&0x0F;
    if(c>=10)  
       Dest[dest_counter++]='A'+c-10;
	else 
	   Dest[dest_counter++]='0'+c; 
	Dest[dest_counter]=0x00;
  }
  
 return dest_counter; 
}
 u16  GSM_AsciitoHEX_Convert(u8 *Src_str,u16 Src_infolen,u8* Out_Str) 
 {
   u16 Counter=0,Out_Str_Len=0;
   u8  C=0;
	 
   Out_Str_Len=0;

     if((u8)Src_infolen&0x01) 
   	{
   	     rt_kprintf("\r\n      接收ASCII信息不正确!   %u  \r\n",Src_infolen);     
   	}
   	
   for(Counter=0;Counter<Src_infolen;Counter++) 
   {   
        //--------------------------------------------------
           if((Counter&0x01)==1)
           	{
	           	 C=HexValue(*(Src_str+Counter));
			 if(C!=0xff)		 
	              {  
	                 Out_Str[Out_Str_Len]+=C;
	                 Out_Str_Len++; 
			  }	
			  else 
			  	  rt_kprintf("\r\n    Convert Error at:   %u  \r\n",Counter);    
           	}
	   else
		{
		     C=((HexValue(*(Src_str+Counter)))<<4);
                      Out_Str[Out_Str_Len]=(HexValue(*(Src_str+Counter)))<<4;	
		      if(C!=0xff)		 
	               {  
	                 Out_Str[Out_Str_Len]=C;
			  }	
			  else 
			  	  rt_kprintf("\r\n    Convert Error at2:   %u  \r\n",Counter);    			  
	   	}

   } 
	return Out_Str_Len;

 } 



void  Data_Send(u8* DataStr, u16  Datalen,u8  Link_Num)
{
    u16  i=0,packet_len=0;
	u8   AT_send_str[20];  
     //  4. Send  


    //  4.1  发送要发送的信息长度  
     //  4.1  发送要发送的信息长度  
	  memset(AT_send_str,0,sizeof(AT_send_str));
	 
	 	if(Link_Num==0)
	    {
	       sprintf((char*)AT_send_str,(char*)Send_TCP, GPRS_infoWr_Tx);
		   Mainlink_send_over=1; // 主链路发送信息
	 	}
	  else
	  	{           
	       sprintf((char*)AT_send_str,(char*)Send_ISP, GPRS_infoWr_Tx); 
	  	} 
	  
	  rt_kprintf(AT_send_str);
	  rt_hw_gsm_output(AT_send_str);  
	  delay_ms(180);// 给模块返回输出时间  

      if(DispContent==2)
	    OutPrint_HEX("GsmSend",DataStr,GPRS_infoWr_Tx);   
	 
	 rt_hw_gsm_output_Data(DataStr,GPRS_infoWr_Tx);	 
	// rt_hw_gsm_putc (0x1A);  
	 WatchDog_Feed();
	 rt_thread_delay(RT_TICK_PER_SECOND/10);//DF_delay_ms(100);     

}

void End_Datalink(void)
{
       if(1==DataLink_EndFlag)          
	{
	   TCP2_Connect=0;
	   DataLink_Online=disable;  // off line

	  DataLink_EndCounter++;
	  if(DataLink_EndCounter==2) 
	  { 	rt_kprintf(" Ready to escape gprs \r\n");	 
		rt_hw_gsm_output(CutDataLnk_str2);
		rt_kprintf(CutDataLnk_str2);
		DataDial.Dial_step_Retry=0;
		DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
	  }
	  else
	  if(DataLink_EndCounter==3)
	  { 		
		rt_hw_gsm_output(CutDataLnk_str3);	// 关闭掉Internet
	       rt_kprintf(CutDataLnk_str3);	// 关闭掉Internet
	       DataDial.Dial_step_Retry=0;
		DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;

	  }    
	  else
	  if(DataLink_EndCounter==5)
	  { 		
		rt_hw_gsm_output(CutDataLnk_str4);	 
		rt_kprintf(CutDataLnk_str4);	 
		DataDial.Dial_step_Retry=0;
		DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
	  }
	    if(DataLink_EndCounter==8)
	  { 		
		rt_hw_gsm_output(CutDataLnk_str5);	 
		rt_kprintf(CutDataLnk_str5);   
		DataDial.Dial_step_Retry=0;
		DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
	  }
	  else
	  if(DataLink_EndCounter>10)   
	  {
		DataLink_EndCounter=0; 
		DataLink_EndFlag=0;
		//-----------------------------
	     if(JT808Conf_struct.Close_CommunicateFlag==0)
	     	{
	               CommAT.Initial_step=0;  
			 CommAT.Total_initial=0;  
	               Redial_Init(); 
			 ModuleStatus &=~Status_GPRS;
		        rt_kprintf(" Redial Start\r\n");   
	     	}
		else
			rt_kprintf("\r\n 关闭通信状态 ，不重播\r\n");   
		      //-----------------------------
	  }
    }     

}

void  ISP_Timer(void)
{
            //--------- Timer  ------------- 
   if((TCP2_ready_dial==1)&&DataLink_Status())     
       {
                 Ready_dial_counter2++;  
		   if(Ready_dial_counter2>5)  
                	{
                         Ready_dial_counter2=0;
			    //DataDial.Dial_step=Dial_ISP;    
			    DataDial.Dial_ON=enable;  //
				
                         Dial_Stage(Dial_ISP);
		       }  
       } 

                  //---------   ISP 心跳包发送 - ----
      /*        if(TCP2_Connect==1)
             {
                   TCP2_Coutner++;
                    if(TCP2_Coutner>=TCP2_sdDuration)
                    	{
                           TCP2_Coutner=0; 
			      TCP2_IC_sdFlag=1;     
                    	}

             }	
      */

}
 
u8  GPRS_GSM_PowerON(void)
{	
	/*
              EM310开关机流程
              powerkey 拉低 50ms 开机  然后再拉低50ms关机
	*/
      if(GSM_PWR.GSM_power_over) 
			  return 1;
	
         GSM_PWR.GSM_powerCounter+=50;   
		 #if   0//M66_GSM
	
               	if((GSM_PWR.GSM_powerCounter>=10)&&(GSM_PWR.GSM_powerCounter<300))
			 {
			    GPIO_ResetBits(GPIOD,GPRS_GSM_Power);    // 关电
			    GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);      //  PWK 低 
			    rt_kprintf(" step 1\r\n");   
			 }	
			 if((GSM_PWR.GSM_powerCounter>=300)&&(GSM_PWR.GSM_powerCounter<400))
			 {
			        GPIO_SetBits(GPIOD,GPRS_GSM_Power);    //  开电
			        GPIO_SetBits(GPIOD,GPRS_GSM_PWKEY);   //  PWK低
				  rt_kprintf(" step 2\r\n");   
				  gps_onoff(1);  // Gps module Power on   GPS 模块开电   
			 }	
			 if((GSM_PWR.GSM_powerCounter>=400)&&(GSM_PWR.GSM_powerCounter<700)) 
			 {
			   GPIO_SetBits(GPIOD,GPRS_GSM_Power);    //  开电
			   GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);    //  PWK 高
			    rt_kprintf(" step 3\r\n");   
			 }	
			 if((GSM_PWR.GSM_powerCounter>=700)&&(GSM_PWR.GSM_powerCounter<900))   
			 {
				GPIO_SetBits(GPIOD,GPRS_GSM_PWKEY);   //  PWK低
				 rt_kprintf(" step 4\r\n");    
			 }	 
			 if((GSM_PWR.GSM_powerCounter>=900)&&(GSM_PWR.GSM_powerCounter<1000))      
			 {
			    GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);      //  PWK 高   
				 rt_kprintf(" step 5\r\n");    
			 }	
			 if(GSM_PWR.GSM_powerCounter>=1000)        
			 {
				rt_kprintf("         %s","--GPRS Power over\r\n ");     
				GSM_PWR.GSM_PowerEnable=0; 
				GSM_PWR.GSM_powerCounter=0;
				GSM_PWR.GSM_power_over=1;  
				   //-------add for re g 
			 }	   
        #endif
		 
		#ifdef M50_GSM

		     if((GSM_PWR.GSM_powerCounter>=10)&&(GSM_PWR.GSM_powerCounter<300))
			 {
			    GPIO_ResetBits(GPIOD,GPRS_GSM_Power);    // 关电
			    GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);      //  PWK 低 
			    rt_kprintf(" step 1\r\n");   
			 }	
			 if((GSM_PWR.GSM_powerCounter>=300)&&(GSM_PWR.GSM_powerCounter<400))
			 {
			        GPIO_SetBits(GPIOD,GPRS_GSM_Power);    //  开电
			        GPIO_SetBits(GPIOD,GPRS_GSM_PWKEY);   //  PWK低
				  rt_kprintf(" step 2\r\n");   
				  gps_onoff(1);  // Gps module Power on   GPS 模块开电   
			 }	
			 if((GSM_PWR.GSM_powerCounter>=400)&&(GSM_PWR.GSM_powerCounter<700)) 
			 {
			   GPIO_SetBits(GPIOD,GPRS_GSM_Power);    //  开电
			   GPIO_SetBits(GPIOD,GPRS_GSM_PWKEY);   //  PWK低
			    rt_kprintf(" step 3\r\n");   
			 }			 
			 if((GSM_PWR.GSM_powerCounter>=700)&&(GSM_PWR.GSM_powerCounter<800))      
			 {
			    GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);      //  PWK 高   
				 rt_kprintf(" step 4\r\n");    
			 }	
			 if(GSM_PWR.GSM_powerCounter>=900)         
			 {
				rt_kprintf("         %s","--GPRS Power over\r\n ");     
				GSM_PWR.GSM_PowerEnable=0; 
				GSM_PWR.GSM_powerCounter=0;
				GSM_PWR.GSM_power_over=1;  
				   //-------add for re g 
			 }
		
		  
		 
		#endif
		
	 return   0;	    
}

void GSM_Module_TotalInitial(void)
{
    
    //----------  Total_initial ------------
   if(CommAT.Total_initial==1)
   {
	 if( CommAT.Execute_enable)
	 {
	   switch(CommAT.Initial_step)
		   {
			 case 0:
					 rt_hw_gsm_output(CommAT_Str1);  
					 if(DispContent)
					 rt_kprintf(CommAT_Str1);  
					 CommAT.Initial_step++;
					 break;
			 case 1:
					 rt_hw_gsm_output(CommAT_Str2);
					 if(DispContent)
					 rt_kprintf(CommAT_Str2); 
					 CommAT.Initial_step++;
					 break;
   
			 case 2:/* Query Operator */
					 rt_hw_gsm_output(CommAT_Str3);  //"AT+COPS?\r\n"
					 if(DispContent)
					 rt_kprintf(CommAT_Str3);  
					 CommAT.Initial_step++;     
					 break;
			 case 3:
					 rt_hw_gsm_output(CommAT_Str4);  
					 if(DispContent)
					 rt_kprintf(CommAT_Str4);	
					 CommAT.Initial_step++;
					 break;
			 case 4:
					 rt_hw_gsm_output(CommAT_Str5);
					 if(DispContent)
					 rt_kprintf(CommAT_Str5);   
					 CommAT.Initial_step++;
					 break;
			 case 5:
					 rt_hw_gsm_output(CommAT_Str6);    
					 if(DispContent)
					 rt_kprintf(CommAT_Str6); 
					 CommAT.Initial_step++; 
					 break;
			 case 6:
					 rt_hw_gsm_output(CommAT_Str7);  
					 if(DispContent)
					 rt_kprintf(CommAT_Str7); 
					 CommAT.Initial_step++;
					 break;
			 case 7:	
					 rt_hw_gsm_output(CommAT_Str11); 
					 if(DispContent)
					 rt_kprintf(CommAT_Str11);  
					 CommAT.Initial_step++;
					 break;		
			 case 8:
					 rt_hw_gsm_output(CommAT_Str9);  
					 rt_kprintf(CommAT_Str9); 
					 CommAT.Initial_step++; 
					 break;	 		 
			 case 9:
					 rt_hw_gsm_output(CommAT_Str10);   
					 if(DispContent)
					 rt_kprintf(CommAT_Str10); 
			 
					 CommAT.Initial_step++;
					 break; 	
			 case 10:
					 rt_hw_gsm_output(CommAT_Str12);    //  配置语音录音参数
					 if(DispContent)
					 rt_kprintf(CommAT_Str12);   
					 CommAT.Initial_step++;
					 break;		 
             case 11:
					 rt_hw_gsm_output(CommAT_Str12);  
					 if(DispContent)
					 rt_kprintf(CommAT_Str12);
					 CommAT.Initial_step++;
					 break; 
			 case 12:
					 rt_hw_gsm_output(CommAT_Str13);
					 if(DispContent)
					 rt_kprintf(CommAT_Str13); 
					 CommAT.Initial_step++;
					 break; 		 
			 case 13:
					 rt_hw_gsm_output(CommAT_Str14);
					 if(DispContent)
					 rt_kprintf(CommAT_Str14);  
					 CommAT.Initial_step++;
					 break; 	
			 case 14:
					 rt_hw_gsm_output(CommAT_Str15); 
					 rt_kprintf(CommAT_Str15);   
					 CommAT.Initial_step++;
					 break; 	
			 case 15:
			                rt_hw_gsm_output(Signal_Intensity_str); 
					 rt_kprintf(Signal_Intensity_str);    
					 break; 			 
			 case 16://  信号强度 /		
			                rt_hw_gsm_output(Signal_Intensity_str); 
					 rt_kprintf(Signal_Intensity_str);    
					 break;
	               case 17:/*开始能拨号*/ 
				     if(JT808Conf_struct.Close_CommunicateFlag==0)
	                         {	
					 rt_kprintf("AT_Start\r\n");   
					 CommAT.Initial_step=0; 
					 CommAT.Total_initial=0;   

					 DataDial.Dial_ON=enable;  //  进入  Data   状态
					 DataDial.Pre_Dial_flag=1;    // Convert to  DataDial State
					 Dial_Stage(Dial_DialInit0);     	
				     	}
				    //  else
					 //   	 rt_kprintf("\r\n 关闭通信状态不dial\r\n");    
					 break; 				 
			 default:					  
					break; 
	 
		   }  
	    CommAT.Execute_enable=0;
	 }
   } 

}

void Redial_Init(void)
{
        //----------- Data Dial ----------------
   memset((u8*)&DataDial,0,sizeof(DataDial)); 
   DataDial.start_dial_stateFLAG=1; //important 
   DataDial.Dial_ON=enable;  //  进入  Data   状态
   DataDial.Pre_Dial_flag=1;    // Convert to  DataDial State
   Dial_Stage(Dial_DialInit0);
}

void  Dial_step_Single_10ms_timer(void)  
{
      //    没在数据状态下，和 成功登陆上后不进行处理
     	if( (DataDial.Dial_ON==0)||(DataDial.Dial_step==Dial_Idle) )
		   return;				
    
		if (DataDial.Dial_step_RetryTimer>= 10)
			DataDial.Dial_step_RetryTimer -= 10;
		else
			DataDial.Dial_step_RetryTimer = 0;
}

void  Get_GSM_HexData(u8*  Src_str,u16 Src_infolen,u8 link_num)
{
     u16   i=0;
     //  1.  Check wether   Instr   is  need   to  convert     Exam:  ASCII_2_HEX	 
                GSM_HEX_len= GSM_AsciitoHEX_Convert(Src_str,Src_infolen,GSM_HEX); 
     //  2 .  Realse   sem
		App_rxGsmData_SemRelease(GSM_HEX, GSM_HEX_len,link_num); 
	      //  for(i=0;i<GSM_HEX_len;i++)
		//		rt_kprintf("%c",GSM_HEX[i]);
} 

void DataLink_Process(void)
{
   u8  len=0,i=0; 
   
  	// state  filter  1:   没在数据状态下，和 成功登陆上后不进行处理
     	if( (DataDial.Dial_ON==0)||(DataDial.Dial_step==Dial_Idle) ) 
		    return;				
	// state  filter  2:  	单步延时还没有到
       if (DataDial.Dial_step_RetryTimer ) 
	   	    return;						   // not time to retry
	// state  filter  3:      	    
        if (DataDial.Dial_step_Retry>= Dial_Step_MAXRetries)		
        {														  
			   //---------------------------------------			  
			   DataDial.Connect_counter++;
			   if( DataDial.Connect_counter>4)     
				   {
					   DataDial.Pre_Dial_flag=1; 	  //--- 重新拨号	
					   DataDial.Pre_dial_counter=0;
					   rt_kprintf("\r\n  RetryDialcounter>=4 重新拨号\r\n");  
				   }	 
			   Dial_Stage(Dial_DialInit0);    // Clear   from  Dial  start 	
			   DataDial.Dial_step_Retry=0;
			   DataDial.Dial_step_RetryTimer=0; 
			   rt_kprintf("\r\nDataDial.Dial_step_Retry>= Dial_Step_MAXRetries ,redial \r\n");
			   return;													   
	  } 
	  //----------  主连接重试2次 失败切换到辅连接	
        if((DataDial.Dial_step==Dial_MainLnk)&&(DataDial.Dial_step_Retry>2))
         {
              i=0;
              if(Dnsr_state)
              {
                   Dial_Stage(Dial_MainLnk);    // 如果是DNSR 连接那么换到mainlink
              }
		else
		{
                    // rt_thread_delay(10);
			rt_kprintf("Dial_MainLnk Retry>2 AT%IPCLOSE=1\r\n");		   
		       rt_hw_gsm_output("AT%IPCLOSE=1\r\n");    	
		       WatchDog_Feed();   
			delay_ms(100);//rt_thread_delay(10);
                     Dial_Stage(Dial_AuxLnk);    
			DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_main,0);	 
		 
		}			 
         }
		
		

	 //  work  on 
       if(DataDial.Dial_ON)
       {
             switch(DataDial.Dial_step)
             	{
                   case Dial_DialInit0: 
				   	 rt_hw_gsm_output(DialInit1);
			               //-----------------------------------------
                                      DataDial.start_dial_stateFLAG=1;   
				         //-----------------------------------------
				         DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
					  DataDial.Dial_step_Retry++;
					  //  Debug
 					  rt_kprintf(DialInit1);   
           
					break;
		      case Dial_DialInit1:    rt_hw_gsm_output((const char*)Dialinit_APN);
                                   DataDial.Dial_step_RetryTimer=Dial_max_Timeout; 
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)Dialinit_APN);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",Dialinit_APN[i]);     

				       break;			
		     case Dial_DialInit2:     rt_hw_gsm_output(DialInit2);
                                   DataDial.Dial_step_RetryTimer=Dial_Timeout;
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit2);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit2[i]);   

				       break;
		     case  Dial_DialInit3:  
			 	       rt_hw_gsm_output(DialInit3);
                                   DataDial.Dial_step_RetryTimer=Dial_max_Timeout; 
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit3);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit3[i]);      

				       break;			   
		     case Dial_DialInit4:    rt_hw_gsm_output(DialInit6); // AT
                                   DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit6);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit6[i]);  

					break;
		     
		    case Dial_DialInit5:    rt_hw_gsm_output(DialInit4);
                                   DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time; 
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit4);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit4[i]);       

				       break;	
		    case Dial_DialInit6:    rt_hw_gsm_output(DialInit8);  // Query  IP 
                                   DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time; 
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit8);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit8[i]);     

				       break;	 	
		   case Dial_DNSR1:    rt_hw_gsm_output(Dialinit_DNSR);  // main DNSR
                                   DataDial.Dial_step_RetryTimer=Dial_max_Timeout; 
					DataDial.Dial_step_Retry++;
					rt_kprintf("\r\n   ----- Link  main  DNSR ----\r\n");
					 len=strlen((const char*)Dialinit_DNSR);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",Dialinit_DNSR[i]);     
                                   Dial_jump_State=7;
					Dnsr_state=1;//表示DNSR 状态下拨号 				   
				       break;	 
                 case Dial_DNSR2:    rt_hw_gsm_output(Dialinit_DNSR2);  // Aux DNSR
                                   DataDial.Dial_step_RetryTimer=Dial_max_Timeout; 
					DataDial.Dial_step_Retry++;
					rt_kprintf("\r\n   ----- Link  Aux  DNSR ----\r\n");
					 len=strlen((const char*)Dialinit_DNSR2);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",Dialinit_DNSR2[i]);       
                                   Dial_jump_State=8; 
					Dnsr_state=1;//表示DNSR 状态下拨号 				   
				       break;	 					   
		    case Dial_DialInit7:    rt_hw_gsm_output(DialInit8);  // Query  IP 
                                   DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time; 
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialInit8);
					for(i=0;i<len;i++)										
					         rt_kprintf("%c",DialInit8[i]);     

				       break;	   
		     case Dial_MainLnk:   // rt_hw_gsm_output(DialStr_Link1); 
		                       if( Dnsr_state==0)//表示DNSR 状态下拨号 	 
		                       	{
		                       	   rt_kprintf("\r\n Dial  orginal   Mainlink\r\n");  
                                            DataLink_MainSocket_set(RemoteIP_main,RemotePort_main,1);
		                       	}
							   
                                   DataDial.Dial_step_RetryTimer=1500; 
					DataDial.Dial_step_Retry++;
					
					if((RemoteIP_main[0]==192)&&(RemoteIP_main[1]==168))
						{  //  如果是内网IP ，则连接辅IP 
						      Dial_Stage(Dial_AuxLnk);    
			                            DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_main,0);	 
                                              
						}
					 len=strlen((const char*)DialStr_Link1); 
					 for(i=0;i<len;i++)										
					   {
					        rt_hw_gsm_putc (DialStr_Link1[i]);     
					   }	
					for(i=0;i<len;i++)										
					   {
					        rt_kprintf("%c",DialStr_Link1[i]);    
					   }		
					break; 
		      case Dial_AuxLnk:   // rt_hw_gsm_output(DialStr_Link2); 
                                   DataDial.Dial_step_RetryTimer=800;
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialStr_LinkAux); 
					 for(i=0;i<len;i++)										
					   {
					        rt_hw_gsm_putc (DialStr_LinkAux[i]);      
					   }	
					for(i=0;i<len;i++)										
					   {
					        rt_kprintf("%c",DialStr_LinkAux[i]);     
					   }		
					break; 	
		     case   Dial_ISP:
				      DataDial.Dial_step_RetryTimer=Dial_max_Timeout;
					DataDial.Dial_step_Retry++;
					 len=strlen((const char*)DialStr_IC_card); 
					 for(i=0;i<len;i++)										
					   {
					        rt_hw_gsm_putc (DialStr_IC_card[i]);     
					   }	
				      rt_kprintf("\r\n   ----- Link  IC ----\r\n");       
					for(i=0;i<len;i++)										
					   {
					        rt_kprintf("%c",DialStr_IC_card[i]);     
					   }	
					break;				
		      default:
			  	       break;
             	}

       }  



}

static void GSM_Process(u8 *instr, u16 len)
{
      	u8	ok = false;
		u8	error = false;
		u8	failed = false; 
		u8	connect = false,connect_2 = false;   		
        u16  i=0;//,len=0;//j=0;
       //  u8 reg_str[80]; 
//----------------------  Debug -------------------------		
  // if(DispContent==2)	 
   memset(GSM_rx,0,sizeof((const char*)GSM_rx));
   memcpy(GSM_rx,instr,len);
  // if((ISP_running_state==0)&&(VocREC.Sate!=VOICEREC_DataRXing)) 
 // if(DispContent!=0) 
   {
      // rt_kprintf("\r\n");      
        for(i=0;i<len;i++)  
		 rt_kprintf("%c",GSM_rx[i]);     	
   }

   //------------------------------------------------------------------------------------------------------------------- 
     /*if (strncmp((char*)GSM_rx, "AT-Command Interpreter ready",20) == 0)  
	{	
		GSM_PWR.GSM_PowerEnable=0;      
		GSM_PWR.GSM_powerCounter=0;
		GSM_PWR.GSM_power_over=1;   
		rt_kprintf("\r\n  GSM 模块启动\r\n");         			
	}*/

      //----------------判断短信状态------------------------	
   /*      if((SMS_send.Msg_sdState==1)&&(SMS_send.Msg_step==2))
         	{
                   if(GSM_rx[0]=='>')
                   	{  
                   	      SMS_send.Msg_step=3;
			      memset(GSM_rx, 0, sizeof(GSM_rx));
	                    GSM_rx_Wr = 0; 			  
			      return ;		  
                   	}
	}*/

    #ifdef  M50_GSM
     if(Voice_data_come==1)	 // 录音数据
	   {		
				memcpy(VocREC.HEX, GSM_rx,Voice_rx_len); 
				VocREC.HEX_len=Voice_rx_len; 
				 VocREC.Sate=VOICEREC_DataRXing;  
				 VocREC.ExcuteFlag=1;   
				 Voice_data_come=0; 
	   }  
	 else
	 if(strncmp(GSM_rx, "CONNECT ", 8 )== 0 ) //CONNECT 1024  
	 {   //  录音相关
              i = sscanf(GSM_rx, "CONNECT %d", &Voice_rx_len);
			 if( i != 1 )
			 {
				 Voice_rx_len= 0;
			 }
			 rt_kprintf("\r\n  Voice rx infolen=%d\r\n",Voice_rx_len); 
			 Voice_data_come=1; 
	  }
	else
    if(strncmp((char*)GSM_rx, "+QTTS:0",7)==0)
	{
                 TTS_Play_End();
		    rt_kprintf("\r\n   TTS  播放完毕\r\n");    		 
	}
	else
	if(strncmp((char*)GSM_rx, "SEND OK",7)==0)	 // 链路 1  TCP 发送OK    
	{												  
	   if(Mainlink_send_over)
          {
              Mainlink_send_over=0;     
	   	  }
	} 
	else
     //   voice  file  size  get
    //+QFLST: "voice.amr",4192  VOICEREC_GET_FILESIZE
    //4806 gsm<+QFLST: "test1.amr",5172
    if(strncmp((char*)GSM_rx, "+QFLST: \"voice.amr\",",20)==0)
    {         
		 i = sscanf(GSM_rx, "+QFLST: \"voice.amr\",%d", &VocREC.filesize);

		 if(i)
		 	{
		 	  rt_kprintf("\r\n VoiceFile Size: %d \r\n",VocREC.filesize); 
			  VocREC.file_write=0;// clear 
		
			  VocREC.Sate=VOICEREC_FileOpen;  
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
		 	}
    }
    else
    if(strncmp((char*)GSM_rx, "+QFOPEN:",8)==0)
    {   
	      //+QFOPEN: 13500416    +QFOPEN: 12976128
          //  get  handle name
          memset(VocREC.handle_name,0, sizeof(VocREC.handle_name)); 
          memcpy(VocREC.handle_name,GSM_rx+9,len-2-9); 
		  rt_kprintf("\r\n --%s--\r\n",VocREC.handle_name);   
		  VocREC.Sate=VOICEREC_Sd_getcmd; 	
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
		  
    }
	#endif
	else
       if(strncmp((char*)GSM_rx, "%IPCLOSE: 2",11) == 0)// ISP close
	 {
		
		rt_kprintf("\r\n");
		rt_kprintf("%s",GSM_rx); 
		rt_kprintf("\r\n");
              #ifdef MULTI_LINK
		     TCP2_Connect=0;										   
		     TCP2_ready_dial=0;  
		     TCP2_login=0;  

              
		     if(DataLink_Status())
			 	  TCP2_ready_dial=1;         
		 #endif	 
        } 
	else
	if(strncmp((char*)GSM_rx, "IPCLOSE",5) == 0)    
	{
		 DataLink_Online=0;
		 ModuleStatus &=~Status_GPRS;
		 rt_kprintf("\r\n所有链接都关闭!\r\n");  
		 DataDial.Pre_Dial_flag=1;
		 //------  交互导航链接状态 ----------
		// ZHinfotype=ZH_01H;
		// subtype=0x2B;	  //  通知显示屏断开   
		 
	}
	else
	if(strncmp((char*)GSM_rx, "1, CLOSED",9) == 0)	   
	{
	   DataLink_Online=0;
		 ModuleStatus &=~Status_GPRS; 
		 rt_kprintf("\r\nTCP连接关闭!\r\n");	 
		 if(CallState==CallState_Idle)		 //  不要再通话的时候操作 
		 {	
		    rt_hw_gsm_output("ATH\r\n"); 		
			DataLink_EndFlag=1; 
		    rt_kprintf("\r\n Datalink end =>收到:%IPCLOSE\r\n"); 	
			//注册不成功连接断开后不鉴权还注册。
			//DEV_Login.Operate_enable=1;//重新鉴权     
		 }
		 
	}	
      if(strncmp((char*)GSM_rx, "%DNSR:",6) == 0)
	{  //%DNSR:113.31.28.100
		//%DNSR:0.0.0.0 
		if(strncmp((char*)GSM_rx, "%DNSR:0.0.0.0",13) == 0)
		  {
		        rt_kprintf("\r\n   域名解析失败\r\n");    
			 if((Dial_jump_State==7)||(Dial_jump_State==8))
				Dial_jump_State=0;    
		        Dnsr_state=0;//表示DNSR 状态下拨号 			  

		  }	  
	        else
	        { 
	                     
	                    
				i =str2ip((char*)GSM_rx+6, RemoteIP_Dnsr);
				if (i <= 3) failed = true; 
			        //memcpy((char*)SysConf_struct.IP_Main,RemoteIP_main,4);
				 // SysConf_struct.Port_main=RemotePort_main;
				 //Api_Config_write(config,ID_CONF_SYS,(u8*)&SysConf_struct,sizeof(SysConf_struct));
			        DataLink_MainSocket_set(RemoteIP_Dnsr,RemotePort_main,1);
                             rt_kprintf("\r\n   域名解析成功\r\n"); 

				  Dial_Stage(Dial_MainLnk);  //--   切换到拨号    
				  return ;
		}
      	}
	else
	if(strncmp((char*)GSM_rx, "+COPS: 0,0,",11)==0)
	{ 
			//联通卡	   +COPS: 0,0,"CHN-CUGSM"
			//移动卡	   +COPS: 0,0,"CHINA  MOBILE"
			//注销的卡	  +COPS: 0
	  	if(strcmp((char*)GSM_rx, "+COPS: 0,0,\"CHN-CUGSM\"")==0)  
			{
				//联通的
				//memset((char *)Dialinit_APN,0,sizeof(Dialinit_APN));
				//strcat((char*)Dialinit_APN, "AT+CGDCONT=1,\"IP\",\"UNINET\"\r\n");  
			}
		  else
		  if(strcmp((char*)GSM_rx, "+COPS: 0,0,\"CHINA  MOBILE\"")==0)	
			{
			   // 移动的
				//memset(Dialinit_APN,0,sizeof(Dialinit_APN));
				//strcat(Dialinit_APN, "AT+CGDCONT=1,\"IP\",\"CMNET\"\r\n");
			} 
	  
		   CommAT.Initial_step++; 
		 //-------------------------------------------------------------------------   
		   
			
	}  
	else
	if(strncmp((char*)GSM_rx, "+COPS: 0",8)==0)  //----该卡被注销了
	{
		COPS_Couter++;
		if(COPS_Couter>=3) 
		{
			COPS_Couter=0;
			CommAT.Initial_step++;   
			rt_kprintf("\r\n  SIM卡搜索不到网络\r\n"); 
		}
	}
	else
	if(strncmp((char*)GSM_rx, "+CME ERROR: 4010",16)==0)	
		{
            if(AT_waitOK)
		                             {
									    VocREC.Sate=VOICEREC_RECORD_START;  
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
									  }
		}
	else
	if(strncmp((char*)GSM_rx, "OK",2) == 0)  
		{				
		    ok = true;
		 // if(DispContent)	
			   rt_kprintf(" OK\r\n");     			
               			 //------------------------------------  
		   if(Send_Rdy4ok==1)
		   	{
		         //  rt_kprintf("\r\n Updata cycle  status !! ----reg for debug\r\n");
			    Send_Rdy4ok=0;	   
		   	}
		    //-------------------------------------------
		    if(DataLink_Status())        //   Online  state  Error    , End Link and Redial     
	              Rx_Error_counter=0;  
			//-------------------------------------------
               switch(VocREC.Sate)
      {
		   case	VOICEREC_DeleteOld:   // 1.  删除old  文件
		                             if(AT_waitOK)
		                             {
									    VocREC.Sate=VOICEREC_RECORD_START;  
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
									  }
								 break; 				
           case VOICEREC_RECORD_START:   //  2.  开始录音
                           
		   	                         if(AT_waitOK)
		                             {
									     VocREC.Sate=VOICEREC_RECORDING;  
										 VocREC.ExcuteFlag=0;  
		                                AT_waitOK=0; 
									  }        			   
								  break;
			case  VOICEREC_REC_STOP:    // 3.  停止录音				       
									  if(AT_waitOK)
		                              {
									    VocREC.Sate=VOICEREC_GET_FILESIZE;  
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
									  }
									   break;
		    case  VOICEREC_GET_FILESIZE: //  4.  获取已录制文件的 大小
				             	     if(AT_waitOK)
		                              {
									    VocREC.Sate=VOICEREC_FileOpen;  
									    VocREC.ExcuteFlag=1;  
		                                AT_waitOK=0; 
									  }
	   	                               break;
			 case  VOICEREC_FileOpen:  //   5.   打开文件 获取句柄
				              		   if(AT_waitOK)
		                              {
		                                 VocREC.ExcuteFlag=0; 
		                                AT_waitOK=0; 
									  }
									    break;
			 case  VOICEREC_DataRXing:
			 	                        Get_voicedata_delay=1;

										
	   	                               break;	 	   
      	}  



			//------------------------------------------ 
        }
	else
	if (strcmp((char*)GSM_rx, "NO DIALTONE") == 0) failed = true;
	else
	if((strcmp((char*)GSM_rx, "NO ANSWER") == 0)|| \
	   (strncmp((char*)GSM_rx, "NO CARRIER",10) == 0)) //NO CARRIER  
	{                      //  NO CARRIER  
		 CallState=CallState_Idle;	
		 failed = true;
		 rt_kprintf("\r\n Callstate=Idle\r\n");    
	}   
	if(strcmp((char*)GSM_rx, "BUSY") == 0)  
	{ 
		// ZHinfotype=ZH_01H;//  车台向终端发送振铃指示
		 //subtype=0x12;	   //挂断
		 CallState=CallState_Idle;	
		 failed = true; 
		// Off_speaker;		// 关闭功放 
		 //Clip_flag=0;
		 //Clip_counter=0; 
	
	}
	else
	if(strncmp((char*)GSM_rx,"+CSQ:",5)==0)	//场强信息
	{ //+CSQ: 27,99
				if ( GSM_rx[7] == ',' )
				{
						ModuleSQ = GSM_rx[6] - 0x30;   
				}
				else if ( GSM_rx[8] == ',' )
				{
						ModuleSQ = ( GSM_rx[6] - 0x30 ) * 10 + GSM_rx[7] - 0x30;
	
						if ( ModuleSQ == 99 )
						{
								ModuleSQ = 0;
						}
				}
	          //-------------------------------------- 
	 	      //  You must   register on 
	 		  if(CommAT.Total_initial==1)
	 			{
	                if(ModuleSQ>8)
	 				{
	 				  CommAT.Initial_step++;
					  rt_kprintf("\r\n CSQ check Pass\r\n");     
	                }
	 			} 				 
		
	}

	if (strncmp((char*)GSM_rx, "89860",5) == 0)	 //-----  %TSIM 1 %TSIM 1
	{
	 
			IMSIGet.Get_state=1;
			rt_kprintf("\r\n  检测到SIM卡\r\n"); 
	}	  
	else
	if (strncmp((char*)GSM_rx, "460",3) == 0)	//----- Nathan Add
	{
	  
	  CommAT.Total_initial=1;	// 进行参数配置
	  //---  add for Test  ----- 
	// memcpy(GSM_rx,"460021022612645",15);     // 河北平台车123         
	 //   memcpy(GSM_rx,"460013602069192",15);     // 河北平台车123   
	// memcpy(GSM_rx,"460012212469640",15);     // 天津运营平台渝000000  
	/*  memcpy(GSM_rx,"460018820130001",15);     // 认证用       
	  rt_kprintf("\r\n获取IMSI 号码:%s \r\n",GSM_rx);       
	  memcpy((char*)IMSI_CODE,(char*)GSM_rx,15);
	  IMSI_Convert_SIMCODE(); //  translate */
	  RTC_TimeShow();
	  IMSIGet.imsi_error_count=0;	 
	  IMSIGet.Get_state=1;
	  GSM_PWR.GSM_power_over=2;     //  get imsi
	  IMSIGet.Get_state=0;
	  CommAT.Total_initial=1;	// 进行参数配置  	  	 

	   //  获取终端属性，终端ID 字段
	   ProductAttribute._4_Dev_ID[0]=0x00;
	   memcpy(ProductAttribute._4_Dev_ID+1,SIM_code,6);  // SIM_code  填充后6 位
	}
	if (strncmp((char*)GSM_rx, "+CCID: \"",8) == 0)
       {  //+CCID: "89860109520220884603"  
           //  获取终端属性  ，ICCID 字段
           ok=0;// 借用做下标
           for(i=0;i<10;i++)
           {
              ProductAttribute._5_Sim_ICCID[ok]=((GSM_rx[8+2*i]-0x30)<<4)+(GSM_rx[9+2*i]-0x30);
              ok++;
           }		  
	}
	else
	if(strncmp((char*)GSM_rx,"RING",4)==0)  //电话  
	{
         ;	 
	}  
	else
	if ((strlen((char*)GSM_rx) >= 7) && (strncmp((char*)GSM_rx, "CONNECT", 7) == 0))
	{
	      connect=true;
	}
                 //1, CONNECT OK
    if ((strncmp((char*)GSM_rx, "1, CONNECT OK", 13) == 0))
	{
	      connect=true;
	}
	else
	if ((strncmp((char*)GSM_rx, "2, CONNECT OK", 13) == 0))
	{
	      connect_2=true;
	}	
	else  //ERROR: 14 
	if((strncmp((char*)GSM_rx,"ERROR: 30",9)==0)||(strncmp((char*)GSM_rx,"ERROR: 20",9)==0)||(strncmp((char*)GSM_rx,"ERROR: 14",14)==0)||(strncmp((char*)GSM_rx,"ERROR: 19",14)==0)\
		||(strncmp((char*)GSM_rx,"ERROR: 6",8)==0)||(strncmp((char*)GSM_rx,"ERROR: 7",8)==0)||(strncmp((char*)GSM_rx,"ERROR: 8",8)==0)||(strncmp((char*)GSM_rx,"ERROR: 10",9)==0))	    
	{
		     if (Dial_jump_State==7)
				{  
				       rt_hw_gsm_output("AT%IPCLOSE=1\r\n");   
					rt_thread_delay(10);   
		                     Dial_Stage(Dial_DNSR1);  // 后边跳跃到ready to  connect aux DNS 
					  rt_kprintf("\r\n   DNSR1  解析 成功-->连接失败\r\n");   	
					   Dnsr_state=0;//表示DNSR 状态下拨号 	
				     	//-------------  End  need	process ---------------------------------------
			               memset(GSM_rx, 0, sizeof(GSM_rx));
	                             GSM_rx_Wr = 0;      
					 return;	   
				}
			       else	 
		             if(Dial_jump_State==8)    
				{      
				        rt_hw_gsm_output("AT%IPCLOSE=1\r\n");    
					 rt_thread_delay(10);
		                      Dial_Stage(Dial_MainLnk);  // ready to  connect mainlink
				        rt_kprintf("\r\n   DNSR2  解析 成功-->连接失败\r\n");  	
				        Dial_jump_State=0;  		
					 Dnsr_state=0;//表示DNSR 状态下拨号 		  
						//-------------  End  need	process ---------------------------------------
			               memset(GSM_rx, 0, sizeof(GSM_rx));
	                             GSM_rx_Wr = 0; 
					  return;	   	  
				}	
				else	 
				 if(DataDial.Dial_step ==Dial_MainLnk)    //mainlink  to  auxlink
				{      
				        // GSM_PutStr("AT%IPCLOSE=1\r\n");  
					//  AT_Stage(AT_Dial_AuxLnk);//AT_Dial_AuxLnk  
					 rt_thread_delay(10);
					 Dial_Stage(Dial_AuxLnk);//  ready to connect auxlink
					
				         //-----------  Below add by Nathan  ----------------------------
                                     DataLink_AuxSocket_set(RemoteIP_aux, RemotePort_main,0);	 
					 //-------------  End  need	process ---------------------------------------
			               memset(GSM_rx, 0, sizeof(GSM_rx));         
 	                             GSM_rx_Wr = 0;  
 					  return;	   
			    	}
				 //--------------------------------
				   else
				   {     
				        DataLink_EndFlag=1; //AT_End();   
                                       rt_kprintf("\r\n Datalink end =>Errors 杂\r\n"); 
				   	}
	} 
	else
	if (strncmp((char*)GSM_rx, "ERROR",5) == 0) 
	{
	   error = true;
	   CallState=CallState_Idle;  
	   //----------------------
	   if(TTS_ACK_Error_Process()==true)
               {     rt_kprintf("\r\n  TTS ack  error \r\n");  Speak_OFF; }
	   else 
	   if(DataLink_Status())        //   Online  state  Error    , End Link and Redial     
	    {
	       Rx_Error_counter++;
		   if(Rx_Error_counter>3)
	        {
	           DataLink_EndFlag=1; 
			   Rx_Error_counter=0;
		   	}

	   	}
	   rt_kprintf("\r\n ERROR\r\n");      
	}   
	else
	if (strncmp((char*)GSM_rx, "Unknown",7) == 0) 
		error = true;	

	//-------------  End  need	process ---------------------------------------
RXOVER:
       memset(GSM_rx, 0, sizeof(GSM_rx));
	GSM_rx_Wr = 0; 
	
	//---------------------  Function  Process Below --------------------------
	if ((DataDial.Dial_step != Dial_MainLnk) && failed) 
	{ 
		 Dial_Stage(Dial_Idle); 
		return;
	}
	
	switch (DataDial.Dial_step)
	{
		case Dial_DialInit0:	if (ok)  Dial_Stage(Dial_DialInit1);
								break;
		case Dial_DialInit1   :   if (ok) Dial_Stage(Dial_DialInit2); 
								break;
		case Dial_DialInit2	:	if (ok)   Dial_Stage(Dial_DialInit3);
								break;
		case Dial_DialInit3	:	if (ok)   Dial_Stage(Dial_DialInit4);
								break;
		case Dial_DialInit4	:	if (ok)  Dial_Stage(Dial_DialInit5);
								break;
		case Dial_DialInit5	:	if (ok)  Dial_Stage(Dial_DialInit6);
								break;
		case Dial_DialInit6	:	if (ok)  
			                                 /*     IC 卡过检验不需要DNSR
			                                   #ifdef MULTI_LINK
			                                         Dial_Stage(Dial_DNSR1);   // 多连接
								#else
							    */	
						        Dial_Stage(Dial_MainLnk);   
		                                      // #endif
								break;						
		case Dial_DNSR1	:	if (ok)
			                           {
                                                         Dial_Stage(Dial_DNSR2);
			                           }   
								break;		
	         case Dial_DNSR2	:	if (ok)
			                           {
                                                          Dial_Stage(Dial_MainLnk); 
			                           }   
								break;							
		case Dial_MainLnk		:		
	    case Dial_AuxLnk	       :	if (failed || error)
								{
									DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;				// try again in 6 seconds
									break;
								} 
								 if (connect) 						
								{   //  below  run once  since  online
								     if(DataDial.Dial_step ==Dial_MainLnk)    //mainlink 
								             rt_kprintf("\r\n连接成功TCP---\r\n");
								     if(DataDial.Dial_step ==Dial_AuxLnk)    //auxlink 	 
								                rt_kprintf("\r\n Aux 连接成功TCP---\r\n");

								 
								            //   注册状态
								            if(1==JT808Conf_struct.Regsiter_Status)  
								                {  
								                   DEV_Login.Operate_enable=1;   	
                                                                           DEV_Login.Enable_sd=1;//鉴权只发一次
                                                                           DEV_Login.Sd_counter=0;
                                                                           DEV_Login.Max_sd_num=0; 
								            	}
								   // connect = true;
								         //  -----  Data  Dial Related ------
									    if((DataDial.Dial_ON)&&(DataDial.Dial_step<Max_DIALSTEP)) 
									    	{
							                            Dial_Stage(Dial_Idle);
										       DataDial.Dial_step_Retry=0;
										       DataDial.Dial_step_RetryTimer=0;  			
									    	}
							              //-----------------------------------
							           //   DataDial.Dial_ON=0; 
								       DataLink_Online=2; 	  
									DialLink_TimeOut_Clear(); 	  

									 Dial_Stage(Dial_Idle); //  state  convert   
									 RTC_TimeShow();	
						             }						           
							      //------------------------------------------							
								 break;  
		     case Dial_ISP:
			 	                           if (failed || error)
								{
									DataDial.Dial_step_RetryTimer=Dial_Dial_Retry_Time;
									break;
								}
								 if (connect_2)  
								{
									 Dial_Stage(Dial_Idle); //	In EM310 Mode										
									 TCP2_Connect=1;		
									 //TCP2_IC_sdFlag=1;
									  TCP2_ready_dial=0;  
									  rt_kprintf("\r\n //---------------------------------");
									  rt_kprintf("\r\n     IC 卡 中心连接成功!\r\n");     
									  rt_kprintf("//---------------------------------\r\n "); 
									  DataDial.Dial_ON=0;  
									  BD_ISP.ISP_running=0; 
								}							 
								break;		 						
		default 			:	 Dial_Stage(Dial_Idle);
	                                          DataDial.Dial_step_Retry=0;	
								break;
	}
    //-----------
}

void  IMSIcode_Get(void)
{

              if((GSM_PWR.GSM_power_over==1) &&(JT808Conf_struct.password_flag ==1) )
		{
                   IMSIGet.Checkcounter++;
		       if(IMSIGet.Checkcounter>30)     //  15*30=450ms      
		   	{
                          IMSIGet.Checkcounter=0; 
			    if(IMSIGet.Get_state==1)
	               {
	                 rt_hw_gsm_output(IMSI_Check_str);    // 查询 IMSI_CODE 号码 
	                 rt_kprintf(IMSI_Check_str);  
		         }  
			  else 
			   {
			  	   rt_hw_gsm_output(SIM_Check_Str);    // 先检查 SIM 卡的存在
		                 rt_kprintf(SIM_Check_Str);    
			   }
		   	} 
		}  
}

void on_spk(void)
{
    Speak_ON;   // 开功放
}
FINSH_FUNCTION_EXPORT(on_spk, Speaker On);
 
void off_spk(void)
{
    Speak_OFF;   // 开功放
}
FINSH_FUNCTION_EXPORT(off_spk, Speaker Off); 

void AT(u8 *str)
{
       rt_hw_gsm_output((const char*)str);     
} 
FINSH_FUNCTION_EXPORT(AT, gsm_ATcmd);

void  redial(void)
{
      DataLink_EndFlag=1; //AT_End();   
        rt_kprintf("\r\n Redial\r\n");      
}
FINSH_FUNCTION_EXPORT(redial, redial);


void Rx_in(u8* instr)
{
    u16  inlen=0;

      if(strncmp(instr,"can1",4)==0)
      	{
           Get_GSM_HexData("7E8103003B013601300001864F060000010004000001F40000010102000A0000010204000000000000010302000000000110080000000058FFD11700000111080000006458FFD017EA7E",148,0);
           OutPrint_HEX("模拟1", GSM_HEX, GSM_HEX_len); 

	 }
        else
      	{
		     inlen=strlen((const char*)instr);
		     Get_GSM_HexData(instr,inlen,0);  
		      OutPrint_HEX("模拟", GSM_HEX, GSM_HEX_len); 
      	}	 

}
FINSH_FUNCTION_EXPORT(Rx_in, Rx_in);


void canrxid(u8 * instr)
{
       u8  REG[4];
      
	GSM_AsciitoHEX_Convert(instr,8,REG); 
       CAN_trans.canid_1_Filter_ID=((REG[0]&0x1F)<<24)+(REG[1]<<16)+(REG[2]<<8) +REG[3]; 
       rt_kprintf("\r\n  Set Filter  ID =%08X\r\n", CAN_trans.canid_1_Filter_ID);        
}
FINSH_FUNCTION_EXPORT(canrxid, canrxid);


void  rt_hw_gsm_init(void)
{

       GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	


      RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD, ENABLE );
      RCC_APB1PeriphClockCmd( RCC_APB1Periph_UART4, ENABLE );



/*uart4 管脚设置*/

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; 
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
       GPIO_InitStructure.GPIO_Pin = GSM_TX_PIN| GSM_RX_PIN;
	GPIO_Init( GPIOC, &GPIO_InitStructure );

       GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_UART4);
       GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_UART4);   

	

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_InitStructure.USART_BaudRate =57600;//115200;   // new M66  57600  
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
	USART_Init(UART4, &USART_InitStructure);
	/* Enable USART */
	USART_Cmd(UART4, ENABLE);
	USART_ITConfig( UART4, USART_IT_RXNE, ENABLE );   

			 
   /* -------------- GPRS  GSM	模块 EM310 操作相关 -----------------	*/
   /*
		   STM32 Pin	 SIM300 gprs  Pin		   Remark
			 PD12		   Power				  PE2 set 1: Power On  set	0: Poweroff 
			 PD13	   PWRKEY				   加了反向三极管	PA5 set 1:Off	   0: On 
	*/
  
   /*  管脚初始化 设置为 推挽输出 */

   RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);


  GPIO_InitStructure.GPIO_Pin = GPRS_GSM_Power ; 
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);	
   
   GPIO_InitStructure.GPIO_Pin = GPRS_GSM_PWKEY;
   GPIO_Init(GPIOD, &GPIO_InitStructure);	
   
   GPIO_InitStructure.GPIO_Pin =GPRS_GSM_RST;		//-----  Reset 常态下置低   高有效  
   GPIO_Init(GPIOD, &GPIO_InitStructure);   


   GPIO_InitStructure.GPIO_Pin =Speak_Ctrl;		//-----  功放
   GPIO_Init(GPIOD, &GPIO_InitStructure);   	 
   //==================================================================== 
   GPIO_ResetBits(GPIOD,GPRS_GSM_RST);   // 常态下置低 Reset	  
   GPIO_ResetBits(GPIOD,GPRS_GSM_Power);
   GPIO_ResetBits(GPIOD,GPRS_GSM_PWKEY);    //GPIO_SetBits(GPIOD,GPRS_GSM_PWKEY);            
   Speak_OFF;//  关闭音频功放

/*
    GPIO_InitStructure.GPIO_Pin =GPIO_Pin_2;		//GPS  串口拉低
   GPIO_Init(GPIOD, &GPIO_InitStructure);   

     GPIO_InitStructure.GPIO_Pin =GPIO_Pin_12;		//GPS  串口拉低
   GPIO_Init(GPIOC, &GPIO_InitStructure);   
     GPIO_ResetBits(GPIOC,GPIO_Pin_12);      
    GPIO_ResetBits(GPIOD,GPIO_Pin_2);      	   

 // Speak_ON;  
 */
}
