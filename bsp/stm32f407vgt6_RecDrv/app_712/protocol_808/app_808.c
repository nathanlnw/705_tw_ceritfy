/*
     App_808.C
*/

#include <rtthread.h> 
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>

#include  <stdlib.h>//����ת�����ַ���
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>
//#include "usbh_usr.h"

#define MQ_RAWINFO_SIZE 4096


/* ��ʱ���Ŀ��ƿ� */
 static rt_timer_t timer_app; 




//----- app_thread   rx     gsm_thread  data  related ----- 	
ALIGN(RT_ALIGN_SIZE)
static MSG_Q_TYPE  app_rx_gsm_infoStruct;  // app   ���մ�gsm  �������ݽṹ
static  struct rt_semaphore app_rx_gsmdata_sem;  //  app �ṩ���� ��gsm�����ź���

//----- app_thread   rx     gps_thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_gps_infoStruct;  // app   ���մ�gsm  �������ݽṹ
//static  struct rt_semaphore app_rx_gps_sem;  //  app �ṩ���� ��gps�����ź���

//----- app_thread   rx    485 _thread  data  related ----- 	
//ALIGN(RT_ALIGN_SIZE)
//static  MSG_Q_TYPE  app_rx_485_infoStruct;  // app   ���մ�gsm  �������ݽṹ
//static  struct rt_semaphore app_rx_485_sem;  //  app �ṩ���� ��gps�����ź��� 

static  struct rt_semaphore SysRst_sem;  // system reset


static u8  OneSec_cout=0;

u8  Udisk_Test_workState=0;  //  Udisk ����״̬
u8  TF_test_workState=0;

rt_device_t   Udisk_dev= RT_NULL;
u8 Udisk_filename[30]; 
int  udisk_fd=0;

	
static  rt_device_t   TF_dev=RT_NULL;

static u8  TF_filename[30];

static  int  TF_fd=0;
u32   sec_num=2;
u32       WarnTimer=0; 
u16   ADC_ConvertedValue=0; //��ص�ѹAD��ֵ  
u16   AD_Volte=0;
u16   AD_2through[2]; //  ����2 ·AD ����ֵ

u8   OneSec_CounterApp=0;
APP_QUE  AppQue; 



 //   ä������
uint8_t					MQ_rawinfo[MQ_INFO_SIZE];
struct rt_messagequeue	mq_MQBuBao;

//  ��Ϣ���� ˳��������
uint8_t					SD_rawinfo[MQ_INFO_SIZE];
struct rt_messagequeue	mq_BDsd;
LENGTH_BUF BDsd_tx;
LENGTH_BUF BDsd_rx;
u8  BD_sequence_send_enable=0;  //   ��������˳����



void App_rxGsmData_SemRelease(u8* instr, u16 inlen,u8 link_num)
{
	/* release semaphore to let finsh thread rx data */
	//rt_sem_release(&app_rx_gsmdata_sem);
	Receive_DataFlag=1; 
	app_rx_gsm_infoStruct.info=instr;
	app_rx_gsm_infoStruct.len=inlen;  
	app_rx_gsm_infoStruct.link_num=link_num;  
}


void Device_RegisterTimer(void)
{
      if(0==JT808Conf_struct.Regsiter_Status)    //ע��   
          {
             DEV_regist.Sd_counter++;
			 if(DEV_regist.Sd_counter>6)
			 	{
                   DEV_regist.Sd_counter=0;
				   DEV_regist.Enable_sd=1;  
			 	}
          }
}


void Device_LoginTimer(void)
{  
  if((1==DEV_Login.Operate_enable)&&(DEV_Login.Max_sd_num<2))
  {
     DEV_Login.Sd_counter++;
	if(DEV_Login.Max_sd_num==0)
	{
		 if(DEV_Login.Sd_counter>=20)   
		 {
	          DEV_Login.Sd_counter=0;
		   DEV_Login.Enable_sd=1;
		   DEV_Login.Max_sd_num++; 
		 }
	}
	else
	if(DEV_Login.Sd_counter>=44)      
	{
	          DEV_Login.Sd_counter=0;
		   DEV_Login.Enable_sd=1;
		   DEV_Login.Max_sd_num++; 
	}
  }
}

//          System  reset  related  
void  System_Reset(void)
{
      rt_sem_release(&SysRst_sem);
}

void   Reset_Saveconfig(void)
{

}
	
void  App808_tick_counter(void) 
{
    Systerm_Reset_counter++;
    if(Systerm_Reset_counter>Max_SystemCounter)
     {   	Systerm_Reset_counter=0;	
	        rt_kprintf("\r\n Sysem  Control   Reset \r\n"); 
               reset(); 
     }  

}



void Udisk_write_buffer(u8 *Inbuf,u16 inlen)
{
    int  fd_res=0;
   if((sec_num>0)&&(sec_num<350)) 
   {   //--- udisk
      if((Udisk_Test_workState)&&(udisk_fd))
         { 
                fd_res=write(udisk_fd,Inbuf,inlen);
	        rt_kprintf("\r\n  wr :%s  resualt=%d\r\n",Udisk_filename,fd_res);		
      	  }
   #if 1	  
      // ----sd	  
      if((TF_test_workState)&&(TF_fd))
        {
             fd_res=write(TF_fd,Inbuf,inlen);
              rt_kprintf("\r\n  TFwr :%s  resualt=%d\r\n",TF_filename,fd_res);	
      	}		 
   #endif	  
   }	  
}

void UDisk_Write_Test(void)
{ 
       rt_err_t res;
	 
      sec_num++; 
      if(sec_num==400)
	  	sec_num=0;  
	if(sec_num==0)
	{   
	      rt_kprintf("\r\n  Sec_num=0\r\n");  
	      //------ U disk
              Udisk_dev=rt_device_find("udisk");
	        if (Udisk_dev != RT_NULL)	    
		{       rt_kprintf("\r\n  Udiskopen");
                       
			  res=rt_device_open(Udisk_dev, RT_DEVICE_OFLAG_RDWR); 	
			 if(res==RT_EOK)
			 {
			    memset(Udisk_filename,0,sizeof(Udisk_filename));
			    RTC_TimeShow();	
			    sprintf((char*)Udisk_filename,"/udisk/ud%d%d%d.txt", time_now.hour,time_now.min, time_now.sec);  	
			    rt_kprintf("\r\n   UdiskFilename:%s\r\n",Udisk_filename);
			    udisk_fd=open((char*)Udisk_filename, O_RDWR|O_CREAT, 0); 
			     Udisk_Test_workState=1;   	 
			 }		 
		}  
	       else
		 { 
                      Udisk_Test_workState=0;
	        }
		  
             //----  TF---
          #if  1   
		/*  if(dfs_mount("spi_sd","/sd","elm",0,0))
		  	{
		  	     rt_kprintf("\r\n   TF errror\r\n");
		  	      return ;
		  	}
		  	*/
		  TF_dev=rt_device_find("spi_sd");
	        if (TF_dev != RT_NULL)	     
		{
                       rt_kprintf("\r\n  TFopen\r\n");  
			   res=rt_device_open(TF_dev, RT_DEVICE_OFLAG_RDWR); 
			  if(res==RT_EOK)
			 {
			   memset(TF_filename,0,sizeof(TF_filename));
			    RTC_TimeShow();	
			   sprintf((char*)TF_filename,"/sd/tf%d%d%d.txt",time_now.hour,time_now.min, time_now.sec); 
			    rt_kprintf("\r\n   TF_Filename:%s\r\n",TF_filename);       
			  TF_fd=open((char*)TF_filename, O_RDWR|O_CREAT, 0); 
			  TF_test_workState=1; 
			  }
		} 
		else
		  { 
		          rt_kprintf("\r\n  TF not find");   
                      TF_test_workState=0;
	        }
			
	  #endif	
	}


	if(sec_num==390)  
	{
	       //----  udisk
	      if((Udisk_Test_workState)&&(udisk_fd)) 
             {
                  close(udisk_fd);
		    rt_kprintf("\r\n   Udisk Filename:%s    close\r\n",Udisk_filename); 
			
	       }
	#if  1	  
              //  ----  tf 
            if((TF_test_workState)&&(TF_fd))
		{
		    close(TF_fd); 
		    rt_kprintf("\r\n  TF_ Filename:%s    close\r\n",TF_filename); 	   
            	}
	#endif		
	}
}

void  SensorPlus_caculateSpeed (void)
{
   u16 plus_reg=0;
     
         plus_reg=TIM2->CNT;
	  TIM_SetCounter(TIM2, 0);		
	  Delta_1s_Plus=plus_reg;
	#if 1	 
	  total_plus+=plus_reg; 


           if(JT808Conf_struct.Speed_GetType)  // ͨ���ٶȴ����� ��ȡ�ٶ�
	       	{ 
	                 //  K_AdjustUseGPS(sp,sp_DISP);  //  ����Kֵ  
	                  GPS_speed=Speed_cacu;      
	       	}  
	  
    if(Spd_senor_Null==0) 
	     //   Speed_cacu=(Delta_1s_Plus*36000)/JT808Conf_struct.Vech_Character_Value;	// ������ٶ�    
	         Speed_cacu=Delta_1s_Plus;	// ������ٶ�     0.1km/h    400HZ==40KM/H      
	 else
	 	{
	 	  Speed_cacu=0;
		  Speed_gps=0;  // GPS Ҳ��� 0
	 	}  
	 
	 if(DispContent==4)  //  disp  ��ʾ   
	 { 
	//   if(DF_K_adjustState)
	 //	rt_kprintf("\r\n    �Զ�У׼���!");
	  // else
	 	//rt_kprintf("\r\n    ��δ�Զ�У׼У׼!");
	 
	   rt_kprintf("\r\n GPS�ٶ�=%d  , �������ٶ�=%d  �ϱ��ٶ�: %d \r\n",Speed_gps,Speed_cacu,GPS_speed);  
	   rt_kprintf("\r\n GPSʵ���ٶ�=%d km/h , ������ʵ���ٶ�=%d km/h �ϱ�ʵ���ٶ�: %d km/h\r\n",Speed_gps/10,Speed_cacu/10,GPS_speed/10);  
	   rt_kprintf("\r\n TIM2->CNT=%d  \r\n",plus_reg); 
	 } 
#endif


}

void  Emergence_Warn_Process(void)
{
  //----------- ����������������صĴ���  ------------------
  if(WARN_StatusGet()) 
  {
     //  ���չ����кʹ�������в��账��                        
	 if((CameraState.status==other)&&(Photo_sdState.photo_sending==0)&&(0==MultiTake.Taking)&&(0==MultiTake.Transfering))
	  {
		  WarnTimer++;
		  if(WarnTimer>=4)
			  {
			      WarnTimer=0;    	
				       //----------  ��·����ͷ���� -------------
					 // MultiTake_Start();
					 // Start_Camera(1);  //����һ·������ʾ   
			  }   
	  }   
     //-------------------------------------------------------
     
	 fTimer3s_warncount++;
	 if(fTimer3s_warncount>=4)			 
	{
		 // fTimer3s_warncount=0;
		 if ( ( warn_flag == 0 ) && ( f_Exigent_warning == 0 )&&(fTimer3s_warncount==4) )
		 {
				 warn_flag = 1; 					// ������־λ
				 Send_warn_times = 0;		  //  ���ʹ��� 
				 rt_kprintf( "�������� ");				 
				 StatusReg_WARN_Enable(); // �޸ı���״̬λ
				 PositionSD_Enable();  
				 Current_UDP_sd=1;   				 
		 }
	}

	 //------------------------------------------------- 
  } 
   else
  { 	   
	   WarnTimer=0;
	   fTimer3s_warncount=0;  
	   //------------ ����Ƿ���Ҫ�������� ------------
	 /*  if(CameraState.status==enable)
	   {		   
		 if((CameraState.camera_running==0)||(0==Photoing_statement))  // �ڲ����䱨��ͼƬ�������ִ��
		 {
			CameraState.status=disable;
		 }
	   }
	   else*/
	   if(CameraState.camera_running==0)
			CameraState.status=other;
  }   

}

void  Recorder_sd_timer(void)
{
    // �г���¼�����ݷ���
    Recode_Obj.timer++;     
    if((Recode_Obj.CountStep==2)&&(Recode_Obj.timer>=3))  
       {    
    	 Recode_Obj.CountStep=1; 
 		 Recode_Obj.timer=0;
       } 
}


static void timeout_app(void *  parameter)
{     //  100ms  =Dur 
    u8  SensorFlag=0,i=0;

     //--------  CAN ID -------------------------
   //  if(CAN_trans.can1_trans_dur>0) 
    // {
        //    CAN_trans.canid_ID_enableGet=1;	 
     //} 	
    App_mq_SendTimer(); 
     //------------------------------------------	
      OneSec_CounterApp++;
     if(OneSec_CounterApp>=10)	 
	{
           /*   if(CAN_trans.can1_trans_dur>0) 
              {
                 if((OneSec_CounterApp%5)==0)  
                       CAN_trans.can1_enable_get=1;   
              } 
*/
             OneSec_CounterApp=0; 
	     //---------------------------------- 	
	        if(DataLink_Status())
		    {
		          Device_RegisterTimer();
		          Device_LoginTimer();	      // ��Ȩֻ����һ��
			   SendMode_ConterProcess(); 	  
		    }	 
		     SensorPlus_caculateSpeed();  	     
		     IO_statusCheck(); 	 
		     Emergence_Warn_Process();   
		     Meida_Trans_Exception();	 
		     Sensor_Keep_Process_Clear();	 
		     local_time();	 
	         SMS_Send_timer();		
		     App_mq_NoAck_counter(); 	
		     MangQu_Timer();	 
			 Recorder_sd_timer(); 
			   
		            //�����ź���״ָ̬ʾ  //ɲ����//��ת//��ת//Զ��//�����//���//����//null
		     
			SensorFlag=0x80;
			for(i=1;i<8;i++)  
			{
				  if(Vehicle_sensor&SensorFlag)   
					    XinhaoStatus[i+10]=0x31;
				   else
					     XinhaoStatus[i+10]=0x30; 
				     SensorFlag=SensorFlag>>1;   
			} 
		       if(DispContent==3)
			       rt_kprintf("\r\n %s   \r\n",XinhaoStatus);     

                   //  system timer
                   App808_tick_counter(); 

                  //---------------------------------- 
            if(BDSD.Enable_Working==1)
            {
		            if( BDSD.SendFlag==RdCycle_SdOver)
				   {	   
				        BDSD.wait__resentTimer++;	   
					 if(BDSD.wait__resentTimer>3)  //5s No resulat 
					 {
					      BDSD.wait__resentTimer=0;  
						  BDSD.SendFlag=RdCycle_RdytoSD;  // ʹ�ܷ���  
					 }		
		           }	   
            }  
	        //====ƣ�ͼ�ʻ===========================
		if(Car_Status[3]&0x01)	 // ƣ�ͼ�ʻ�ǻ���ACC��������½��е� 
		       Tired_Check();     

		 //  --- ���ٱ���  -----------
		  SpeedWarnJudge();	// �ٶȱ����ж�    
		  //-------------------------------------------------------	
		  
     	 }	
         //   Media 
           if(OneSec_CounterApp>>1)   //  ����2 Ϊ1	   
	           Media_Timer_Service();        
	   //----------------------------------	

        if(Get_voicedata_delay==1)
        	{
                 GEt_voice_Counter++;
				 if(GEt_voice_Counter>4)
				 	{
				 	  GEt_voice_Counter=0;
                      Get_voicedata_delay=0;
					  //-------------------------------------
									    VocREC.Sate=VOICEREC_Sd_getcmd;   
									    VocREC.ExcuteFlag=1; 
				 	}
        	}

}

u8    Udisk_Find(void)
{
       rt_err_t res;

	if(  USB_Disk_RunStatus()==USB_CONNECT_NOTFIND)
	{
              Udisk_dev=rt_device_find("udisk");
	        if (Udisk_dev != RT_NULL)	    
		{     
		      rt_kprintf("\r\n  Udiskopen");                       
		      res=rt_device_open(Udisk_dev, RT_DEVICE_OFLAG_RDWR); 	
		      if(res==RT_EOK)
			 {
                      
	                   Udisk_Test_workState=1;
			     USB_DeviceFind();	
				rt_kprintf("\r\n Udisk Find ok\r\n");  	 
			      return       USB_FIND;		   
			 }	
			  return false;
		}  
	       else
		 { 
                      Udisk_Test_workState=0;
			 return  false;		  
	        }
	}
	else
	if( USB_Disk_RunStatus()==USB_FIND)	
		return  USB_FIND;
	else
		Udisk_Test_workState=0;
	return false; 
}

 void   MainPower_cut_process(void)
 {
	Powercut_Status=0x02;
	//----------�ϵ���  ------------
	//---------------------------------------------------
	//        D4  D3
	//        1   0  ����Դ���� 
	StatusReg_POWER_CUT(); 
	//----------------------------------------------------
	Power_485CH1_OFF;  // ��һ·485�ĵ�			 �ص繤��
	//-----------------------------------------------------			    
	//------- ACC ������ ----------
	Vehicle_RunStatus|=0x01;
	//   ACC ON		 ��� 
	StatusReg_ACC_ON();  // ACC  ״̬�Ĵ���   
	Sleep_Mode_ConfigExit(); // �������
	//-------  ��Ƿѹ--------------
	Warn_Status[3]&=~0x80; //ȡ��Ƿѹ����  
	SleepCounter=0; 

 }


 void  MainPower_Recover_process(void)
 { 
	//----------------------------------------------
	StatusReg_POWER_NORMAL();
	//----------------------------------------------
	Power_485CH1_ON;  // ��һ·485�ĵ� 		  ���繤��
}

void  Recorder_init(void)
{
  Recode_Obj.Float_ID=0;     //  ������ˮ��
  Recode_Obj.CMD=0;     //  ���ݲɼ� 
  Recode_Obj.SD_Data_Flag=0; //  ���ͷ��������ر�־
  Recode_Obj.CountStep=0;  //  ����������Ҫһ��һ������ 
  Recode_Obj.timer=0;
  //--------- add on  5-4 
  Recode_Obj.Devide_Flag=0;//  ��Ҫ�ְ��ϴ���־λ
  Recode_Obj.Total_pkt_num=0;   // �ְ��ܰ���
  Recode_Obj.Current_pkt_num=0; // ��ǰ���Ͱ��� �� 1  ��ʼ
  Recode_Obj.fcs=0;

  
  //-------  ��¼���б��ش�---  
  Recode_Obj.RSD_State=0;     //  �ش�״̬   0 : �ش�û������   1 :  �ش���ʼ    2  : ��ʾ˳���굫�ǻ�û�յ����ĵ��ش�����
  Recode_Obj.RSD_Timer=0;     //  ��״̬�µļ�����   
  Recode_Obj.RSD_Reader=0;    //  �ش���������ǰ��ֵ 
  Recode_Obj.RSD_total=0;     //  �ش�ѡ����Ŀ    
   

}
 ALIGN(RT_ALIGN_SIZE)
char app808_thread_stack[4096];      
struct rt_thread app808_thread;

static void App808_thread_entry(void* parameter) 
{
    MSG_Q_TYPE  AppMsgQ_struct;  
    MSG_Q_TYPE  MsgQ_gsmCome;	 	
    u8   Dialstr[40];  	
//    u32  a=0;	

     //  finsh_init(&shell->parser);
	  rt_kprintf("\r\n ---> app808 thread start !\r\n");  
  
	  TIM2_Configuration();	   
       //  step 1:  Init Dataflash
         DF_init();
         ProductAttribute_init();  
       //  step 2:   process config data   
         SysConfiguration();    // system config   	    
       //  step 3:    usb host init	   	    	//  step  4:   TF card Init    
       //  	 spi_sd_init();	    
          usbh_init();    
	   APP_IOpinInit();
          Init_ADC(); 
		  
       //  	 tf_open();      // open device  
	// pos=dfs_mount("spi_sd","/sd","elm",0,0);	
       //   if(pos)
	//  	rt_kprintf("\r\n TF_result=%d\r\n",pos); 
	//rt_kprintf("\r\n tf ok\r\n");  

	       //BUZZER
	   GPIO_Config_PWM();
	   TIM_Config_PWM();  
	   buzzer_onoff(0);  
	   
        /* watch dog init */ 
	WatchDogInit();                        
    GNSS_RawDataTrans_Init(0);
	CAN_struct_init();     
	SMS_send_init(); 
	Mangqu_Init();  
	Recorder_init(); 

       AppQue.write_num=0;
	AppQue.read_num=0;
	AppQue.sd_enable_flag=0;  
	AppQue.abnormal_counter=0;
	AppQue.re_send_flag=0; 
	AppQue.send_timer=0;
	
	while (1)
	{

  
		//--------------------------------------------------	
         if(Receive_DataFlag==1)
		 {
                        memcpy( UDP_HEX_Rx,app_rx_gsm_infoStruct.info,app_rx_gsm_infoStruct.len);
			  UDP_hexRx_len=app_rx_gsm_infoStruct.len;	 
			  if(app_rx_gsm_infoStruct.link_num)
			  	   rt_kprintf("\r\n Linik 2 info \r\n");  
			  
			  OutPrint_HEX("GSM RX",UDP_HEX_Rx,UDP_hexRx_len); 
              TCP_RX_Process(app_rx_gsm_infoStruct.link_num);        
			  Receive_DataFlag=0;	 	   
         }	
			     
                //    ISP  service  
		  Api_CHK_ReadCycle_status();//   ѭ���洢״̬���		
		  //-------- 808   Send data   		
           if(DataLink_Status()&&(CallState==CallState_Idle)&&(VocREC.Sate!=VOICEREC_DataRXing))   
		   {   		
		        //------- send
		          // if(App_mq_Resend()==false)
                            //      App_mq_Read_Process(); 
		             Do_SendGPSReport_GPRS();   
		   } 
           else
		   if(CallState==CallState_rdytoDialLis)
			  {
				  CallState=CallState_Dialing;
				  memset(Dialstr,0,sizeof(Dialstr));
				  memcpy(Dialstr,"ATD",3);
				  memcpy(Dialstr+3,JT808Conf_struct.LISTEN_Num,strlen((const char*)JT808Conf_struct.LISTEN_Num));
				  memcpy(Dialstr+3+strlen((const char*)JT808Conf_struct.LISTEN_Num),";\r\n",3);	
				  rt_kprintf((char*)Dialstr);
				  rt_hw_gsm_output((const char *)Dialstr); 	 
		  
		  }  
         //  ä���洢 
          // 5. ---------------  ˳��洢 GPS  -------------------		    
		if(GPS_getfirst)	 //------������������γ��
		{
			    if(Current_SD_Duration>1)// �������10s �洢˳���ϱ��� С��10 ���洢�ϱ� 
			    {                                                 //  �������ݲ�����flash
					   Save_GPS();       
			    } 
		}
		   MangQU_true_Save();
	     //============  ״̬��� =======================
           ACC_status_Check();
	     //---------------------------------------------------------------------------------------------	  
                rt_thread_delay(10);
          //  ����˳��洢�ϱ�          
		  BD_send_Mque_Rx();
		  WatchDog_Feed();
	}
}



/* init app808  */
void Protocol_app_init(void)
{
        rt_err_t result;

        
		rt_mq_init( &mq_MQBuBao, "mq_MQBB", &MQ_rawinfo[0], 64 - sizeof( void* ), MQ_RAWINFO_SIZE, RT_IPC_FLAG_FIFO );
        rt_mq_init( &mq_BDsd, "mq_BDsd", &SD_rawinfo[0], 128 - sizeof( void* ), MQ_RAWINFO_SIZE, RT_IPC_FLAG_FIFO );
			
		rt_sem_init(&SysRst_sem,"SysRst",0,0);  
        rt_sem_init(&app_rx_gsmdata_sem, "appRxSem", 0, 0);   		
       //---------  timer_app ----------
	         // 5.1. create  timer     100ms=Dur
	      timer_app=rt_timer_create("tim_app",timeout_app,RT_NULL,10,RT_TIMER_FLAG_PERIODIC); 
	        //  5.2. start timer
	      if(timer_app!=RT_NULL)
	           rt_timer_start(timer_app);      


       //------------------------------------------------------
	result=rt_thread_init(&app808_thread, 
		"app808", 
		App808_thread_entry, RT_NULL,
		&app808_thread_stack[0], sizeof(app808_thread_stack),  
		Prio_App808, 10); 
	   

    if (result == RT_EOK)
    {
           rt_thread_startup(&app808_thread); 
   	    rt_kprintf("\r\n app808  thread initial sucess!\r\n");    // nathan add
    	}
    else
	    rt_kprintf("\r\napp808  thread initial fail!\r\n");    // nathan add	   
}



