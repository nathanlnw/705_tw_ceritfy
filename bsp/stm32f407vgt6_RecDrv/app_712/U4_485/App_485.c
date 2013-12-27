/*
       APP_485.C
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
#include "App_moduleConfig.h"
#include "spi_sd.h"
#include "Usbh_conf.h"
#include <dfs_posix.h>  
#include <finsh.h>

#define FileSize_mp3   5616 

/*
                  拍照相关  
*/
 _MultiTake     MultiTake;	  //  多路拍照状态位 
 u8  SingleCamera_TakeRetry=0; // 单路摄像头拍照时，重拍次数计数
 Camera_state CameraState; 
 u8    TX_485const_Enable=0;   // 使能发送标志位  
 u8 	  last_package=0; // 拍照最后一包标识

/*
              语音盒相关
*/
 VOICE_DEV Dev_Voice;  

/*
      写固有音频MP3
*/
WT_MP3  WriteMP3;  // 写固有音频
u8    mp3_buf[512],mp3_read[512];
int	 fd_mp3 = -1;




//---------------------------Photo Related Function --------------------------------------

void Init_Camera(void)
{
   CameraState.block_counter=0;  // clear   
   CameraState.status=other;    
   CameraState.create_Flag=0; 
}
void End_Camera(void)
{
  CameraState.block_counter=0;
  CameraState.status=other;
  CameraState.camera_running=0; 
  CameraState.timeoutCounter=0;
  CameraState.OperateFlag=0;
  CameraState.create_Flag=0; 
  Dev_Voice.Poll_Enble=1;//开始发送语音盒轮询 
  _485_RXstatus._485_receiveflag=IDLE_485;  
   #ifdef LCD_5inch
  DwinLCD_work_Enable();//  使能5 寸屏
  #endif
}
u8 Start_Camera(u8  CameraNum)
{
    if(1==ISP_running_Status())    // 远程更新过程中不许拍照
		return false;
		
        //----------------------------------------------------
		Photo_TakeCMD_Update(CameraNum);
	    //---------------------------------	
		Photo_FetchCMD_Update(CameraNum);
        
		CameraState.create_Flag=0;  
        //-------------------------------------------------------
                 //if((CameraState.camera_running==0)||(Photo_sdState.photo_sending==enable))// 在不传输报警图片的情况且没有图片传输的情况下执行
                  if((CameraState.camera_running==0)&&(Photo_sdState.photo_sending==0))// 在不传输报警图片的情况且没有图片传输的情况下执行 
	 	          {
				        CameraState.camera_running=1; 
					 CameraState.status=enable;
					 TX_485const_Enable=1;
					  _485_RXstatus._485_receiveflag=IDLE_485; 
					 Dev_Voice.Poll_Enble=0;//停止发送语音盒轮询   
                                    #ifdef LCD_5inch 
					 DwinLCD_work_Disable() ;  //  停止5 寸屏
					 #endif
					
					 return true;
				  }  
				  else
				  {
				    CameraState.status=other; 
					return false;
				  }
				  
}

void  Camra_Take_Exception(void)
{
         if(CameraState.camera_running==1)
         {
	           if((CameraState.timeoutCounter++)>=3)  // 每发送一包，清除，只有单包发送超过3秒没返回才认为失败 
			   {  
			      //------------  Normal  Process  --------------
			      //FeedDog;
			      End_Camera();		
				  
				 // Power_485CH1_OFF;  // 第一路485的电			关电工作
				  rt_kprintf("\r\n  Camera %d  Error\r\n",Camera_Number);     
				  WatchDog_Feed();
				  delay_ms(20);  // end--duration--new start	
				  WatchDog_Feed();
				  //Power_485CH1_ON;  // 第一路485的电			关电工作      
                  //------------  Multi Take Process  ------------  
                  if(1==MultiTake.Taking)
                  {
                      switch(Camera_Number)  
                      	{
                           case  1:  
						   	       MultiTake.Take_retry++;
								   if(MultiTake.Take_retry>=3) 
								   	{
								   	   //-------- old process---------------
                                       MultiTake.TakeResult[0]=Take_Fail;   // 表示第一路摄像头拍照失败
									   //--------- new   process-------------
                                           MultiTake.Take_retry=0;
									       //----------拍照下一路摄像头-----------									   
										   Camera_Number=2;  
									       //-------------------------
									       Start_Camera(Camera_Number);
								   	}
								   else
								   	{    // continue operate this  camera 
                                          Start_Camera(Camera_Number);
								   	}

							       break;
						    case  2:  
						   	       MultiTake.Take_retry++;
								   if(MultiTake.Take_retry>=3) 
								   	{
								   	   //-------- old process---------------
                                       MultiTake.TakeResult[1]=Take_Fail;   // 表示第一路摄像头拍照失败
									   //--------- new   process-------------
                                           MultiTake.Take_retry=0;
									       //----------拍照下一路摄像头-----------									   
										   Camera_Number=3;  
									       //-------------------------
									       Start_Camera(Camera_Number);
								   	}
								   else
								   	{    // continue operate this  camera 
                                          Start_Camera(Camera_Number);
								   	}

							       break;
						    case  3:   
						   	       MultiTake.Take_retry++; 
								   if(MultiTake.Take_retry>=3) 
								   	{
								   	   //-------- old process---------------
                                       MultiTake.TakeResult[2]=Take_Fail;   // 表示第一路摄像头拍照失败
									   //--------- new   process-------------
                                           MultiTake.Take_retry=0;
									       //----------拍照下一路摄像头-----------									   
										   Camera_Number=4;  
									       //-------------------------
									       Start_Camera(Camera_Number);
								   	}
								   else
								   	{    // continue operate this  camera 
                                          Start_Camera(Camera_Number);
								   	}

							       break;		   
						   case  4:
						   	       MultiTake.Take_retry++;
								   if(MultiTake.Take_retry>=3)
								   	{
								   	   //-------- old process---------------
                                       MultiTake.TakeResult[3]=Take_Fail;   // 表示第一路摄像头拍照失败
									   //--------- new   process-------------
                                           MultiTake.Take_retry=0;
									       /*
									       Taking End, Start Transfering 
									        */
									    MultiTake.Taking=0;  // Taking  State  Over									    
									    Check_MultiTakeResult_b4Trans();
								   	}
								   else
								   	{    // continue operate this  camera 
                                          Start_Camera(Camera_Number);
								   	}

							       break;			
						   default:
						   	       MultiTake_End();   // clear and  exit Mutitake state
						   	       break;
                      	}


                  }
				 else
				 {    //  单次拍照
                    SingleCamera_TakeRetry++;
					if(SingleCamera_TakeRetry>=3)
						{
                             SingleCamera_TakeRetry=0;//clear
							 rt_kprintf("\r\n     单路拍照超过最大次数!\r\n");  
						}
					else
					   	{    // continue operate this  camera 
                              Start_Camera(Camera_Number);
							  //printf("\r\n  重新拍 %d \r\n",Camera_Number); 
					   	}
				 }
				  
	           }
         }




}

void MultiTake_Start(void)
{
    u8 i=0;
	
    MultiTake.Taking=1;     // enable
	MultiTake.Transfering=0;
	MultiTake.CurretCameraNum=1;
	for(i=0;i<Max_CameraNum;i++)
	   MultiTake.TakeResult[i]=Take_idle;
	MultiTake.Take_retry=0;  
	MultiTake.Take_success_counter=0;
	//----------------------------------	
        Camera_Number=1;   // 从 1  Start from  1  Camera
	    Start_Camera(Camera_Number);  // 开始执行拍照   

     //------------------
     SingleCamera_TakeRetry=0;  // add later    		
     
} 

void MultiTake_End(void) 
{
    u8 i=0;
	
    MultiTake.Taking=0;     
	MultiTake.Transfering=0;
	MultiTake.CurretCameraNum=1;
	for(i=0;i<Max_CameraNum;i++)
	   MultiTake.TakeResult[i]=Take_idle;
    MultiTake.Take_retry=0;	
	MultiTake.Take_success_counter=0;

}


u8   Camera_Take_Enable(void)
{
         //  在拍照状态空闲， 没有多路拍照和多路传输的情况下，可以拍照
          if((CameraState.status==other)&&(0==MultiTake.Taking)&&(0==MultiTake.Transfering))
                return  true;
	   else
	   	   return false;
   }

// ----------------------  语音盒处理程序  ------------------------------
void Voice_Dev_Init(void)
{
  Dev_Voice.Poll_Enble=1;  
  Dev_Voice.CMD_Type='0';
  Dev_Voice.Work_State=0;
  Dev_Voice.Sd_Timer=0;
  Dev_Voice.info_sdFlag=0;
  Dev_Voice.Rec_runFlag=0; 
  Dev_Voice.Voice_FileOperateFlag=0;
  Dev_Voice.Centre_RecordFlag=0; 
   Dev_Voice.Voice_PageCounter=0;  
}
void  Voice_Dev_Rxprocess(void)
{
    u16  len=0;//,wrlen=0;i=0;
#if 0	
  len=(_485_content[3]<<8)+_485_content[4]-1; 
  switch(_485_content[5])
  	{
 	   case '0':  // 停止采集语音盒数据  应答 	    
 	              if((Dev_Voice.CMD_Type!='0')&&(Dev_Voice.info_sdFlag==0))  // 只有不等且信息标志为0才置 
				      Dev_Voice.CMD_Type='0'; 
				  Dev_Voice.Work_State=0; 		  		  
	              break;
	   case '1':  // 语音数据		
	              Dev_Voice.info_sdFlag=0;   
				  if(Dev_Voice.Voice_FileOperateFlag==1)  // 在命令为 1 时 采集记录
				  {
				     // rt_kprintf("\r\n Rxlen= %d \r\n",len);                       
                      //-----------  每包有效字节500 但是写 512   后边的12字节是空的  --------------
                      memset(Dev_Voice.Voice_Reg, 0,512); 
					  memcpy(Dev_Voice.Voice_Reg, _485_content+6,len);  //写内容					  
					  if(TF_Card_Status()==1)
						 {	;	
						    /*          rt_kprintf("\r\n -s\r\n");
						            //rt_kprintf("Head-wr : %X %X %X %X \r\n",Dev_Voice.Voice_Reg[0],Dev_Voice.Voice_Reg[1],Dev_Voice.Voice_Reg[2],Dev_Voice.Voice_Reg[3]);
									Dev_Voice.Voice_FileSize+=len; //有数据时是 500 没有时候为O
									if(len==0) 
										 break;   
									
									//edit_file(Dev_Voice.FileName,Dev_Voice.Voice_Reg,512); 	  //写信息到TF  
									DF_WriteFlashDirect(SoundStart_offdet+Dev_Voice.Voice_PageCounter,0,Dev_Voice.Voice_Reg,500);  
									Dev_Voice.Voice_PageCounter++; 
									//DF_ReadFlash(SoundStart_offdet+Dev_Voice.Voice_PageCounter-1,0,Dev_Voice.Voice_Reg,500);  
						            //rt_kprintf("Head-Rd : %X %X %X %X \r\n",Dev_Voice.Voice_Reg[0],Dev_Voice.Voice_Reg[1],Dev_Voice.Voice_Reg[2],Dev_Voice.Voice_Reg[3]);
									
							if(Dev_Voice.Centre_RecordFlag)
								{	
									
								   if(Dev_Voice.Voice_FileSize>=15000) //  15秒 的数据  
								   {
									  Dev_Voice.CMD_Type='0';
									  Dev_Voice.info_sdFlag=0;	  
									  Dev_Voice.Voice_FileOperateFlag=0;
									  Dev_Voice.Centre_RecordFlag=0; // 清空中心录音标志位                                       
									  rt_kprintf("\r\n ---------   中心录音地址	  VoiceFileSize %d Bytes \r\n",Dev_Voice.Voice_FileSize);
									  Dev_Voice.Centre_RecordFlag=0;

									  Sound_SaveEnd();
									  Sound_send_start(); //开始上传
									  
								   	}
								} */   
						 }  
				  	}    				  
                  break;
	   case '2':  // 收到语音播报应答
                  Dev_Voice.CMD_Type='0'; 
				  Dev_Voice.Work_State=0;   
				  Dev_Voice.info_sdFlag=0;
	              break;
	   default:
	   	          break;  	   
  	}
  #endif
 /* 
  rt_kprintf("\r\n RX content:");    
  for(i=0;i<len+8;i++)
     rt_kprintf(" %x",_485_content[i]); 
  rt_kprintf("\r\n"); 
 */
}
									 
void  Voice_Dev_Txprocess(void)
{
   u8 Send[250];
   u16  Vinfolen=0,i, V_wr=0;
   u8  Vfcs=0;

    memset(Send,0,sizeof(Send));
	V_wr=0;
    Send[V_wr++]=0x40;
	Send[V_wr++]=0x40;
	Send[V_wr++]=0x81; // 主机
    if(Dev_Voice.info_sdFlag==1)
    {
       Vinfolen=1+strlen((const char*)Dev_Voice.Play_info);
	   Send[V_wr++]=Vinfolen>>8;
	   Send[V_wr++]=Vinfolen; 
	   Send[V_wr++]=Dev_Voice.CMD_Type; 
	   memcpy(Send+6,Dev_Voice.Play_info,strlen((const char*)Dev_Voice.Play_info));   
	   V_wr+=strlen((const char*)Dev_Voice.Play_info);
    }
	else
	{      
	  Vinfolen=1;
	  Send[V_wr++]=Vinfolen>>8;
	  Send[V_wr++]=Vinfolen; 
	  Send[V_wr++]=Dev_Voice.CMD_Type;       
	}
     Vfcs=0;
	 for(i=0;i<Vinfolen+2;i++) // 长度 2 个字节 所以加 2
	 	{
          Vfcs^=Send[3+i];
	 	}
	 Send[V_wr++]=Vfcs;
	 Send[V_wr++]=0x0D;
	 Send[V_wr++]=0x0A; 

      if(Dev_Voice.CMD_Type=='2')
      {  
		  rt_device_write(&Device_485,0,( const char*)Send,V_wr); 
		  //------------ below add  clear over  ----------------
		  Dev_Voice.CMD_Type='0'; 
		  Dev_Voice.Work_State=0;   
		  Dev_Voice.info_sdFlag=0;     
		  
      }	   
	  else
	        rt_device_write(&Device_485,0,Send,V_wr);
    //---- Add for debug ----
    /*  
        rt_kprintf("\r\n   ----> 485:");  
	   for(i=0;i<V_wr;i++)
	     	rt_kprintf(" %x",Send[i]);  
	 */ 
}


 u8  Check_MultiTakeResult_b4Trans(void)
{ 
    u8 i=0;

	 MultiTake.Take_success_counter=0; 
     for(i=0;i<Max_CameraNum;i++)
	 {
	 	 if(Take_Success==MultiTake.TakeResult[i])
	 	 {
	 	    MultiTake.Take_success_counter++;
	 	 }
	 }
	// Read_picNum=MultiTake.Take_success_counter-1; // 0  是最新一幅      
	 if(MultiTake.Take_success_counter)
	 {
	 	    MultiTake.Transfering=1;  // Set Transfering 
	 	    MultiTake.Take_success_counter--;  // 开始上传就得减 1
	 	    //----------- 查找拍照成功的摄像头序号  上报数据要填充-----------
	 	     for(i=0;i<Max_CameraNum;i++) 
			 {
				 	 if(Take_Success==MultiTake.TakeResult[i])
				 	 { 
				 	    Camera_Number=i+1;
						MultiTake.TakeResult[i]=Take_idle;// clear state
						break;
				 	 }
	         }
			rt_kprintf("\r\n            多路开始 Camera=%d  \r\n",Camera_Number);  
			Photo_send_start(Camera_Number);   //开始准备发送      
			rt_kprintf("\r\n            多媒体事件上传MediaObj.SD_media_Flag=%d , MediaObj.SD_Eventstate=%d  \r\n",MediaObj.SD_media_Flag,MediaObj.SD_Eventstate);  
			return  1;
	 }					 
	 else
	 {
	    MultiTake_End();
		return  0;
	 }	

}

void Send_const485(u8  flag) 
{

  if(!TX_485const_Enable)
  	return;
 
  switch(CameraState.status)
  	{
       case enable:
                     rt_device_write(&Device_485,0,(char const*)Take_photo, 10);  // 发送拍照命令 
                    // rt_hw_485_Output_Data(Take_photo, 10);  // 发送拍照命令 
                    CameraState.OperateFlag=1;
					CameraState.create_Flag=1; // 需要创建新图片文件 
                    last_package=0; 
					 rt_kprintf("\r\n  Camera: %d  发送拍照命令\r\n",Camera_Number);
					 CameraState.timeoutCounter=0; // send clear
				    break; 
	   case  transfer:
	             	//----------_485_content_wr=0;
				  _485_RXstatus._485_receiveflag=IDLE_485;  
					// rt_kprintf("\r\n 发送图片读取命令\r\n"); 
                                  rt_device_write(&Device_485,0,(char const*)Fectch_photo, 10); // 发送取图命令
                            //   rt_hw_485_Output_Data(Fectch_photo, 10);  // 发送拍照命令   
                               CameraState.OperateFlag=1;
				   CameraState.timeoutCounter=0;// sendclear
					//rt_kprintf("\r\n  发送取图命令\r\n");					
		            break;
	   case  other	:
	   	
	   	            break;
	    default:    
	   	            break;
  	}  
   TX_485const_Enable=0;  // clear     
}
 
void takephoto(u8* str) 
{
  
  if (strlen((const char*)str)==0)
  {
	 
	  rt_kprintf("\r\n 请输入要拍照的摄像头序号\r\n");	  
	  return ;
  }
  else 
  {  
     Camera_Number=(str[0]-0x30);
     rt_kprintf("\r\n CameraNum=  %d\r\n",Camera_Number);	   
     Start_Camera(Camera_Number);    
	 return ;  
  }
} 

FINSH_FUNCTION_EXPORT(takephoto,Take photo); 


int str2ipport(char *buf, u8 *ip, u16 *port)
{	// convert an ip:port string into a binary values
	int	i;
	u16	_ip[4], _port;
	

	_port = 0;
	memset(_ip, 0, sizeof(_ip));

	strtrim((u8*)buf, ' ');
   
	i = sscanf(buf, "%u.%u.%u.%u:%u", (u32*)&_ip[0], (u32*)&_ip[1], (u32*)&_ip[2], (u32*)&_ip[3], (u32*)&_port);

	*(u8*)(ip + 0) = (u8)_ip[0];
	*(u8*)(ip + 1) = (u8)_ip[1];
	*(u8*)(ip + 2) = (u8)_ip[2];
	*(u8*)(ip + 3) = (u8)_ip[3];
	*port = _port;

	return i;
}


void  mp3write(void)
{
       	if( rt_device_find( "udisk" ) == RT_NULL ) /*没有找到*/
		{
		       rt_kprintf("\r\n 没有找到udisk\r\n");	  
			rt_thread_delay( RT_TICK_PER_SECOND );
		}else
		{
			rt_kprintf("\r\n 找到udisk，ch12.mp3 ok\r\n");
			WriteMP3.write_flag=1;
			WriteMP3.run_step=1;
			WriteMP3.Page_count=0;
			WriteMP3.File_size=FileSize_mp3;  //固定文件大小 
		}   
}
FINSH_FUNCTION_EXPORT(mp3write,mp3write ); 
//-------------------------------------------------------------------------
void enalemp3(void)
{
       WriteMP3.Trans_mp3=1;
     rt_kprintf("\r\nEnable 上传固有音频\r\n");	 
}
FINSH_FUNCTION_EXPORT(enalemp3,enalemp3 ); 

void transmp3(void)
{
     WriteMP3.Trans_mp3=1;
     MP3_send_start();
     rt_kprintf("\r\n Start 上传固有音频\r\n"); 	 
}
FINSH_FUNCTION_EXPORT(transmp3,transmp3 ); 

void Mp3_run(void)
{
    rt_err_t	res;
    u16            read_size,lastpacket=0,i=0;	

     switch(WriteMP3.run_step)
     	{
     	       case  1:    Api_DFdirectory_Delete(mp3);   //clear  mp3  DF  area 
     	                        WriteMP3.run_step=2;
     	                        break;
               case 2:
			   	   	fd_mp3= open( "/udisk/ch12.mp3", O_RDONLY, 0 );
					if( fd_mp3 >= 0 )
					{	 
					      WriteMP3.run_step=3;
                                          rt_kprintf("\r\n 读取ch12.mp3文件\r\n");
					}
					else
					{
						       WriteMP3.write_flag=0;
							WriteMP3.run_step=0;
							WriteMP3.File_size=0; 
							 WriteMP3.Page_count=0;
							rt_kprintf("\r\n ch12.mp3文件不存在");
							return;
					}
			   	    break;
		case  3:  //--------------------------------------------------
                            rt_thread_delay(RT_TICK_PER_SECOND/2);
				res=0;
				if((WriteMP3.File_size-WriteMP3.File_left)>=512)
					         read_size=512;
				else	         
					{      
					           read_size=WriteMP3.File_size-WriteMP3.File_left;
                                              lastpacket=1; 
					}
				
			      res = read( fd_mp3, mp3_buf,read_size );
				if(res<0)
					{
					             WriteMP3.write_flag=0;
							WriteMP3.run_step=0;
							WriteMP3.File_size=0; 
							WriteMP3.Page_count=0;
							rt_kprintf("\r\n ch12.mp3文件读取错误");
							return;
					}
				else           /*判断是否为最后一包*/
					{
                                           WriteMP3.File_left+=read_size;
                                           OutPrint_HEX("udisk数据",mp3_buf,read_size);
										   
                                           // -----------
					        WatchDog_Feed();

						 WriteMP3.Page_count++;				   
						 DF_WriteFlashDirect(DF_MP3_offset+WriteMP3.Page_count,0,mp3_buf,read_size);
						 DF_ReadFlash(DF_MP3_offset+WriteMP3.Page_count,0,mp3_read,read_size);
						// OutPrint_HEX(" DF read数据",mp3_read,read_size);
						 for(i=0;i<read_size;i++)
				              {
							if(mp3_buf[i]!=mp3_read[i])
								{
								      rt_kprintf("\r\n  error at page=%d,i=%d,write=%X,read=%X",WriteMP3.Page_count,i,mp3_buf[i],mp3_read[i]);
								       WriteMP3.write_flag=0;
									WriteMP3.run_step=0;
									WriteMP3.File_size=0; 
									WriteMP3.Page_count=0;
									rt_kprintf("\r\n ch12.mp3文件读取错误");
							return; 
								}
						}


						 if(1==lastpacket)
						  {	   
						       DF_WriteFlashDirect(DF_MP3_offset,0,"mp3",3); //  写标志
						       WriteMP3.write_flag=0;
							WriteMP3.run_step=0;
							WriteMP3.File_size=0; 
							WriteMP3.Page_count=0;
							rt_kprintf("\r\n ch12.mp3写入完毕!");
						 }
					}	 
                            //--------------------------------------------------
			          break;
		   default: 
		   	           WriteMP3.write_flag=0;
				    WriteMP3.run_step=0;
				    WriteMP3.File_size=0; 
				    WriteMP3.Page_count=0;
				    break;			


     	}

}





