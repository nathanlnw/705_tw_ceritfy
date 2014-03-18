#include "App_moduleConfig.h"




//--------   顺序读取发送相关  ------------
u8    ReadCycle_status=RdCycle_Idle;  
u8    ReadCycle_timer=0;   // 超时判断

u32     cycle_write=0, cycle_read=0,delta_0704_rd=0,mangQu_read_reg=0,Mq_total_pkg=0,CurrentTotal=0;  // 循环存储记录
                                                            //currentTotal 是满足补报情况下再生成的过程中还会有新的数据存储，所以总包数还会增加
u32     cycle_writeAbnormal_counter=0;  // 写数据异常
u32    AvrgSpdPerMin_write=0,AvrgSpdPerMin_Read=0; // 车辆每分钟平均速度记录
u32    AvrgSpdPerSec_write=0,AvrgSpdPerSec_Read=0; // 车辆每秒平均速度记录
u32    AvrgMintPosit_write=0,AvrgMintPosit_Read=0; // 车辆单位小时内每分钟位置记录
u32    ErrorLog_write=0,ErrorLog_Read=0;           // 设备异常记录
u32    Recorder_write=0,Recorder_Read=0;           // 事故疑点
u32    Login_write=0,Login_Read=0;                 // 登录记录
u32    Powercut_write=0,Powercut_read=0;           // 外部电源断开
u32    Settingchg_write=0,Settingchg_read=0;       // 参数修改
u32    TiredDrv_write=0, TiredDrv_read=0;  // 疲劳驾驶存储记录
u32    ExpSpdRec_write=0, ExpSpdRec_read=0;  // 超速报警存储记录
u32    OnFireRec_write=0, OnFireRec_read=0;  // 车辆打火存储记录
u32    pic_write=0,pic_read=0,pic_current_page=0,pic_PageIn_offset=0,pic_size=0;;       // 图片存储记录 
u32   	Distance_m_u32=0;	 // 行车记录仪运行距离	  单位米
u32     DayStartDistance_32=0; //每天起始里程数目


//-----------------------------------------------------------------------------------------------------------------------------
u8 SaveCycleGPS(u32 cyclewr,u8 *content ,u16 saveLen) 
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   reg[1]={0};

  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(cycle_write>>4);                // 计算出 Page 偏移  除以16  
     InPageoffset=cycle_write-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<5);           // 计算出页内 字节   乘以 32 (每个记录32个字节)
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Sector  被移除到下一个Sector  1Sector=8Page  
     {
              WatchDog_Feed();
		SST25V_SectorErase_4KByte((pageoffset+CycleStart_offset)*PageSIZE);      // erase Sector	
		WatchDog_Feed();
		DF_delay_ms(80); 
	    rt_kprintf("\r\n Erase Cycle Sector : %d\r\n",(pageoffset>>3));       
	 }
  //	   2. Filter write  area    
       DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,reg,1); 
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址增然后从新开始寻找直到找到为止
	   	{
              //  cyclewr++;
		  cycle_write++;		  
		  if(cycle_write>=Max_CycleNum)
			   cycle_write=0;  
		  rt_kprintf("\r\n    *******   写区域 Write area : %d   is   Dirity! --EraseAgain \r\n",cycle_write);   
                //  Exception process
		    cycle_writeAbnormal_counter++;
		    if(cycle_writeAbnormal_counter>3)
		    	{
                            WatchDog_Feed();
				SST25V_SectorErase_4KByte((pageoffset+CycleStart_offset)*PageSIZE);      // erase current Sector		
				DF_delay_ms(20); 
			       rt_kprintf("\r\n write error exceed 3 ,Erase current Cycle Sector : %d\r\n",(pageoffset>>3));    
				//----------------------------   
                           cycle_writeAbnormal_counter=0;  
		    	}				
		  //---------------------------
		  PositionSD_Enable(); 
		  Current_State=1; // 使能即时上报 		 		 
		  return false;		  		 
	   	}       
  //   3. Write Record Content  
       WatchDog_Feed(); 
        Current_State=0;   //clear state  
        cycle_writeAbnormal_counter=0;  // normal sate  clear
        DF_WriteFlashDirect(pageoffset+CycleStart_offset,InPageAddr,content,saveLen);  //   写入信息
        DF_delay_ms(2);  
	// rt_kprintf("\r\n SaveGPS Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",CycleStart_offset,pageoffset,InPageoffset); 
		return true;  
  //-------------------------------------------------------------------------------------------- 
}  


u8 ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen)
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8  i=0,FCS=0;;

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
     pageoffset=(u32)(cycle_read>>4);                // 计算出 Page 偏移  除以16 
     InPageoffset=cycle_read-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<5);            // 计算出页内 字节   乘以 32 (每个记录32个字节)
  //   2. Write Record Content 
     DF_ReadFlash(pageoffset+CycleStart_offset,InPageAddr,content,ReadLen); 
     DF_delay_us(20);
    // DF_delay_ms(10); 
	// rt_kprintf("\r\n  ReadGPS Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d  \r\n",CycleStart_offset,pageoffset,InPageoffset);  
     WatchDog_Feed();
  if(DispContent)
  {
	  //for(i=0;i<ReadLen;i++)
	  	//rt_kprintf("%2X ",content[i]);  
	  	 for(i=8;i<17;i++)
	  	     rt_kprintf("%2X ",content[i]);  
		 
		  rt_kprintf("  ");

		 for(i=22;i<28;i++)  
	  	     rt_kprintf("%2X ",content[i]);         
	 rt_kprintf("\r\n");  
  }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   }			  
	  if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和   
	    { 	      
		  if(content[0]==0xFF)
		  {
			 rt_kprintf("\r\n  content[0]==0xFF   read=%d,  write=%d  \r\n",cycle_read,cycle_write);   
			 //cycle_read=cycle_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。
                       
			  cycle_read++;	
		        if(cycle_read>=Max_CycleNum)
		  	      cycle_read=0; 
			 return false;	
		  }  
                 cycle_read++;	
		  if(cycle_read>=Max_CycleNum)
		  	cycle_read=0;
		 // rt_kprintf("\r\n   *******  该条记录内容不对 *******  \r\n");      
		  return false; 
     	}	  
 
	//------------------------------------------------------------------------------------------   	 
	    return true;  
   
  //-------------------------------------------------------------------------------------------- 
} 

u8 BDSD_SaveCycleGPS(u32 cyclewr,u8 *content ,u16 saveLen) 
{
  /*
         //old  NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
         NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 128 Bytes ;  1page= 4Records   1Sector=8Page=32Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
//	u8   reg[1]={0};
	u8   rd_back[128];
	u16  i=0,retry=0;

  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(BDSD.write>>2);                // 计算出 Page 偏移  除以4 (每个page能放4条记录)
     InPageoffset=BDSD.write-(u32)(pageoffset<<2);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<7);           // 计算出页内 字节   乘以 128 (每个记录128个字节)
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Sector  被移除到下一个Sector  1Sector=8Page  
     {
        WatchDog_Feed();
		SST25V_SectorErase_4KByte((pageoffset+Pos_accuracy_offset)*PageSIZE);      // erase Sector		
		DF_delay_ms(20); 
	    rt_kprintf("\r\n Erase BDSD Sector : %d\r\n",(pageoffset>>3));       
	 }
  //	   2. write  and read back    
  SV_RTRY:
      if(retry>=2)
	  	  return false;
	   delay_ms(5);
	   WatchDog_Feed(); 
	   DF_WriteFlashDirect(pageoffset+Pos_accuracy_offset,InPageAddr,content,saveLen);  //   写入信息
	   DF_delay_us(30);   
       DF_ReadFlash(pageoffset+Pos_accuracy_offset,InPageAddr,rd_back,saveLen);  //   读取信息
  //  compare 
       for(i=0;i<saveLen;i++)
       	{
             if(content[i]!=rd_back[i])
			 {
			     BDSD.write++;		  
		         if(BDSD.write>=Max_BDSD_Num)
			       BDSD.write=0; 
				 if(retry==0)
				 {
				     retry++;
			         goto SV_RTRY;
				 }
				 else
				 {
				    return false;
				 }
             }
       	}
		return true;  
  //-------------------------------------------------------------------------------------------- 
}  


u8  BDSD_ReadCycleGPS(u32 cycleread,u8 *content ,u16 ReadLen)
{
  /*
     NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
  */
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8  i=0,FCS=0;
	u8  Len_read=0;  // 信息长度

  /*
      上报的每一包数第一个字节是有效信息的长度，从第二个字节是信息内容，
      信息内容的后边是一个字节额校验(校验从长度开始到内容最后一个字节)
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
     pageoffset=(u32)(BDSD.read>>2);                 // 计算出 Page 偏移  除以4 (每个page能放4条记录)
     InPageoffset=BDSD.read-(u32)(pageoffset<<2);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset<<7);            // 计算出页内 字节   乘以 128 (每个记录128个字节)
  //   2. Write Record Content 
     DF_ReadFlash(pageoffset+Pos_accuracy_offset,InPageAddr,content,ReadLen); 
     DF_delay_us(20);
	 //  获取信息长度
	 Len_read=content[0];
	 
  if(DispContent==2)
  {
   	 OutPrint_HEX("读取BDSD_GPS 内容为 ",content,Len_read+1);
  }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 ------- 
	  FCS = 0;
	   for ( i = 0; i < Len_read; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *( content + i );  
	   } 
	   //rt_kprintf("\r\n  FCS=%d \r\n",FCS); 
	  if(((content[Len_read]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和   
	    { 	      
		  if(content[0]==0xFF)
		  {
			 rt_kprintf("\r\n  content[0]==0xFF   read=%d,  write=%d  \r\n",BDSD.read,BDSD.write);   
			 //cycle_read=cycle_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。
                       
			  BDSD.read++;	
		        if(BDSD.read>=Max_BDSD_Num)
		  	      BDSD.read=0; 
			 return false;	
		  }  
		  //------------------------------------------------
          BDSD.read++;	
		  if(BDSD.read>=Max_BDSD_Num)
		  	BDSD.read=0;
		  return false; 
     	}	  
	//------------------------------------------------------------------------------------------   	 
	    return true;     
  //-------------------------------------------------------------------------------------------- 
} 

//-----------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------



u8  Common_WriteContent(u32 In_write,u8 *content ,u16 saveLen, u8 Type) 
{
  //-----------------------------------------------------
  u8     reg[1];
  //u8   regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0; 

   //--------------------------------------------------	
    //memset(regStr,0,sizeof(regStr)); 
  //  1.   Classify
    switch(Type)
    {
	
		case TYPE_TiredDrvAdd:
							 Start_offset=TiredDrvStart_offset;
							// memcpy(regStr,"疲劳驾驶",25);
							 break;
		case TYPE_ExpSpdAdd:
							// memcpy(regStr,"超速报警",25);
							 break; 					 
		case TYPE_AccFireAdd:
							// memcpy(regStr,"ACC打火",25);
							 break;
		case TYPE_ErrorLogAdd:
							 //memcpy(regStr,"异常LOG",25); 
							 break; 
		case TYPE_LogInAdd:
							// memcpy(regStr,"登录记录",25);   
							 break; 
		case TYPE_PowerCutAdd:
							// memcpy(regStr,"外部断电记录",25); 
							 break; 
		case TYPE_SettingChgAdd:
							 //memcpy(regStr,"参数修改",25);       
							 break; 					 
		default :
							 return false;		  					 
    }
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		pageoffset=(u32)(In_write>>4);				 // 计算出 Page 偏移  除以16  
		InPageoffset=In_write-(u32)(pageoffset<<4);	 // 计算出 页内偏移地址 
		InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block	1Block=8Page  
		{
        SST25V_SectorErase_4KByte((pageoffset+Start_offset)*PageSIZE);      // erase Sector	
        DF_delay_ms(10);
		//  rt_kprintf("\r\n Common --- Erase Cycle Block : %d\r\n",(pageoffset>>6));    
		}
	 // 	  2. Filter write  area    
		DF_ReadFlash(pageoffset+Start_offset,InPageAddr,reg,1); 
		  if(reg[0]!=0xff)	// 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
		   {
			 In_write++;
			 if(In_write>=Max_CommonNum)
				  In_write=0;  
			         // rt_kprintf("\r\n	 *******   Common 写区域 Write area : %d	is	 Dirity!  \r\n",In_write);  
				 return false;
		   }	   
	 //   3. Write Record Content 
        DF_WriteFlashDirect(pageoffset+Start_offset,InPageAddr,content,saveLen);  //   写入信息
		DF_delay_ms(10);
		//rt_kprintf("\r\n Common Starpageoffset=%d	PageOffset= %d ,  InPageAddr= %d  TYPE= %s \r\n",CycleStart_offset,pageoffset,InPageoffset,regStr);  

     //   4. end  
        switch(Type) 
    {
	
		case TYPE_TiredDrvAdd:							 
							 TiredDrv_write=In_write;
							 break;
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_write=In_write;
							 break; 					 
		case TYPE_AccFireAdd:
							 OnFireRec_write=In_write;
							 break;
		case TYPE_AvrgSpdAdd:
							 AvrgSpdPerMin_write=In_write;
							 break;
		case TYPE_ErrorLogAdd:
							 ErrorLog_write=In_write;  
							 break; 
		case TYPE_LogInAdd:
							 Login_write=In_write;
							 break; 
		case TYPE_PowerCutAdd:
							 Powercut_write=In_write;
							 break; 
		case TYPE_SettingChgAdd:
							 Settingchg_write=In_write;     
							 break; 					 
		default :
							 return false;	 						 
    }

	return true; 
	     	     
}


u8  Common_ReadContent(u32 In_read,u8 *content ,u16 ReadLen, u8 Type) 
{
  //-----------------------------------------------------
  //u8    regStr[25];   
  //-----------------------------------------------------
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u32  Start_offset=0;     
	u8   i=0,FCS=0;;
   //--------------------------------------------------	
 
	// memset(regStr,0,sizeof(regStr)); 
   //  1.	Classify
	 switch(Type)
	 {
	 
		 case TYPE_TiredDrvAdd:
							  Start_offset=TiredDrvStart_offset;
							//  memcpy(regStr,"疲劳驾驶",25);
							  break;
		 case TYPE_ExpSpdAdd:
							//  memcpy(regStr,"超速报警",25);
							  break;					  
		 case TYPE_AccFireAdd:
							//  memcpy(regStr,"ACC打火",25);
							  break;
		 case TYPE_ErrorLogAdd:
							//  memcpy(regStr,"异常LOG",25); 
							  break; 
		  case TYPE_LogInAdd:
							//   memcpy(regStr,"登录记录",25);   
							   break; 
		  case TYPE_PowerCutAdd:
							//   memcpy(regStr,"外部断电记录",25); 
							   break; 
		  case TYPE_SettingChgAdd:
							 //  memcpy(regStr,"参数修改",25);	   
							   break;					   
		 					  
		 default :
							  return false; 						  
	 } 
 
	//----------------------------------------------------------------------------------------------
	//	 2. caculate address 
	 
		   pageoffset=(u32)(In_read>>4);				  // 计算出 Page 偏移  除以64  
		   InPageoffset=In_read-(u32)(pageoffset<<4);   // 计算出 页内偏移地址 
		   InPageAddr=(u16)(InPageoffset<<5);			 // 计算出页内 字节   乘以 32 (每个记录32个字节)
		  //	 2. Write Record Content 
		   DF_ReadFlash(pageoffset+Start_offset,InPageAddr,content,ReadLen);    
		   DF_delay_us(10);
		 //  rt_kprintf("\r\n  Common Starpageoffset=%d	PageOffset= %d ,  InPageAddr= %d  TYPE= %s\r\n",Start_offset,pageoffset,InPageoffset,regStr);  
		 if(DispContent)
		 {
			   rt_kprintf("\r\n  读取Common 内容为 :\r\n");  
				for(i=0;i<ReadLen;i++)
				  rt_kprintf("%2X ",content[i]);  
			   rt_kprintf("\r\n");   
		 }   
		//	3. Judge FCS  
		  //--------------- 过滤已经发送过的信息 -------
			FCS = 0;
			 for ( i = 0; i < ReadLen-1; i++ )	 //计算读取信息的异或和
			 {
					 FCS ^= *( content + i );  
			 }				
			 if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))  // 判断异或和 
			  { 
			   
			   if(content[0]==0xFF)
			   {
				  //如果是内容是0xFF ，读指针和写指针相等，不再触发上报。	
				         switch(Type)
					    {
						
							case TYPE_TiredDrvAdd:							 
												 TiredDrv_read=TiredDrv_write;
												 break;
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							case TYPE_AccFireAdd:
												 OnFireRec_read=OnFireRec_write;
												 break;
							case TYPE_AvrgSpdAdd:
												 AvrgSpdPerMin_Read=AvrgSpdPerMin_write;
												 break;
							case TYPE_ErrorLogAdd:
												 ErrorLog_Read=ErrorLog_write;        
												 break; 
							 case TYPE_LogInAdd:
												 Login_Read=Login_write;
												  break; 
							 case TYPE_PowerCutAdd:
												 Powercut_read=Powercut_write;
												  break; 
							 case TYPE_SettingChgAdd:
							 	                 Settingchg_read=Settingchg_write;
												  break;					  
												 
							default :
												 return false;							 
					    
				         	}	
						return false; 
			   }
				In_read++; 
				if(In_read>=Max_CycleNum)
				  In_read=0;
				rt_kprintf("\r\n   *******	该条记录内容不对 *******  \r\n");  
				return false;
			  }
			 if(content[0]==0xFF)
			 {
               rt_kprintf("\r\n  读取内容为 0xFF \r\n");
			        switch(Type)
					    {
						
							case TYPE_TiredDrvAdd:							 
												 TiredDrv_read=TiredDrv_write;
												 break;
							case TYPE_ExpSpdAdd:
												 ExpSpdRec_read=ExpSpdRec_write;
												 break; 					 
							case TYPE_AccFireAdd:
												 OnFireRec_read=OnFireRec_write;
												 break;
							case TYPE_AvrgSpdAdd:
												 AvrgSpdPerMin_Read=AvrgSpdPerMin_write;
												 break;
							case TYPE_ErrorLogAdd:
												 ErrorLog_Read=ErrorLog_write;        
												 break; 
							 case TYPE_LogInAdd:
												 Login_Read=Login_write;
												  break; 
							 case TYPE_PowerCutAdd:
												 Powercut_read=Powercut_write;
												  break; 
							 case TYPE_SettingChgAdd:
												 Settingchg_read=Settingchg_write;
												  break;					  
												 
							default :
												 return false;							 
					    
				         	}	
               return false;
			 } 
			 
			 	
     //   4. end  
        switch(Type)
    {
	
		case TYPE_TiredDrvAdd:							 
							 TiredDrv_read=In_read;
							 break;
		case TYPE_ExpSpdAdd:
							 ExpSpdRec_read=In_read;
							 break; 					 
		case TYPE_AccFireAdd:
							 OnFireRec_read=In_read;
							 break;
		case TYPE_AvrgSpdAdd:
							 AvrgSpdPerMin_Read=In_read;
							 break;
		case TYPE_ErrorLogAdd:
							 ErrorLog_Read=In_read;        
							 break; 
		 case TYPE_LogInAdd:
							 Login_Read=In_read;
							  break; 
		 case TYPE_PowerCutAdd:
							 Powercut_read=In_read;
							  break; 
		 case TYPE_SettingChgAdd:
		 	                 Settingchg_read=In_read; 
							  break;					  
							 
		default :
							 return false;							 
    }

	return true; 
	     	     
}


u8 Save_PerMinContent(u32 In_wr,u8 *content ,u16 saveLen)
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   reg[1]={0};
  //----------------------------------------------------------------------------------------------
  //   1. Judge  Whether  needs to Erase page 
  
     pageoffset=(u32)(In_wr/7);                // 计算出 Page 偏移  除以7  
     InPageoffset=In_wr-(u32)(pageoffset*7);   // 计算出 页内偏移地址 
     InPageAddr=(u16)(InPageoffset*70);           // 计算出页内 字节   乘以 70 (每个记录70个字节)    
     if(((pageoffset%8)==0)&&(InPageoffset==0))  // 判断是否需要擦除Block  被移除到下一个Block  1Block=8Page  
     {
	  SST25V_SectorErase_4KByte((pageoffset+AverageSpdStart_offset)*PageSIZE);	  // erase Sector 
	  DF_delay_ms(10);
	  // rt_kprintf("\r\n Erase Cycle Block : %d\r\n",(pageoffset>>6));    
	 }
  //	   2. Filter write  area    
      DF_ReadFlash(pageoffset+AverageSpdStart_offset,InPageAddr,reg,1);
	   if(reg[0]!=0xff)  // 如果要写入的区域 dirty  ，则地址增然后从新开始寻找知道找到为止
	   	{
	   	  In_wr++;
		  AvrgSpdPerMin_write++;		  
		  if(AvrgSpdPerMin_write>=Max_SPDSperMin)
			   AvrgSpdPerMin_write=0;  
		 // rt_kprintf("\r\n    *******  每分钟速度 写区域 Write area : %d   is   Dirity!  \r\n",AvrgSpdPerMin_write);  
          //--------------------------------
		  	return false;
	   	}       
  //   3. Write Record Content 
     DF_WriteFlashDirect(pageoffset+AverageSpdStart_offset,InPageAddr,content,saveLen);  //   写入信息
     DF_delay_ms(5);
	// rt_kprintf("\r\n Write PerMit Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d \r\n",AverageSpdStart_offset,pageoffset,InPageoffset); 
		return true; 
  //-------------------------------------------------------------------------------------------- 
}

u8 Read_PerMinContent(u32 In_read,u8 *content ,u16 ReadLen) 
{
	/*
	   NOTE : Flash  1 page = 512 Bytes  ; 1 Record = 32 Bytes ;  1page= 16 Records   1Sector=8Page=128Records
	*/
    u32  pageoffset=0;   //Page 偏移
    u32  InPageoffset;   //页内Record偏移
    u16  InPageAddr=0;   //页内 地址偏移 
	u8   FCS=0;
	u16  i=0;  

  /*
      上报的每一包数据位31个字节， 每条记录为32个字节，用每条记录的最后一个字节来作为是否上报过的标志位。没有上报该标志为0xFF
      上报过后该标志为被写为0x01
  */
  //----------------------------------------------------------------------------------------------
  //   1. caculate address 
  pageoffset=(u32)(In_read/7);				 // 计算出 Page 偏移  除以7  
  InPageoffset=In_read-(u32)(pageoffset*7);	 // 计算出 页内偏移地址  
  InPageAddr=(u16)(InPageoffset*70);		   // 计算出页内 字节	乘以 70 (每个记录70个字节)	  
  //   2. Write Record Content 
    DF_ReadFlash(pageoffset+AverageSpdStart_offset,InPageAddr,content,ReadLen);     
    DF_delay_us(10);
	// rt_kprintf("\r\n   Read PerMintSPD  Starpageoffset=%d  PageOffset= %d ,  InPageAddr= %d   每分钟平均速度 \r\n",AverageSpdStart_offset,pageoffset,InPageoffset);  
   if(DispContent==2)
   {
	rt_kprintf("\r\n  >>>>>>>>>>读取每分钟平均速度   PerMintSPD 内容为 :\r\n");       
   	  for(i=0;i<ReadLen;i++)
   	  	rt_kprintf("%2X ",content[i]);   
   	 rt_kprintf("\r\n");    
   }	 
  //  3. Judge FCS	
	//--------------- 过滤已经发送过的信息 -------
	  FCS = 0;
	   for ( i = 0; i < ReadLen-1; i++ )   //计算读取信息的异或和
	   {
			   FCS ^= *(content + i );  
	   }			  
	   if(((content[ReadLen-1]!=FCS)&&(content[0]!=0xFF))||(content[0]==0xFF))	// 判断异或和 
	    { 
	    
		    if(content[0]==0xFF)
	 		{             
	 		       rt_kprintf("\r\n  读取内容为0xFF --1\r\n"); 	 		    
				   AvrgSpdPerMin_Read=AvrgSpdPerMin_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。   		  
	               return false; 
			} 
          AvrgSpdPerMin_Read++;	
		  if(AvrgSpdPerMin_Read>=Max_SPDSperMin) 
		  	AvrgSpdPerMin_Read=0; 
		  //rt_kprintf("\r\n   ******* PerMit  该条记录内容不对 *******  \r\n");    
		  return false; 
     	}  
	    if(content[0]==0xFF)
 		{             
 		       rt_kprintf("\r\n  读取内容为0xFF \r\n"); 	 		    
			   AvrgSpdPerMin_Read=AvrgSpdPerMin_write;//如果是内容是0xFF ，读指针和写指针相等，不再触发上报。   		  
               return false; 
		} 
	//------------------------------------------------------------------------------------------  
    return true; 
    //-------------------------------------------------------------------------------------------- 
}
//----------------------------------------------------------------------  
void  CHK_ReadCycle_status(void) 
{  
   if(BDSD.Enable_Working==0)
   	  return;
   
  		if(RdCycle_Idle==BDSD.SendFlag)
			  {
			       
				   if(BDSD.write!=BDSD.read)
					 {
						BDSD.SendFlag=RdCycle_RdytoSD; 
					 }
				    else
						BDSD.SendFlag=RdCycle_Idle; 
				
			  } 
}


void MediaIndex_Init(void)  
{
   u8 i=0;
   
   memset((u8*)&MediaIndex,0,sizeof(MediaIndex)); 
   
   for(i=0;i<8;i++)
   {
     MediaIndex.ID=i+1;
	if(i==0)
     {
       MediaIndex.Type=0;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"pic.jpg",7);
       DF_WriteFlashSector(DF_PicIndex_Page,0,(u8*)&MediaIndex,sizeof(MediaIndex));  
	   DF_delay_ms(50);
	   MediaIndex.Type=1;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"sound.wav",9);
       DF_WriteFlashSector(DF_SoundIndex_Page,0,(u8*)&MediaIndex,sizeof(MediaIndex));  
	   DF_delay_ms(50);
	 }  
	else 
     {
       MediaIndex.Type=0;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"pic.jpg",7);
       DF_WriteFlashDirect(DF_PicIndex_Page+i, 0,(u8*)&MediaIndex, sizeof(MediaIndex));    
       DF_delay_ms(10);    
	   MediaIndex.Type=1;
	   memset(MediaIndex.FileName,0,sizeof(MediaIndex.FileName));
	   memcpy(MediaIndex.FileName,"sound.wav",9);
       DF_WriteFlashDirect(DF_SoundIndex_Page+i,0,(u8*)&MediaIndex,sizeof(MediaIndex));      
	   DF_delay_ms(10);
	 }  
   }
}

void Save_Common(u32 In_write,u8 Type)
{
  u8 content[32],regDateTime[6];
  u8 wr_add=0,FCS=0,i;

  
  /*
   if(TYPE_AccFireAdd==Type)   //   第30字节
   	{
      content[wr_add++]=OnFire_Status+0x30;
   	}
   else
   	{      
      content[wr_add++]=0xFF;
   	}
  */
  
   //-----------------
    memset(content,0,sizeof(content));
    wr_add=0; 
   //-----------------
   switch(Type)
   	{
   	   
	   case	 TYPE_ExpSpdAdd:  // 附录A 中没有超速记录报警，但是我们有
	                           break;
	   case  TYPE_TiredDrvAdd:
		                      break;
       
	   case  TYPE_AccFireAdd:	 // 附录A 中没有汽车打火记录报警，但是我们有
						  break;
							  
	  case  TYPE_LogInAdd:
	  case  TYPE_PowerCutAdd: 
	  case	TYPE_SettingChgAdd:
	  	                        Time2BCD(regDateTime);  
	                            memcpy(content+wr_add,regDateTime,6);
							    wr_add+=6; 
								memcpy(content+wr_add,JT808Conf_struct.Driver_Info.DriverCard_ID,18);
							    wr_add+=18;
								switch(Type)  
									{
                                       case  TYPE_LogInAdd:  
									   	                     content[wr_add++]=Login_Status;
															 break;
									   case  TYPE_PowerCutAdd: 
									   	                     content[wr_add++]=Powercut_Status;
															 break;
									   case	TYPE_SettingChgAdd:
									   	                     content[wr_add++]=Settingchg_Status;
															 break;
									}
								break;
     default:
	 	         rt_kprintf("\r\n  Save common Type Error!\r\n");
	 	         return;
   	}
   //-----------------------
   FCS = 0;
   for ( i = 0; i < 32; i++ )  
   {
		 FCS ^=content[i];
   }			  //求上边数据的异或和
   content[wr_add++] = FCS;	  // 第31字节    
    if(DispContent)
   rt_kprintf("\r\n Common  WriteLen:  %d  \r\n ",wr_add);   
   Common_WriteContent( In_write, content, wr_add, Type);       	
}

