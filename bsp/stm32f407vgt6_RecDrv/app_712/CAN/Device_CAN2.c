/*
       Device  CAN    2  
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
#include "Device_CAN.h"
#include <finsh.h>

u8   U3_Rx[100];
u8   U3_content[100];
u16   U3_content_len=0; 
u8   U3_flag=0;
u16   U3_rxCounter=0; 
u16 U3_rx_timer=0;




#ifdef RT_USING_DEVICE 
 struct rt_device  Device_CAN2;  
#endif

//     С��ͨ�Ų�����  ������   115200bps

/*
        1.CAN ���

         1    1       1       2    content                     ( IN  content   �ο�JT/T 808  ������12  Page 16)
        7E  33   Type    ID      n             7E

        Type:  01  <->  ����������   
                                    ���� 2 bytes   HL    ��λ KB <=> ID    (û��ID  ��ֱ���ǲ���������)
                  02  <->  JT/T 808 ���� ID   +  ���� 

        2.  ͸��
        7E  34  01    content   7E
*/

u16  Protocol_808_Decode_Good(u8 *Instr ,u8* Outstr,u16  in_len);


void  DeviceData_Encode_Send( u8 DeviceID, u8 CMD_TYPE, u8 *Srcstr, u16 inlen )
{
	u16 out_len = 0, wr = 0, i = 0;
	u8	addFCS	= 0;
	u8	Reg[100];
	u8	reg2[100];

	//  stuff orginal info
	Reg[0]	= 0;                        // ����У��
	Reg[1]	= 0x00;                     // �汾���
	Reg[2]	= 0x01;
	Reg[3]	= 0x01;                     // ����ID
	Reg[4]	= 0x00;
	Reg[5]	= DeviceID;                 // ��������
	Reg[6]	= CMD_TYPE;                 // ��������
	memcpy( Reg + 7, Srcstr, inlen );   //  ��Ϣ����
	wr = 7 + inlen;

	// caculate  add fcs
	for( i = 3; i < wr; i++ )
	{
		Reg[0] += Reg[i];
	}
	rt_kprintf( "\r\n // Send  -------------------------" );
	OutPrint_HEX( "ԭʼ��Ϣ", Reg, wr );
	//  Encode 1       reg2 is  A.2.1  ��ʽ����
	memset( reg2, 0, 100 );
	reg2[0]		= 0x7e;                                                 // head
	i			= Protocol_808_Encode( reg2 + 1, Reg, wr );             // Encode content
	reg2[i + 1] = 0x7e;                                                 // tail
	OutPrint_HEX( "u3������Ϣ", reg2, i + 2 );
	rt_device_write( &Device_CAN2, 0, ( const char*)reg2, i + 2  );
	rt_kprintf( "// Send over -----------------------\r\n" );

}



void U3_RxProcess(void)  
{
   u8  iRX;
	uint16_t	i,j, len;
   uint8_t fcs;
   uint8_t buf[32]; 

       // 1.  Debug  out
             
	   rt_kprintf( "\r\n // U3 _RX  -------------------------" );	   
	   OutPrint_HEX( "U3���յ�������", U3_Rx,U3_rxCounter ); 
       rt_kprintf( "\r\n // U3 _end -------------------------" );  	 
	   
       //  2. normal   process
       
		   /*'5',1,<ת����ic������,��0x7e���>*/
		   len=Protocol_808_Decode_Good( U3_Rx, U3_content,U3_rxCounter); 
		   rt_kprintf("\r\n******************\r\n");
		   OutPrint_HEX( "U3 ��ת������", U3_content,len ); 
		   rt_kprintf("\r\n******************");
	   
		   if(U3_content[0]!=0x7e) return;
		   if(U3_content[len-1]!=0x7e) return; 
		   /*�����ۼӺ�*/
		   fcs=0;
		   //for(i=4;i<len-1;i++)  fcs+=U3_content[i]; 
		   //if(fcs!=U3_content[1])
		   //{
			   //rt_kprintf("\r\n%s(line:%d)>",__func__,__LINE__);
		   //}
		   if(U3_content[6]!=0x0B) return; // IC �����ͼ�飬�����򷵻�
		   
		   switch(U3_content[7])				   
			 {	
				 case		  0x40:  // �״β忨����Ϣ
							   {	// 40H	 
									 switch(U3_content[8])
									   {
												  case 0x00:
												 rt_kprintf("\r\n IC �������ɹ�\r\n");
												 IC_MOD.IC_Status=1;
												 JT808Conf_struct.Driver_Info.BD_IC_rd_res=0x00;
												 time_now=Get_RTC(); 
												 Time2BCD(JT808Conf_struct.Driver_Info.BD_IC_inoutTime);  
						   
						   if( DataLink_Status( ) && ( TCP2_Connect ) )
						   {													   //  Online	   Trans  64 Data  to  Centre  , wait for 1 or 25 byte result,
							   memset( IC_MOD.IC_Tx40H, 0, sizeof( IC_MOD.IC_Tx40H ) );
							   memcpy( IC_MOD.IC_Tx40H, U3_content + 9, 64 );  //��ȡ64���ֽڵĿ�Ƭ��Ϣ
							   IC_MOD.Trans_0900Flag = 1;					   // ����͸��
							   rt_kprintf( "\r\n IC get 64 Bytes\r\n" );
							   return;
						   }else
						   {													   //Off line
							   buf[0] = 0x01;
							   DeviceData_Encode_Send( 0x0B, 0x40, buf, 1 );
							   TTS_Get_Data("��δ��������",12);
							   return;
						   }
												 break;
					   case 0x01:  /*�ȴ�20���ӣ�ʹ��0x43��������������ȡ*/
						   rt_kprintf( "\r\n IC��δ����\r\n" );
	   
						   time_now 								   = Get_RTC( );
												 Time2BCD(JT808Conf_struct.Driver_Info.BD_IC_inoutTime);  
																				   //  send back
												 buf[0]=0x03;
												 DeviceData_Encode_Send(0x0B,0x40,buf,1);
												 break; 			 
												 case 0x02:
						   rt_kprintf( "\r\n IC����ȡʧ��\r\n" );
												  //  send back
												 buf[0]=0x03;
												 DeviceData_Encode_Send(0x0B,0x40,buf,1);
						   TTS_Get_Data("IC����ȡʧ��",12);
												 break;
										case 0x03:
												 rt_kprintf("\r\n �Ǵ�ҵ�ʸ�֤��\r\n");
												  //  send back
												 buf[0]=0x03;
												 DeviceData_Encode_Send(0x0B,0x40,buf,1);
						   TTS_Get_Data("�Ǵ�ҵ�ʸ�֤��",14);
												 break; 		 
										case 0x04:
						   rt_kprintf( "\r\n IC��������\r\n" );
												 //  send back
												 buf[0]=0x03; 
												 DeviceData_Encode_Send(0x0B,0x40,buf,1); 
						   JT808Conf_struct.Driver_Info.BD_IC_rd_res   = 0x01;
						   SD_ACKflag.f_DriverInfoSD_0702H = 1;    // ʹ���ϱ�
	   
						   TTS_Get_Data("IC��������",10);
						   break;
				   }
			   }
			   break;
			   case    0x41:								   //  IC �ڼ�ʻԱ����Ϣ
			   {
				   /*
				   7E 55 00 01 01 00 0B 41 00 
				   06 C2 DE B3 A4 C0 D6 
				   36 32 30 31 32 33 31 39 37 33 30 35 30 33 39 31 31 32 00 00 
				   14 C0 BC D6 DD CA D0 B9 AB C2 B7 D4 CB CA E4 B9 DC C0 ED B4 A6 
				   20 15 05 01 
				   7E 
				   */
	   
				   if( U3_content[8] == 0x00 )				   // ��ȡ��ʻ�����Ϣ����
				   {
				   
					   rt_kprintf( "\r\n ��ȡ��ʻԱ�����Ϣ�ɹ�\r\n" );
					   JT808Conf_struct.Driver_Info.BD_IC_status=0x01;
					   i=*(U3_content + 9);    /*��������*/
					   JT808Conf_struct.Driver_Info.BD_DriveName_Len=i;    /*��������*/
					   memcpy(JT808Conf_struct.Driver_Info.BD_DriveName,U3_content + 10,i);
					   JT808Conf_struct.Driver_Info.BD_DriveName[i]=0;
					   memcpy(JT808Conf_struct.Driver_Info.BD_Drv_CareerID,U3_content + 10+i,20);
					   j=*(U3_content + 30+i); /*��֤���س���*/
					   JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len=j;
					   memcpy(JT808Conf_struct.Driver_Info.BD_Confirm_agentID,U3_content + 31+i,j);
					   JT808Conf_struct.Driver_Info.BD_Confirm_agentID[j]=0;
					   memcpy(JT808Conf_struct.Driver_Info.BD_ExpireDate,U3_content + 31+i+j,4);
					   
					   rt_kprintf("\r\n--����:%s",JT808Conf_struct.Driver_Info.BD_DriveName);
					   rt_kprintf("\r\n--֤����:%s",JT808Conf_struct.Driver_Info.BD_Drv_CareerID);
					   rt_kprintf("\r\n--��֤����:%s",JT808Conf_struct.Driver_Info.BD_Confirm_agentID);
					   memcpy(buf,JT808Conf_struct.Driver_Info.BD_ExpireDate,4);
					   rt_kprintf("\r\n--��Ч����:%02x%02x-%02x-%02x",buf[0],buf[1],buf[2],buf[3]);
					   
					   memcpy( IC_MOD.IC_TX41H, U3_content + 9, len - 9 );
					   IC_MOD.IC_TX41H_len			   = len - 9;
					   SD_ACKflag.f_DriverInfoSD_0702H = 1;    // ʹ���ϱ�
				   }
				   else
				   {
					   rt_kprintf( "\r\n IC����ʻԱ��Ϣ��ȡʧ��\r\n" );
					   TTS_Get_Data("IC����ʻԱ��Ϣ��ȡʧ��",22);
				   }
				   //  send back
				   DeviceData_Encode_Send( 0x0B, 0x41, NULL, 0 );
				   return;
			   }
			   break;
			   case    0x42:								   // ��Ƭ�γ�֪ͨ
			   {
				   time_now = Get_RTC( );
				   Time2BCD( JT808Conf_struct.Driver_Info.BD_IC_inoutTime );
				   IC_MOD.IC_Status = 0;
				   if( DataLink_Status( ) )
				   {
					   JT808Conf_struct.Driver_Info.BD_IC_rd_res   = 0x00;
					   SD_ACKflag.f_DriverInfoSD_0702H = 1;    // ʹ���ϱ�
				   }
				   // send back
				   DeviceData_Encode_Send( 0x0B, 0x42, NULL, 0 );
				   return;
			   }
		   }
			
       

}

u16  Protocol_808_Decode_Good(u8 *Instr ,u8* Outstr,u16  in_len)  // ����ָ��buffer :  UDP_HEX_Rx  
{
	//-----------------------------------
	  u16 i=0, decode_len=0;

    // 1.  clear  write_counter
	  decode_len=0;//clear DecodeLen

	// 2   decode process   
	for(i=0;i<in_len;i++)
	 {
		if((Instr[i]==0x7d)&&(Instr[i+1]==0x02))
		{
		   Outstr[decode_len]=0x7e;
		   i++;
		}
		else
		if((Instr[i]==0x7d)&&(Instr[i+1]==0x01))
		{
		   Outstr[decode_len]=0x7d;
		   i++;
		}
		else  
		{
		  Outstr[decode_len]=Instr[i];    
		}
	    decode_len++;
	 }	
    //  3.  The  End
        return decode_len;
}

void  U3_rx_timeout(void)
{//10ms
     if(U3_flag==1)
     {
         U3_rx_timer++;
		 if(U3_rx_timer>12) 
		 	{
                	  
               U3_RxProcess();
						   
			   U3_flag=0;
			   U3_rxCounter=0;
			   U3_rx_timer=0;
		 	}		 
     }
}
	

void U3_RxHandler(unsigned char rx_data)
{
     //if(((rx_data==0x40)||(rx_data==0x41)||(rx_data==0x42))&&(U3_flag==0))
     if(U3_flag==0)
      	{
      	   U3_Rx[U3_rxCounter++]=rx_data;
      	   U3_flag=1;
		   U3_rx_timer=0;
      	}	
	 else
	  if(U3_flag==1)
	  	{
            U3_Rx[U3_rxCounter++]=rx_data;
	  	}
	  else
	  	 U3_rxCounter=0;
}

void CAN2_putc(char c)
{
	//USART_SendData(USART3,  c); 
	while (!(USART3->SR & USART_FLAG_TXE));  
	USART3->DR = (c & 0x1FF);    
	 //	USART_SendData( USART3, c );
	//	while( USART_GetFlagStatus( USART3, USART_FLAG_TC ) == RESET )  
	//	{
	//	}   
	//	while( USART_GetFlagStatus( USART3, USART_FLAG_TC )  == RESET )   
		//{
	//	}
		//USART_SendData( USART3, c );

}

static rt_err_t   Device_CAN2_init( rt_device_t dev )
{
      GPIO_InitTypeDef  GPIO_InitStructure;
      USART_InitTypeDef USART_InitStructure;    
     NVIC_InitTypeDef NVIC_InitStructure;	  


       //  1 . Clock	  
	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Enable USART3 clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

      //   2.  GPIO    
       	/* Configure USART3 Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure USART3 Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	    /* Connect alternate function */
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
      GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);  

     //  3.  Interrupt
	/* Enable the USART3 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	
    //   4.  uart  Initial
       USART_InitStructure.USART_BaudRate = 9600;    //CAN2    
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); 

	/* Enable USART */
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag( USART3, USART_FLAG_TC );     
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);           

    //  5.  Serial 2  power
         RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	 

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;  
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL; 	 
   //	OUT
	//------------------- PE7 -----------------------------
	GPIO_InitStructure.GPIO_Pin	 = GPIO_Pin_7;				//------���ſ���״̬  0 ��Ч  ��̬��Ϊ��   
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;   //���ֻ��ɲ�����Ǿ���PE5��ɲ������ 
	GPIO_Init(GPIOE, &GPIO_InitStructure); 

     GPIO_SetBits(GPIOE,GPIO_Pin_7);  // ��RS232  ����  

	return RT_EOK;
}

static rt_err_t Device_CAN2_open( rt_device_t dev, rt_uint16_t oflag )  
{
         return RT_EOK;
}
static rt_err_t Device_CAN2_close( rt_device_t dev )
{
        return RT_EOK;
}

static rt_size_t Device_CAN2_read( rt_device_t dev, rt_off_t pos, void* buff, rt_size_t count )
{

        return RT_EOK;
}

static rt_size_t Device_CAN2_write( rt_device_t dev, rt_off_t pos, const void* buff, rt_size_t count )
 {
        unsigned int  Info_len485=0;
	 const char		*p	= (const char*)buff;
	

	Info_len485=(unsigned int)count; 
    	/* empty console output */
		//--------  add by  nathanlnw ---------
  while (Info_len485)
	{
		CAN2_putc (*p++);   
		Info_len485--;
	}
       //--------  add by  nathanlnw  --------	
        return RT_EOK;
  }
static rt_err_t Device_CAN2_control( rt_device_t dev, rt_uint8_t cmd, void *arg )
{
     return RT_EOK;
}


void  Device_CAN2_regist(void ) 
{
       Device_CAN2.type	= RT_Device_Class_Char;
	Device_CAN2.init	=   Device_CAN2_init;
	Device_CAN2.open	=  Device_CAN2_open; 
	Device_CAN2.close	=  Device_CAN2_close;
	Device_CAN2.read	=  Device_CAN2_read;
	Device_CAN2.write	=  Device_CAN2_write;
	Device_CAN2.control =Device_CAN2_control;

	rt_device_register( &Device_CAN2, "CAN2", RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_STANDALONE );
	rt_device_init( &Device_CAN2 );
}
