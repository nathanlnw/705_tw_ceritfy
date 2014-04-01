/*
     Protocol_808.C
 */

#include <rtthread.h>
#include <rthw.h>
#include "stm32f4xx.h"
#include "usart.h"
#include "board.h"
#include <serial.h>
#include "math.h"
#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include "App_moduleConfig.h"
#include "math.h"
#include "stdarg.h"
#include "string.h"

#define   SYSID 0x037a

#define    ROUTE_DIS_Default 0x3F000000

#define   BLIND_NUM        10100  
#define   MQ_PKNUM 20

//----   ��ý�巢��״̬ -------
_Media_SD_state Photo_sdState;      //  ͼƬ����״̬
_Media_SD_state Sound_sdState;      //��������
_Media_SD_state Video_sdState;      //��Ƶ����
_Media_SD_state DrvRecoder_sdState; //�г���¼�Ƿ���״̬
_Media_SD_state BlindZone_sdState;  //ä������

//------ Photo -----
u32 PicFileSize = 0;                // ͼƬ�ļ���С
u8	PictureName[40];

//------  voice -----
u8	local_trig	= 0;                //  ���ض�ʱ�ϱ�
u32 local_timer = 0;                // ���ض�ʱ������

//------  video  --------
u8 line_warn_enable = 0;            // ʹ��·�߱���


/*
             ��
 */
//------ phone
u8 CallState = CallState_Idle;      // ͨ��״̬

//   ASCII  to   GB    ---- start
//0-9        10
u8 arr_A3B0[20] = { 0xA3, 0xB0, 0xA3, 0xB1, 0xA3, 0xB2, 0xA3, 0xB3, 0xA3, 0xB4, 0xA3, 0xB5, 0xA3, 0xB6, 0xA3, 0xB7, 0xA3, 0xB8, 0xA3, 0xB9 };

//@ A-O      16
u8 arr_A3C0[32] = { 0xA3, 0xC0, 0xA3, 0xC1, 0xA3, 0xC2, 0xA3, 0xC3, 0xA3, 0xC4, 0xA3, 0xC5, 0xA3, 0xC6, 0xA3, 0xC7, 0xA3, 0xC8, 0xA3, 0xC9, 0xA3, 0xCA, 0xA3, 0xCB, 0xA3, 0xCC0, 0xA3, 0xCD, 0xA3, 0xCE, 0xA3, 0xCF };

//P-Z         11��
u8 arr_A3D0[22] = { 0xA3, 0xD0, 0xA3, 0xD1, 0xA3, 0xD2, 0xA3, 0xD3, 0xA3, 0xD4, 0xA3, 0xD5, 0xA3, 0xD6, 0xA3, 0xD7, 0xA3, 0xD8, 0xA3, 0xD9, 0xA3, 0xDA };

//.  a-0       16
u8 arr_A3E0[32] = { 0xA3, 0xE0, 0xA3, 0xE1, 0xA3, 0xE2, 0xA3, 0xE3, 0xA3, 0xE4, 0xA3, 0xE5, 0xA3, 0xE6, 0xA3, 0xE7, 0xA3, 0xE8, 0xA3, 0xE9, 0xA3, 0xEA, 0xA3, 0xEB, 0xA3, 0xEC, 0xA3, 0xED, 0xA3, 0xEE, 0xA3, 0xEF };

//p-z          11
u8 arr_A3F0[22] = { 0xA3, 0xF0, 0xA3, 0xF1, 0xA3, 0xF2, 0xA3, 0xF3, 0xA3, 0xF4, 0xA3, 0xF5, 0xA3, 0xF6, 0xA3, 0xF7, 0xA3, 0xF8, 0xA3, 0xF9, 0xA3, 0xFA };
//-------  ASCII to GB ------

u8	spd_dex[420];
u8	Latiude_hex[420];

u8  _700H_buffer[700]; 


/*
   //------------------ ��λ����process    --------------------------------------------
   u16 spd_dex[420]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
                  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,8,17,26,35,
                  44,53,62,71,80,89,98,107,116,125,134,143,152,161,170,179,188,197,206,
                  215,224,233,242,252,261,270,279,288,297,306,315,324,333,342,351,360,
                  369,378,387,396,405,414,423,432,441,450,459,468,477,486,495,504,513,
                  522,531,540,549,558,567,576,585,594,603,612,621,630,639,648,657,666,
                  675,684,693,702,711,720,729,738,747,756,765,774,783,792,801,810,819,
                  828,837,846,855,864,873,882,891,900,909,918,927,936,945,954,963,972,
                  981,990,999,1008,1017,1026,1035,1044,1053,1062,1071,1080,1080,1080,1080
                  ,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,1080,
                  1080,1080,1080,1080,1080,1071,1062,1053,1044,1035,1026,1017,1008,999,990,
                  981,972,963,954,945,936,927,918,909,900,891,882,873,864,855,846,837,828,819,
                  810,801,792,783,774,765,756,747,738,729,720,711,702,693,684,675,666,657,648,
                  639,630,621,612,603,594,585,576,567,558,549,540,531,522,513,504,495,486,477,
                  468,459,450,441,432,423,414,405,396,387,378,369,360,351,342,333,324,315,306,
                  297,288,279,270,261,252,243,233,224,215,206,197,188,179,170,161,152,143,134,
                  125,116,107,98,89,80,71,62,53,44,35,26,17,8};
   u32 Latiude_hex[420]={0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,
   0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,
   0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,
   0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x2625A00,0x26259FF,0x26259FC,
   0x26259F6,0x26259EE,0x26259E4,0x26259D8,0x26259C9,0x26259B8,0x26259A5,0x2625990,0x2625978,0x262595E,0x2625942,0x2625924,0x2625903,0x26258E0,0x26258BB,0x2625894,0x262586A,0x262583E,
   0x2625810,0x26257E0,0x26257AD,0x2625778,0x2625741,0x2625707,0x26256CC,0x262568E,0x262564E,0x262560B,0x26255C7,0x2625580,0x2625537,0x26254EB,0x262549D,0x262544E,0x26253FB,0x26253A7,
   0x2625350,0x26252F7,0x262529C,0x262523F,0x26251DF,0x262517D,0x2625119,0x26250B3,0x262504A,0x2624FDF,0x2624F72,0x2624F02,0x2624E91,0x2624E1D,0x2624DA6,0x2624D2E,0x2624CB3,0x2624C36,
   0x2624BB7,0x2624B36,0x2624AB2,0x2624A2C,0x26249A4,0x2624919,0x262488D,0x26247FE,0x262476C,0x26246D9,0x2624643,0x26245AB,0x2624511,0x2624474,0x26243D6,0x2624335,0x2624291,0x26241EC,
   0x2624144,0x262409A,0x2623FEE,0x2623F3F,0x2623E8F,0x2623DDC,0x2623D26,0x2623C6F,0x2623BB5,0x2623AF9,0x2623A3B,0x262397A,0x26238B8,0x26237F3,0x262372B,0x2623662,0x2623596,0x26234C8,
   0x26233F8,0x2623325,0x2623250,0x2623179,0x26230A0,0x2622FC5,0x2622EE7,0x2622E07,0x2622D24,0x2622C40,0x2622B59,0x2622A70,0x2622985,0x2622897,0x26227A7,0x26226B5,0x26225C1,0x26224CA,
   0x26223D2,0x26222D7,0x26221D9,0x26220DA,0x2621FD8,0x2621ED4,0x2621DCD,0x2621CC5,0x2621BBA,0x2621AAD,0x262199F,0x2621891,0x2621782,0x2621674,0x2621566,0x2621458,0x262134A,0x262123B,
   0x262112D,0x262101F,0x2620F11,0x2620E03,0x2620CF5,0x2620BE6,0x2620AD8,0x26209CA,0x26208BC,0x26207AE,0x262069F,0x2620591,0x2620483,0x2620375,0x2620267,0x2620159,0x262004A,0x261FF3C,
   0x261FE2E,0x261FD20,0x261FC12,0x261FB03,0x261F9F5,0x261F8E7,0x261F7D9,0x261F6CB,0x261F5BC,0x261F4AE,0x261F3A0,0x261F292,0x261F184,0x261F076,0x261EF67,0x261EE59,0x261ED4B,0x261EC3D,
   0x261EB2F,0x261EA20,0x261E912,0x261E804,0x261E6F6,0x261E5E8,0x261E4DA,0x261E3CB,0x261E2BD,0x261E1AF,0x261E0A1,0x261DF93,0x261DE84,0x261DD76,0x261DC68,0x261DB5A,0x261DA4C,0x261D93D,
   0x261D82F,0x261D721,0x261D613,0x261D505,0x261D3F7,0x261D2E8,0x261D1DA,0x261D0CC,0x261CFBE,0x261CEB0,0x261CDA1,0x261CC93,0x261CB85,0x261CA77,0x261C969,0x261C85B,0x261C74C,0x261C63E,
   0x261C530,0x261C422,0x261C314,0x261C205,0x261C0F7,0x261BFE9,0x261BEDB,0x261BDCD,0x261BCBE,0x261BBB0,0x261BAA2,0x261B994,0x261B886,0x261B778,0x261B669,0x261B55B,0x261B44D,0x261B33F,
   0x261B231,0x261B122,0x261B014,0x261AF06,0x261ADF8,0x261ACEA,0x261ABDB,0x261AACD,0x261A9BF,0x261A8B1,0x261A7A3,0x261A695,0x261A586,0x261A478,0x261A36A,0x261A25C,0x261A14E,0x261A03F,
   0x2619F31,0x2619E23,0x2619D15,0x2619C07,0x2619AFA,0x26199EF,0x26198E6,0x26197E0,0x26196DC,0x26195DA,0x26194DA,0x26193DD,0x26192E2,0x26191E9,0x26190F3,0x2618FFE,0x2618F0C,0x2618E1C,
   0x2618D2F,0x2618C44,0x2618B5B,0x2618A74,0x261898F,0x26188AD,0x26187CD,0x26186EF,0x2618614,0x261853A,0x2618463,0x261838E,0x26182BC,0x26181EC,0x261811E,0x2618052,0x2617F88,0x2617EC1,
   0x2617DFC,0x2617D39,0x2617C79,0x2617BBA,0x2617AFE,0x2617A45,0x261798D,0x26178D8,0x2617825,0x2617774,0x26176C6,0x2617619,0x261756F,0x26174C8,0x2617422,0x261737F,0x26172DE,0x261723F,
   0x26171A3,0x2617108,0x2617070,0x2616FDB,0x2616F47,0x2616EB6,0x2616E27,0x2616D9A,0x2616D10,0x2616C88,0x2616C02,0x2616B7E,0x2616AFC,0x2616A7D,0x2616A00,0x2616986,0x261690D,0x2616897,
   0x2616823,0x26167B1,0x2616742,0x26166D5,0x261666A,0x2616601,0x261659B,0x2616536,0x26164D4,0x2616475,0x2616417,0x26163BC,0x2616363,0x261630D,0x26162B8,0x2616266,0x2616216,0x26161C8,
   0x261617D,0x2616134,0x26160ED,0x26160A8,0x2616066,0x2616026,0x2615FE8,0x2615FAC,0x2615F73,0x2615F3C,0x2615F07,0x2615ED4,0x2615EA4,0x2615E75,0x2615E4A,0x2615E20,0x2615DF9,0x2615DD3,
   0x2615DB1,0x2615D90,0x2615D72,0x2615D55,0x2615D3C,0x2615D24,0x2615D0F,0x2615CFB,0x2615CEB,0x2615CDC,0x2615CD0,0x2615CC5,0x2615CBE,0x2615CB8,0x2615CB5};




 */

//----------- �г���¼�����  -----------------
Avrg_MintSpeed	Avrgspd_Mint;
u32				PerMinSpdTotal	= 0;    //��¼ÿ�����ٶ�����
u8				avgspd_Mint_Wr	= 0;    // ��дÿ����ƽ���ٶȼ�¼�±�
u8				avgspd_Sec_Wr	= 0;    // ��дÿ����ƽ���ٶȼ�¼�±�
u8				avgWriteOver	= 0;    // д�����־λ
u8				AspdCounter		= 0;    // ÿ�����ٶ���Ч����������
u8				Vehicle_sensor	= 0;    // ����������״̬   0.2s  ��ѯһ��


/*
   D7  ɲ��
   D6  ��ת��
   D5  ��ת��
   D4  Զ���
   D3  �����
   D2  ���
   D1  ����
   D0  Ԥ��
 */
u8			Vehicle_sensor_BAK = 0;     // ����������״̬	0.2s  ��ѯһ��

DOUBT_TYPE	Sensor_buf[100];            // 20s ״̬��¼
u8			save_sensorCounter	= 0, sensor_writeOverFlag = 0;;
u32			total_plus			= 0;

//   -------  CAN BD new  --------------
CAN_TRAN	CAN_trans;

u8			Camera_Number	= 1;
u8			DispContent		= 1;    // ����ʱ�Ƿ���ʾ��������


/*
            1 <->  ������ʾ
            2 <->  ��ʾ������Ϣ��
            3 <->  ��ʾ ������������
            0<-> ����ʾ���������ֻ��ʾЭ������
 */

u8	TextInforCounter = 0;           //�ı���Ϣ����

u8	FCS_GPS_UDP = 0;                //UDP ��������
u8	FCS_RX_UDP	= 0;                // UDP ���ݽ���У��

u8	Centre_IP_modify	= 0;        //  ���޸�IP��
u8	IP_change_counter	= 0;        //   �����޸�IP ������
u8	Down_Elec_Flag		= 0;        //   ���Ͷϵ�ʹ�ܱ�־λ

//------------ ���ٱ���---------------------
SPD_EXP		speed_Exd;

GPRMC_PRO	GPRMC_Funs =
{
	Time_pro,
	Status_pro,
	Latitude_pro,
	Lat_NS_pro,
	Longitude_pro,
	Long_WE_pro,
	Speed_pro,
	Direction_pro,
	Date_pro
};

//--------  GPS prototcol----------------------------------------------------------------------------------
static u32	fomer_time_seconds, tmp_time_secnonds, delta_time_seconds;
u8			UDP_dataPacket_flag = 0x03;             /*V	   0X03      ;		   A	  0X02*/
u8			GPS_getfirst		= 0, Shoushi = 0;;  //  �״��о�γ��
u8			HDOP_value			= 99;               //  Hdop ��ֵ
u8			Satelite_num		= 0;                // ���ǿ���
u8			CurrentTime[3];
u8			BakTime[3];
u8			Sdgps_Time[3];                          // GPS ���� ʱ���¼   BCD ��ʽ
u8			gps_log[50];                            //gps log

//static u8      UDP_AsciiTx[1800];
ALIGN( RT_ALIGN_SIZE )
u8 GPRS_info[3000];
u16 GPRS_infoWr_Tx = 0;

ALIGN( RT_ALIGN_SIZE )
u8 UDP_HEX_Rx[1024];                                    // EM310 ��������hex
u16		UDP_hexRx_len		= 0;                        // hex ���� ����
u16		UDP_DecodeHex_Len	= 0;                        // UDP���պ�808 ���������ݳ���

GPS_RMC GPRMC;                                          // GPMC��ʽ
BD_SEND  BDSD;  //    ����˳����


/*                         pGpsRmc->status,\
   pGpsRmc->latitude_value,\
   pGpsRmc->latitude,\
   pGpsRmc->longtitude_value,\
   pGpsRmc->longtitude,\
   pGpsRmc->speed,\
   pGpsRmc->azimuth_angle);
 */

//----------808 Э�� -------------------------------------------------------------------------------------
u16			GPS_Hight		= 0;                        //   808Э��-> �߳�   m
u16			GPS_speed		= 0;                        //   808Э��-> �ٶ�   0.1km/h
u16			GPS_direction	= 0;                        //   808Э��-> ����   ��
u16			Centre_FloatID	= 0;                        //  ������Ϣ��ˮ��
u16			Centre_CmdID	= 0;                        //  ��������ID

u8			Original_info[1024];                        // û��ת�崦��ǰ��ԭʼ��Ϣ
u16			Original_info_Wr = 0;                       // ԭʼ��Ϣд��ַ
//---------- ��GPSУ׼����ϵ����� ----------------------------
u8			Speed_area			= 60;                   // У��Kֵ��Χ
u16			Speed_gps			= 0;                    // ͨ��GPS����������ٶ� 0.1km/h
u8			Speed_Rec			= 0;                    // �ٶȴ����� У��K�õĴ洢��
u16			Speed_cacu			= 0;                    // ͨ��Kֵ����������ٶ�
u16			Spd_adjust_counter	= 0;                    // ȷ������״̬������
u16			Former_DeltaPlus[K_adjust_Duration];        // ǰ�����������
u8			Former_gpsSpd[K_adjust_Duration];           // ǰ������ٶ�
u8			DF_K_adjustState = 0;                       // ����ϵ���Զ�У׼״̬˵��  1:�Զ�У׼��    0:��δ�Զ�У׼
//-----  ��̨ע�ᶨʱ��  ----------
DevRegst	DEV_regist;                                 // ע��
DevLOGIN	DEV_Login;                                  //  ��Ȩ

//------- �ı���Ϣ�·� -------
TEXT_INFO	TextInfo;                                   // �ı���Ϣ�·�
//------- �¼� ----
EVENT		EventObj;                                   // �¼�
EVENT		EventObj_8[8];                              // �¼�
//-------�ı���Ϣ-------
MSG_TEXT	TEXT_Obj;
MSG_TEXT	TEXT_Obj_8[8], TEXT_Obj_8bak[8];

//------ ����  --------
CENTRE_ASK		ASK_Centre;                             // ��������
//------  ��Ϣ�㲥  ---
MSG_BRODCAST	MSG_BroadCast_Obj;                      // ��Ϣ�㲥
MSG_BRODCAST	MSG_Obj_8[8];                           // ��Ϣ�㲥
//------  �绰��  -----
PHONE_BOOK		PhoneBook, Rx_PhoneBOOK;                //  �绰��
PHONE_BOOK		PhoneBook_8[20];

//-----  �������� ------
VEHICLE_CONTROL Vech_Control;                           //  ��������
//-----  ����Χ��  -----
POLYGEN_RAIL	Rail_Polygen;                           // �����Χ��
RECT_RAIL		Rail_Rectangle;                         // ����Χ��
RECT_RAIL       Rail_Rectangle_multi[8]; // ����Χ��
CIRCLE_RAIL     Rail_Cycle_multi[8];     // Բ��Χ��

CIRCLE_RAIL		Rail_Cycle;                             // Բ��Χ��
//------- ��·���� -----
POINT			POINT_Obj;                              // ·�ߵĹյ�
ROUTE			ROUTE_Obj;                              // ·�����
//-------    �г���¼��  -----
RECODER			Recode_Obj;                             // �г���¼��
//-------  ����  ----
CAMERA			Camera_Obj;                             //  �����������
//-----   ¼��  ----
VOICE_RECODE	VoiceRec_Obj;                           //  ¼������
//------ ��ý��  --------
MULTIMEDIA		MediaObj;                               // ��ý����Ϣ
//-------  ������Ϣ͸��  -------
DATATRANS		DataTrans;                              // ������Ϣ͸��
//-------  ����Χ��״̬ --------
INOUT			InOut_Object;                           // ����Χ��״̬
//-------- ��ý�����  ------------
MEDIA_INDEX		MediaIndex;                             // ��ý����Ϣ
//------- ��������״̬ ---------------
u8				CarLoadState_Flag = 1;                  //ѡ�г���״̬�ı�־   1:�ճ�   2:���   3:�س�

//------- ��ý����Ϣ����---------------
u8	Multimedia_Flag = 1;                                //��Ҫ�ϴ��Ķ�ý����Ϣ����   1:��Ƶ   2:��Ƶ   3:ͼ��
u8	SpxBuf[SpxBuf_Size];
u16 Spx_Wr			= 0, Spx_Rd = 0;
u8	Duomeiti_sdFlag = 0;

//------- ¼����ʼ���߽���---------------
u8 Recor_Flag = 1;                                      //  1:¼����ʼ   2:¼������

//----------808Э�� -------------------------------------------------------------------------------------
u8				SIM_code[6];                            // Ҫ���͵�IMSI	����
u8				IMSI_CODE[15]	= "000000000000000";    //SIM ����IMSI ����
u8				Warn_Status[4]	=
{
	0x00, 0x00, 0x00, 0x00
};                                                      //  ������־λ״̬��Ϣ
u8				Car_Status[4] =
{
	0x00, 0x00, 0x00, 0x00
};                                                      //  ����״̬��Ϣ
T_GPS_Info_GPRS Gps_Gprs, Bak_GPS_gprs;
T_GPS_Info_GPRS Temp_Gps_Gprs;

u8   EverySecond_Time_Get=0;                   // ��ȡÿ��ʱ���־  0: not get  1: get  day not change  2 : get  day need change
u8   Lati_Get=0;                               // γ��GGA  �����
u8   Longi_Get=0;                              //  ���� GGA  ���� ��






u8				A_time[6];                              // ��λʱ�̵�ʱ��

u8				ReadPhotoPageTotal	= 0;
u8				SendPHPacketFlag	= 0;                ////�յ���������������һ��blockʱ��λ

//-------- �������� --------
u8	warn_flag			= 0;
u8	f_Exigent_warning	= 0;                            //0;     //�Ŷ� ��������װ�� (INT0 PD0)
u8	Send_warn_times		= 0;                            //   �豸�������ϱ��������������3 ��
u32 fTimer3s_warncount	= 0;

//------  ���ſ������� -------
DOORCamera DoorOpen;                                    //  ���س�������

//------- ������չЭ��  ------------
GNSS_RAW			GNSS_rawdata;                       //  GNSS ��ϸ�����ϱ�
BD_EXTEND			BD_EXT;                             //  ������չЭ��
DETACH_PKG			Detach_PKG;                         // �ְ��ش����
SET_QRY				Setting_Qry;                        //  �ն˲�����ѯ
PRODUCT_ATTRIBUTE	ProductAttribute;                   // �ն�����
HUMAN_CONFIRM_WARN	HumanConfirmWarn;                   // �˹�ȷ�ϱ���

// ---- �յ� -----
u16 Inflexion_Current		= 0;
u16 Inflexion_Bak			= 0;
u16 Inflexion_chgcnter		= 0;                        //�仯������
u16 InflexLarge_or_Small	= 0;                        // �ж�curent �� Bak ��С    0 equql  1 large  2 small
u16 InflexDelta_Accumulate	= 0;                        //  ��ֵ�ۼ�

// ----����״̬  ------------
u8	SleepState		= 0;                                //   0  ������ACC on            1  ����Acc Off
u8	SleepConfigFlag = 0;                                //  ����ʱ���ͼ�Ȩ��־λ

//---- �̶��ļ���С ---
u32 mp3_fsize		= 5616;
u8	mp3_sendstate	= 0;
u32 wmv_fsize		= 25964;
u8	wmv_sendstate	= 0;

//-------------------   ���� ---------------------------------------
static u8	GPSsaveBuf[40];             // �洢GPS buffer
static u8	ISP_buffer[520];
static u16	GPSsaveBuf_Wr = 0;

POSIT		Posit[60];                  // ÿ����λ����Ϣ�洢
u8			PosSaveFlag = 0;            // �洢Pos ״̬λ

NANDSVFlag	NandsaveFlg;
A_AckFlag	Adata_ACKflag;              // ����GPRSЭ�� ������� RS232 Э�鷵��״̬�Ĵ���
TCP_ACKFlag SD_ACKflag;                 // ����GPRSЭ�鷵��״̬��־
u32			SubCMD_8103H	= 0;        //  02 H���� ���ü�¼�ǰ�װ�����ظ� ������
u32			SubCMD_FF01H	= 0;        //  FF02 ������Ϣ��չ
u32			SubCMD_FF03H	= 0;        //  FF03  ������չ�ն˲�������1

u8			SubCMD_10H			= 0;    //  10H   ���ü�¼�Ƕ�λ�澯����
u8			OutGPS_Flag			= 0;    //  0  Ĭ��  1  ���ⲿ��Դ����
u8			Spd_senor_Null		= 0;    // �ֶ��������ٶ�Ϊ0
u32			Centre_DoubtRead	= 0;    //  ���Ķ�ȡ�¹��ɵ����ݵĶ��ֶ�
u32			Centre_DoubtTotal	= 0;    //  ���Ķ�ȡ�¹��ɵ�����ֶ�
u8			Vehicle_RunStatus	= 0;    //  bit 0: ACC �� ��             1 ��  0��
//  bit 1: ͨ���ٶȴ�������֪    1 ��ʾ��ʻ  0 ��ʾֹͣ
//  bit 2: ͨ��gps�ٶȸ�֪       1 ��ʾ��ʻ  0 ��ʾֹͣ
u8	Status_TiredwhRst = 0;              //  ����λʱ ƣ�ͼ�ʻ��״̬   0 :ͣ��  1:ͣ����û���� 2:�����˻�û����

u32 SrcFileSize		= 0, DestFilesize = 0, SrcFile_read = 0;
u8	SleepCounter	= 0;

u16 DebugSpd		= 0;                //������GPS�ٶ�
u8	MMedia2_Flag	= 0;                // �ϴ�������Ƶ ��ʵʱ��Ƶ  �ı�־λ    0 ������ 1 ��ʵʱ

u8	tts_bro_tired_flag = 0;             // Ϊ����֤  �� ֻ����һ�� ƣ�ͼ�ʻ�Ѿ����

//-----  ISP    Զ��������� -------
ISP_BD BD_ISP;                          //  BD   ������

//------ IC ��ģ�� ---
IC_MODULE			IC_MOD;             //IC ģ��

unsigned short int	FileTCB_CRC16	= 0;
unsigned short int	Last_crc		= 0, crc_fcs = 0;

//---------  ����Ӧ��  -----------
u8				Send_Rdy4ok = 0;
unsigned char	Rstart_time = 0;

MQU				MangQU;
MQU             MQ_TrueUse; // ������ä������ģʽ     

//---------- SMS SD ------------------
SMS_SD SMS_send;

//---------------  �ٶ��������--------------
u16			Delta_1s_Plus	= 0;
u16			Delta_1s_Plus2	= 0;
u16			Sec_counter		= 0;

LENGTH_BUF	Rx_reg;
LENGTH_BUF	app_rx;
u32			Appringbuf_wr	= 0;            // ѭ���洢buffer
u32			AppRingbuf_rd	= 0;


//---------74CH595  Q5   control Power----
u8   Print_power_Q5_enable=0;   
u8   Buzzer_on_Q7_enable=0; 

u32     MQsend_counter=0;
//++++++++++++++++++++++++++++++++++++++++++++++++++++�г���¼��+++++
u8 t_hour=0,t_min=0,t_second=0;

void K_AdjustUseGPS( u32 sp, u32 sp_DISP ); // ͨ��GPS У׼  K ֵ  (������ʻ1KM ��������Ŀ)


u16 Protocol_808_Encode( u8 *Dest, u8 *Src, u16 srclen );


void Protocol_808_Decode( void );           // ����ָ��buffer :  UDP_HEX_Rx


void Photo_send_end( void );


void Sound_send_end( void );


void Video_send_end( void );
void mq_true_enable(u8 value);



unsigned short int CRC16_file( unsigned short int num );


void Spd_ExpInit( void );


void AvrgSpd_MintProcess( u8 hour, u8 min, u8 sec );


u32 Distance_Point2Line( u32 Cur_Lat, u32 Cur_Longi, u32 P1_Lat, u32 P1_Longi, u32 P2_Lat, u32 P2_Longi );


void RouteRail_Judge( u8* LatiStr, u8* LongiStr );


void app_queenable( u8* instr );
u32 Get_MQ_true_CuurentTotal_packets(void);



//  A.  Total

void  Mangqu_Init( void )
{
	MangQU.PacketNum	= 0;
	MangQU.Sd_timer		= 0;
	MangQU.Sd_flag		= 0;
	//---  5-3
	MangQU.Enable_SD_state = 0;

    //  another  struct  init	
	MQ_TrueUse.PacketNum	= 0;
	MQ_TrueUse.Sd_timer		= 0;
	MQ_TrueUse.Sd_flag		= 0;
	//---  5-3
	MQ_TrueUse.Enable_SD_state = 0; 
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void delay_us( u16 j )
{
	u8 i;
	while( j-- )
	{
		i = 3;
		while( i-- )
		{
			;
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void delay_ms( u16 j )
{
	while( j-- )
	{
		DF_delay_us( 2000 ); // 1000
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void App_que_process( void )
{
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Do_SendGPSReport_GPRS( void )
{
	unsigned short int	crc_file	= 0;
	u16					count		= 0, str_len = 0;
	u8					i			= 0;
	u8					reg_gps[30], reg_wr = 0;
	u8					packet_type = 0;

     //  ˳���ϱ�BD ��Ϣ
        if(BDSD.Enable_Working==1)
            {
		         if( BDSD.SendFlag==RdCycle_RdytoSD)
		         {
                   Stuff_BDSD_0200H();
                   BDSD.SendFlag=RdCycle_SdOver;
				   return true;
		         }
        	}
	// 1.  GNSS ��ϸ�����ϴ�----------------------------------------------------
	//          ��������
    if( MQ_TrueUse.Enable_SD_state == 1 )
	{
		if( ( MQ_TrueUse.Sd_flag == 1 ) && ( DEV_Login.Operate_enable == 2 ) )
		{
		                
		               
		                Stuff_MangQu_Packet_Send_0704H_True();  
						MQ_TrueUse.Sd_flag=2;  //2;  
		    return true;            
		}		
	}
	//--------------------------------------------------------------------------------
	if( MangQU.Enable_SD_state == 1 )
	{
		if( ( MangQU.Sd_flag == 1 ) && ( DEV_Login.Operate_enable == 2 ) )
		{
		                Stuff_MangQu_Packet_Send_0704H();  
						//MangQU.Sd_flag=0;
		                MangQU.Sd_flag=2;  //2;  
		              /*  MangQU.PacketNum++;
					    if(MangQU.PacketNum>=(BLIND_NUM/MQ_PKNUM))
									  	    	{
									  	    	   MangQU.PacketNum=0;
												   MangQU.Enable_SD_state=0; 
												   rt_kprintf("\r\nReturn Normal all\r\n");  
									  	    	}
									  	    	*/
				return true;					  	    	
		}		
	}
	if( ( 1 == GNSS_rawdata.WorkEnable ) && ( GNSS_rawdata.save_status ) )
	{
		if( GNSS_rawdata.save_status & ( 1 << GNSS_rawdata.rd_num ) )   // ��鵱ǰrd   ��ֵ״̬
		{
			str_len = strlen( GNSS_rawdata.Raw[GNSS_rawdata.rd_num] );
			/*
			rt_kprintf("\r\n ----------Rx info--------------\r\n");
			 for(count=0;count<str_len;count++)
			   rt_kprintf("%c",GNSS_rawdata.Raw[GNSS_rawdata.rd_num][count]); 
			 rt_kprintf("\r\n ----------Rx info end--------------\r\n");
			Stuff_GNSSRawData_0900H( GNSS_rawdata.Raw[GNSS_rawdata.rd_num], str_len );
			//---------------------------
			*/
			GNSS_rawdata.rd_num++;
			if( GNSS_rawdata.rd_num > 3 )
			{
				GNSS_rawdata.rd_num = 0;
			}
			GNSS_rawdata.save_status &= ~( 1 << GNSS_rawdata.rd_num ); //clear
		}
		return true;
	}
	//2.  ��Ȩ--------------------------------------------------------------------
	if( DEV_Login.Operate_enable != 2 )
	{
		if( 1 == DEV_Login.Enable_sd )
		{
			Stuff_DevLogin_0102H( ); //  ��Ȩ   ==2 ʱ��Ȩ���
			DEV_Login.Enable_sd = 0;
			//------ ���ͼ�Ȩ���ж� ------------------
			//DEV_Login.Operate_enable=2;  //  �����жϼ�Ȩ��
			return true;
		}
	}
	// 3. ��ý�������ϴ�
	if( MediaObj.Media_transmittingFlag == 2 )
	{
		if( 1 == MediaObj.SD_Data_Flag )
		{
			Stuff_MultiMedia_Data_0801H( );
			MediaObj.SD_Data_Flag = 0;
			return true;
		}
		return true;                        // ����808 Э��Ҫ�� �������ý������в������ͱ����Ϣ��
	}
	//  4.   �ն�ע��
	if( 1 == DEV_regist.Enable_sd )
	{
		Stuff_RegisterPacket_0100H( 0 );    // ע��
		JT808Conf_struct.Msg_Float_ID = 0;
		Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
		DEV_regist.Enable_sd = 0;
		//  JT808Conf_struct.Regsiter_Status=1; //��עע�ᣬ�����洢
		return true;
	}
	// 5.  �ն�ע��
	if( 1 == DEV_regist.DeRegst_sd )
	{
		Stuff_DeviceDeregister_0101H( );
		DEV_regist.DeRegst_sd = 0;
		return true;
	}
#if   0
	//  6.  �ն�������
	if( ( 1 == JT808Conf_struct.DURATION.Heart_SDFlag ) && ( DataLink_Status( ) ) && ( SleepState == 0 ) && ( stopNormal == 0 ) )   //  ����
	{
		Stuff_DeviceHeartPacket_0002H( );
		JT808Conf_struct.DURATION.Heart_SDFlag	= 0;
		JT808Conf_struct.DURATION.TCP_SD_state	= 1;                                                                                //��������� 1
		return true;
	}
	// 7.  ����ʱ�ն�������
	if( ( 1 == SleepConfigFlag ) && ( DataLink_Status( ) ) && ( SleepState == 1 ) )                                                 //  ����ʱ����
	{
		Stuff_DevLogin_0102H( );                                                                                                    //  ��Ȩ   ==2 ʱ��Ȩ���
		rt_kprintf( "\r\n	 ����ʱ�ü�Ȩ�������� ! \r\n");
		SleepConfigFlag = 0;
		return true;
	}
#endif
	//  8.   ��λ���������ϴ�
	if( 1 == SD_ACKflag.f_BD_BatchTrans_0704H )                                                                                     //  ��λ���������ϴ�
	{
		Stuff_BatchDataTrans_BD_0704H( );
		SD_ACKflag.f_BD_BatchTrans_0704H = 0;
		return true;
	}
	// 9.  ������������Ӧ��
	if( 1 == SD_ACKflag.f_BD_CentreTakeAck_0805H )                                                                                  // ������������Ӧ��
	{
		Stuff_CentreTakeACK_BD_0805H( );
		SD_ACKflag.f_BD_CentreTakeAck_0805H = 0;
		return true;
	}
	//   10.  �ն������ϴ�
	if( 1 == SD_ACKflag.f_BD_DeviceAttribute_8107 )                                                                                 //  �ն������ϴ�
	{
		Stuff_DeviceAttribute_BD_0107H( );
		SD_ACKflag.f_BD_DeviceAttribute_8107 = 0;
		return true;
	}
	//   11.  λ����Ϣ��ѯ
	if( 1 == SD_ACKflag.f_CurrentPosition_0201H )                                                                                   // λ����Ϣ��ѯ
	{
		Stuff_Current_Data_0201H( );
		SD_ACKflag.f_CurrentPosition_0201H = 0;
		return true;
	}
	//   12.   �¼�����
	if( 1 == SD_ACKflag.f_CurrentEventACK_0301H )                                                                                   //  �¼�����
	{
		Stuff_EventACK_0301H( );
		SD_ACKflag.f_CurrentEventACK_0301H = 0;
		return true;
	}
	//  13. ����Ӧ��
	if( 2 == ASK_Centre.ASK_SdFlag )                                                                                                //  ����Ӧ��
	{
		Stuff_ASKACK_0302H( );
		ASK_Centre.ASK_SdFlag = 0;
		return true;
	}
	// 14. ����Ӧ�����
	if( 1 == Vech_Control.ACK_SD_Flag )                                                                                             //  ����Ӧ�����
	{
		Stuff_ControlACK_0500H( );
		Vech_Control.ACK_SD_Flag = 0;
		return true;
	}
	//  15.  �г���¼�������ϴ�
	if( ( 1 == Recode_Obj.SD_Data_Flag ) && ( 1 == Recode_Obj.CountStep )&&((0==Recode_Obj.RSD_State)||(3==Recode_Obj.RSD_State)) ) 
	{
		//  1. clear   one  packet  flag
		switch( Recode_Obj.CMD ) 
		{
			/*                divide  not  stop
			   case 0x08:
			   case 0x09:
			   case 0x10:
			   case 0x11:
			   case 0x12:
			   case 0x15:
			 */
			case 0x07:
			case 0x13:
			case 0x14:
				Recode_Obj.SD_Data_Flag = 0;
				Recode_Obj.CountStep	= 0;
				break;
		}
		//  2.  stuff   recorder   infomation
		//  judge  packet  type
		if( Recode_Obj.Devide_Flag == 1 )
		{
			packet_type = Packet_Divide;
		} else
		{
			packet_type = Packet_Normal;
		}

        
		rt_kprintf( "\r\n ��¼�� CMD_ID =0x%2X \r\n", Recode_Obj.CMD );
		if( packet_type == Packet_Divide )
		{
		  if((Recode_Obj.RSD_State==0)||(Recode_Obj.RSD_State==3))  // �ڷ��б��ش�����½��� bak  
		   {
		       Recode_Obj.Bak_current_num=Recode_Obj.Current_pkt_num;
               Recode_Obj.Bak_fcs= Recode_Obj.fcs;   
			   Recode_Obj.Bak_CMD=Recode_Obj.CMD;
		   }
		  
			rt_kprintf( "\r\n              current =%d  Total: %d \r\n", Recode_Obj.Current_pkt_num, Recode_Obj.Total_pkt_num );
		}
		Stuff_RecoderACK_0700H( packet_type ); //   �г���¼�������ϴ�
		//  3. step  by  step  send   from  00H  ---  07H
		switch( Recode_Obj.CMD )
		{
			case 0x00:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x01;
				break;
			case 0x01:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x02;
				break;
			case 0x02:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x03;
				break;
			case 0x03:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x04;
				break;
			case 0x04:    Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x05;
				break;
			case 0x05:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x06;
				break;
			//-----------------------------------------
			case 0x06:   Recode_Obj.SD_Data_Flag	= 1;
				Recode_Obj.CMD						= 0x07;
				break;
				//-----------------------------------------
		}

		if( Recode_Obj.CountStep == 1 )
		{
			Recode_Obj.CountStep = 2; Recode_Obj.timer = 0;
		}

		return true;
	}

       //15.2     ��¼�� �б��ش�
       if((1 == Recode_Obj.RSD_State)&&(Recode_Obj.SD_Data_Flag==1))  
       	{
            
			if( Recode_Obj.Devide_Flag == 1 )
			{
				packet_type = Packet_Divide;
			} else
			{
				packet_type = Packet_Normal;
			}			
			rt_kprintf( "\r\n ��¼���б��ش� CMD_ID =0x%2X \r\n", Recode_Obj.CMD );
			if( packet_type == Packet_Divide )
			{
				rt_kprintf( "\r\n             current =%d  RsendTotal: %d  CurrentRsd=%d\r\n", Recode_Obj.Current_pkt_num, Recode_Obj.RSD_total,Recode_Obj.RSD_Reader);
			}
			Stuff_RecoderACK_0700H( packet_type ); //	�г���¼�������ϴ�

            if( Recode_Obj.RSD_Reader == Recode_Obj.RSD_total )
			{
				Recorder_init(1); //  ��λ�ȴ�״̬���ȴ��������ٷ��ش�ָ��
				rt_kprintf( "\r\n ��¼���б��ش�����!  CMD_ID =0x%2X  RsendTotal:%d CurrentRsd=%d\r\n", Recode_Obj.CMD,Recode_Obj.RSD_total,Recode_Obj.RSD_Reader);
                /*
                          if(Recode_Obj.Transmit_running==1)
	                	{
	                	  Rcorder_Recover(); 
	                      rt_kprintf( "\r\n ˳��ִ����δ���  current=%d  total=%d\r\n",Recode_Obj.Current_pkt_num,Recode_Obj.Total_pkt_num);
						  return true;
	                	}
				*/
			}			
			Recode_Obj.SD_Data_Flag=0;// clear   
			return true;

       	}
	
	// 16. ��Ϣ�㲥
	if( SD_ACKflag.f_MsgBroadCast_0303H == 1 ) // ��Ϣ�㲥
	{
		Stuff_MSGACK_0303H( );
		SD_ACKflag.f_MsgBroadCast_0303H = 0;
		return true;
	}
	//17. ��ý���¼���Ϣ�ϴ�
	if( 1 == MediaObj.SD_media_Flag )
	{
		Stuff_MultiMedia_InfoSD_0800H( ); // ��ý���¼���Ϣ�ϴ�
		MediaObj.SD_media_Flag = 0;
		return true;
	}
	//  18.����͸�� ��Զ������
	if( CAN_trans.canid_0705_sdFlag )
	{
		Stuff_CANDataTrans_BD_0705H( );
		CAN_trans.canid_0705_sdFlag = 0;    // clear
		//   DataTrans_Init();     //clear
		return true;
	}
	// 19. ��ý�������ϱ�
	if( SD_ACKflag.f_MediaIndexACK_0802H )
	{
		Stuff_MultiMedia_IndexAck_0802H( ); // ��ý�������ϱ�
		SD_ACKflag.f_MediaIndexACK_0802H = 0;
		return true;
	}
	// 20.��ʻԱ��Ϣ�ϱ�
	if( SD_ACKflag.f_DriverInfoSD_0702H )
	{
		Stuff_DriverInfoSD_0702H( );        //  ��ʻԱ��Ϣ�ϱ�
		SD_ACKflag.f_DriverInfoSD_0702H = 0;
		return true;
	}
	//   21.�����˵�
	if( SD_ACKflag.f_Worklist_SD_0701H )
	{
		Stuff_Worklist_0701H( );            //   �����˵�
		SD_ACKflag.f_Worklist_SD_0701H = 0;
		return true;
	}
	//    22. �ն�ͨ��Ӧ��
	if( SD_ACKflag.f_CentreCMDack_0001H )
	{
		Stuff_DevCommmonACK_0001H( );
		if( SD_ACKflag.f_CentreCMDack_0001H == 2 )  //  �޸�IP��������Ҫ�ز�
		{
			Close_DataLink( );   rt_kprintf( "\r\n Datalink end =>�����޸�IP\r\n" );
		}                                           //  AT_END
		else
		if( SD_ACKflag.f_CentreCMDack_0001H == 3 )  //   Զ�̸�λ
		{
			Systerm_Reset_counter	= Max_SystemCounter;
			ISP_resetFlag			= 2;            //   ����Զ�������������Ƹ�λϵͳ
		}else
		if( SD_ACKflag.f_CentreCMDack_0001H == 5 )  //   �ر�����ͨ��
		{
			Close_DataLink( );
			Stop_Communicate( );
			rt_kprintf( "\r\n Datalink end =>�ر�ͨ��n\r\n" );
		}
		SD_ACKflag.f_CentreCMDack_0001H		= 0;
		SD_ACKflag.f_CentreCMDack_resualt	= 0;

		return true;
	}
	//  23.   ��ѯ�ն˲���
	if( SD_ACKflag.f_SettingPram_0104H )
	{
		Stuff_SettingPram_0104H( SD_ACKflag.f_SettingPram_0104H );
		SD_ACKflag.f_SettingPram_0104H = 0;
		return true;
	}
	//  24 .  �ն���������ϱ�
	if( 1 == SD_ACKflag.f_BD_ISPResualt_0108H )
	{
		Stuff_ISP_Resualt_BD_0108H( );
		SD_ACKflag.f_BD_ISPResualt_0108H = 0;
		return true;
	}
	//  25.  IC ��͸����Ϣ
	if( ( 1 == IC_MOD.Trans_0900Flag ) && ( 1 == TCP2_Connect ) )
	{
		Stuff_DataTrans_0900_BD_ICinfo( );
		IC_MOD.Trans_0900Flag = 0;
		return true;
	}
	//           λ�������ϱ�
	if( MangQU.Enable_SD_state == 1 )
	{
		return true;
	}
	//--------------------------------------------------------------------------------------


	/*    if(app_que_enable==1)
	     {
	             //--------save
	           // if(PositionSD_Status()&&(DEV_Login.Operate_enable==2)&&((enable==BD_EXT.Trans_GNSS_Flag)||(DispContent==6))||(Current_UDP_sd&&PositionSD_Status()&&(DEV_Login.Operate_enable==2))||((DF_LOCK==enable)&&PositionSD_Status()&&(DEV_Login.Operate_enable==2)))	  //�״ζ�λ�ٷ�
	            if(Current_UDP_sd&&PositionSD_Status())	  //�״ζ�λ�ٷ�
	   {
	                 //  clear  flag
	                 PositionSD_Disable();
	   Current_UDP_sd=0;

	   // stuff      content
	   memset(app_rx.body,0,sizeof(app_rx.body));
	   app_rx.wr=Reg_save_gps(app_rx.body);
	   //  send queue
	   if((AppQue.write_num*28)>APP_RAWINFO_SIZE )
	   {
	   rt_kprintf("\r\n exceed max appsize !!\r\n ");
	   return true;
	   }
	   memcpy(APP_serialinfo+AppQue.write_num*28,app_rx.body,app_rx.wr);
	                   //  rt_mq_send(&mq_APPs, (void*)&app_rx,  app_rx.wr+2);
	   AppQue.write_num++;
	   rt_kprintf("\r\n msg-save len=%d\r\n ", app_rx.wr);
	           }
	   return true;
	     }
	 */
	if( app_que_enable == 1 )
	{
		return true;
	}
	//-----------------------------------------------------------------------------------
	// if((Current_SD_Duration<=10)||(Current_State==1))   // ����ʱ30  ʵ����10 Current_SD_Duration
	// {
	if( stopNormal == 2 )
	{
		return true;
	}
	if( PositionSD_Status( ) && ( DEV_Login.Operate_enable == 2 )&&(VocREC.running==0) && ( ( enable == BD_EXT.Trans_GNSS_Flag ) || ( DispContent == 6 ) ) || ( Current_UDP_sd && PositionSD_Status( ) && ( DEV_Login.Operate_enable == 2 )&&(VocREC.running==0) ) || ( ( DF_LOCK == enable ) && PositionSD_Status( ) && ( DEV_Login.Operate_enable == 2 )&&(VocREC.running==0) ) ) //�״ζ�λ�ٷ�
	//  if((PositionSD_Status())&&(DataLink_Status())&&(DEV_Login.Operate_enable==2))	                                                                                                                     // DF  �������͵�ǰλ����Ϣ
	{
		PositionSD_Disable( );
		Current_UDP_sd = 0;
		//1.   ʱ�䳬ǰ�ж�
		//if(Time_FastJudge()==false)
		//		return false;
		// 2.
		Stuff_Current_Data_0200H( ); // �ϱ���ʱ����
		//----Ӧ����� ----
		// ACKFromCenterCounter++; // ֻ��עӦ������������Ӧ��ʱ��
		//---------------------------------------------------------------------------------
		if( SleepState == 1 )
		{
			rt_kprintf( "\r\n����ʱ����ʱ�� %d-%d-%d %02d:%02d:%02d\r\n", time_now.year + 2000, time_now.month, time_now.day, \
			            time_now.hour, time_now.min, time_now.sec );
		}
		if( DispContent )
		{
			rt_kprintf( "\r\n ���� GPS -current !\r\n" );
		}

		return true;
	}
	//-------------------------------------------------------------
	return false;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void strtrim( u8 *s, u8 c )
{
	u8	*p1, *p2;
	u16 i, j;

	if( s == 0 )
	{
		return;
	}

	// delete the trailing characters
	if( *s == 0 )
	{
		return;
	}
	j	= strlen( (char const*)s );
	p1	= s + j;
	for( i = 0; i < j; i++ )
	{
		p1--;
		if( *p1 != c )
		{
			break;
		}
	}
	if( i < j )
	{
		p1++;
	}
	*p1 = 0; // null terminate the undesired trailing characters

	// delete the leading characters
	p1 = s;
	if( *p1 == 0 )
	{
		return;
	}
	for( i = 0; *p1++ == c; i++ )
	{
		;
	}
	if( i > 0 )
	{
		p2 = s;
		p1--;
		for(; *p1 != 0; )
		{
			*p2++ = *p1++;
		}
		*p2 = 0;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
int str2ip( char *buf, u8 *ip )
{   // convert an ip:port string into a binary values
	int i;
	u16 _ip[4];

	memset( _ip, 0, sizeof( _ip ) );

	strtrim( (u8*)buf, ' ' );

	i = sscanf( buf, "%u.%u.%u.%u", (u32*)&_ip[0], (u32*)&_ip[1], (u32*)&_ip[2], (u32*)&_ip[3] );

	*(u8*)( ip + 0 )	= (u8)_ip[0];
	*(u8*)( ip + 1 )	= (u8)_ip[1];
	*(u8*)( ip + 2 )	= (u8)_ip[2];
	*(u8*)( ip + 3 )	= (u8)_ip[3];

	return i;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
int IP_Str( char *buf, u32 IP )
{
	T_IP_Addr ip;

	if( !buf )
	{
		return 0;
	}

	ip.ip32 = IP;

	return sprintf( buf, "%u.%u.%u.%u", ip.ip8[0], ip.ip8[1], ip.ip8[2], ip.ip8[3] );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u16 AsciiToGb( u8 *dec, u8 InstrLen, u8 *scr )
{
	u16 i			= 0, j = 0, m = 0;
	u16 Info_len	= 0;

	for( i = 0, j = 0; i < InstrLen; i++, j++ )
	{
		m = scr[i];
		if( ( m >= 0x30 ) && ( m <= 0x39 ) )
		{
			memcpy( &dec[j], &arr_A3B0[( m - '0' ) * 2], 2 );
			j++;
		}else if( ( m >= 0x41 ) && ( m <= 0x4f ) )
		{
			memcpy( &dec[j], &arr_A3C0[( m - 0x41 + 1 ) * 2], 2 );
			j++;
		}else if( ( m >= 0x50 ) && ( m <= 0x5a ) )
		{
			memcpy( &dec[j], &arr_A3D0[( m - 0x50 ) * 2], 2 );
			j++;
		}else if( ( m >= 0x61 ) && ( m <= 0x6f ) )
		{
			memcpy( &dec[j], &arr_A3E0[( m - 0x61 + 1 ) * 2], 2 );
			j++;
		}else if( ( m >= 0x70 ) && ( m <= 0x7a ) )
		{
			memcpy( &dec[j], &arr_A3F0[( m - 0x70 ) * 2], 2 );
			j++;
		}else
		{
			dec[j] = m;
		}
	}
	Info_len = j;
	return Info_len;
}

// B.   Protocol

//==================================================================================================
// ��һ���� :   ������GPS ����ת����غ���
//==================================================================================================

void Time_pro( u8 *tmpinfo, u8 hour, u8 min, u8 sec )
{
	//---- record  to memory
	GPRMC.utc_hour	= hour;
	GPRMC.utc_min	= min;
	GPRMC.utc_sec	= sec;

	CurrentTime[0]	= hour;
	CurrentTime[1]	= min;
	CurrentTime[2]	= sec;

	//-----------  ���ͨЭ�� -------------
	Temp_Gps_Gprs.Time[0]	= hour;
	Temp_Gps_Gprs.Time[1]	= ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
	Temp_Gps_Gprs.Time[2]	= ( tmpinfo[4] - 0x30 ) * 10 + tmpinfo[5] - 0x30;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Status_pro( u8 *tmpinfo )
{
	GPRMC.status = tmpinfo[0];

	//-------------------------���ͨЭ��-----------------------------
	if( tmpinfo[0] == 'V' || tmpinfo[0] == 'v' )
	{
		UDP_dataPacket_flag = 0X03;
		StatusReg_GPS_V( );
	}else if( tmpinfo[0] == 'A' || tmpinfo[0] == 'a' )
	{
		UDP_dataPacket_flag = 0X02;
		StatusReg_GPS_A( );
	}

	//---------------------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Latitude_pro( u8 *tmpinfo )
{
	u32 latitude;
	GPRMC.latitude_value = atof( (char*)tmpinfo );


	/*     Latitude
	       ddmm.mmmm
	 */

	//--------	808 Э�� --------------------
	if( UDP_dataPacket_flag == 0X02 ) //��ȷ�������֮һ��
	{
		//------------  dd part   --------
		latitude = ( u32 )( ( tmpinfo[0] - 0x30 ) * 10 + ( u32 )( tmpinfo[1] - 0x30 ) ) * 1000000;
		//------------  mm  part  -----------


		/*    ת���ɰ����֮һ��
		      mm.mmmm   *  1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
		 */
		latitude = latitude + ( u32 )( ( ( tmpinfo[2] - 0x30 ) * 100000 + ( tmpinfo[3] - 0x30 ) * 10000 + ( tmpinfo[5] - 0x30 ) * 1000 + ( tmpinfo[6] - 0x30 ) * 100 + ( tmpinfo[7] - 0x30 ) * 10 + ( tmpinfo[8] - 0x30 ) ) * 5 / 3 );

		if( latitude == 0 )
		{
			GPS_getfirst = 0;
			StatusReg_GPS_V( );
			return;
		}

		Temp_Gps_Gprs.Latitude[0]	= ( u8 )( latitude >> 24 );
		Temp_Gps_Gprs.Latitude[1]	= ( u8 )( latitude >> 16 );
		Temp_Gps_Gprs.Latitude[2]	= ( u8 )( latitude >> 8 );
		Temp_Gps_Gprs.Latitude[3]	= ( u8 )latitude;
	}
	//----------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Lat_NS_pro( u8 *tmpinfo )
{
	GPRMC.latitude = tmpinfo[0];
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Longitude_pro( u8 *tmpinfo )
{
	u32 longtitude;
	GPRMC.longtitude_value = atof( (char*)tmpinfo );


	/*     Latitude
	        dddmm.mmmm
	 */
	//--------  808Э��  ---------
	if( UDP_dataPacket_flag == 0X02 ) //��ȷ�������֮һ��
	{
		//------  ddd part -------------------
		longtitude = ( u32 )( ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 ) ) * 1000000;
		//------  mm.mmmm --------------------


		/*    ת���ɰ����֮һ��
		   mm.mmmm	 *	1000000/60=mm.mmmm*50000/3=mm.mmmm*10000*5/3
		 */
		longtitude = longtitude + ( u32 )( ( ( tmpinfo[3] - 0x30 ) * 100000 + ( tmpinfo[4] - 0x30 ) * 10000 + ( tmpinfo[6] - 0x30 ) * 1000 + ( tmpinfo[7] - 0x30 ) * 100 + ( tmpinfo[8] - 0x30 ) * 10 + ( tmpinfo[9] - 0x30 ) ) * 5 / 3 );
		if( longtitude == 0 )
		{
			GPS_getfirst = 0; StatusReg_GPS_V( ); return;
		}

		//---jiade --guojingdu yong ---
		if( app_que_enable == 1 )
		{
			longtitude = 116 * 1000000 - ( CSQ_counter % 8 ) + ( TIM1_Timer_Counter % 3 );
		}

		Temp_Gps_Gprs.Longitude[0]	= ( u8 )( longtitude >> 24 );
		Temp_Gps_Gprs.Longitude[1]	= ( u8 )( longtitude >> 16 );
		Temp_Gps_Gprs.Longitude[2]	= ( u8 )( longtitude >> 8 );
		Temp_Gps_Gprs.Longitude[3]	= ( u8 )longtitude;
	}

	//---------------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Long_WE_pro( u8 *tmpinfo )
{
	GPRMC.longtitude = tmpinfo[0];
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Speed_pro( u8 *tmpinfo, u8 Invalue, u8 Point )
{
	u32 sp	= 0, sp_DISP = 0;
	u32 reg = 0;

	//-------------------------------------------------------------------------------------------------------------
	if( Invalue == INIT )
	{
		return;
	}else //---------------------------------------------------------------------------------------------------------
	{
		GPRMC.speed = atof( (char*)tmpinfo );
		//---------------------------------------------------
		if( UDP_dataPacket_flag == 0x02 )
		{
			//-----808 Э�� --------------
			//�����ֽڵ�λ0.1 km/h
			if( Point == 1 )                                                    //0.0-9.9=>
			{
				//++++++++  Nathan Modify on 2008-12-1   ++++++++++
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[2] >= 0x30 ) && ( tmpinfo[2] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 );    //����10��
				}else
				{
					return;
				}
			}else if( Point == 2 )                                              //10.0-99.9
			{
				//++++++++  Nathan Modify on 2008-12-1   ++++++++++
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[1] >= 0x30 ) && ( tmpinfo[1] <= 0x39 ) && ( tmpinfo[3] >= 0x30 ) && ( tmpinfo[3] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + tmpinfo[3] - 0x30;
				}else
				{
					return;
				}
			}else if( Point == 3 ) //100.0-999.9
			{
				//++++++++  Nathan Modify on 2008-12-1	++++++++++
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[1] >= 0x30 ) && ( tmpinfo[1] <= 0x39 ) && ( tmpinfo[2] >= 0x30 ) && ( tmpinfo[2] <= 0x39 ) && ( tmpinfo[4] >= 0x30 ) && ( tmpinfo[4] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 ) * 1000 + ( tmpinfo[1] - 0x30 ) * 100 + ( tmpinfo[2] - 0x30 ) * 10 + tmpinfo[4] - 0x30;
				}else
				{
					return;
				}
			}else
			{
				if( JT808Conf_struct.Speed_GetType == 0 )
				{
					GPS_speed = 0;
				} else
				{
					GPS_speed = Speed_cacu;
				}
			}

			// --------  sp ��ǰ��0.1 knot------------------
			sp = (u32)( sp * 185.6 );                                   //  1 ����=1.856 ǧ��  ������m/h

			if( sp > 220000 )                                           //ʱ�ٴ���220km/h���޳�
			{
				return;
			}

			sp_DISP = sp / 100;                                         //  sp_Disp ��λ�� 0.1km/h

			//------------------------------ ͨ��GPSģ�����ݻ�ȡ�����ٶ� --------------------------------
			Speed_gps = (u16)sp_DISP;
			//---------------------------------------------------------------------------
			if( JT808Conf_struct.Speed_GetType )                        // ͨ���ٶȴ����� ��ȡ�ٶ�
			{
				GPS_speed = Speed_cacu;
#if 0                                                                   //  ������֤�Ȳ����Զ�У׼
				K_AdjustUseGPS( sp, sp_DISP );                          //  ����Kֵ
				if( JT808Conf_struct.DF_K_adjustState == 0 )
				{
					// ---  ��δУ׼ǰ����õ����ٶ���ͨ��GPS����õ���
					GPS_speed = Speed_gps;
					//------- GPS	��̼���--------
					if( sp >= 5000 )                                    //	�������Ư��  �ٶȴ���
					{
						reg								= sp / 3600;    // ����3600 ��m/s
						JT808Conf_struct.Distance_m_u32 += reg;
						if( JT808Conf_struct.Distance_m_u32 > 0xFFFFFF )
						{
							JT808Conf_struct.Distance_m_u32 = 0;        //������ô����
						}
						//----- ����ش�����---
						if( 1 == JT808Conf_struct.SD_MODE.DIST_TOTALMODE )
						{
							DistanceAccumulate += reg;
							if( DistanceAccumulate >= Current_SD_Distance )
							{
								DistanceAccumulate = 0;
								PositionSD_Enable( ); //����
								Current_UDP_sd = 1;
							}
						}
						//------- ���ദ����� -----
					}
				}
#endif
			}else
			{                                                       // ��GPS ȡ�ٶ�
				//------- GPS	��̼���--------
				if( sp >= 5000 )                                    //	�������Ư��  �ٶȴ���
				{
					JT808Conf_struct.Distance_m_u32 += sp / 3600;   // ����3600 ��m/s
					if( JT808Conf_struct.Distance_m_u32 > 0xFFFFFF )
					{
						JT808Conf_struct.Distance_m_u32 = 0;        //������ô����
					}
					//----- ����ش�����---
					if( 1 == JT808Conf_struct.SD_MODE.DIST_TOTALMODE )
					{
						DistanceAccumulate += reg;
						if( DistanceAccumulate >= Current_SD_Distance )
						{
							DistanceAccumulate = 0;
							PositionSD_Enable( );   //����
							Current_UDP_sd = 1;
						}
					}
					//------- ���ദ����� -----
				}

				GPS_speed = Speed_gps;              // ��GPS���ݼ���õ��ٶ� ��λ0.1km/h

				//-----------------------------------------------
			}
			// if(DispContent==2)
			//  rt_kprintf("\r\n				  �ٶ�: %d Km/h\r\n",GPS_speed/10);
		}
		else if( UDP_dataPacket_flag == 0x03 )
		{
			if( 0 == JT808Conf_struct.Speed_GetType )
			{
				//----- GPS ��ʱ�ٶ�	km/h  ---------
				GPS_speed = 0;
			}
			if( JT808Conf_struct.Speed_GetType ) // ͨ���ٶȴ����� ��ȡ�ٶ�
			{
				//  K_AdjustUseGPS(sp,sp_DISP);  //  ����Kֵ
				GPS_speed = Speed_cacu;
			}
			Speed_gps = 0;
		//	if( DispContent == 2 )
			//{
			//	rt_kprintf( "\r\n 2 GPSû��λ\r\n" );
			//}
		}
	}
	//---------------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Direction_pro( u8 *tmpinfo, u8 Invalue, u8 Point )
{
	u32 sp = 0;
	//------------------------------------------------------------------------------------------------
	if( Invalue == INIT )
	{
		return;
	}else //-------------------------------------------------------------------------------------------
	{
		GPRMC.azimuth_angle = atof( (char*)tmpinfo );

		//--------------808 Э��  1 ��-------------------------
		if( UDP_dataPacket_flag == 0x02 )
		{
			if( Point == 1 ) //5.8
			{
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[2] >= 0x30 ) && ( tmpinfo[2] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 );
				} else
				{
					return;
				}
			}else if( Point == 2 ) // 14.7
			{
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[1] >= 0x30 ) && ( tmpinfo[1] <= 0x39 ) && ( tmpinfo[3] >= 0x30 ) && ( tmpinfo[3] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 ) * 10 + ( tmpinfo[1] - 0x30 );
				} else
				{
					return;
				}
			}else //357.38
			if( Point == 3 )
			{
				if( ( tmpinfo[0] >= 0x30 ) && ( tmpinfo[0] <= 0x39 ) && ( tmpinfo[1] >= 0x30 ) && ( tmpinfo[1] <= 0x39 ) && ( tmpinfo[2] >= 0x30 ) && ( tmpinfo[2] <= 0x39 ) && ( tmpinfo[4] >= 0x30 ) && ( tmpinfo[4] <= 0x39 ) )
				{
					sp = ( tmpinfo[0] - 0x30 ) * 100 + ( tmpinfo[1] - 0x30 ) * 10 + ( tmpinfo[2] - 0x30 );
				} else
				{
					return;
				}
			}else
			{
				sp = 0;
			}
			GPS_direction = sp; //  ��λ 1��

			//----------  �յ㲹�����   ----------
			// Inflexion_Process();
		}else if( UDP_dataPacket_flag == 0x03 )
		{
			GPS_direction = 0;
		}

		return;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Date_pro( u8 *tmpinfo, u8 fDateModify, u8 hour, u8 min, u8 sec )
{
	uint8_t		year = 0, mon = 0, day = 0;
	TDateTime	now;

	day		= ( ( tmpinfo[0] - 0x30 ) * 10 ) + ( tmpinfo[1] - 0x30 );
	mon		= ( ( tmpinfo[2] - 0x30 ) * 10 ) + ( tmpinfo[3] - 0x30 );
	year	= ( ( tmpinfo[4] - 0x30 ) * 10 ) + ( tmpinfo[5] - 0x30 );

	if( fDateModify )
	{
		//sscanf(tmpinfo,"%2d%2d%2d",&day,&mon,&year);
		day++;
		if( mon == 2 )
		{
			if( ( year % 4 ) == 0 )
			{
				if( day == 30 )
				{
					day = 1; mon++;
				}
			}else if( day == 29 )
			{
				day = 1; mon++;
			}
		}else if( ( mon == 4 ) || ( mon == 6 ) || ( mon == 9 ) || ( mon == 11 ) )
		{
			if( day == 31 )
			{
				mon++; day = 1;
			}
		}else
		{
			if( day == 32 )
			{
				mon++; day = 1;
			}
			if( mon == 13 )
			{
				mon = 1; year++;
			}
		}
	}
	GPRMC.utc_year	= year;
	GPRMC.utc_mon	= mon;
	GPRMC.utc_day	= day;
	if( ( ( sec == 0 ) && ( GPRMC.status == 'A' ) ) || ( Shoushi == 1 ) )
	{
		now.year	= year;
		now.month	= mon;
		now.day		= day;
		now.hour	= hour;
		now.min		= min;
		now.sec		= sec;
		now.week	= 1;
		Device_RTC_set( now );
		Shoushi = 0;
	}
	//------------------------------------------------
	if( GPRMC.status == 'A' )                                               //  ��¼��λʱ��
	{
		Time2BCD( A_time );
		//------- Debug �洢 ÿ��ľ�γ��  || ʵ��Ӧ���� �洢ÿ���ӵ�λ��  -----
		//  ���ݳ���55��ÿ����£���Ĵ����м�¼������ÿ���������һ����λ�ľ�γ�� ,Ԥ��5�����ڴ洢��һСʱ��λ��
		if( sec < 55 )
		{
			memcpy( Posit[min].latitude_BgEnd, Gps_Gprs.Latitude, 4 );      //��γ
			memcpy( Posit[min].longitude_BgEnd, Gps_Gprs.Longitude, 4 );    //����
			Posit[min].longitude_BgEnd[0] |= 0x80;                          //  ����
		}
		if( ( min == 59 ) && ( sec == 55 ) )
		{                                                                   // ÿ��Сʱ��λ����Ϣ
			NandsaveFlg.MintPosit_SaveFlag = 1;
		}
	}
	//---- �洢��ǰ����ʼ���  ����ʱ------------
	if( ( hour == 0 ) && ( min == 0 ) && ( sec < 3 ) )                      // �洢3��ȷ���洢�ɹ�
	{
		JT808Conf_struct.DayStartDistance_32 = JT808Conf_struct.Distance_m_u32;
		Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
	}

	//-------------------------------------------------
	//---------  ���ͨЭ��  -------

	//if(systemTick_TriggerGPS==0)
	{
		Temp_Gps_Gprs.Date[0]	= year;
		Temp_Gps_Gprs.Date[1]	= mon;
		Temp_Gps_Gprs.Date[2]	= day;
	}

	//-------------- ������ƽ���ٶ� ----------------
	AvrgSpd_MintProcess( hour, min, sec );
}

//---------  GGA --------------------------
void HDop_pro( u8 *tmpinfo )
{
	float dop;

	dop			= atof( (char*)tmpinfo );
	HDOP_value	= dop;          //  Hdop ��ֵ
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  GPS_Delta_DurPro( void )  //��GPS �����ϱ�������
{
	if( line_warn_enable == 1 )
	{
		rt_kprintf( "\r\n  %d-%d-%d %d:%d:%d %s", Temp_Gps_Gprs.Date[0], Temp_Gps_Gprs.Date[1], Temp_Gps_Gprs.Date[2], \
		            Temp_Gps_Gprs.Time[0], Temp_Gps_Gprs.Time[1], Temp_Gps_Gprs.Time[2], gps_log );
	}

	if( 1 == JT808Conf_struct.SD_MODE.DUR_TOTALMODE ) // ��ʱ�ϱ�ģʽ
	{
		//----- ��һ�����ݼ�¼��ʱ��
		fomer_time_seconds = ( u32 )( BakTime[0] * 60 * 60 ) + ( u32 )( BakTime[1] * 60 ) + ( u32 )BakTime[2];

		//-----  ��ǰ���ݼ�¼��ʱ��
		tmp_time_secnonds = ( u32 )( CurrentTime[0] * 60 * 60 ) + ( u32 )( CurrentTime[1] * 60 ) + ( u32 )CurrentTime[2];

		//һ��86400��

		if( tmp_time_secnonds > fomer_time_seconds )
		{
			delta_time_seconds = tmp_time_secnonds - fomer_time_seconds;
			//systemTickGPS_Clear();
		}else if( tmp_time_secnonds < fomer_time_seconds )
		{
			delta_time_seconds = 86400 - fomer_time_seconds + tmp_time_secnonds;
			//systemTickGPS_Clear();
		}else
		{
			// systemTickGPS_Set();
			UDP_dataPacket_flag = 0X03;
			StatusReg_GPS_V( );
		}

		if( ( SleepState == 1 ) && ( delta_time_seconds == ( Current_SD_Duration - 5 ) ) )  //  --  ����ʱ �ȷ���Ȩ
		{
			SleepConfigFlag = 1;                                                            //����ǰ5 ����һ����Ȩ
		}

		if( ( delta_time_seconds >= Current_SD_Duration ) && ( local_trig == 0 ) )          //limitSend_idle
		{
			if( app_que_enable == 0 )                                                       // app �������ϱ�
			{
				if( stop_current == 0 )
				{
					                                                  // ÿ�ζ��Ǽ�ʱ�ϱ�
	                 if(MQ_TrueUse.Enable_SD_state) 
	                 {
		                  if(MQ_TrueUse.Enable_SD_state==2)  // �洢ʱ��洢�� �ϱ�ʱ�򲻴洢
		                  	{      
		                  	      //  upgrade    CurrentTotal  
		                  	       Get_MQ_true_CuurentTotal_packets(); 
								  //---- ä��д�����process--------
									MangQU_true_create(Temp_Gps_Gprs);
								    MQsend_counter++;
									//if(GpsStatus.Raw_Output==1) 
									rt_kprintf( "\r\n                 %d-%d-%d %d:%d:%d   sd=%d     Timeenable:%d  lati:%d  longi:%d\r\n",Temp_Gps_Gprs.Date[0], Temp_Gps_Gprs.Date[1], Temp_Gps_Gprs.Date[2], \
												Temp_Gps_Gprs.Time[0], Temp_Gps_Gprs.Time[1], Temp_Gps_Gprs.Time[2],MQsend_counter,EverySecond_Time_Get,Lati_Get,Longi_Get);  
		                  	}
	                 }
					 else
					 {
						 if(BDSD.Enable_Working==1)
						 	{   // ˳��洢�жϲ���
	                           BD_send_Mque_Tx(Temp_Gps_Gprs);
						 	} 
					    else
					 	 {
	                        PositionSD_Enable( );
						    Current_UDP_sd = 1;   
					 	 }
					}	
				}
			}
			memcpy( BakTime, CurrentTime, 3 );                                              // update

			if( stopNormal == 1 )
			{
				gps_sd_coutner++;
				rt_kprintf( "\r\n %d    %d-%d-%d %d:%d:%d %s", gps_sd_coutner, Temp_Gps_Gprs.Date[0], Temp_Gps_Gprs.Date[1], Temp_Gps_Gprs.Date[2], \
				            Temp_Gps_Gprs.Time[0], Temp_Gps_Gprs.Time[1], Temp_Gps_Gprs.Time[2], gps_log );
			}else
			{
				gps_sd_coutner = 0;
			}
			// rt_kprintf("\r\ntmp_time=%d,fomer_time=%d,delta_time=%d,Current=%d",tmp_time_secnonds,fomer_time_seconds,delta_time_seconds,Current_SD_Duration);
		}
	}

	//------------------------------ do this every  second-----------------------------------------
	memcpy( (char*)&Gps_Gprs, (char*)&Temp_Gps_Gprs, sizeof( Temp_Gps_Gprs ) );

	//------  ����Χ�� �ж�  ----------


	/*  if((Temp_Gps_Gprs.Time[2]%20)==0) //   ��֤ʱ�����Բ�ε���Χ��
	   {
	      CycleRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
	   //rt_kprintf("\r\n --- �ж�Բ�ε���Χ��");
	   }	*/
	// if((Temp_Gps_Gprs.Time[2]==5)||(Temp_Gps_Gprs.Time[2]==25)||(Temp_Gps_Gprs.Time[2]==45)) //

	//  if(Temp_Gps_Gprs.Time[2]%2==0)//    ��֤ʱҪ��2 ��
	// {
	     RectangleRail_Judge( Temp_Gps_Gprs.Latitude, Temp_Gps_Gprs.Longitude );
	//rt_kprintf("\r\n -----�жϾ��ε���Χ��");
	//    }

	if( line_warn_enable == 1 )
	{
		RouteLineWarn_judge( Temp_Gps_Gprs.Latitude, Temp_Gps_Gprs.Longitude );
	}

	/*
	     if((Temp_Gps_Gprs.Time[2]%3)==0) //     ·���ж�
	   {
	         // printf("\r\n --- �ж�Բ�ε���Χ��");
	          RouteRail_Judge(Temp_Gps_Gprs.Latitude,Temp_Gps_Gprs.Longitude);
	   }
	 */
	//rt_kprintf("\r\n Delta_seconds %d \r\n",delta_time_seconds);

	
    EverySecond_Time_Get=0; // clear
    Longi_Get=0;
	Lati_Get=0; 
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void local_time( void )
{
	if( local_trig == 1 )                                                           //  GPS ʱ��
	{
		local_timer++;

		if( ( SleepState == 1 ) && ( local_timer == ( Current_SD_Duration - 5 ) ) ) //  --  ����ʱ �ȷ���Ȩ
		{
			SleepConfigFlag = 1;                                                    //����ǰ5 ����һ����Ȩ
		}

		if( ( local_timer >= Current_SD_Duration ) && ( local_trig == 0 ) )         //limitSend_idle
		{
			PositionSD_Enable( );
			// if(SleepState==1)
			Current_UDP_sd	= 1;
			local_timer		= 0;                                                    // clear

			memcpy( BakTime, CurrentTime, 3 );                                      // update
			rt_kprintf( "\r\n-->local time trig\r\n" );
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  Exact_Test_time( void )
{
	if( local_trig == 2 )                                                                                                                                                                               //  GPS ʱ��
	{
		local_timer++;
		if( local_timer >= 9 )                                                                                                                                                                          //900ms
		{
			local_timer = 0;
			PositionSD_Enable( );
			// if(SleepState==1)
			Current_UDP_sd	= 1;
			local_timer		= 0;                                                                                                                                                                        // clear
			memcpy( BakTime, CurrentTime, 3 );                                                                                                                                                          // update
			rt_kprintf( "\r\n->DWJD-\r\n" );
		}
	}
}

//----------------------------------------------------------------------------------------------------
void  SpeedSensorProcess( void )                                                                                                                                                                        // ͨ���������ٶȴ�������� �ٶ� ���������
{
	u32 Distance_1s_m = 0;                                                                                                                                                                              // һ�������� ������
//  u32 sp=0;
	//  1. ��Kֵ�����ٶ�   -----------------------------------------------------------


	/*
	      K ��ʾ ÿ���� ������
	      1��/������ :   K/1000
	      Delta_1s_Plus: ÿ���Ӳɼ�����������
	      ÿ����ʻ����:  Delta_1s_Plus *1000/K
	      => Delta_1s_Plus *1000/K*3.6*10 ��λ:0.1  km/h  =>Delta_1s_Plus *36000/K  ��λ:0.1 km/h
	 */
	Speed_cacu		= ( Delta_1s_Plus * 36000 ) / JT808Conf_struct.Vech_Character_Value;                                                                                                                // ͨ������õ����ٶ�
	GPS_speed		= Speed_cacu;                                                                                                                                                                       //�Ѽ���õ��Ĵ������ٶȸ� Э�� �Ĵ���
	Distance_1s_m	= ( Delta_1s_Plus * 1000 ) / JT808Conf_struct.Vech_Character_Value;                                                                                                                 // ÿ�����ж�����
	// 2. ����������  -------------------------------------------------------------
	//------------------------------------
	ModuleStatus |= Status_Pcheck;

	//------- GPS  ��̼���  --------
	JT808Conf_struct.Distance_m_u32 += Distance_1s_m;                                                                                                                                                   // ����3600 ��m/s
	if( JT808Conf_struct.Distance_m_u32 > 0xFFFFFF )
	{
		JT808Conf_struct.Distance_m_u32 = 0;                                                                                                                                                            //������ô����
	}
	// ------------------------------------------------------------------------------
}

//---------------------------------------------------------------------------------------------------
void K_AdjustUseGPS( u32 sp, u32 sp_DISP )                                                                                                                                                              // ͨ��GPS У׼  K ֵ  (������ʻ1KM ��������Ŀ)
{
	u32 Reg_distance	= 0;
	u32 Reg_plusNum		= 0;
	u16 i				= 0;

	if( JT808Conf_struct.DF_K_adjustState )                                                                                                                                                             // ֻ��ûУ׼ʱ����Ч
	{
		return;
	}

	Speed_Rec = (u8)( sp_DISP / 10 );                                                                                                                                                                   // GPS�ٶ�    ��λ:km/h
	// -------	Ҫ���ٶ���60��65km/h  -------------
	if( ( ( Speed_Rec >= Speed_area ) && ( Speed_Rec <= ( Speed_area + 8 ) ) ) || ( ( Speed_Rec >= 40 ) && ( Speed_Rec <= ( 40 + 8 ) ) ) || ( ( Speed_Rec >= 70 ) && ( Speed_Rec <= ( 70 + 8 ) ) ) )    // Speed_area=60
	// if(Speed_Rec>=Speed_area)
//   if((Speed_Rec>=40)&&(Speed_Rec<=48))   // Speed_area=60
	{
		Spd_adjust_counter++;
		if( Spd_adjust_counter > K_adjust_Duration )                                                                                                                                                    //�������ٶ���60~65����Ϊ�Ѿ���������
		{
			// �û�ȡ��������GPS�ٶ���Ϊ��׼���͸��ݴ���������������ٶȣ���Kֵ��У׼
			Reg_distance	= 0;                                                                                                                                                                        // clear
			Reg_plusNum		= 0;                                                                                                                                                                        // clear
			for( i = 0; i < K_adjust_Duration; i++ )
			{
				Reg_distance	+= Former_gpsSpd[i];                                                                                                                                                    // ����3.6km/h ��ʾ���������˶�����
				Reg_plusNum		+= Former_DeltaPlus[i];
			}


			/*
			     ��һ���ж�  �� ����ٶȴ����������ã� ��ô���أ�
			 */
			if( Reg_plusNum < 20 )
			{
				Spd_adjust_counter = 0;
				rt_kprintf( "\r\n    �ٶȴ����� û������!\r\n" );
				return;
			}
			//===================================================================
			// ת���ɸ���GPS�ٶȼ�����ʻ�˶����ף�(�ܾ���) ������ڳ���3.6 ��Ϊ�˼��㷽�� ��x10  �ٳ���36
			Reg_distance = (u32)( Reg_distance * 10 / 36 ); // ת���ɸ���GPS�ٶȼ�����ʻ�˶����ף�(�ܾ���)
			// (Reg_plusNum/Reg_distance) ��ʾ�������������Ծ���(��)= ÿ�ײ������ٸ����� ����ΪKֵ��1000��������������Ӧ�ó���1000
			JT808Conf_struct.Vech_Character_Value = 1000 * Reg_plusNum / Reg_distance;
			//-------  �洢�µ�����ϵ�� --------------------------------
			JT808Conf_struct.DF_K_adjustState	= 1;        // clear  Flag
			ModuleStatus						|= Status_Pcheck;
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );

			Spd_adjust_counter = 0;                         // clear  counter
		}else
		{                                                   //-------- ��¼�涨ʱ���ڵ���������GPS�ٶ�----------
			Former_gpsSpd[Spd_adjust_counter]		= Speed_Rec;
			Former_DeltaPlus[Spd_adjust_counter]	= Delta_1s_Plus;
		}
	}else
	{
		Spd_adjust_counter = 0;                             // ֻҪ�ٶȳ���Ԥ�跶Χ ��������0
	}
}

//==================================================================================================
// �ڶ����� :   �������ⲿ������״̬���
//==================================================================================================


/*
     -----------------------------
     2.1   ��Э����صĹ��ܺ���
     -----------------------------
 */

void IMSI_Convert_SIMCODE( void )
{
	SIM_code[0] = IMSI_CODE[3] - 0X30;
	SIM_code[0] <<= 4;
	SIM_code[0] |= IMSI_CODE[4] - 0X30;

	SIM_code[1] = IMSI_CODE[5] - 0X30;
	SIM_code[1] <<= 4;
	SIM_code[1] |= IMSI_CODE[6] - 0X30;

	SIM_code[2] = IMSI_CODE[7] - 0X30;
	SIM_code[2] <<= 4;
	SIM_code[2] |= IMSI_CODE[8] - 0X30;

	SIM_code[3] = IMSI_CODE[9] - 0X30;
	SIM_code[3] <<= 4;
	SIM_code[3] |= IMSI_CODE[10] - 0X30;

	SIM_code[4] = IMSI_CODE[11] - 0X30;
	SIM_code[4] <<= 4;
	SIM_code[4] |= IMSI_CODE[12] - 0X30;

	SIM_code[5] = IMSI_CODE[13] - 0X30;
	SIM_code[5] <<= 4;
	SIM_code[5] |= IMSI_CODE[14] - 0X30;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void GNSS_RawDataTrans_Init( u8 mode )  // �������
{
	if( mode )
	{
		GNSS_rawdata.WorkEnable = 1;    //��ʼ��ʹ��
	}else
	{
		GNSS_rawdata.WorkEnable = 0;    //��ʼ����ʹ��
	}
	GNSS_rawdata.first_record	= 1;
	GNSS_rawdata.rd_num			= 0;
	GNSS_rawdata.wr_num			= 0;
	GNSS_rawdata.save_status	= 0;
	GNSS_rawdata.Raw_wr			= 0;
}

/*
     -----------------------------
    2.4  ��ͬЭ��״̬�Ĵ����仯
     -----------------------------
 */

void StatusReg_WARN_Enable( void )
{
	//     ��������״̬�� �Ĵ����ı仯
	Warn_Status[3] |= 0x01;     //BIT( 0 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_WARN_Clear( void )
{
	//     ��������Ĵ���
	Warn_Status[3] &= ~0x01;    //BIT( 0 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_ACC_ON( void )
{                               //    ACC ��
	Car_Status[3] |= 0x01;      //  Bit(0)     Set  1  ��ʾ ACC��
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_ACC_OFF( void )
{                               //    ACC ��
	Car_Status[3] &= ~0x01;     //  Bit(0)     Set  01  ��ʾ ACC��
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_POWER_CUT( void )
{                               //  ����Դ�Ͽ�
	Warn_Status[2]	|= 0x01;    //BIT( 0 );
	ModuleStatus	|= Status_Battery;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_POWER_NORMAL( void )
{                               // ����Դ����
	Warn_Status[2]	&= ~0x01;   //BIT( 0 );
	ModuleStatus	&= ~Status_Battery;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_GPS_A( void )
{                               // GPS ��λ
	if( GPS_getfirst == 0 )
	{
#ifdef LCD_5inch
		DwinLCD.Type = LCD_SETTIME;
#endif
		Shoushi = 1;
		rt_kprintf( "\r\n firs A  Set RTC\r\n" );
	}
	GPS_getfirst	= 1;
	Car_Status[3]	|= 0x02;    //Bit(1)
	ModuleStatus	|= Status_GPS;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_GPS_V( void )
{                               //  GPS ����λ
	Car_Status[3]	&= ~0x02;   //Bit(1)
	ModuleStatus	&= ~Status_GPS;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_SPD_WARN( void )
{                               //  ���ٱ���
	Warn_Status[3] |= 0x02;     //BIT( 1 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_SPD_NORMAL( void )
{                               //  �ٶ�����
	Warn_Status[3] &= ~0x02;    //BIT( 1 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_Relay_Cut( void )
{                               // ���Ͷϵ�״̬
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_Relay_Normal( void )
{                               //  ���͵�״̬����
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void StatusReg_Default( void )
{                               //   ״̬�Ĵ�����ԭĬ������
	Warn_Status[0]	= 0x00;     //HH
	Warn_Status[1]	= 0x00;     //HL
	Warn_Status[2]	= 0x00;     //LH
	Warn_Status[3]	= 0x00;     //LL
}

//==================================================================================================
// �������� :   ������GPRS���ߴ������Э��
//==================================================================================================
void BD_send_Init(void)
{
  BDSD.Enable_Working=0;
  BDSD.read=0;
  BDSD.SendFlag=0; 
  BDSD.wait__resentTimer=0;
  BDSD.write=0;

}

void BD_send_Mque_Tx(T_GPS_Info_GPRS Gps_Gprs)
{
  
  u16 counter_mainguffer, i;
  u32 Dis_01km=0;
   
  
	  //----------------------- Save GPS --------------------------------------
	  memset( (u8*)BDsd_tx.body, 0, sizeof(BDsd_tx.body) );
	  BDsd_tx.wr= 0;
	  BDsd_tx.wr++;// ����
	  //------------------------------- Stuff ----------------------------------------
	  counter_mainguffer = BDsd_tx.wr;
	  // 1. �澯״̬   4 Bytes
	  memcpy( ( char* ) BDsd_tx.body+ BDsd_tx.wr, ( char* )Warn_Status, 4 );
	  BDsd_tx.wr += 4;
	  // 2. ����״̬   4 Bytes
	  memcpy( ( char* )BDsd_tx.body + BDsd_tx.wr, ( char* )Car_Status, 4 );
	  BDsd_tx.wr += 4;
	  // 3.   γ��	   4 Bytes
	  memcpy( ( char* )BDsd_tx.body + BDsd_tx.wr, ( char* )Gps_Gprs.Latitude, 4 );   //γ��   modify by nathan
	  BDsd_tx.wr += 4;
	  // 4.   ����	   4 Bytes
	  memcpy( ( char* )BDsd_tx.body + BDsd_tx.wr, ( char* )Gps_Gprs.Longitude, 4 );  //����	����  Bit 7->0	  ���� Bit 7 -> 1
	  BDsd_tx.wr += 4;
	  // 5.  �߶�	2 Bytes    m
	  BDsd_tx.body[BDsd_tx.wr++] = ( GPS_Hight >> 8 ); 							  // High
	  BDsd_tx.body[BDsd_tx.wr++] = (u8)GPS_Hight;									  // Low
	  // 6.  �ٶ�	2 Bytes 	0.1Km/h
	  BDsd_tx.body[BDsd_tx.wr++] = ( Speed_gps >> 8 ); 							  // High
	  BDsd_tx.body[BDsd_tx.wr++] = (u8)Speed_gps;									  // Low
	  // 7.  ����	2 Bytes 	  1��
	  BDsd_tx.body[BDsd_tx.wr++] = ( GPS_direction >> 8 ); 						  //High
	  BDsd_tx.body[BDsd_tx.wr++] = GPS_direction;									  // Low
	  // 8.  ����ʱ��	6 Bytes
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	  BDsd_tx.body[BDsd_tx.wr++] = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );





   #if 1
	  //--------------------------------------------------------------	  
	  //----------- ������Ϣ  ------------
	  //  ������Ϣ 1  -----------------------------
	  //  ������Ϣ ID
	  BDsd_tx.body[BDsd_tx.wr++] = 0x03; // ��ʻ��¼�ǵ��ٶ�
	  //  ������Ϣ����
	  BDsd_tx.body[BDsd_tx.wr++] = 2;
	  //  ����
	  BDsd_tx.body[BDsd_tx.wr++]   = (u8)( Speed_cacu >> 8 );
	  BDsd_tx.body[BDsd_tx.wr++]   = (u8)( Speed_cacu );
	  //rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu);
	  //  ������Ϣ 2  -----------------------------
	  //  ������Ϣ ID
	  BDsd_tx.body[BDsd_tx.wr++] = 0x01; // ���ϵ���ʻ���
	  //  ������Ϣ����
	  BDsd_tx.body[BDsd_tx.wr++] = 4;
	  //  ����
	  Dis_01km							  = JT808Conf_struct.Distance_m_u32 / 100;
	  BDsd_tx.body[BDsd_tx.wr++]   = ( Dis_01km >> 24 );
	  BDsd_tx.body[BDsd_tx.wr++]   = ( Dis_01km >> 16 );
	  BDsd_tx.body[BDsd_tx.wr++]   = ( Dis_01km >> 8 );
	  BDsd_tx.body[BDsd_tx.wr++]   = Dis_01km;
	  
	  //  ������Ϣ 3
	  if( Warn_Status[1] & 0x10 )
	  {
		  //  ������Ϣ ID
		  BDsd_tx.body[BDsd_tx.wr++] = 0x12; //	��������/·�߱���
		  //  ������Ϣ����
		  BDsd_tx.body[BDsd_tx.wr++] = 6;
		  //  ����
		  BDsd_tx.body[BDsd_tx.wr++]   = InOut_Object.TYPE;
		  BDsd_tx.body[BDsd_tx.wr++]   = ( InOut_Object.ID >> 24 );
		  BDsd_tx.body[BDsd_tx.wr++]   = ( InOut_Object.ID >> 16 );
		  BDsd_tx.body[BDsd_tx.wr++]   = ( InOut_Object.ID >> 8 );
		  BDsd_tx.body[BDsd_tx.wr++]   = InOut_Object.ID;
		  BDsd_tx.body[BDsd_tx.wr++]   = InOut_Object.InOutState;
		  rt_kprintf( "\r\n ----- 0x0200 current ������Ϣ \r\n" );
	  }
	  
	  //  ������Ϣ4
	  if( Warn_Status[3] & 0x02 )
	  {
		  //  ������Ϣ ID
		  BDsd_tx.body[BDsd_tx.wr++] = 0x11; 				  //  ��������/·�߱���
		  //  ������Ϣ����
		  BDsd_tx.body[BDsd_tx.wr++] = 1;
		  //  ����
		  BDsd_tx.body[BDsd_tx.wr++] = 0;					  //  ���ض�λ��
	  }
	  // 5. ������Ϣ
	  BDsd_tx.body[BDsd_tx.wr++]   = 0x25;					  //ID	��չ�����ź�״̬λ
	  BDsd_tx.body[BDsd_tx.wr++]   = 4;						  //LEN
	  BDsd_tx.body[BDsd_tx.wr++]   = 0x00;
	  BDsd_tx.body[BDsd_tx.wr++]   = 0x00;
	  BDsd_tx.body[BDsd_tx.wr++]   = 0x00;
	  BDsd_tx.body[BDsd_tx.wr++]   = BD_EXT.Extent_IO_status;
	  //6. ������Ϣ  :
	  if( SleepState == 1 ) 										  //����
	  {
		  BDsd_tx.body[BDsd_tx.wr++]   = 0x2A;				  //ID	 IO״̬λ��1��	 bit0:�������	 bit1:���ߡ�0������
		  BDsd_tx.body[BDsd_tx.wr++]   = 2;					  //LEN
		  BDsd_tx.body[BDsd_tx.wr++]   = 0x02;
		  BDsd_tx.body[BDsd_tx.wr++]   = 0;
	  }
	  //  ������Ϣ 7  -----------------------------
	  //  ������Ϣ ID
	  BDsd_tx.body[BDsd_tx.wr++] = 0x2B; 					  //ģ����
	  //  ������Ϣ����
	  BDsd_tx.body[BDsd_tx.wr++] = 4;
	  //  ����
	  BDsd_tx.body[BDsd_tx.wr++]   = ( AD_2through[1] >> 8 );  // AD1
	  BDsd_tx.body[BDsd_tx.wr++]   = AD_2through[1];
	  BDsd_tx.body[BDsd_tx.wr++]   = ( AD_2through[0] >> 8 );  // AD0
	  BDsd_tx.body[BDsd_tx.wr++]   = AD_2through[0];
	  //  ������Ϣ 8  -----------------------------
	  //  ������Ϣ ID
	  BDsd_tx.body[BDsd_tx.wr++] = 0x30; 					  //����ͨ�������ź�ǿ��
	  //  ������Ϣ����
	  BDsd_tx.body[BDsd_tx.wr++] = 1;
	  //  ����
	  BDsd_tx.body[BDsd_tx.wr++] = ModuleSQ;
	  
	  //  �����ź� 9  ---------------------------------
	  BDsd_tx.body[BDsd_tx.wr++]   = 0x31;					  // GNSS ��λ���ǿ���
	  BDsd_tx.body[BDsd_tx.wr++]   = 1;						  // len
	  BDsd_tx.body[BDsd_tx.wr++]   = Satelite_num;
	#endif  

		BDsd_tx.body[0]=BDsd_tx.wr;  

		//-------------  Caculate  FCS  -----------------------------------
		FCS_GPS_UDP=0;  
		for ( i = 0; i < BDsd_tx.wr; i++ )  
		{
				FCS_GPS_UDP ^= *( BDsd_tx.body + i ); 
		}			   //���ϱ����ݵ�����
		BDsd_tx.body[BDsd_tx.wr++] = FCS_GPS_UDP;  

	  //-------------------------------- stuff msg_queue------------------------------------------
	  rt_mq_send( &mq_BDsd, (void*)&BDsd_tx, BDsd_tx.wr+2); 
  
	  //OutPrint_HEX("BD-send",BDsd_tx.body,BDsd_tx.wr);

}

u8 BD_send_Mque_Rx(void)
{
    rt_err_t	res;

    memset(BDsd_rx.body,0,sizeof(BDsd_rx.body));
   	res = rt_mq_recv( &mq_BDsd, (void*)&BDsd_rx, 128,5 ); //�ȴ�100ms,ʵ���Ͼ��Ǳ䳤����ʱ,�100ms
	if( res == RT_EOK )                                                     //�յ�һ������
	{

               WatchDog_Feed();
       if( BDSD_SaveCycleGPS(BDSD.write,BDsd_rx.body,BDsd_rx.wr))
	     { //---- updata pointer   -------------		
			BDSD.write++;  	
		       if(BDSD.write>=Max_BDSD_Num)
		  	               BDSD.write=0;  
			DF_Write_RecordAdd(BDSD.write,BDSD.write,TYPE_BDsdAdd);   
			DF_delay_ms(20);  
	      //-------------------------------	
	        if(DispContent) 	
					       rt_kprintf("\r\n    BDsq succed\r\n");
            }  
	    else
	    	{
	    	  if(DispContent) 	
					       rt_kprintf("\r\n    BDsq fail\r\n");

	    	}
           //  OutPrint_HEX("BD-RX",BDsd_rx.body,BDsd_rx.wr);  
             return true;
	}

}


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void MangQU_true_create(T_GPS_Info_GPRS Gps_Gprs)
{
	
	u16 counter_mainguffer, i;


		//----------------------- Save GPS --------------------------------------
		memset( GPSsaveBuf, 0, 40 );
		GPSsaveBuf_Wr = 0;
		//------------------------------- Stuff ----------------------------------------
		counter_mainguffer = GPSsaveBuf_Wr;
		// 1. �澯״̬   4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Warn_Status, 4 );
		GPSsaveBuf_Wr += 4;
		// 2. ����״̬   4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Car_Status, 4 );
		GPSsaveBuf_Wr += 4;
		// 3.   γ��     4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Gps_Gprs.Latitude, 4 );   //γ��   modify by nathan
		GPSsaveBuf_Wr += 4;
		// 4.   ����     4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Gps_Gprs.Longitude, 4 );  //����    ����  Bit 7->0	���� Bit 7 -> 1
		GPSsaveBuf_Wr += 4;
		// 5.  �߶�	  2 Bytes    m
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( GPS_Hight >> 8 );                               // High
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)GPS_Hight;                                    // Low
		// 6.  �ٶ�	  2 Bytes     0.1Km/h
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( Speed_gps >> 8 );                               // High
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)Speed_gps;                                    // Low
		// 7.  ����	  2 Bytes	    1��
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( GPS_direction >> 8 );                           //High
		GPSsaveBuf[GPSsaveBuf_Wr++] = GPS_direction;                                    // Low
		// 8.  ����ʱ��	  6 Bytes
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );
		//-------------  Caculate  FCS  -----------------------------------
		FCS_GPS_UDP = 0;
		for( i = counter_mainguffer; i < 30; i++ )
		{
			FCS_GPS_UDP ^= *( GPSsaveBuf + i );
		}                                                           //���ϱ����ݵ�����
		GPSsaveBuf[30] = FCS_GPS_UDP;

		
		//-------------------------------- stuff msg_queue------------------------------------------
		rt_mq_send( &mq_MQBuBao, (void*)&GPSsaveBuf, 32); 

		//OutPrint_HEX("MQ-Genernate",GPSsaveBuf,32);
		
}

void MangQU_true_Save(void)
{
    rt_err_t	res;
	u8  Rx_MQ[40];

    memset(Rx_MQ,0,40);
   	res = rt_mq_recv( &mq_MQBuBao, (void*)&Rx_MQ, 32,5 ); //�ȴ�100ms,ʵ���Ͼ��Ǳ䳤����ʱ,�100ms
	if( res == RT_EOK )                                                     //�յ�һ������
	{
       	if( Api_cycle_write( GPSsaveBuf, 31 ) ) 
		{
			if( DispContent )
			{
				rt_kprintf( "\r\n    MQ Save succed wr=%d\r\n",cycle_write ); 
			}
		}else
		{
			WatchDog_Feed( );
			if( DispContent )
			{
				rt_kprintf( "\r\n MQ save fail\r\n" );
			}

			if( Api_cycle_write( GPSsaveBuf, 31 ) )
			{
				rt_kprintf( "\r\n MQ save retry ok\r\n" ); 
			}
		}
      // OutPrint_HEX("MQ-save",Rx_MQ,32);  
	}

}


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void MangQU_read( u32 packetnum, u8 *str, u32 length )
{
    u32  i=0,Size_pkt=0, pagein_offset=0;
    u8  latiBuf[8*MQ_PKNUM];  // 16*8
    u8   regstr[4];	
	u8   reg8str[8];
	u8	lati[4], longi[4];
	u32 lati_stuff	= 0, longi_stuff = 0;
	u16 spd_stuff	= 0;
	u8	time_stuff[3];
	u32 timervalue = 0;

	   Size_pkt=8*MQ_PKNUM;	
	memset( latiBuf, 0, sizeof( latiBuf ) );

	for( i = 0; i < Size_pkt; i++ )
	{
		latiBuf[i] = SST25V_ByteRead( ( (u32)( DF_MQ_Page ) * 512 ) + (u32)( packetnum * Size_pkt + i ) ); //512bytes һ����λ
	}
		//OutPrint_HEX("lati",latiBuf,175);
        // value start      18:13:14   18*3600+13*60+14=65594       
		timervalue=65594+packetnum*MQ_PKNUM; 
		for(i=0;i<MQ_PKNUM;i++)
		{
		     //--------- convert latitude ---------------------
			 memcpy(reg8str,latiBuf+8*i,8);
		//OutPrint_HEX("ascii",reg8str,8);
		//------------------------------------
		regstr[0]	= ( HexValue( reg8str[0] ) << 4 ) + HexValue( reg8str[1] );
		regstr[1]	= ( HexValue( reg8str[2] ) << 4 ) + HexValue( reg8str[3] );
		regstr[2]	= ( HexValue( reg8str[4] ) << 4 ) + HexValue( reg8str[5] );
		regstr[3]	= ( HexValue( reg8str[6] ) << 4 ) + HexValue( reg8str[7] );
		//------------------------------------
		// OutPrint_HEX("hex",regstr,4);

		//----------  ��Ϣ���� --------------------------
		Original_info[Original_info_Wr++]	= 0;
		Original_info[Original_info_Wr++]	= 32;
		//----------------0200 -----------------------
		// 1. �澯��־  4
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
		Original_info_Wr += 4;
		// 2. ״̬  4
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
		Original_info_Wr += 4;
		// 3.  γ��
		lati_stuff	= ( regstr[0] << 24 ) + ( regstr[1] << 16 ) + ( regstr[2] << 8 ) + ( regstr[3] ) - ( CSQ_counter % 25 ) + ( TIM1_Timer_Counter % 17 );  // need change	a  little
		lati[0]		= ( u8 )( lati_stuff >> 24 );
		lati[1]		= ( u8 )( lati_stuff >> 16 );
		lati[2]		= ( u8 )( lati_stuff >> 8 );
		lati[3]		= ( u8 )lati_stuff;
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )lati, 4 );                                                                              //γ��	 modify by nathan
		Original_info_Wr += 4;
		// 4.  ����

		longi_stuff = 116 * 1000000 - ( CSQ_counter % 12 ) + ( TIM1_Timer_Counter % 3 );
		longi[0]	= ( u8 )( longi_stuff >> 24 );
		longi[1]	= ( u8 )( longi_stuff >> 16 );
		longi[2]	= ( u8 )( longi_stuff >> 8 );
		longi[3]	= ( u8 )longi_stuff;
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )longi, 4 );                             //����	  ����	Bit 7->0   ���� Bit 7 -> 1
		Original_info_Wr += 4;
		// 5.  �߳�
		GPS_Hight							= 50 + ( CSQ_counter % 8 ) - ( TIM1_Timer_Counter % 6 );
		Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
		Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
		// 6.  �ٶ�	 0.1 Km/h
		spd_stuff							= 1080 - ( CSQ_counter % 10 ) + ( TIM1_Timer_Counter % 12 );    // need change	a  little  76
		Original_info[Original_info_Wr++]	= (u8)( spd_stuff >> 8 );                                       //(GPS_speed>>8);
		Original_info[Original_info_Wr++]	= (u8)( spd_stuff );                                            //GPS_speed;
		// 7. ����   ��λ 1��
		Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );                                       //High
		Original_info[Original_info_Wr++]	= GPS_direction;                                                // Low
		// 8.  ����ʱ��
		Original_info[Original_info_Wr++]	= 0x13;                                                         //(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);
		Original_info[Original_info_Wr++]	= 0x03;                                                         //((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10);
		Original_info[Original_info_Wr++]	= 0x05;                                                         // ((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);

			  timervalue=65594+packetnum*MQ_PKNUM+i;
		time_stuff[0]	= timervalue / 3600;
		time_stuff[1]	= ( timervalue % 3600 ) / 60;
		time_stuff[2]	= ( timervalue % 60 );
		// rt_kprintf("\r\n %d stufftime %d:%d:%d ",i,time_stuff[0],time_stuff[1],time_stuff[2]);

		Original_info[Original_info_Wr++]	= ( ( time_stuff[0] / 10 ) << 4 ) + ( time_stuff[0] % 10 ); //((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
		Original_info[Original_info_Wr++]	= ( ( time_stuff[1] / 10 ) << 4 ) + ( time_stuff[1] % 10 );
		Original_info[Original_info_Wr++]	= ( ( time_stuff[2] / 10 ) << 4 ) + ( time_stuff[2] % 10 );

		//----------- ������Ϣ  ------------
		//  ������Ϣ 1  -----------------------------
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x03;                                                       // ��ʻ��¼�ǵ��ٶ�
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 2;
		//  ����
		Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
		Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void latrd( u8* instr )
{
	u32 i = 0, Size_pkt = 0, pagein_offset = 0;
	u8	latiBuf[7 * MQ_PKNUM]; // 15*7
	u8	regstr[4];
	u8	reg8str[8];
	u16 packetnum = 0;

	Size_pkt = 7 * MQ_PKNUM;

	sscanf( instr, "%d", (u32*)&packetnum );
	rt_kprintf( "\r\n inputpacket %d \r\n", packetnum );

	memset( latiBuf, 0, sizeof( latiBuf ) );

	for( i = 0; i < Size_pkt; i++ )
	{
		latiBuf[i] = SST25V_ByteRead( ( (u32)( DF_Record_Page + 1 ) * 512 ) + (u32)( packetnum * Size_pkt + i ) ); //512bytes һ����λ
	}
	OutPrint_HEX( "lati", latiBuf, Size_pkt );

	for( i = 0; i < MQ_PKNUM; i++ )
	{
		reg8str[0] = '0';
		memcpy( reg8str + 1, latiBuf + 7 * i, 7 );
		OutPrint_HEX( "ascii", reg8str, 8 );
		//------------------------------------
		regstr[0]	= ( HexValue( reg8str[0] ) << 4 ) + HexValue( reg8str[1] );
		regstr[1]	= ( HexValue( reg8str[2] ) << 4 ) + HexValue( reg8str[3] );
		regstr[2]	= ( HexValue( reg8str[4] ) << 4 ) + HexValue( reg8str[5] );
		regstr[3]	= ( HexValue( reg8str[6] ) << 4 ) + HexValue( reg8str[7] );
		//------------------------------------
		OutPrint_HEX( "hex", regstr, 4 );
	}
}

FINSH_FUNCTION_EXPORT( latrd, latrd );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  MangquSave_GPS( void )
{
	u16 counter_mainguffer, i;
	u8	readbuf[MANGQsize]; //,regstatus;

	if( Current_UDP_sd && PositionSD_Status( ) )
	{
		PositionSD_Disable( );
		Current_UDP_sd = 0;
		//-------------------------------------------------------
		//1.   ʱ�䳬ǰ�ж�
		//  if(Time_FastJudge()==false)
		//		return ;
		//----------------------- Save GPS --------------------------------------
		memset( GPSsaveBuf, 0, 40 );
		GPSsaveBuf_Wr = 0;
		//------------------------------- Stuff ----------------------------------------
		counter_mainguffer = GPSsaveBuf_Wr;
		// 1. �澯״̬   4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Warn_Status, 4 );
		GPSsaveBuf_Wr += 4;
		// 2. ����״̬   4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Car_Status, 4 );
		GPSsaveBuf_Wr += 4;
		// 3.   γ��     4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Gps_Gprs.Latitude, 4 );   //γ��   modify by nathan
		GPSsaveBuf_Wr += 4;
		// 4.   ����     4 Bytes
		memcpy( ( char* )GPSsaveBuf + GPSsaveBuf_Wr, ( char* )Gps_Gprs.Longitude, 4 );  //����    ����  Bit 7->0	���� Bit 7 -> 1
		GPSsaveBuf_Wr += 4;
		// 5.  �߶�	  2 Bytes    m
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( GPS_Hight >> 8 );                               // High
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)GPS_Hight;                                    // Low
		// 6.  �ٶ�	  2 Bytes     0.1Km/h
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( Speed_gps >> 8 );                               // High
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)Speed_gps;                                    // Low
		// 7.  ����	  2 Bytes	    1��
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( GPS_direction >> 8 );                           //High
		GPSsaveBuf[GPSsaveBuf_Wr++] = GPS_direction;                                    // Low
		// 8.  ����ʱ��	  6 Bytes
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );
		//----------- ������Ϣ  ------------
		//  ������Ϣ 1  -----------------------------
		//	������Ϣ ID
		GPSsaveBuf[GPSsaveBuf_Wr++] = 0x03; // ��ʻ��¼�ǵ��ٶ�
		//	������Ϣ����
		GPSsaveBuf[GPSsaveBuf_Wr++] = 2;
		//	����
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)( Speed_cacu >> 8 );
		GPSsaveBuf[GPSsaveBuf_Wr++] = (u8)( Speed_cacu );

		//	 memcpy(MangQU.Buffer+MangQU.buf_wr,GPSsaveBuf,GPSsaveBuf_Wr);
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Reg_save_gps( u8 *instr )
{
	u8 wr = 0;
	// 1. �澯״̬   4 Bytes
	memcpy( ( char* )instr + wr, ( char* )Warn_Status, 4 );
	wr += 4;
	// 2. ����״̬   4 Bytes
	memcpy( ( char* )instr + wr, ( char* )Car_Status, 4 );
	wr += 4;
	// 3.   γ��     4 Bytes
	memcpy( ( char* )instr + wr, ( char* )Gps_Gprs.Latitude, 4 );   //γ��   modify by nathan
	wr += 4;
	// 4.   ����     4 Bytes
	memcpy( ( char* )instr + wr, ( char* )Gps_Gprs.Longitude, 4 );  //����    ����  Bit 7->0	���� Bit 7 -> 1
	wr += 4;
	// 5.  �߶�	  2 Bytes    m
	instr[wr++] = ( GPS_Hight >> 8 );                               // High
	instr[wr++] = (u8)GPS_Hight;                                    // Low
	// 6.  �ٶ�	  2 Bytes     0.1Km/h
	instr[wr++] = ( Speed_gps >> 8 );                               // High
	instr[wr++] = (u8)Speed_gps;                                    // Low
	// 7.  ����	  2 Bytes	    1��
	instr[wr++] = ( GPS_direction >> 8 );                           //High
	instr[wr++] = GPS_direction;                                    // Low
	// 8.  ����ʱ��	  6 Bytes
	instr[wr++] = ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	instr[wr++] = ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	instr[wr++] = ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	instr[wr++] = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	instr[wr++] = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	instr[wr++] = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

	return wr;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  App_mq_Read_Process( void )
{
	char		ch;
//-----------------------------------------------------
	rt_err_t	res;

	if( AppQue.sd_enable_flag == 1 )
	{
		Stuff_Normal_Data_0200H( AppQue.read_num );
		AppQue.sd_enable_flag = 2;
		rt_kprintf( "\r\n DWJD     %d  send\r\n", AppQue.read_num );
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void App_mq_NoAck_counter( void )           // Que  ��ʽ�� û���µ�Ӧ��ʱ�ж�
{
	if( ( AppQue.sd_enable_flag == 2 ) && ( app_que_enable == 1 ) )
	{
		AppQue.abnormal_counter++;
		if( AppQue.abnormal_counter >= 4 )  // 2s
		{
			AppQue.abnormal_counter = 0;
			AppQue.sd_enable_flag	= 1;
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void App_mq_SendTimer( void )
{
	if( ( app_que_enable == 1 ) && ( AppQue.sd_enable_flag == 0 ) )
	{
		AppQue.send_timer++;
		if( AppQue.send_timer >= 10 ) // 100ms
		{
			AppQue.send_timer		= 0;
			AppQue.abnormal_counter = 0;
			AppQue.sd_enable_flag	= 1;
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  App_mq_Resend( void )           // ����Queue �ط�
{
	if( AppQue.re_send_flag == 1 )
	{
		rt_kprintf( "\r\n + msg_resend ->app msg  wr=%d  rd=%d \r\n", AppQue.write_num, AppQue.read_num );
		Stuff_AppMq_Data_0200H( Rx_reg.body );
		AppQue.re_send_flag = 2;    // enable   wait for  Ack
		return true;
	}else
	{
		return false;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void MangQu_Timer( void )
{
	//------------------------------------------------------------
	if( ( JT808Conf_struct.Close_CommunicateFlag == 0 ))
	{
		//   �洢��ʽä��
		if( MangQU.Enable_SD_state == 1 )
		{
		               // ----- normal -----
						if( MangQU.Sd_flag == 0 )
						{
							MangQU.Sd_timer++;
								  if(MangQU.Sd_timer>=2) 
							{
								MangQU.Sd_timer = 0;
										 MangQU.NoAck_timer=0;
								MangQU.Sd_flag	= 1;
							}
						}
		              //------- no  ack  process -----
		              if(MangQU.Sd_flag==2)
		              	{
		              	  MangQU.NoAck_timer++;
						  if( MangQU.NoAck_timer>=5)  
						  {
						      MangQU.NoAck_timer=0;
							  MangQU.Sd_timer=0;
		                      MangQU.Sd_flag=1;		
							  rt_kprintf("\r\n MQ Noack rensend! \r\n");
						  }

		              	}
		 } 
	  // ����ä�������ӷ���ʱ��
	  	if( MQ_TrueUse.Enable_SD_state == 1 )
		{
		               // ----- normal -----
						if( MQ_TrueUse.Sd_flag == 0 )
						{
							MQ_TrueUse.Sd_timer++;
								  if(MQ_TrueUse.Sd_timer>=2) 
							{
								MQ_TrueUse.Sd_timer = 0;
										 MQ_TrueUse.NoAck_timer=0;
								MQ_TrueUse.Sd_flag	= 1;
							}
						}
		              //------- no  ack  process -----
		              if(MQ_TrueUse.Sd_flag==2)
		              	{
		              	  MQ_TrueUse.NoAck_timer++;
						  if( MQ_TrueUse.NoAck_timer>=2)  
						  {
						      MQ_TrueUse.NoAck_timer=0;
							  MQ_TrueUse.Sd_timer=0;
							  cycle_read=mangQu_read_reg;   //   ��ԭread���·���
				              MQ_TrueUse.Sd_flag=1; //enable����
							  rt_kprintf("\r\n MQ_true Noack rensend! \r\n");
						  }

		              	}
		 } 
				
	}
}

//----------------------------------------------------------------------
u8  Protocol_Head( u16 MSG_ID, u8 Packet_Type )
{
    u16  CurrentID=0; 
	//----  clear --------------
	Original_info_Wr = 0;
	//	1. Head

	//  original info
	Original_info[Original_info_Wr++]	= ( MSG_ID >> 8 );                                              // ��ϢID
	Original_info[Original_info_Wr++]	= (u8)MSG_ID;

	Original_info[Original_info_Wr++]	= 0x00;                                                         // �ְ������ܷ�ʽ��״̬λ
	Original_info[Original_info_Wr++]	= 28;                                                           // ��Ϣ�峤��   λ����Ϣ����Ϊ28���ֽ�

	memcpy( Original_info + Original_info_Wr, SIM_code, 6 );                                            // �ն��ֻ��� ���豸��ʶID	BCD
	Original_info_Wr += 6;


    switch(Recode_Obj.CMD)
    {
        case 0x08:CurrentID=0x8000+Recode_Obj.Current_pkt_num;
			      Original_info[Original_info_Wr++]   = ( CurrentID>> 8 ); 					  //��Ϣ��ˮ��
				  Original_info[Original_info_Wr++]   =CurrentID;
			       break;
		case 0x09:CurrentID=0x9000+Recode_Obj.Current_pkt_num;
					Original_info[Original_info_Wr++]	= ( CurrentID>> 8 );					//��Ϣ��ˮ��
					Original_info[Original_info_Wr++]	=CurrentID;

                   break;
		case 0x10:
			       CurrentID=0xA000+Recode_Obj.Current_pkt_num;
				   Original_info[Original_info_Wr++]   = ( CurrentID>> 8 ); 				   //��Ϣ��ˮ��
				   Original_info[Original_info_Wr++]   =CurrentID;

			       break;
		case 0x11:
			       CurrentID=0xB000+Recode_Obj.Current_pkt_num;
				   Original_info[Original_info_Wr++]   = ( CurrentID>> 8 ); 				   //��Ϣ��ˮ��
				   Original_info[Original_info_Wr++]   =CurrentID;
			       break;
		case 0x12:
			       CurrentID=0xC000+Recode_Obj.Current_pkt_num;
				   Original_info[Original_info_Wr++]   = ( CurrentID>> 8 ); 				   //��Ϣ��ˮ��
				   Original_info[Original_info_Wr++]   =CurrentID;
			       break;
		case  0x15:
			      CurrentID=0xD000+Recode_Obj.Current_pkt_num;
			      Original_info[Original_info_Wr++]   = ( CurrentID>> 8 ); 					  //��Ϣ��ˮ��
				  Original_info[Original_info_Wr++]   =CurrentID;
 
				   break;  
		default:
			     if(MSG_ID==MSG_0x0704)
			     	{                      
					  CurrentID=0x5000+MQ_TrueUse.PacketNum;
					  Original_info[Original_info_Wr++]   = ( CurrentID>> 8 );					  //��Ϣ��ˮ��
					  Original_info[Original_info_Wr++]   =CurrentID;

			     	}
				 else
			     if(MediaObj.Media_Type<3) // ����  ͼƬ����Ƶ
			     {
				    Original_info[Original_info_Wr++]   = ( JT808Conf_struct.Msg_Float_ID >> 8 ); 					  //��Ϣ��ˮ��
				    Original_info[Original_info_Wr++]   = JT808Conf_struct.Msg_Float_ID;
			     }
				  break;

    } 


	if( Packet_Type == Packet_Divide )
	{
		switch( MediaObj.Media_Type )
		{
			case 0:                                                                                     // ͼ��
				MediaObj.Media_totalPacketNum	= Photo_sdState.Total_packetNum;                        // ͼƬ�ܰ���
				MediaObj.Media_currentPacketNum = Photo_sdState.SD_packetNum;                           // ͼƬ��ǰ����
				MediaObj.Media_ID				= 1;                                                    //  ��ý��ID
				MediaObj.Media_Channel			= Camera_Number;                                        // ͼƬ����ͷͨ����
                
				
			    CurrentID=0xF000+Camera_Number*0x0100+Photo_sdState.SD_packetNum;
				Original_info[Original_info_Wr++]=( CurrentID>>8); //��Ϣ��ˮ��
				Original_info[Original_info_Wr++]=	CurrentID; 			   

				break;
			case 1:                                                                                     // ��Ƶ
				MediaObj.Media_totalPacketNum	= Sound_sdState.Total_packetNum;                        // ��Ƶ�ܰ���
				MediaObj.Media_currentPacketNum = Sound_sdState.SD_packetNum;                           // ��Ƶ��ǰ����
				MediaObj.Media_ID				= 1;                                                    //  ��ý��ID
				MediaObj.Media_Channel			= 1;                                                    // ��Ƶͨ����
                CurrentID=0xE000+Sound_sdState.SD_packetNum;
			    Original_info[Original_info_Wr++]=( CurrentID>>8); //��Ϣ��ˮ��
	            Original_info[Original_info_Wr++]=  CurrentID; 
				break;
			case 2:                                                                                     // ��Ƶ
				MediaObj.Media_totalPacketNum	= Video_sdState.Total_packetNum;                        // ��Ƶ�ܰ���
				MediaObj.Media_currentPacketNum = Video_sdState.SD_packetNum;                           // ��Ƶ��ǰ����
				MediaObj.Media_ID				= 1;                                                    //  ��ý��ID
				MediaObj.Media_Channel			= 1;                                                    // ��Ƶͨ����
				break;
			case  3:                                                                                    //�г���¼��
				MediaObj.Media_totalPacketNum	= Recode_Obj.Total_pkt_num;                             // ��¼���ܰ���
				MediaObj.Media_currentPacketNum = Recode_Obj.Current_pkt_num;                           // ��¼�ǵ�ǰ����

				break;
			case  4:                                                                                    // ä������
				MediaObj.Media_totalPacketNum	= BlindZone_sdState.Total_packetNum;                    // ä���ܰ���
				MediaObj.Media_currentPacketNum = BlindZone_sdState.SD_packetNum;                       // ä����ǰ����

				break;
			default:
				return false;
		}

		Original_info[Original_info_Wr++]	= ( MediaObj.Media_totalPacketNum & 0xff00 ) >> 8;          //��block
		Original_info[Original_info_Wr++]	= (u8)MediaObj.Media_totalPacketNum;                        //��block

		Original_info[Original_info_Wr++]	= ( ( MediaObj.Media_currentPacketNum ) & 0xff00 ) >> 8;    //��ǰblock
		Original_info[Original_info_Wr++]	= (u8)( ( MediaObj.Media_currentPacketNum ) & 0x00ff );     //��ǰblock
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Protocol_End( u8 Packet_Type, u8 LinkNum )
{
	u16 packet_len	= 0;
	u16 i			= 0;    //Ҫ���͵�UDP �������ݵĳ���
	u8	Gfcs		= 0;
	u16 Msg_bodyLen = 0;    //  Э�������Ϣֻ��ʾ��Ϣ��     ��������Ϣͷ ��ϢͷĬ�ϳ�����12 , �ְ���Ϣͷ���� 20

	Gfcs = 0;               //  �������Ϣͷ��ʼ��У��ǰ���ݵ�����  808Э��У��  1Byte
	//---  ��д��Ϣ���� ---
	if( Packet_Normal == Packet_Type )
	{
		Msg_bodyLen			= Original_info_Wr - 12;
		Original_info[2]	= ( Msg_bodyLen >> 8 ); 
		Original_info[3]	= Msg_bodyLen;
	}else
	if( Packet_Divide == Packet_Type )
	{
		Msg_bodyLen = Original_info_Wr - 16;
		// rt_kprintf("\r\n Divide Infolen=%d  \r\n",Msg_bodyLen);
		Original_info[2]	= ( Msg_bodyLen >> 8 ) | 0x20; // Bit 13  0x20 ����Bit 13
		Original_info[3]	= Msg_bodyLen;
	}
	//---- ����У��  -----
/*�г���¼�Ƕ������ʱ��ֻ�����ż�У��*/

	for( i = 0; i < Original_info_Wr; i++ )
	{
		Gfcs ^= Original_info[i];
	}
	Original_info[Original_info_Wr++] = Gfcs;   // ��дGУ��λ

	// 1.stuff start
	GPRS_infoWr_Tx				= 0;
	GPRS_info[GPRS_infoWr_Tx++] = 0x7e;         // Start ��ʶλ
	if( Packet_Divide == Packet_Type )
	{
		//rt_kprintf("\r\n Tx=%d  Divide Infolen=%d  \r\n",GPRS_infoWr_Tx,Original_info_Wr);


		/* rt_kprintf("\r\n PacketContent: ");
		    for(i=0;i<Original_info_Wr;i++)
		                    rt_kprintf(" %X",Original_info[i]);
		    rt_kprintf("\r\n");
		 */
	}
	// 2.  convert
	packet_len		= Protocol_808_Encode( GPRS_info + GPRS_infoWr_Tx, Original_info, Original_info_Wr );
	GPRS_infoWr_Tx	+= packet_len;
	if( Packet_Divide == Packet_Type )
	{
		//rt_kprintf("\r\n Divide  Send Infolen=%d  \r\n",packet_len);


		/* rt_kprintf("\r\n EncodeContent: ");
		   for(i=0;i<packet_len;i++)
		                      rt_kprintf(" %X",GPRS_info[i+1]);
		   rt_kprintf("\r\n");
		 */
		//rt_kprintf("\r\n GPRStx  Send Infolen=%d  \r\n",GPRS_infoWr_Tx+1);
	}
	GPRS_info[GPRS_infoWr_Tx++] = 0x7e; //  End  ��ʶ
	//  4. Send

	// 4.1 ������Ϣ����1
	// 4.2   MsgQueue
	WatchDog_Feed( );

#ifdef BITTER
	rt_kprintf( "\r\n App to GSM info:" );
	for( i = 0; i < GPRS_infoWr_Tx; i++ )
	{
		rt_kprintf( " %02X", GPRS_info[i] );
	}
	rt_kprintf( "\r\n" );
#endif

	Gsm_rxAppData_SemRelease( GPRS_info, GPRS_infoWr_Tx, LinkNum );
	//--------��Ϣ��� ���� --------
	JT808Conf_struct.Msg_Float_ID++;
	//------------------------------
}

u8  Stuff_BDSD_0200H(void)  
{
  u8 spd_sensorReg[2];
  u8  rd_infolen=0;
 //  1. Head  
   if(!Protocol_Head(MSG_0x0200,Packet_Normal)) 
 	  return false;  
 // 2. content 
 WatchDog_Feed();
 if( BDSD_ReadCycleGPS( BDSD.read,Original_info+Original_info_Wr, 128)==false)              
  {
     rt_kprintf("\r\n  ��ȡ false\r\n "); 
    return false;
  }
 // ��ȡ��Ϣ����
 rd_infolen=Original_info[Original_info_Wr];
 //OutPrint_HEX("read -1",Original_info+Original_info_Wr,rd_infolen+1);
 memcpy(Original_info+Original_info_Wr,Original_info+Original_info_Wr+1,rd_infolen);
 //OutPrint_HEX("read -2",Original_info+Original_info_Wr,rd_infolen+1); 
 Original_info_Wr+=rd_infolen-1;   // ���ݳ��� �޳���һ�������ֽ� 

 
 //  3. Send 
 Protocol_End(Packet_Normal ,0);
   	
 return true; 

}


//------------------------------------------------------------------------------------
u8  Stuff_MangQu_Packet_Send_0704H( void )
{
	u16 Qsize = 0;

	// 1. Head
	if( !Protocol_Head( MSG_0x0704, Packet_Normal ) )
	{
		return false;                           //��λ���������ϴ�
	}
	//  2.  Content
	//-----  �ж����-------
	//  2.1    ���������
	Original_info[Original_info_Wr++]	= 0x00; //  10000=0x2710
	Original_info[Original_info_Wr++]	= MQ_PKNUM;

	//  2.2    ��������     1  ä������    0:   ����λ�������㱨
	Original_info[Original_info_Wr++] = 1;
	//  3.   ���ݰ�
	//  3.1   ���ݻ㱨�峤��+ ����

	// 3.2   ��������

	MangQU_read( MangQU.PacketNum, Original_info + Original_info_Wr, Qsize );
	//  Original_info_Wr += Qsize;

	//   4 .  end
	Protocol_End( Packet_Normal, 0 );

	rt_kprintf( "\r\n	0704H  current=%d  total=%d  \r\n", MangQU.PacketNum, BLIND_NUM / MQ_PKNUM );
}


u8  Stuff_MangQu_Packet_Send_0704H_True( void )
{
    /*  Note:
                      ��ȡ�洢ʱ�п��ܴ��ڶ�ȡУ�鲻��ȷ�Ŀ��ܣ�
                      ���������rd_error Ҫ��¼�������
                      ͬʱ���ļ�¼��ҲҪ�ݼ�
      */
    u8   Rd_error_Counter=0;  // ��ȡ���������
    u8   i=0;
	u8   rd_infolen=0;
	u8	 reg_128[128];  // 0704 �Ĵ���
   
	u16 Qsize = 0;
    u32   Area_ID=0;

       //0 .  congifrm   batch  num
        if(cycle_read<cycle_write)
        {
            delta_0704_rd=cycle_write-cycle_read;
			// �ж�ƫ���¼�����Ƿ��������¼��
            if(delta_0704_rd>=MQ_PKNUM)
				 delta_0704_rd=MQ_PKNUM;   

        }
		else   // write С�� read
		{	    
			delta_0704_rd=Max_CycleNum-cycle_read; 
		}

		 rt_kprintf("\r\n	 delat_0704=%d\r\n",delta_0704_rd); 

	// 1. Head
	 if(!Protocol_Head(MSG_0x0704,Packet_Normal)) 
		  return false; 
	  // 2. content   
	  //  2.1	 ���������
	  Original_info[Original_info_Wr++]   = 0x00; //  10000=0x2710
	  Original_info[Original_info_Wr++]   = delta_0704_rd;
	  
	  //  2.2	 ��������	  1  ä������	 0:   ����λ�������㱨
	  Original_info[Original_info_Wr++] = 1;
      
	  rd_infolen=Original_info_Wr;
	  //	2.3  ������Ŀ
		mangQu_read_reg=cycle_read;   //   �洢��ǰ�ļ�¼
		 for(i=0;i<delta_0704_rd;i++)
		 {
			//	 ��ȡ��Ϣ
		   memset(reg_128,0,sizeof(reg_128)); 
		   if( ReadCycleGPS(cycle_read,reg_128, 32)==false)	 // ʵ������ֻ��28���ֽ�
		   {  
			  Rd_error_Counter++;
			  continue; 
		   } 
		   cycle_read++; 
			//----------  ������Ϣ���� --------------------------		   
			Original_info[Original_info_Wr++]	= 0;
			Original_info[Original_info_Wr++]	= 40; // 28+ ������Ϣ����
	
			memcpy(Original_info+Original_info_Wr,reg_128,28);
			Original_info_Wr+=28; 	// ���ݳ��� �޳���һ�������ֽ�	

			//----------- ������Ϣ	------------
			//	������Ϣ 1	-----------------------------
			//	������Ϣ ID
			Original_info[Original_info_Wr++] = 0x03;														// ��ʻ��¼�ǵ��ٶ�
			//	������Ϣ����
			Original_info[Original_info_Wr++] = 2;
			//	����
			Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
			Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );


		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12; //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		Area_ID=1;
		Original_info[Original_info_Wr++]	= 2; //����Χ��
		Original_info[Original_info_Wr++]	= ( Area_ID >> 24 );
		Original_info[Original_info_Wr++]	= (Area_ID>> 16 );
		Original_info[Original_info_Wr++]	= ( Area_ID>> 8 );
		Original_info[Original_info_Wr++]	= Area_ID;
		Original_info[Original_info_Wr++]	= 0;  // ��Χ��


			
			//OutPrint_HEX("read -1"reg_128,rd_infolen+1);
			
			//OutPrint_HEX("read -2",reg_128+1,rd_infolen); 
	//==================================================	
		}
		// rt_kprintf("\r\n   rd_infolen=%d   delta=%d\r\n",rd_infolen,Original_info_Wr-rd_infolen); 

	//   3 .  end
	Protocol_End( Packet_Normal, 0 );

    
	if(DispContent)
			rt_kprintf("\r\n  ��λ���������ϴ�	delta=%d   read=%d  write=%d current=%d  ConstTotal=%d   CurrentTotal=%d\r\n",delta_0704_rd,cycle_read,cycle_write,MQ_TrueUse.PacketNum, Mq_total_pkg,CurrentTotal);  
	if(MQ_TrueUse.PacketNum==Mq_total_pkg)
		    rt_kprintf("\r\n  ä��True�洢�����Ѿ����\r\n");
	if(cycle_read==cycle_write) 
		{ 
           MQ_TrueUse.PacketNum=0;
		   MQ_TrueUse.Enable_SD_state=0; 
		   // ÿ��Ӧ��洢��ؼ�¼��Ŀ
		   DF_Write_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd);  
		   rt_kprintf("\r\nTrue Return Normal_place2\r\n");
		}

}

//--------------------------------------------------------------------------------------
u8  Stuff_DevCommmonACK_0001H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0001, Packet_Normal ) )
	{
		return false; //�ն�ͨ��Ӧ��
	}
	// 2. content  is null
	//   float ID
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;
	//  cmd  ID
	Original_info[Original_info_Wr++]	= (u8)( Centre_CmdID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Centre_CmdID;
	//   resualt
	Original_info[Original_info_Wr++] = SD_ACKflag.f_CentreCMDack_resualt;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Common CMD ACK! \r\n");
	}
	return true;
}

//-------------------------------------------------------------------------
u8  Stuff_RegisterPacket_0100H( u8 LinkNum )
{
	u8 i = 0;
// 1. Head
	if( !Protocol_Head( MSG_0x0100, Packet_Normal ) )
	{
		return false;
	}

	// 2. content
	//  province ID
	//  Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vechicle_Info.Dev_ProvinceID>>8);
	// Original_info[Original_info_Wr++]=(u8)JT808Conf_struct.Vechicle_Info.Dev_ProvinceID;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 12;               // �����
	//  county  ID
	// Original_info[Original_info_Wr++]=(u8)(JT808Conf_struct.Vechicle_Info.Dev_CityID>>8);
	//Original_info[Original_info_Wr++]=(u8)JT808Conf_struct.Vechicle_Info.Dev_CityID;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 107;              // ������
	//  ����ID
	memcpy( Original_info + Original_info_Wr, "70420", 5 ); //TW705
	Original_info_Wr += 5;
	//  �ն��ͺ� 20 Bytes      -- ����Э����������
	//memcpy(Original_info+Original_info_Wr,"Tianjin TCB TW701-BD",20);
	// Original_info_Wr+=20;
	memcpy( Original_info + Original_info_Wr, "TW705", 5 ); //  ��ʽ  ----
	// memcpy(Original_info+Original_info_Wr,"SW703",5);      // ����
	Original_info_Wr += 5;
	for( i = 0; i < 15; i++ )
	{
		Original_info[Original_info_Wr++] = 0x00;
	}

	//  �ն�ID   7 Bytes    ,
	memcpy( Original_info + Original_info_Wr, IMSI_CODE + 8, 7 );                   //000013601300001
	Original_info_Wr += 7;
	//   Original_info_Wr+=3;
	// Original_info[Original_info_Wr++]='0';
	// memcpy(Original_info+Original_info_Wr,IMSI_CODE+12,3);
	// Original_info_Wr+=3;
	//  ������ɫ
	Original_info[Original_info_Wr++] = JT808Conf_struct.Vechicle_Info.Dev_Color;   // ������ɫ
	//  ����
	/*
	memcpy( Original_info + Original_info_Wr, "TST000", 6 ); 
	Original_info_Wr					+= 6;
	Original_info[Original_info_Wr++]	= IMSI_CODE[14];
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	*/
	memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Vechicle_Info.Vech_Num,8);
	Original_info_Wr+=8;

	//  3. Send
	Protocol_End( Packet_Normal, LinkNum );
	if( DispContent )
	{
		rt_kprintf( "\r\n	SEND Reigster Packet! \r\n");
	}
	return true;
}

//--------------------------------------------------------------------------------------
u8  Stuff_DeviceHeartPacket_0002H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0002, Packet_Normal ) )
	{
		return false;
	}
	// 2. content  is null

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Send Dev Heart! \r\n");
	}
	return true;
}

//--------------------------------------------------------------------------------------
u8  Stuff_DeviceDeregister_0101H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0003, Packet_Normal ) )
	{
		return false; //�ն�ע��
	}
	// 2. content  is null
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Deregister  ע��! \r\n");
	}
	return true;
}

//------------------------------------------------------------------------------------
u8  Stuff_DevLogin_0102H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0102, Packet_Normal ) )
	{
		return false;                                                                                                               //�ն˼�Ȩ
	}
	// 2. content

	memcpy( Original_info + Original_info_Wr, JT808Conf_struct.ConfirmCode, strlen( (const char*)JT808Conf_struct.ConfirmCode ) );  // ��Ȩ��  string Type
	Original_info_Wr += strlen( (const char*)JT808Conf_struct.ConfirmCode );
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 ���ͼ�Ȩ! \r\n");
	}
	return true;
}

//--------------------------------------------------------------------------------------
u8  Stuff_Normal_Data_0200H( u16 InNUM )
{
	u8	status = 0, U8_STUFF = 0;
	u8	spd_sensorReg[2];
	u32 Dis_01km = 0;
	// ---- add  new  --------
	u8	lati[4], longi[4];
	u32 lati_stuff	= 0, longi_stuff = 0;
	u16 spd_stuff	= 0;
	u8	time_stuff[2];

	//  1. Head
	if( !Protocol_Head( MSG_0x0200, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//------------------------------- Stuff ----------------------------------------
	// 1. �澯��־  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
	Original_info_Wr += 4;
	// 3.	γ��
	lati_stuff	= Latiude_hex[InNUM] - ( CSQ_counter % 7 ) + ( TIM1_Timer_Counter % 2 );    // need change  a  little
	lati[0]		= ( u8 )( lati_stuff >> 24 );
	lati[1]		= ( u8 )( lati_stuff >> 16 );
	lati[2]		= ( u8 )( lati_stuff >> 8 );
	lati[3]		= ( u8 )lati_stuff;
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )lati, 4 );                  //γ��	modify by nathan
	Original_info_Wr += 4;
	// 4.	����

	longi_stuff = 116 * 1000000 - ( CSQ_counter % 8 ) + ( TIM1_Timer_Counter % 3 );
	longi[0]	= ( u8 )( longi_stuff >> 24 );
	longi[1]	= ( u8 )( longi_stuff >> 16 );
	longi[2]	= ( u8 )( longi_stuff >> 8 );
	longi[3]	= ( u8 )longi_stuff;
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )longi, 4 ); //����	 ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.	�߳�
	GPS_Hight							= 50 + ( CSQ_counter % 5 ) - ( Satelite_num % 4 );
	Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
	Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
	// 6.	�ٶ�	0.1 Km/h
	if( spd_dex[InNUM] > 11 )
	{
		spd_stuff = spd_dex[InNUM] - ( CSQ_counter % 10 ) + ( TIM1_Timer_Counter % 3 ); // need change  a  little  76
	}else
	{
		spd_stuff = spd_dex[InNUM];
	}
	Original_info[Original_info_Wr++]	= (u8)( spd_stuff >> 8 );                       //(GPS_speed>>8);
	Original_info[Original_info_Wr++]	= (u8)( spd_stuff );                            //GPS_speed;
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );                       //High
	Original_info[Original_info_Wr++]	= GPS_direction;                                // Low
	// 8.	����ʱ��
	Original_info[Original_info_Wr++]	= 0x13;                                         //(((Gps_Gprs.Date[0])/10)<<4)+((Gps_Gprs.Date[0])%10);
	Original_info[Original_info_Wr++]	= 0x03;                                         //((Gps_Gprs.Date[1]/10)<<4)+(Gps_Gprs.Date[1]%10);
	Original_info[Original_info_Wr++]	= 0x05;                                         // ((Gps_Gprs.Date[2]/10)<<4)+(Gps_Gprs.Date[2]%10);
	Original_info[Original_info_Wr++]	= 0x18;                                         //((Gps_Gprs.Time[0]/10)<<4)+(Gps_Gprs.Time[0]%10);
	time_stuff[0]						= 12 + ( 30 + InNUM ) / 60;
	Original_info[Original_info_Wr++]	= ( ( time_stuff[0] / 10 ) << 4 ) + ( time_stuff[0] % 10 );
	time_stuff[1]						= ( 30 + InNUM ) % 60;
	Original_info[Original_info_Wr++]	= ( ( time_stuff[1] / 10 ) << 4 ) + ( time_stuff[1] % 10 );

	//----------- ������Ϣ  ------------
	//  ������Ϣ 1  -----------------------------
	//	������Ϣ ID
	Original_info[Original_info_Wr++] = 0x03; // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++] = 2;
	//	����
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu);
	//  ������Ϣ 2  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x01; // ���ϵ���ʻ���
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Dis_01km							= JT808Conf_struct.Distance_m_u32 / 100;
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 24 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 16 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 8 );
	Original_info[Original_info_Wr++]	= Dis_01km;

	//  ������Ϣ 3
	if( Warn_Status[1] & 0x10 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12; //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		Original_info[Original_info_Wr++]	= InOut_Object.TYPE;
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 24 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 16 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 8 );
		Original_info[Original_info_Wr++]	= InOut_Object.ID;
		Original_info[Original_info_Wr++]	= InOut_Object.InOutState;
		rt_kprintf( "\r\n ----- 0x0200 current ������Ϣ \r\n" );
	}

	//  ������Ϣ4
	if( Warn_Status[3] & 0x02 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x11;                   //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 1;
		//  ����
		Original_info[Original_info_Wr++] = 0;                      //  ���ض�λ��
	}
	// 5. ������Ϣ
	Original_info[Original_info_Wr++]	= 0x25;                     //ID  ��չ�����ź�״̬λ
	Original_info[Original_info_Wr++]	= 4;                        //LEN
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= BD_EXT.Extent_IO_status;
	//6. ������Ϣ  :
	if( SleepState == 1 )                                           //����
	{
		Original_info[Original_info_Wr++]	= 0x2A;                 //ID   IO״̬λ��1��   bit0:�������   bit1:���ߡ�0������
		Original_info[Original_info_Wr++]	= 2;                    //LEN
		Original_info[Original_info_Wr++]	= 0x02;
		Original_info[Original_info_Wr++]	= 0;
	}
	//  ������Ϣ 7  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x2B;                       //ģ����
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Original_info[Original_info_Wr++]	= ( AD_2through[1] >> 8 );  // AD1
	Original_info[Original_info_Wr++]	= AD_2through[1];
	Original_info[Original_info_Wr++]	= ( AD_2through[0] >> 8 );  // AD0
	Original_info[Original_info_Wr++]	= AD_2through[0];
	//  ������Ϣ 8  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x30;                       //����ͨ�������ź�ǿ��
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 1;
	//  ����
	Original_info[Original_info_Wr++] = ModuleSQ;

	//  �����ź� 9  ---------------------------------
	Original_info[Original_info_Wr++]	= 0x31;                     // GNSS ��λ���ǿ���
	Original_info[Original_info_Wr++]	= 1;                        // len
	Original_info[Original_info_Wr++]	= Satelite_num;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );

	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_AppMq_Data_0200H( u8 *instr )
{
	u8	status		= 0, U8_STUFF = 0;
	u32 Dis_01km	= 0;
	//  1. Head
	if( !Protocol_Head( MSG_0x0200, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	memcpy( Original_info + Original_info_Wr, instr, 28 );
	Original_info_Wr += 28;                     // ����ֻ��28

	//----------- ������Ϣ  ------------
	//  ������Ϣ 1  -----------------------------
	//	������Ϣ ID
	Original_info[Original_info_Wr++] = 0x03;   // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++] = 2;
	//	����
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu);
	//  ������Ϣ 2  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x01; // ���ϵ���ʻ���
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Dis_01km							= JT808Conf_struct.Distance_m_u32 / 100;
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 24 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 16 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 8 );
	Original_info[Original_info_Wr++]	= Dis_01km;

	//  ������Ϣ 3
	if( Warn_Status[1] & 0x10 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12; //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		Original_info[Original_info_Wr++]	= InOut_Object.TYPE;
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 24 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 16 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 8 );
		Original_info[Original_info_Wr++]	= InOut_Object.ID;
		Original_info[Original_info_Wr++]	= InOut_Object.InOutState;
		rt_kprintf( "\r\n ----- 0x0200 current ������Ϣ \r\n" );
	}

	//  ������Ϣ4
	if( Warn_Status[3] & 0x02 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x11;                   //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 1;
		//  ����
		Original_info[Original_info_Wr++] = 0;                      //  ���ض�λ��
	}
	// 5. ������Ϣ
	Original_info[Original_info_Wr++]	= 0x25;                     //ID  ��չ�����ź�״̬λ
	Original_info[Original_info_Wr++]	= 4;                        //LEN
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= BD_EXT.Extent_IO_status;
	//6. ������Ϣ  :
	if( SleepState == 1 )                                           //����
	{
		Original_info[Original_info_Wr++]	= 0x2A;                 //ID   IO״̬λ��1��   bit0:�������   bit1:���ߡ�0������
		Original_info[Original_info_Wr++]	= 2;                    //LEN
		Original_info[Original_info_Wr++]	= 0x02;
		Original_info[Original_info_Wr++]	= 0;
	}
	//  ������Ϣ 7  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x2B;                       //ģ����
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Original_info[Original_info_Wr++]	= ( AD_2through[1] >> 8 );  // AD1
	Original_info[Original_info_Wr++]	= AD_2through[1];
	Original_info[Original_info_Wr++]	= ( AD_2through[0] >> 8 );  // AD0
	Original_info[Original_info_Wr++]	= AD_2through[0];
	//  ������Ϣ 8  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x30;                       //����ͨ�������ź�ǿ��
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 1;
	//  ����
	Original_info[Original_info_Wr++] = ModuleSQ;

	//  �����ź� 9  ---------------------------------
	Original_info[Original_info_Wr++]	= 0x31;                     // GNSS ��λ���ǿ���
	Original_info[Original_info_Wr++]	= 1;                        // len
	Original_info[Original_info_Wr++]	= Satelite_num;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );

	return true;
}

//------------------------------------------------------------------------------------
u8  Stuff_Current_Data_0200H( void )   //  ���ͼ�ʱ���ݲ��洢���洢����
{
	u8	status		= 0, U8_STUFF = 0;
	u32 Dis_01km	= 0;

	if( GPS_speed <= ( JT808Conf_struct.Speed_warn_MAX * 10 ) )
	{
		StatusReg_SPD_NORMAL( );
	}

	//  1. Head
	if( !Protocol_Head( MSG_0x0200, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//------------------------------- Stuff ----------------------------------------
	// 1. �澯��־  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
	Original_info_Wr += 4;
	// 3.  γ��
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Latitude, 4 );     //γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Longitude, 4 );    //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
	Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps >> 8 );                           //(GPS_speed>>8);
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps );                                //GPS_speed;
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );                           //High
	Original_info[Original_info_Wr++]	= GPS_direction;                                    // Low
	// 8.  ����ʱ��
	Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

	//----------- ������Ϣ  ------------
	//  ������Ϣ 1  -----------------------------
	//	������Ϣ ID
	Original_info[Original_info_Wr++] = 0x03; // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++] = 2;
	//	����
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu);
	//  ������Ϣ 2  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x01; // ���ϵ���ʻ���
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Dis_01km							= JT808Conf_struct.Distance_m_u32 / 100;
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 24 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 16 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 8 );
	Original_info[Original_info_Wr++]	= Dis_01km;

	//  ������Ϣ 3
	if( Warn_Status[1] & 0x10 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12; //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		Original_info[Original_info_Wr++]	= InOut_Object.TYPE;
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 24 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 16 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 8 );
		Original_info[Original_info_Wr++]	= InOut_Object.ID;
		Original_info[Original_info_Wr++]	= InOut_Object.InOutState;
		rt_kprintf( "\r\n ----- 0x0200 current ������Ϣ \r\n" );
	}

	if( Warn_Status[1] & 0x80 )                     // �� ·�߱���
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12;   //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		InOut_Object.ID						= 100;  // �µ���100
		Original_info[Original_info_Wr++]	= 4;
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 24 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 16 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 8 );
		Original_info[Original_info_Wr++]	= InOut_Object.ID;
		Original_info[Original_info_Wr++]	= 1;
		rt_kprintf( "\r\n ----- 0x0200 ��·���� ������Ϣ \r\n" );
	}

	//  ������Ϣ4
	if( Warn_Status[3] & 0x02 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x11;                   //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 1;
		//  ����
		Original_info[Original_info_Wr++] = 0;                      //  ���ض�λ��
	}
	// 5. ������Ϣ
	Original_info[Original_info_Wr++]	= 0x25;                     //ID  ��չ�����ź�״̬λ
	Original_info[Original_info_Wr++]	= 4;                        //LEN
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= BD_EXT.Extent_IO_status;
	//6. ������Ϣ  :
	if( SleepState == 1 )                                           //����
	{
		Original_info[Original_info_Wr++]	= 0x2A;                 //ID   IO״̬λ��1��   bit0:�������   bit1:���ߡ�0������
		Original_info[Original_info_Wr++]	= 2;                    //LEN
		Original_info[Original_info_Wr++]	= 0x02;
		Original_info[Original_info_Wr++]	= 0;
	}
	//  ������Ϣ 7  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x2B;                       //ģ����
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Original_info[Original_info_Wr++]	= ( AD_2through[1] >> 8 );  // AD1
	Original_info[Original_info_Wr++]	= AD_2through[1];
	Original_info[Original_info_Wr++]	= ( AD_2through[0] >> 8 );  // AD0
	Original_info[Original_info_Wr++]	= AD_2through[0];
	//  ������Ϣ 8  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x30;                       //����ͨ�������ź�ǿ��
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 1;
	//  ����
	Original_info[Original_info_Wr++] = ModuleSQ;

	//  �����ź� 9  ---------------------------------
	Original_info[Original_info_Wr++]	= 0x31;                     // GNSS ��λ���ǿ���
	Original_info[Original_info_Wr++]	= 1;                        // len
	Original_info[Original_info_Wr++]	= Satelite_num;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	return true;
}

//-----------------------------------------------------------------------
u8  Stuff_Current_Data_0201H( void )   //   λ����Ϣ��ѯ��Ӧ
{
	u8	status		= 0, U8_STUFF = 0;
	u32 Dis_01km	= 0;
	//  1. Head
	if( !Protocol_Head( MSG_0x0201, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//------------------------------- Stuff ----------------------------------------
	//   float ID                                                // ��Ӧ����Ӧ����Ϣ����ˮ��
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;

	// 1. �澯��־  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
	Original_info_Wr += 4;
	// 3.  γ��
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Latitude, 4 );     //γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Longitude, 4 );    //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
	Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Speed_gps;
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );   //High
	Original_info[Original_info_Wr++]	= GPS_direction;            // Low
	// 8.  ����ʱ��
	Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

	//----------- ������Ϣ  ------------
	//  ������Ϣ 1  -----------------------------
	//	������Ϣ ID
	Original_info[Original_info_Wr++] = 0x03; // ��ʻ��¼�ǵ��ٶ�
	//	������Ϣ����
	Original_info[Original_info_Wr++] = 2;
	//	����
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu >> 8 );
	Original_info[Original_info_Wr++]	= (u8)( Speed_cacu );
	//rt_kprintf("\r\n GPS�ٶ�=%d km/h , �������ٶ�=%d km/h\r\n",Speed_gps,Speed_cacu);
	//  ������Ϣ 2  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x01; // ���ϵ���ʻ���
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Dis_01km							= JT808Conf_struct.Distance_m_u32 / 100;
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 24 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 16 );
	Original_info[Original_info_Wr++]	= ( Dis_01km >> 8 );
	Original_info[Original_info_Wr++]	= Dis_01km;

	//  ������Ϣ 3
	if( Warn_Status[1] & 0x10 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x12; //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 6;
		//  ����
		Original_info[Original_info_Wr++]	= InOut_Object.TYPE;
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 24 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 16 );
		Original_info[Original_info_Wr++]	= ( InOut_Object.ID >> 8 );
		Original_info[Original_info_Wr++]	= InOut_Object.ID;
		Original_info[Original_info_Wr++]	= InOut_Object.InOutState;
		rt_kprintf( "\r\n ----- 0x0200 current ������Ϣ \r\n" );
	}

	//  ������Ϣ4
	if( Warn_Status[3] & 0x02 )
	{
		//  ������Ϣ ID
		Original_info[Original_info_Wr++] = 0x11;                   //  ��������/·�߱���
		//  ������Ϣ����
		Original_info[Original_info_Wr++] = 1;
		//  ����
		Original_info[Original_info_Wr++] = 0;                      //  ���ض�λ��
	}
	// 5. ������Ϣ       ��չ�����ź���
	Original_info[Original_info_Wr++]	= 0x25;                     //ID  ��չ�����ź�״̬λ
	Original_info[Original_info_Wr++]	= 4;                        //LEN
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= 0x00;
	Original_info[Original_info_Wr++]	= BD_EXT.Extent_IO_status;
	//6. ������Ϣ  :
	if( SleepState == 1 )                                           //����
	{
		Original_info[Original_info_Wr++]	= 0x2A;                 //ID   IO״̬λ��1��   bit0:�������   bit1:���ߡ�0������
		Original_info[Original_info_Wr++]	= 2;                    //LEN
		Original_info[Original_info_Wr++]	= 0x02;
		Original_info[Original_info_Wr++]	= 0;
	}
	//  ������Ϣ 7  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x2B;                       //ģ����
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 4;
	//  ����
	Original_info[Original_info_Wr++]	= ( AD_2through[1] >> 8 );  // AD1
	Original_info[Original_info_Wr++]	= AD_2through[1];
	Original_info[Original_info_Wr++]	= ( AD_2through[0] >> 8 );  // AD0
	Original_info[Original_info_Wr++]	= AD_2through[0];
	//  ������Ϣ 8  -----------------------------
	//  ������Ϣ ID
	Original_info[Original_info_Wr++] = 0x30;                       //����ͨ�������ź�ǿ��
	//  ������Ϣ����
	Original_info[Original_info_Wr++] = 1;
	//  ����
	Original_info[Original_info_Wr++] = ModuleSQ;

	//  �����ź� 9  ---------------------------------
	Original_info[Original_info_Wr++]	= 0x31;                     // GNSS ��λ���ǿ���
	Original_info[Original_info_Wr++]	= 1;                        // len
	Original_info[Original_info_Wr++]	= Satelite_num;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	SEND GPS CMD=81H ! \r\n");
	}

	return true;
}

//-----------------------------------------------------------------------
u8  Sub_stuff_AppointedPram_0104( void )
{
	//   ��������
	Original_info[Original_info_Wr++] = Setting_Qry.Num_pram;
	//   �����б�
}

//-----------------------------------------------------------------------
u8  Stuff_SettingPram_0104H( u8 in )
{
	u8 reg_str[30];

	//  1. Head
	if( !Protocol_Head( MSG_0x0104, Packet_Normal ) )
	{
		return false; // �ն˲����ϴ�
	}
	//  2. content
	//   float ID
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;
//if(in==1)
	{
		//   ��������
		Original_info[Original_info_Wr++] = 4;
		//   �����б�

		//   2.1   ���ƺ�


		/* Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x83;
		   Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num); // ��������
		   memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.Vechicle_Info.Vech_Num,strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num) ); // ����ֵ
		   Original_info_Wr+=strlen((const char*)JT808Conf_struct.Vechicle_Info.Vech_Num);
		 */
		//   2.2  ��������IP
		Original_info[Original_info_Wr++]	= 0x00; // ����ID 4Bytes
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x13;
		// ��������
		memset( reg_str, 0, sizeof( reg_str ) );
		IP_Str( (char*)reg_str, *( u32* )RemoteIP_aux );
		Original_info[Original_info_Wr++] = strlen( (const char*)reg_str );
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )reg_str, strlen( (const char*)reg_str ) );          // ����ֵ
		Original_info_Wr += strlen( (const char*)reg_str );

		//   2.3   ������TCP�˿�
		Original_info[Original_info_Wr++]	= 0x00;                                                                     // ����ID 4Bytes
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x18;
		Original_info[Original_info_Wr++]	= 4;                                                                        // ��������
		Original_info[Original_info_Wr++]	= 0x00;                                                                     // ����ֵ
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= ( RemotePort_main >> 8 );
		Original_info[Original_info_Wr++]	= RemotePort_main;

		//   2.4  APN �ַ���
		Original_info[Original_info_Wr++]	= 0x00;                                                                     // ����ID 4Bytes
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x10;
		Original_info[Original_info_Wr++]	= strlen( (const char*)APN_String );                                        // ��������
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )APN_String, strlen( (const char*)APN_String ) );    // ����ֵ
		Original_info_Wr += strlen( (const char*)APN_String );

		//  2.5   ����IP
		Original_info[Original_info_Wr++]	= 0x00;                                                                     // ����ID 4Bytes
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x17;
		// ��������
		memset( reg_str, 0, sizeof( reg_str ) );
		IP_Str( (char*)reg_str, *( u32* )RemoteIP_aux );
		Original_info[Original_info_Wr++] = strlen( (const char*)reg_str );
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )reg_str, strlen( (const char*)reg_str ) ); // ����ֵ
		Original_info_Wr += strlen( (const char*)reg_str );


		/*
		   //   2.4   ȱʡʱ���ϱ����
		   Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x29;
		   Original_info[Original_info_Wr++]=4  ; // ��������
		   Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>24);   // ����ֵ
		   Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>16);
		   Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur>>8);
		   Original_info[Original_info_Wr++]=(JT808Conf_struct.DURATION.Default_Dur);

		   //   2.5   ���ļ�غ���
		   Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x40;
		   Original_info[Original_info_Wr++]=strlen((const char*)JT808Conf_struct.LISTEN_Num); // ��������
		   memcpy( ( char * ) Original_info+ Original_info_Wr, ( char * )JT808Conf_struct.LISTEN_Num,strlen((const char*)JT808Conf_struct.LISTEN_Num)); // ����ֵ
		   Original_info_Wr+=strlen((const char*)JT808Conf_struct.LISTEN_Num);
		   //   2.6   ����ٶ�����
		   Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x55;
		   Original_info[Original_info_Wr++]=4  ; // ��������
		   Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>24);   // ����ֵ
		   Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>16);
		   Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX>>8);
		   Original_info[Original_info_Wr++]=( JT808Conf_struct.Speed_warn_MAX);
		   //   2.7   ������ʻ����
		   Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x57;
		   Original_info[Original_info_Wr++]=4  ; // ��������
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>24);   // ����ֵ
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>16);
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec>>8);
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_DrvKeepingSec);
		   //   2.8   ��С��Ϣʱ��
		   Original_info[Original_info_Wr++]=0x00;   // ����ID 4Bytes
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x00;
		   Original_info[Original_info_Wr++]=0x59;
		   Original_info[Original_info_Wr++]=4  ; // ��������
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>24);	 // ����ֵ
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>16);
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec>>8);
		   Original_info[Original_info_Wr++]=(TiredConf_struct.TiredDoor.Door_MinSleepSec);
		 */
	}


/*else
   if(in==2)
   {
   Sub_stuff_AppointedPram_0104();
   }*/
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	���Ͳ�����ѯ��Ϣ! \r\n");
	}

	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_DeviceAttribute_BD_0107H( void )
{
	u16 infoLen = 0;
	// 1. Head
	if( !Protocol_Head( MSG_0x0107, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	Original_info[Original_info_Wr++] = ProductAttribute._1_DevType;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._2_ProducterID, 5 );
	Original_info_Wr += 5;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._3_Dev_TYPENUM, 20 );
	Original_info_Wr += 20;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._4_Dev_ID, 7 );
	Original_info_Wr += 7;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._5_Sim_ICCID, 10 );
	Original_info_Wr += 10;

	Original_info[Original_info_Wr++] = ProductAttribute._6_HardwareVer_Len;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._7_HardwareVer, ProductAttribute._6_HardwareVer_Len );
	Original_info_Wr += ProductAttribute._6_HardwareVer_Len;

	Original_info[Original_info_Wr++] = ProductAttribute._8_SoftwareVer_len;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._9_SoftwareVer, ProductAttribute._8_SoftwareVer_len );
	Original_info_Wr += ProductAttribute._8_SoftwareVer_len;

	Original_info[Original_info_Wr++] = ProductAttribute._10_FirmWareVer_len;
	memcpy( ( char* )Original_info + Original_info_Wr, (u8*)ProductAttribute._11_FirmWare, ProductAttribute._10_FirmWareVer_len );
	Original_info_Wr += ProductAttribute._10_FirmWareVer_len;

	Original_info[Original_info_Wr++]	= ProductAttribute._12_GNSSAttribute;
	Original_info[Original_info_Wr++]	= ProductAttribute._13_ComModuleAttribute;


	/*
	   infoLen=sizeof(ProductAttribute);
	   memcpy( ( char * ) Original_info+ Original_info_Wr,(u8*)&ProductAttribute,infoLen);
	   Original_info_Wr+=infoLen;
	 */

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	�����ն����� \r\n");
	}

	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_ISP_Resualt_BD_0108H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0108, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	BD_ISP.ISP_running = 0;                                     // clear
	//----------------------------------------------------
	Original_info[Original_info_Wr++]	= BD_ISP.Update_Type;   // ��������
	Original_info[Original_info_Wr++]	= 0;                    //BD_ISP.Update_Type;  // �������

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Զ����������ϱ� \r\n");
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_BatchDataTrans_BD_0704H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0704, Packet_Normal ) ) 
	{
		return false;
	}
	// 2. content
	// 2.1   ���������
	Original_info[Original_info_Wr++] = 1;
	// 2.2   λ����������
	Original_info[Original_info_Wr++] = 0; //  0: �����ϱ�  1: ä������
	// 2.3   λ�û㱨������
	// 1. �澯��־  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
	Original_info_Wr += 4;
	// 3.  γ��
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Latitude, 4 );     //γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Longitude, 4 );    //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
	Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps >> 8 );                           //(GPS_speed>>8);
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps );                                //GPS_speed;
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );                           //High
	Original_info[Original_info_Wr++]	= GPS_direction;                                    // Low
	// 8.  ����ʱ��
	Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	��λ���������ϴ�\r\n");
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_CANDataTrans_BD_0705H( void )
{
	u16 DataNum = 0, i = 0;
	u32 read_rd = 0;
	// 1. Head
	if( !Protocol_Head( MSG_0x0705, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	// ���������
	DataNum								= ( CAN_trans.canid_1_SdWr >> 3 ); // ����8
	Original_info[Original_info_Wr++]	= ( DataNum >> 8 );
	Original_info[Original_info_Wr++]	= DataNum;
	Can_sdnum							+= DataNum;
	//����ʱ��
	Original_info[Original_info_Wr++]	= ( ( time_now.hour / 10 ) << 4 ) + ( time_now.hour % 10 );
	Original_info[Original_info_Wr++]	= ( ( time_now.min / 10 ) << 4 ) + ( time_now.min % 10 );
	Original_info[Original_info_Wr++]	= ( ( time_now.sec / 10 ) << 4 ) + ( time_now.sec % 10 );
	Original_info[Original_info_Wr++]	= ( Can_same % 10 ); //0x00;  // ms ����
	Original_info[Original_info_Wr++]	= 0x00;
	//  CAN ����������
	read_rd = 0;
	for( i = 0; i < DataNum; i++ )
	{
		/*
		   Original_info[Original_info_Wr++]= (CAN_trans.canid_1_Filter_ID>>24)|0x40;// ����͸�����ݵ�����
		   Original_info[Original_info_Wr++]=(CAN_trans.canid_1_Filter_ID>>16);
		   Original_info[Original_info_Wr++]=(CAN_trans.canid_1_Filter_ID>>8);
		   Original_info[Original_info_Wr++]=CAN_trans.canid_1_Filter_ID;
		 */

		Original_info[Original_info_Wr++]	= ( CAN_trans.canid_1_ID_SdBUF[i] >> 24 ) | 0x40; // ����͸�����ݵ�����
		Original_info[Original_info_Wr++]	= ( CAN_trans.canid_1_ID_SdBUF[i] >> 16 );
		Original_info[Original_info_Wr++]	= ( CAN_trans.canid_1_ID_SdBUF[i] >> 8 );
		Original_info[Original_info_Wr++]	= CAN_trans.canid_1_ID_SdBUF[i];

		//--------------------------------------------------------------------------
		memcpy( Original_info + Original_info_Wr, CAN_trans.canid_1_Sdbuf + read_rd, 8 );
		Original_info_Wr	+= 8;
		read_rd				+= 8;
	}
	CAN_trans.canid_1_SdWr = 0;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	CAN ���������ϴ� ������=%d   Rxnum=%d  Sd_num=%d  Can_loudiao=%d  Can_notsame=%d Same=%d\r\n", DataNum, Can_RXnum, Can_sdnum, Can_loudiao, Can_same, Can_notsame );
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_CentreTakeACK_BD_0805H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0805, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//   float ID
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );      //  Ӧ����ˮ��
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;
	Original_info[Original_info_Wr++]	= (u8)SingleCamra_TakeResualt_BD;   // ����Ӧ����
	Original_info[Original_info_Wr++]	= 0x00;                             //  �ɹ����ն�ý�����    1
	Original_info[Original_info_Wr++]	= 1;
	Original_info[Original_info_Wr++]	= 0;                                // ��ý��ID �б�
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 0;
	Original_info[Original_info_Wr++]	= 1;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	����ͷ��������Ӧ�� \r\n");
	}
}

//--------------------------------------------------------------------------
u8  Stuff_EventACK_0301H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0301, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	Original_info[Original_info_Wr++] = EventObj.Event_ID; // �����¼�ID

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 �¼��������  \r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_ASKACK_0302H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0302, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//  Ӧ����ˮ��
	Original_info[Original_info_Wr++]	= ( ASK_Centre.ASK_floatID >> 8 ); // �����¼�ID
	Original_info[Original_info_Wr++]	= ASK_Centre.ASK_floatID;
	Original_info[Original_info_Wr++]	= ASK_Centre.ASK_answerID;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	��������ѡ���� \r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_MSGACK_0303H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0303, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//  Ӧ����ˮ��
	Original_info[Original_info_Wr++]	= MSG_BroadCast_Obj.INFO_TYPE;
	Original_info[Original_info_Wr++]	= MSG_BroadCast_Obj.INFO_PlyCancel; //  0  ȡ��  1 �㲥
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	�㲥ȡ���ظ�  \r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Stuff_ControlACK_0500H( void )   //   ��������Ӧ��
{
	//  1. Head
	if( !Protocol_Head( MSG_0x0500, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//------------------------------- Stuff ----------------------------------------
	//   float ID                                                // ��Ӧ����Ӧ����Ϣ����ˮ��
	Original_info[Original_info_Wr++]	= (u8)( Vech_Control.CMD_FloatID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Vech_Control.CMD_FloatID;

	// 1. �澯��־  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
	Original_info_Wr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
	Original_info_Wr += 4;
	// 3.  γ��
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Latitude, 4 );     //γ��   modify by nathan
	Original_info_Wr += 4;
	// 4.  ����
	memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Longitude, 4 );    //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Original_info_Wr += 4;
	// 5.  �߳�
	Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
	Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Original_info[Original_info_Wr++]	= (u8)( Speed_gps >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Speed_gps;
	// 7. ����   ��λ 1��
	Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );   //High
	Original_info[Original_info_Wr++]	= GPS_direction;            // Low
	// 8.  ����ʱ��
	Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	SEND  Vech  Control  ! \r\n");
	}

	return true;
}

//  �г���¼�� :�ɼ�ָ����λ����Ϣ��¼
u8  REC_09H_Stuff( void )
{
	u8 i = 0;
#if 0
	// 1.  ���1 Сʱ�ĵ�λ����Ϣ��¼
	// 1.1   ��ʼʱ��
	Time2BCD( Original_info + Original_info_Wr ); // ��¼��ʵʱʱ��
	Original_info_Wr += 6;
	for( i = 0; i < 60; i++ )
	{
		//  1.2  ��ʼ���һ���ӵ�λ��
		//  longitude    3905.292651,N,11733.124913,E        long: 043438B1  lat: 0165DCFE
		Original_info[Original_info_Wr++]	= 0x04;
		Original_info[Original_info_Wr++]	= 0x34;
		Original_info[Original_info_Wr++]	= 0x38;
		Original_info[Original_info_Wr++]	= 0xB1;
		// lat
		Original_info[Original_info_Wr++]	= 0x01;
		Original_info[Original_info_Wr++]	= 0x65;
		Original_info[Original_info_Wr++]	= 0xDC;
		Original_info[Original_info_Wr++]	= 0xFE;
		// high
		Original_info[Original_info_Wr++]	= 0x00;
		Original_info[Original_info_Wr++]	= 0x65;
		// spd
		Original_info[Original_info_Wr++] = 0x00;
	}
#endif
}



/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:  wwww
***********************************************************/
u8  Stuff_RecoderACK_0700H( u8 PaketType )  //   �г���¼�������ϴ�
{
	u16 SregLen				= 0, Swr = 0;   //,Gwr=0; // S:serial  G: GPRS
	u16 Reg_len_position	= 0;
	u8	Sfcs				= 0;
	u16 i					= 0;
	u32 regdis				= 0, reg2 = 0;
	u8	Reg[70];
	u8	QueryRecNum = 0;                    // ��ѯ��¼��Ŀ

	//  1. Head
	if( !Protocol_Head( MSG_0x0700, PaketType ) )
	{
		return false;
	}

	switch( Recode_Obj.CMD )
	{
		case   0x00:                                                //  ִ�б�׼�汾���
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;   // ������
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;             // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x00;             //������
			SregLen								= 0x02;             // ��Ϣ����
			Original_info[Original_info_Wr++]	= 0x00;             // Hi
			Original_info[Original_info_Wr++]	= 2;                // Lo

			Original_info[Original_info_Wr++]	= 0x00;             // ������
			Original_info[Original_info_Wr++]	= 0x12;             //  12  ���׼ 
			Original_info[Original_info_Wr++]	= 0x00;

			break;
		//---------------- �ϴ�������  -------------------------------------
		case 0x01:
			//  01  ��ǰ��ʻԱ���뼰��Ӧ�Ļ�������ʻ֤��
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // ������
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x01;                         //������

			SregLen								= 0x00;                         // ��Ϣ����
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 18;                           // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // ������

			memcpy( Original_info + Original_info_Wr, "410727198503294436", 18 );  //��Ϣ����
			Original_info_Wr					+= 18;
			/*
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			*/
			break;
		case 0x02:                                                              //  02  �ɼ���¼�ǵ�ʵʱʱ��
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // ������
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x02;                         //������

			SregLen								= 0x00;                         // ��Ϣ����
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 6;                            // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // ������
			Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );
			//+++++++++++++++++++++++++++++++++++++++++++++shi jian ++++++++++++++++++++++
			t_hour = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
			t_min = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
			t_second = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );
			/*
			Original_info[Original_info_Wr++] = 0x14;
			Original_info[Original_info_Wr++] = 0x03;
			Original_info[Original_info_Wr++] = 0x20;
			Original_info[Original_info_Wr++] =	0x08;
			Original_info[Original_info_Wr++] = 0x17;
			Original_info[Original_info_Wr++] = 0x30;
			*/
			/*
			Time2BCD( Original_info + Original_info_Wr );                       //��¼��ʱ��
			Original_info_Wr += 6;
			*/
			break;
		case 0x03:                                                              // 03 �ɼ�360h�����
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // ������
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;
			Original_info[Original_info_Wr++]	= 0x03;                         //������

			SregLen								= 0x00;                         // ��Ϣ����
			Original_info[Original_info_Wr++]	= 0x00;                         // Hi
			Original_info[Original_info_Wr++]	= 20;                           // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // ������
			//	��Ϣ����
			/*
			Time2BCD( Original_info + Original_info_Wr );                       // ��¼��ʵʱʱ��
			Original_info_Wr					+= 6;
			*/
			Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
			Original_info[Original_info_Wr++]	= t_hour;
			Original_info[Original_info_Wr++]	= t_min;
			Original_info[Original_info_Wr++]	= t_second;
			/*
			Original_info[Original_info_Wr++] = 0x14;
			Original_info[Original_info_Wr++] = 0x03;
			Original_info[Original_info_Wr++] = 0x20;
			Original_info[Original_info_Wr++] =	0x08;
			Original_info[Original_info_Wr++] = 0x17;
			Original_info[Original_info_Wr++] = 0x30;
			*/
			
			Original_info[Original_info_Wr++]	= 0x13;                         // ���ΰ�װʱ��
			Original_info[Original_info_Wr++]	= 0x04;
			Original_info[Original_info_Wr++]	= 0x01;
			Original_info[Original_info_Wr++]	= 0x08;
			Original_info[Original_info_Wr++]	= 0x30;
			Original_info[Original_info_Wr++]	= 0x26;
			//--- ��ʼ���
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			// -- �ۻ���� 3���ֽ� ��λ0.1km    6λ
			regdis								= JT808Conf_struct.Distance_m_u32 / 100; //��λ0.1km
			reg2								= regdis / 100000;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= ( reg2 << 4 ) + ( regdis % 100000 / 10000 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 10000 / 1000 ) << 4 ) + ( regdis % 1000 / 100 );
			Original_info[Original_info_Wr++]	= ( ( regdis % 100 / 10 ) << 4 ) + ( regdis % 10 );
			break;

		case 0x04:                                                                  // 04  �ɼ���¼������ϵ��
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;                   // ������
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                             // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x04;                               //������

			SregLen								= 0x00;                             // ��Ϣ����
			Original_info[Original_info_Wr++]	= 0x00;                             // Hi
			Original_info[Original_info_Wr++]	= 8;                                // Lo

			Original_info[Original_info_Wr++] = 0x00;                               // ������

			//  ��Ϣ����
			/*
			Time2BCD( Original_info + Original_info_Wr );                           // ��¼��ʵʱʱ��
			Original_info_Wr					+= 6;
			*/
			Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
			Original_info[Original_info_Wr++]	= t_hour;
			Original_info[Original_info_Wr++]	= t_min;
			Original_info[Original_info_Wr++]	= t_second;
			/*
			Original_info[Original_info_Wr++] = 0x14;
			Original_info[Original_info_Wr++] = 0x03;
			Original_info[Original_info_Wr++] = 0x20;
			Original_info[Original_info_Wr++] =	0x08;
			Original_info[Original_info_Wr++] = 0x17;
			Original_info[Original_info_Wr++] = 0x30;
			*/
			
			Original_info[Original_info_Wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value >> 8 );
			Original_info[Original_info_Wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value );
			break;
		case 0x05:                                                                  //05      ������Ϣ�ɼ�
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;                   // ������
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                             // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x05;                               //������

			SregLen								= 41;                               // ��Ϣ����
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );             // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;                      // Lo	  65x7

			Original_info[Original_info_Wr++] = 0x00;                               // ������
			//-----------	��Ϣ����  --------------
			memcpy( Original_info + Original_info_Wr, "LGBK22E70AY100102", 17 );    //��Ϣ����
			Original_info_Wr += 17;
			memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Vechicle_Info.Vech_Num, 8 );               // ���ƺ�
			Original_info_Wr					+= 7;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			//��������
			/*Original_info[Original_info_Wr++]	= 0x03; //С�ͳ�
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;*/
			memcpy( Original_info + Original_info_Wr, "С�ͳ�", 6 );   
			Original_info_Wr+=6;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;  
			
			break;
		case   0x06:                                                    // 06-09
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // ������
			Swr									= Original_info_Wr;
			//  06 �ź�������Ϣ
			Original_info[Original_info_Wr++]	= 0x55;                 // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x06;                   //������

			SregLen								= 87;                   // ��Ϣ����
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo

			Original_info[Original_info_Wr++] = 0x00;                   // ������

			Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
			Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
			Original_info[Original_info_Wr++]	= t_hour;
			Original_info[Original_info_Wr++]	= t_min;
			Original_info[Original_info_Wr++]	= t_second;
			/*
			Time2BCD( Original_info + Original_info_Wr );               //��¼��ʵʱʱ��
			Original_info_Wr += 6;
			*/
			//-------  ״̬�ָ���----------------------
			Original_info[Original_info_Wr++] = 0;// 8//�޸�Ϊ0
			//---------- ״̬������-------------------


			/*
			   -------------------------------------------------------------
			          F4  �г���¼�� TW705   �ܽŶ���
			   -------------------------------------------------------------
			   ��ѭ  GB10956 (2012)  Page26  ��A.12  �涨
			   -------------------------------------------------------------
			 | Bit  |      Note       |  �ر�|   MCUpin  |   PCB pin  |   Colour | ADC
			   ------------------------------------------------------------
			    D7      ɲ��           *            PE11             9                ��
			    D6      ��ת��     *             PE10            10               ��
			    D5      ��ת��     *             PC2              8                ��
			    D4      Զ���     *             PC0              4                ��
			    D3      �����     *             PC1              5                ��
			    D2      ���          add          PC3              7                ��      *
			    D1      ����          add          PA1              6                ��      *
			    D0      Ԥ��
			 */
			memcpy( Original_info + Original_info_Wr, "Ԥ��      ", 10 );       // D0
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "Ԥ��      ", 10 );       // D1
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "Ԥ��      ", 10 );       // D2
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "�����    ", 10 );       // D3
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "Զ���    ", 10 );       // D4
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "��ת��    ", 10 );       // D5
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "��ת��    ", 10 );       // D6
			Original_info_Wr += 10;
			memcpy( Original_info + Original_info_Wr, "ɲ��      ", 10 );       // D7
			Original_info_Wr += 10;
			break;

		case 0x07:                                                              //07  ��¼��Ψһ���
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;               // ������
			Swr									= Original_info_Wr;

			Original_info[Original_info_Wr++]	= 0x55;                         // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x07;                           //������

			SregLen								= 30;                           //206;		 // ��Ϣ����
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );         // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;                  // Lo

			Original_info[Original_info_Wr++] = 0x00;                           // ������
			//------- ��Ϣ���� ------
			memcpy( Original_info + Original_info_Wr, "7654321", 7 );           // 3C ��֤����
			Original_info_Wr += 7;
			memcpy( Original_info + Original_info_Wr, "TW705   TW705   ", 16 ); // ��Ʒ�ͺ�
			Original_info_Wr					+= 16;
			Original_info[Original_info_Wr++]	= 0x13;                         // ��������
			Original_info[Original_info_Wr++]	= 0x03;
			Original_info[Original_info_Wr++]	= 0x01;
			Original_info[Original_info_Wr++]	= 0x00;                         // ������ˮ��
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= 0x00;
			Original_info[Original_info_Wr++]	= IMSI_CODE[14] - 0x30;
			break;

		case 0x08:                                                              //  08   �ɼ�ָ������ʻ�ٶȼ�¼
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // ������
		    }
				
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;
				Original_info[Original_info_Wr++]	= 0x08;                     //������

				SregLen								= 504;//504;//630;                      // ��Ϣ����       630
				Original_info[Original_info_Wr++]	= SregLen >> 8;             // Hi
				Original_info[Original_info_Wr++]	= SregLen;                  // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // ������
			//	��Ϣ����
			//WatchDog_Feed( );
			get_08h( Original_info + Original_info_Wr,Recode_Obj.Current_pkt_num);                        //126*5=630        num=576  packet

			Original_info_Wr += 504; //630;
			//  ������Ҫ�ְ�����  -----nate
			break;
		case   0x09:                                                            // 09   ָ����λ����Ϣ��¼
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // ������
			}
				Swr									= Original_info_Wr;
				Original_info[Original_info_Wr++]	= 0x55;                     // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x09;                       //������

				SregLen								= 666;//666 ;                  // ��Ϣ����
				Original_info[Original_info_Wr++]	= SregLen >> 8;             // Hi      666=0x29A
				Original_info[Original_info_Wr++]	= SregLen;                  // Lo

				//Original_info[Original_info_Wr++]=0;    // Hi      666=0x29A
				//Original_info[Original_info_Wr++]=0;	   // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // ������
			//	��Ϣ����
			//WatchDog_Feed( ); 
		#if 0	
			if(Recode_Obj.Current_pkt_num%2) 
			{    
			  get_09h(_700H_buffer,Recode_Obj.Current_pkt_num);
			  memcpy(Original_info + Original_info_Wr,_700H_buffer,333);
			}
			else
               memcpy(Original_info + Original_info_Wr,_700H_buffer+333,333); 
			
			Original_info_Wr += 333;
	  #endif
	        get_09h( Original_info + Original_info_Wr,Recode_Obj.Current_pkt_num); 			
			Original_info_Wr += 666; 
			break;
		case   0x10:                                                            // 10-13     10   �¹��ɵ�ɼ���¼
			//�¹��ɵ�����
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // ������
			}		
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x10;                       //������

				SregLen								= 234;                //0		 // ��Ϣ����
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo

				Original_info[Original_info_Wr++] = 0x00;                       // ������
			
			//------- ��Ϣ���� ------
			//WatchDog_Feed( );
			delay_ms( 3 );
			get_10h( Original_info + Original_info_Wr );                        //234  packetsize      num=100
			Original_info_Wr += 234;

			break;

		case  0x11:                                                             // 11 �ɼ�ָ���ĵĳ�ʱ��ʻ��¼
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // ������
			}
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x11;                       //������

				SregLen								= 50;                 // ��Ϣ����
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo    65x7

				Original_info[Original_info_Wr++] = 0x00;                       // ������


			/*
			       ÿ�� 50 bytes  ��100 ��    ��ȡ����ÿ10 ����һ��  500 packet    Totalnum=10
			 */
			 WatchDog_Feed( );
			get_11h( Original_info + Original_info_Wr,Recode_Obj.Current_pkt_num); 		                       //50  packetsize      num=100
			Original_info_Wr += 50;
			break;
		case  0x12:                                                             // 12 �ɼ�ָ����ʻ����ݼ�¼  ---Devide
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;           // ������

			}
				Swr									= Original_info_Wr;

				Original_info[Original_info_Wr++]	= 0x55;                     // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x12;                       //������

				SregLen								= 500;                 // ��Ϣ����
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 );     // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;              // Lo    65x7

				Original_info[Original_info_Wr++] = 0x00;                       // ������
			//------- ��Ϣ���� ------


			/*
			        ��ʻԱ��ݵ�¼��¼  ÿ��25 bytes      200��      20��һ�� 500 packetsize  totalnum=10
			 */
			 WatchDog_Feed( );
			get_12h( Original_info + Original_info_Wr );
			Original_info_Wr += 500;
			break;
		case  0x13:                                                     // 13 �ɼ���¼���ⲿ�����¼
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // ������
			Swr									= Original_info_Wr;
			Original_info[Original_info_Wr++]	= 0x55;                 // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x13;                   //������

			SregLen								= 700;                  // ��Ϣ����
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

			Original_info[Original_info_Wr++] = 0x00;                   // ������
			//------- ��Ϣ���� ------


			/*
			   �ⲿ�����¼   7 ���ֽ�  100 ��       һ��������
			 */
			 WatchDog_Feed( );
			get_13h( Original_info + Original_info_Wr );
			Original_info_Wr += 700;

			break;
		case   0x14:
			Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
			Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // ������
			Swr									= Original_info_Wr;
			// 14 ��¼�ǲ����޸ļ�¼
			Original_info[Original_info_Wr++]	= 0x55;                 // ��ʼͷ
			Original_info[Original_info_Wr++]	= 0x7A;

			Original_info[Original_info_Wr++] = 0x14;                   //������

			SregLen								= 700;                  // ��Ϣ����
			Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
			Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

			Original_info[Original_info_Wr++] = 0x00;                   // ������
			//------- ��Ϣ���� ------

			/*
			       ÿ�� 7 ���ֽ�   100 ��    1��������
			 */
			 WatchDog_Feed( );
			get_14h( Original_info + Original_info_Wr );
			Original_info_Wr += 700;
			break;

		case     0x15:                                                      // 15 �ɼ�ָ�����ٶ�״̬��־     --------Divde
			if( ( PaketType == Packet_Divide ) && ( Recode_Obj.Current_pkt_num == 1 ) )
			{
				Original_info[Original_info_Wr++]	= (u8)( Recode_Obj.Float_ID >> 8 );
				Original_info[Original_info_Wr++]	= (u8)Recode_Obj.Float_ID;
				Original_info[Original_info_Wr++]	= Recode_Obj.CMD;       // ������
			}
			
				Swr									= Original_info_Wr;
				Original_info[Original_info_Wr++]	= 0x55;                 // ��ʼͷ
				Original_info[Original_info_Wr++]	= 0x7A;

				Original_info[Original_info_Wr++] = 0x15;                   //������

				SregLen								= 133 ;             // ��Ϣ����
				Original_info[Original_info_Wr++]	= (u8)( SregLen >> 8 ); // Hi
				Original_info[Original_info_Wr++]	= (u8)SregLen;          // Lo    65x7

				Original_info[Original_info_Wr++] = 0x00;                   // ������


			/*
			       ÿ�� 133 ���ֽ�   10 ��    1��������     5*133=665        totalnum=2
			 */
			WatchDog_Feed( );
			get_15h( Original_info + Original_info_Wr,Recode_Obj.Current_pkt_num);
			Original_info_Wr += 133;

			break;
	}
	//---------------  ��д���� A Э��	Serial Data   У��λ  -------------------------------------

	Sfcs = 0;                            //  ����SУ�� ��Ox55 ��ʼ
	for( i = Swr; i < Original_info_Wr; i++ )
	{
		Sfcs ^= Original_info[i];
	}
	Original_info[Original_info_Wr++] = Sfcs;               // ��дFCS 

/*bitter:���һ������fcs*/
#if 0
	if( PaketType == Packet_Divide )
	{
		Recode_Obj.fcs ^= Sfcs;
		if( Recode_Obj.Current_pkt_num == Recode_Obj.Total_pkt_num )
		{
			Original_info[Original_info_Wr++] =Sfcs ;//Recode_Obj.fcs; // ��дFCS
		}
	}else
	{
		Original_info[Original_info_Wr++] = Sfcs;               // ��дFCS
	}
#endif

	//  3. Send
	Protocol_End( PaketType, 0 );

	//  4.     ����Ƿְ� �жϽ���
	if(( Recode_Obj.Devide_Flag == 1 )&&((Recode_Obj.RSD_State==0)||(Recode_Obj.RSD_State==3)))   // ����Ӧ�� ,���б��ش�״̬�½���
	{
		if( Recode_Obj.Current_pkt_num >= Recode_Obj.Total_pkt_num )
		{
		  // ����Ƿ��й��б��ش�
		  if(Recode_Obj.RSD_State==3)
             Recode_Obj.RSD_State=1;  // enable   
		  else
			 Recorder_init(1);           //  clear
		}else
		{
			Recode_Obj.Current_pkt_num++;
		}
	}

	if( DispContent )
	{
		rt_kprintf( "\r\n	SEND Recorder Data ! \r\n");
	}

	return true;
}

//------------------------------------------------------------------

u8  Stuff_GNSSRawData_0900H( u8*Instr, u16 len )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0900, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	Original_info[Original_info_Wr++] = 0x00; // ����͸�����ݵ�����
	memcpy( Original_info + Original_info_Wr, Instr, len );
	Original_info_Wr += len;
	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	����͸��  \r\n");
	}
	return true;
}

//----------------------------------------------------------------------
u8  Stuff_MultiMedia_InfoSD_0800H( void )
{
	// 1. Head
	if( !Protocol_Head( MSG_0x0800, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	switch( MediaObj.Media_Type )
	{
		case 0:                                                                 // ͼ��
			MediaObj.Media_totalPacketNum	= Photo_sdState.Total_packetNum;    // ͼƬ�ܰ���
			MediaObj.Media_currentPacketNum = Photo_sdState.SD_packetNum;       // ͼƬ��ǰ����
			MediaObj.Media_ID				= 1;                                //  ��ý��ID
			MediaObj.Media_Channel			= Camera_Number;                    // ͼƬ����ͷͨ����
			break;
		case 1:                                                                 // ��Ƶ

			MediaObj.Media_totalPacketNum	= Sound_sdState.Total_packetNum;    // ͼƬ�ܰ���
			MediaObj.Media_currentPacketNum = Sound_sdState.SD_packetNum;       // ͼƬ��ǰ����
			MediaObj.Media_ID				= 1;                                //  ��ý��ID
			MediaObj.Media_Channel			= 1;                                // ͼƬ����ͷͨ����
			// rt_kprintf(" \r\n �����ϴ���Ƶ��Ϣ \r\n");
			break;
		case 2:                                                                 // ��Ƶ
			MediaObj.Media_totalPacketNum	= Video_sdState.Total_packetNum;    // ͼƬ�ܰ���
			MediaObj.Media_currentPacketNum = Video_sdState.SD_packetNum;       // ͼƬ��ǰ����
			MediaObj.Media_ID				= 1;                                //  ��ý��ID
			MediaObj.Media_Channel			= 1;                                // ͼƬ����ͷͨ����
			// rt_kprintf(" \r\n �����ϴ���Ƶ��Ϣ \r\n");

			break;
		default:
			return false;
	}

	//  MediaID
	Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 24 ); // �����¼�ID
	Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 16 );
	Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 8 );
	Original_info[Original_info_Wr++]	= MediaObj.Media_ID;
	//  Type
	Original_info[Original_info_Wr++] = MediaObj.Media_Type;
	//  MediaCode Type
	Original_info[Original_info_Wr++]	= MediaObj.Media_CodeType;
	Original_info[Original_info_Wr++]	= MediaObj.Event_Code;
	Original_info[Original_info_Wr++]	= MediaObj.Media_Channel;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	���Ͷ�ý���¼���Ϣ�ϴ�  \r\n");
	}
	return true;
}

//--------------------------------------------------------------------------
u8  Stuff_MultiMedia_Data_0801H( void )
{
	u16 inadd = 0, readsize = 0; //,soundpage=0,sounddelta=0;
//	u8  instr[SpxGet_Size];

	//  rt_kprintf("\r\n  1--- pic_total_num:  %d	current_num:  %d  MediaObj.Media_Type: %d \r\n ",MediaObj.Media_totalPacketNum,MediaObj.Media_currentPacketNum,MediaObj.Media_Type);
	// 1. Head
	if( !Protocol_Head( MSG_0x0801, Packet_Divide ) )
	{
		return false;
	}
	// 2. content1  ==>  MediaHead
	if( MediaObj.Media_currentPacketNum == 1 )
	{
		//  MediaID
		Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 24 );  //  ��ý��ID
		Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 16 );
		Original_info[Original_info_Wr++]	= ( MediaObj.Media_ID >> 8 );
		Original_info[Original_info_Wr++]	= MediaObj.Media_ID;
		//  Type
		Original_info[Original_info_Wr++] = MediaObj.Media_Type;            // ��ý������
		//  MediaCode Type
		Original_info[Original_info_Wr++]	= MediaObj.Media_CodeType;      // ��ý������ʽ
		Original_info[Original_info_Wr++]	= MediaObj.Event_Code;          // ��ý���¼�����
		Original_info[Original_info_Wr++]	= MediaObj.Media_Channel;       // ͨ��ID

		//  Position Inifo
		//  �澯��־  4
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Warn_Status, 4 );
		Original_info_Wr += 4;
		// . ״̬  4
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Car_Status, 4 );
		Original_info_Wr += 4;
		//   γ��
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Latitude, 4 );     //γ��   modify by nathan
		Original_info_Wr += 4;
		//   ����
		memcpy( ( char* )Original_info + Original_info_Wr, ( char* )Gps_Gprs.Longitude, 4 );    //����    ����  Bit 7->0   ���� Bit 7 -> 1
		Original_info_Wr += 4;
		//   �߳�
		Original_info[Original_info_Wr++]	= (u8)( GPS_Hight << 8 );
		Original_info[Original_info_Wr++]	= (u8)GPS_Hight;
		//   �ٶ�    0.1 Km/h
		Original_info[Original_info_Wr++]	= (u8)( Speed_gps >> 8 );                           //(GPS_speed>>8);
		Original_info[Original_info_Wr++]	= (u8)( Speed_gps );                                //GPS_speed;
		//   ����   ��λ 1��
		Original_info[Original_info_Wr++]	= ( GPS_direction >> 8 );                           //High
		Original_info[Original_info_Wr++]	= GPS_direction;                                    // Low
		//   ����ʱ��
		Original_info[Original_info_Wr++]	= ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
		Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
		Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
		Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
		Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
		Original_info[Original_info_Wr++]	= ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );

		if( 1 == MediaObj.Media_Type ) //  ��Ƶͷ
		{
			//  AMR �ļ�ͷ     23 21 41 4D 52 0A
			Original_info[Original_info_Wr++]	= 0x23;
			Original_info[Original_info_Wr++]	= 0x21;
			Original_info[Original_info_Wr++]	= 0x41;
			Original_info[Original_info_Wr++]	= 0x4D;
			Original_info[Original_info_Wr++]	= 0x52;
			Original_info[Original_info_Wr++]	= 0x0A;
		}
	}
	// 4. content3  ==> Media Info
	switch( MediaObj.Media_Type )
	{
		case 0:                                     // ͼ��

			if( ( ( Photo_sdState.photo_sending ) == enable ) && ( ( Photo_sdState.SD_flag ) == enable ) )
			{
				Photo_sdState.SD_flag = disable;    // clear
			}else
			{
				return false;
			}
			//  ---------------  ��д����  ---------------
			//			read		Photo_sdState.SD_packetNum��1��ʼ����
			//			content_startoffset     picpage_offset				 contentpage_offset
			if( TF_Card_Status( ) == 0 )
			{
				if( Camera_Number == 1 )
				{
					Api_DFdirectory_Read( camera_1, Original_info + Original_info_Wr, 512, 1, MediaObj.Media_currentPacketNum );
				} else
				if( Camera_Number == 2 )
				{
					Api_DFdirectory_Read( camera_2, Original_info + Original_info_Wr, 512, 1, MediaObj.Media_currentPacketNum );
				} else
				if( Camera_Number == 3 )
				{
					Api_DFdirectory_Read( camera_3, Original_info + Original_info_Wr, 512, 1, MediaObj.Media_currentPacketNum );
				} else
				if( Camera_Number == 4 )
				{
					Api_DFdirectory_Read( camera_4, Original_info + Original_info_Wr, 512, 1, MediaObj.Media_currentPacketNum );
				}

				inadd = ( Photo_sdState.SD_packetNum - 1 ) << 9; //����512
				if( PicFileSize > inadd )
				{
					if( ( PicFileSize - inadd ) > 512 )
					{
						readsize = 512;
					} else
					{
						readsize = PicFileSize - inadd; // ���һ��
						rt_kprintf( "\r\n   ���һ�� readsize =%d \r\n", readsize );
					}
				}else
				{
					return false;
				}
			}else
			if( TF_Card_Status( ) == 1 )
			{
				;


				/* inadd=(Photo_sdState.SD_packetNum-1)<<9; //����512
				   if(PicFileSize>inadd)
				   {
				                   if((PicFileSize-inadd)>512)
				                        readsize=512;
				   else
				   {
				   readsize=PicFileSize-inadd; // ���һ��
				   rt_kprintf("\r\n   ���һ�� readsize =%d \r\n",readsize);
				   }
				   }
				   else
				   return false;
				             i=read_file(PictureName,inadd,readsize,Original_info + Original_info_Wr);
				   if(i==false)
				   {
				                 rt_kprintf("\r\n ͼƬ�ļ�: %s   ��ȡʧ��\r\n",PictureName);
				                 return false;
				   } */
			}
			Original_info_Wr += readsize;           //
			break;
		case 1:                                     // ��Ƶ
			if( ( ( Sound_sdState.photo_sending ) == enable ) && ( ( Sound_sdState.SD_flag ) == enable ) )
			{
				Sound_sdState.SD_flag = disable;    // clear
			}else
			{
				return false;
			}
			//------------------------------------------------------------------------
			//  ---------------  ��д����  ---------------
			//			read		Photo_sdState.SD_packetNum��1��ʼ����
			//			content_startoffset     picpage_offset				 contentpage_offset
			if( TF_Card_Status( ) == 0 )
			{
				Api_DFdirectory_Read( voice, Original_info + Original_info_Wr, 512, 1, MediaObj.Media_currentPacketNum );
				inadd = ( Sound_sdState.SD_packetNum - 1 ) << 9; //����512
				if( SrcFileSize > inadd )
				{
					if( ( SrcFileSize - inadd ) > 512 )
					{
						readsize = 512;
					} else
					{
						readsize = SrcFileSize - inadd; // ���һ��
						rt_kprintf( "\r\n   ���һ�� readsize =%d \r\n", readsize );
					}
				}else
				{
					return false;
				}
			}
			rt_kprintf( "\r\n Sound_sdState.SD_packetNum= %d   filesize=%d  readsize=%d  \r\n", Sound_sdState.SD_packetNum, SrcFileSize, SrcFileSize - inadd );
			Original_info_Wr += readsize;

			//-------------------------------------------------------------------------


			/*
			   //  ---------------	��д����  ---------------
			   if(TF_Card_Status()==1)
			   {
			   if(mp3_sendstate==0)
			   {
			   if(Sound_sdState.SD_packetNum==1)
			   {  // wav tou

			   inadd=WaveFile_EncodeHeader(SrcFileSize ,Original_info + Original_info_Wr);
			   Original_info_Wr+=inadd;
			   rt_kprintf("\r\n д���ļ�ͷ��СΪ wav fileheadersize=%d  \r\n",inadd);

			   }
			   //---------------------------------------------------------
			   soundpage=(Sound_sdState.SD_packetNum-1)/5;// �õ�page
			   sounddelta=((Sound_sdState.SD_packetNum-1)%5)*SpxGet_Size; // �õ�ҳ��ƫ��
			              rt_kprintf("\r\n inadd=%d  soundpage =%d  inpageoffset=%d \r\n",inadd,soundpage,sounddelta);
			   //  i=read_file(SpxSrcName,(soundpage<<9),512,SpxBuf);
			   //  if(i==false)
			   // {
			   // rt_kprintf("\r\n spx�ļ�: %s   ��ȡʧ��--2\r\n",SpxSrcName);
			   //return false;
			   //}
			   Api_Config_read(voice, Sound_sdState.SD_packetNum, SpxBuf,500);
			   memcpy(instr,SpxBuf+sounddelta,SpxGet_Size);
			            //---------  spx Decode  5  �� ---------
			   speachDecode(instr, Original_info + Original_info_Wr);
			   Original_info_Wr+=160;
			   speachDecode(instr+20, Original_info + Original_info_Wr);
			   Original_info_Wr+=160;
			   speachDecode(instr+40, Original_info + Original_info_Wr);
			   Original_info_Wr+=160;
			   speachDecode(instr+60, Original_info + Original_info_Wr);
			   Original_info_Wr+=160;
			   speachDecode(instr+80, Original_info + Original_info_Wr);
			   Original_info_Wr+=160;
			   }
			   else
			   if(mp3_sendstate==1)
			   {
			   inadd=(Sound_sdState.SD_packetNum-1)<<9; //����512
			   if(mp3_fsize>inadd)
			   {
			   if((mp3_fsize-inadd)>512)
			    readsize=512;
			   else
			   {
			   readsize=mp3_fsize-inadd; // ���һ��
			   rt_kprintf("\r\n	 ���һ�� mp3size =%d \r\n",readsize);
			   }
			   }
			   else
			   return false;
			   //rt_kprintf("\r\n ��ȡ�ļ�\r\n");
			   i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr);
			   //rt_kprintf("\r\n ��ȡ�ļ����\r\n");
			   if(i==false)
			   {
			   rt_kprintf("\r\n mp3�ļ�: %s	��ȡʧ��\r\n",SpxSrcName);
			   return false;
			   }
			   Original_info_Wr+=readsize;//


			   }
			   }
			       else
			   return false;
			 */
			break;
		case 2: // ��Ƶ
			if( TF_Card_Status( ) == 1 )
			{
				;


				/*	inadd=(Video_sdState.SD_packetNum-1)<<9; //����512
				   if(wmv_fsize>inadd)
				   {
				   if((wmv_fsize-inadd)>512)
				     readsize=512;
				   else
				    {
				    readsize=wmv_fsize-inadd; // ���һ��
				    rt_kprintf("\r\n	 ���һ�� wmvsize =%d \r\n",readsize);
				    }
				   }
				   else
				   return false;
				   i=read_file(SpxSrcName,inadd,readsize,Original_info + Original_info_Wr);
				   if(i==false)
				   {
				   rt_kprintf("\r\n mp3�ļ�: %s	��ȡʧ��\r\n",SpxSrcName);
				   return false;
				   }
				   Original_info_Wr+=readsize;
				 */
			}else
			{
				return false;
			}

			break;
		default:
			return false;
	}

	if( MediaObj.Media_currentPacketNum > MediaObj.Media_totalPacketNum )
	{
		return false;
	}
	//  5. Send
	Protocol_End( Packet_Divide, 0 );
	WatchDog_Feed( );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Send Media Data \r\n");
	}
	//  else
	{
		rt_kprintf( "\r\n pic_total_num:  %d   current_num:  %d   \r\n ", MediaObj.Media_totalPacketNum, MediaObj.Media_currentPacketNum );
		if( MediaObj.Media_currentPacketNum >= MediaObj.Media_totalPacketNum )
		{
			rt_kprintf( "\r\n Media ���һ��block\r\n" );

			if( 0 == MediaObj.RSD_State ) // �����˳����ģʽ�£����Ϊֹͣ״̬,�ȴ��������ش�
			{
				MediaObj.RSD_State	= 2;
				MediaObj.RSD_Timer	= 0;
			}
		}
	}
	//----------  �ۼӷ��ͱ��� --------------------
	if( 0 == MediaObj.RSD_State )
	{
		if( MediaObj.Media_currentPacketNum < MediaObj.Media_totalPacketNum )
		{
			//  ͼƬ
			if( Photo_sdState.photo_sending == enable )
			{
				Photo_sdState.SD_packetNum++;
			}
			//  ��Ƶ
			if( Sound_sdState.photo_sending == enable )
			{
				Sound_sdState.SD_packetNum++;
			}
			//��Ƶ
			if( Video_sdState.photo_sending == enable )
			{
				Video_sdState.SD_packetNum++;
			}
		}
	}


/*	 else
   if(1==MediaObj.RSD_State)
   {
    MediaObj.RSD_Reader++;
     rt_kprintf("\r\n	  MediaObj.RSD_Reader++  =%d\r\n",MediaObj.RSD_Reader);
    if(MediaObj.RSD_Reader==MediaObj.RSD_total)
         MediaObj.RSD_State=2; //  ��λ�ȴ�״̬���ȴ��������ٷ��ش�ָ��
   }
 */
	//----------  ����  -------------------
	return true;
}

//----------------------------------------------------------------------
u8  Stuff_MultiMedia_IndexAck_0802H( void )
{
	u16 totalNum	= 0, lenregwr = 0;
	u16 i			= 0;

	// 1. Head
	if( !Protocol_Head( MSG_0x0802, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//   float ID  Ӧ����ˮ��
	Original_info[Original_info_Wr++]	= (u8)( Centre_FloatID >> 8 );
	Original_info[Original_info_Wr++]	= (u8)Centre_FloatID;

	//------- ��ý�������� ----
	lenregwr							= Original_info_Wr;
	Original_info[Original_info_Wr++]	= (u8)( totalNum >> 8 );                    // ��ʱռ��λ��
	Original_info[Original_info_Wr++]	= (u8)totalNum;

	//----- ������ЧЧλ�� ----
	totalNum = 0;
	for( i = 0; i < 8; i++ )
	{
		if( SD_ACKflag.f_MediaIndexACK_0802H == 1 )                                 // ͼ��
		{
			Api_RecordNum_Read( pic_index, i, (u8*)&MediaIndex, sizeof( MediaIndex ) );
		}else
		if( SD_ACKflag.f_MediaIndexACK_0802H == 2 )                                 // ��Ƶ
		{
			Api_RecordNum_Read( voice_index, i, (u8*)&MediaIndex, sizeof( MediaIndex ) );
		}
		// rt_kprintf("\r\n Effective_Flag %d  f_QueryEventCode %d  EventCode %d  \r\n",MediaIndex.Effective_Flag,SD_ACKflag.f_QueryEventCode,MediaIndex.EventCode);
		if( ( MediaIndex.Effective_Flag == 1 ) && ( SD_ACKflag.f_QueryEventCode == MediaIndex.EventCode ) )
		{                                                                           //  ������Ч�����������Ӧ���͵�����
			Original_info[Original_info_Wr++]	= (u8)( MediaIndex.MediaID >> 24 ); //  ��ý��ID dworrd
			Original_info[Original_info_Wr++]	= (u8)( MediaIndex.MediaID >> 16 );
			Original_info[Original_info_Wr++]	= (u8)( MediaIndex.MediaID >> 8 );
			Original_info[Original_info_Wr++]	= (u8)( MediaIndex.MediaID );
			Original_info[Original_info_Wr++]	= MediaIndex.Type;                  //  ��ý������
			Original_info[Original_info_Wr++]	= MediaIndex.ID;                    //  ͨ��
			Original_info[Original_info_Wr++]	= MediaIndex.EventCode;
			memcpy( Original_info + Original_info_Wr, MediaIndex.PosInfo, 28 );
			Original_info_Wr += 28;
			totalNum++;
		}
	}

	//---------   ����������  --------
	Original_info[lenregwr]		= (u8)( totalNum >> 8 );
	Original_info[lenregwr + 1] = totalNum;

	//  3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Send Media Index \r\n");
	}
	return true;
}

//--------------------------------------------------------------------------------------
u8  Stuff_DriverInfoSD_0702H( void )
{
	u8 i = 0;
	// 1. Head
	if( !Protocol_Head( MSG_0x0702, Packet_Normal ) )
	{
		return false;
	}

	// 2. content

#if  0                                                                                              // old  not   right


	/*Original_info[Original_info_Wr++]=JT808Conf_struct.Driver_Info.BD_IC_status;
	   memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.BD_IC_inoutTime,6);
	   Original_info_Wr+=6;
	   Original_info[Original_info_Wr++]=JT808Conf_struct.Driver_Info.BD_IC_rd_res;   //��Ƭ���γ�
	 */Original_info[Original_info_Wr++] = JT808Conf_struct.Driver_Info.BD_DriveName_Len; //  ��������
	memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_DriveName, JT808Conf_struct.Driver_Info.BD_DriveName_Len );
	Original_info_Wr += JT808Conf_struct.Driver_Info.BD_DriveName_Len;
	memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_Drv_CareerID, 20 );   //  ��ҵ�ʸ�֤����
	Original_info_Wr					+= 20;
	Original_info[Original_info_Wr++]	= JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len;
	memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_Confirm_agentID, JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len );
	Original_info_Wr += JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len;


	/* memcpy(Original_info+Original_info_Wr,JT808Conf_struct.Driver_Info.BD_ExpireDate,4);  //YYYYMMDD		Original_info_Wr+=i;
	   Original_info_Wr+=4;	*/
#endif

#if   1            //  new  BD format
	//  ״̬
	if( IC_MOD.IC_Status == 1 )
	{
		JT808Conf_struct.Driver_Info.BD_IC_status	= 0x01;
		Original_info[Original_info_Wr++]			= JT808Conf_struct.Driver_Info.BD_IC_status;
		memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_IC_inoutTime, 6 );
		Original_info_Wr					+= 6;
		Original_info[Original_info_Wr++]	= JT808Conf_struct.Driver_Info.BD_IC_rd_res;                    //��Ƭ���γ�
		if( JT808Conf_struct.Driver_Info.BD_IC_rd_res == 0 )
		{
			Original_info[Original_info_Wr++] = JT808Conf_struct.Driver_Info.BD_DriveName_Len;              //	��������
			memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_DriveName, JT808Conf_struct.Driver_Info.BD_DriveName_Len );
			Original_info_Wr += JT808Conf_struct.Driver_Info.BD_DriveName_Len;
			memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_Drv_CareerID, 20 );   //	��ҵ�ʸ�֤����
			Original_info_Wr					+= 20;
			Original_info[Original_info_Wr++]	= JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len;
			memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_Confirm_agentID, JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len );
			Original_info_Wr += JT808Conf_struct.Driver_Info.BD_Confirm_agentID_Len;
			memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_ExpireDate, 4 );      //YYYYMMDD		Original_info_Wr+=i;
			Original_info_Wr += 4;
		}
	} else
	{
		JT808Conf_struct.Driver_Info.BD_IC_status	= 0x02;
		Original_info[Original_info_Wr++]			= JT808Conf_struct.Driver_Info.BD_IC_status;
		memcpy( Original_info + Original_info_Wr, JT808Conf_struct.Driver_Info.BD_IC_inoutTime, 6 );
		Original_info_Wr += 6;
	}
#endif

	// 3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Send Driver Info \r\n");
	}
	return true;
}

//---------------------------------------------------------------------------------
u8  Stuff_Worklist_0701H( void )
{
	u32 listlen = 215;
	// 1. Head
	if( !Protocol_Head( MSG_0x0701, Packet_Normal ) )
	{
		return false;
	}

	// 2. content
	//   ��Ϣ����
	listlen								= 207;
	Original_info[Original_info_Wr++]	= ( listlen >> 24 ); // �����¼�ID
	Original_info[Original_info_Wr++]	= ( listlen >> 16 );
	Original_info[Original_info_Wr++]	= ( listlen >> 8 );
	Original_info[Original_info_Wr++]	= listlen;

	memcpy( Original_info + Original_info_Wr, "���˵�λ:�����һ��ͨ�Ź㲥���޹�˾ �绰:022-26237216  ", 55 );
	Original_info_Wr += 55;
	memcpy( Original_info + Original_info_Wr, "���˵�λ:����������乫˾ �绰:022-86692666  ", 45 );
	Original_info_Wr += 45;
	memcpy( Original_info + Original_info_Wr, "��Ʒ����:GPS�����ն�  ��װ��ʽ:  ��ʽ   ÿ������: 20   ����: 30��  ", 67 );
	Original_info_Wr += 67;
	memcpy( Original_info + Original_info_Wr, "����:��ʽС���� �˴����� :  2012-1-11   ", 40 );
	Original_info_Wr += 40;

	// 3. Send
	Protocol_End( Packet_Normal, 0 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	Send Worklist  \r\n");
	}
	return true;
}

//------------------------------------------------------------------------------------
u8 Stuff_DataTrans_0900_BD_GNSSData( void )
{
	u16 TX_NUM	= 0, Rec_Num = 0, i = 0;
	u8	Gfcs	= 0;
	//---------------------------------------------
	// 1. Head
	if( !Protocol_Head( MSG_0x0900, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//	Ӧ����ˮ��
	Original_info[Original_info_Wr++] = 0x00; // ����͸�����ݵ����� 1	��ʾԶ������
	// 3. ����
	// GNSS ��Ϣ
	//	4. Send
	Protocol_End( Packet_Normal, 1 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 ����͸�� GNSS Data\r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8 Stuff_DataTrans_0900_BD_ICinfo( void )
{
	u16 TX_NUM	= 0, Rec_Num = 0, i = 0;
	u8	Gfcs	= 0;
	//---------------------------------------------
	// 1. Head
	if( !Protocol_Head( MSG_0x0900, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//	Ӧ����ˮ��
	Original_info[Original_info_Wr++] = 0x0B;                           //IC ����Ϣ
	// 3. ����
	// IC ��Ϣ
	memcpy( Original_info + Original_info_Wr, IC_MOD.IC_Tx40H, 64 );    //IC 64 �ֽ���Ϣ
	Original_info_Wr += 64;
	//	4. Send
	Protocol_End( Packet_Normal, 1 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 ����͸��IC ����Ϣ\r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8 Stuff_DataTrans_0900_BD_Serial1( void )
{
	u16 TX_NUM	= 0, Rec_Num = 0, i = 0;
	u8	Gfcs	= 0;
	//---------------------------------------------
	// 1. Head
	if( !Protocol_Head( MSG_0x0900, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//	Ӧ����ˮ��
	Original_info[Original_info_Wr++] = 0x41; // ����1 ͸����Ϣ
	// 3. ����
	// ����1 ��Ϣ
	//	4. Send
	Protocol_End( Packet_Normal, 1 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 ����͸������1��Ϣ\r\n");
	}
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8 Stuff_DataTrans_0900_BD_Serial2( void )
{
	u16 TX_NUM	= 0, Rec_Num = 0, i = 0;
	u8	Gfcs	= 0;
	//---------------------------------------------
	// 1. Head
	if( !Protocol_Head( MSG_0x0900, Packet_Normal ) )
	{
		return false;
	}
	// 2. content
	//	Ӧ����ˮ��
	Original_info[Original_info_Wr++] = 0x42; // ����2 ͸����Ϣ
	// 3. ����
	// ����1 ��Ϣ
	//	4. Send
	Protocol_End( Packet_Normal, 1 );
	if( DispContent )
	{
		rt_kprintf( "\r\n	 ����͸������2 ��Ϣ\r\n");
	}
	return true;
}

//----------------------------------------------
u8  Update_HardSoft_Version_Judge( u8 * instr )
{
	//   ��ȡ��50 ҳ��Ϣ
	Device_type		= ( (u32)instr[1] << 24 ) + ( (u32)instr[2] << 16 ) + ( (u32)instr[3] << 8 ) + (u32)instr[4];
	Firmware_ver	= ( (u32)instr[5] << 24 ) + ( (u32)instr[6] << 16 ) + ( (u32)instr[7] << 8 ) + (u32)instr[8];
	rt_kprintf( "	\r\n �豸����: %x  ����汾:%x \r\n", Device_type, Firmware_ver );

	if( Device_type != STM32F407_Recoder_32MbitDF )
	{
		rt_kprintf( "\r\n �豸���Ͳ�ƥ�䲻�����" );
		return false;
	}    else
	{
		return true;
	}
}

//-------------------- ISP Check  ---------------------------------------------
void  ISP_file_Check( void )
{
	memset( ISP_buffer, 0, sizeof( ISP_buffer ) );
	ISP_Read( ISP_APP_Addr, ISP_buffer, PageSIZE );


	/*
	   ���   �ֽ���	����			  ��ע
	   1          1    ���±�־      1 ��ʾ��Ҫ����   0 ��ʾ����Ҫ����
	   2-5			  4   �豸����				 0x0000 0001  ST712   TWA1
	        0x0000 0002   STM32  103  ��A1
	        0x0000 0003   STM32  101  ������
	        0x0000 0004   STM32  A3  sst25
	        0x0000 0005   STM32  �г���¼��
	   6-9		 4	   ����汾      ÿ���豸���ʹ�  0x0000 00001 ��ʼ���ݰ汾���ε���
	   10-29      20	����		' mm-dd-yyyy HH:MM:SS'
	   30-31      2    ��ҳ��		   ��������Ϣҳ
	   32-35      4    ������ڵ�ַ
	   36-200	   165	  Ԥ��
	   201-		  n    �ļ���

	 */
	//------------   Type check  ---------------------
	Device_type		= ( (u32)ISP_buffer[1] << 24 ) + ( (u32)ISP_buffer[2] << 16 ) + ( (u32)ISP_buffer[3] << 8 ) + (u32)ISP_buffer[4];
	Firmware_ver	= ( (u32)ISP_buffer[5] << 24 ) + ( (u32)ISP_buffer[6] << 16 ) + ( (u32)ISP_buffer[7] << 8 ) + (u32)ISP_buffer[8];
	rt_kprintf( "	\r\n �豸����: %x  ����汾:%x \r\n", Device_type, Firmware_ver );

	if( Device_type != STM32F407_Recoder_32MbitDF )
	{
		rt_kprintf( "\r\n �豸���Ͳ�ƥ�䲻�����" );
		ISP_buffer[0] = 0; // ���������صĳ���
		ISP_Write( ISP_APP_Addr, ISP_buffer, PageSIZE );
	}

	rt_kprintf( "\r\n �ļ�����: " );
	rt_kprintf( "%20s", (const char*)( ISP_buffer + 10 ) );
	rt_kprintf( "\r\n" );
	rt_kprintf( "\r\n �ļ���: " );
	rt_kprintf( "%100s", (const char*)( ISP_buffer + 201 ) );
	rt_kprintf( "\r\n" );
	if( Device_type == STM32F407_Recoder_32MbitDF )
	{
		Systerm_Reset_counter	= ( Max_SystemCounter - 5 );    // ׼�������������³���
		ISP_resetFlag			= 1;                            //׼������
		rt_kprintf( "\r\n ׼���������³���!\r\n" );
	}
}

//----------------------------------------------------------------------------------
void Stuff_O200_Info_Only( u8* Instr )
{
	u8 Infowr = 0;

	// 1. �澯��־  4
	memcpy( ( char* )Instr + Infowr, ( char* )Warn_Status, 4 );
	Infowr += 4;
	// 2. ״̬  4
	memcpy( ( char* )Instr + Infowr, ( char* )Car_Status, 4 );
	Infowr += 4;
	// 3.  γ��
	memcpy( ( char* )Instr + Infowr, ( char* )Gps_Gprs.Latitude, 4 );   //γ��   modify by nathan
	Infowr += 4;
	// 4.  ����
	memcpy( ( char* )Instr + Infowr, ( char* )Gps_Gprs.Longitude, 4 );  //����    ����  Bit 7->0   ���� Bit 7 -> 1
	Infowr += 4;
	// 5.  �߳�
	Instr[Infowr++] = (u8)( GPS_Hight << 8 );
	Instr[Infowr++] = (u8)GPS_Hight;
	// 6.  �ٶ�    0.1 Km/h
	Instr[Infowr++] = (u8)( Speed_gps >> 8 );
	Instr[Infowr++] = (u8)Speed_gps;
	// 7. ����   ��λ 1��
	Instr[Infowr++] = ( GPS_direction >> 8 );   //High
	Instr[Infowr++] = GPS_direction;            // Low
	// 8.  ����ʱ��
	Instr[Infowr++] = ( ( ( Gps_Gprs.Date[0] ) / 10 ) << 4 ) + ( ( Gps_Gprs.Date[0] ) % 10 );
	Instr[Infowr++] = ( ( Gps_Gprs.Date[1] / 10 ) << 4 ) + ( Gps_Gprs.Date[1] % 10 );
	Instr[Infowr++] = ( ( Gps_Gprs.Date[2] / 10 ) << 4 ) + ( Gps_Gprs.Date[2] % 10 );
	Instr[Infowr++] = ( ( Gps_Gprs.Time[0] / 10 ) << 4 ) + ( Gps_Gprs.Time[0] % 10 );
	Instr[Infowr++] = ( ( Gps_Gprs.Time[1] / 10 ) << 4 ) + ( Gps_Gprs.Time[1] % 10 );
	Instr[Infowr++] = ( ( Gps_Gprs.Time[2] / 10 ) << 4 ) + ( Gps_Gprs.Time[2] % 10 );
}

//-----------------------------------------------------
u8  Save_MediaIndex( u8 type, u8* name, u8 ID, u8 Evencode )
{
	u8 i = 0;

	if( ( type != 1 ) && ( type != 0 ) )
	{
		return false;
	}

	//----- ������Чλ�� ----
	for( i = 0; i < 8; i++ )
	{
		if( type == 0 ) // ͼ��
		{
			Api_RecordNum_Read( pic_index, i, (u8*)&MediaIndex, sizeof( MediaIndex ) );
		}else
		if( type == 1 ) // ��Ƶ
		{
			Api_RecordNum_Read( voice_index, 1, (u8*)&MediaIndex, sizeof( MediaIndex ) );
		}
		if( MediaIndex.Effective_Flag == 0 )
		{
			break;
		}
	}
	if( i == 8 ) // �����������ӵ�һ����ʼ
	{
		i = 0;
	}
	//----  ��д��Ϣ -------------
	memset( (u8*)&MediaIndex, 0, sizeof( MediaIndex ) );
	MediaIndex.MediaID			= JT808Conf_struct.Msg_Float_ID;
	MediaIndex.Type				= type;
	MediaIndex.ID				= ID;
	MediaIndex.Effective_Flag	= 1;
	MediaIndex.EventCode		= Evencode;
	memcpy( MediaIndex.FileName, name, strlen( (const char*)name ) );
	Stuff_O200_Info_Only( MediaIndex.PosInfo );

	if( type == 0 ) // ͼ��
	{
		Api_RecordNum_Write( pic_index, i, (u8*)&MediaIndex, sizeof( MediaIndex ) );
	}else
	if( type == 1 ) // ��Ƶ
	{
		Api_RecordNum_Write( voice_index, i, (u8*)&MediaIndex, sizeof( MediaIndex ) );
	}
	return true;
}

//------------------------------------------------------------------
u8  CentreSet_subService_8103H( u32 SubID, u8 infolen, u8 *Content )
{
	u8	i = 0;
	u8	reg_str[80];
	u8	reg_in[20];
	u32 resualtu32 = 0;

	rt_kprintf( "\r\n    �յ������������� SubID=%X \r\n", SubID );

	switch( SubID )
	{
		case 0x0001: // �ն����������ͼ��  ��λ:s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.Heart_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ���������: %d s\r\n", JT808Conf_struct.DURATION.Heart_Dur );
			break;
		case 0x0002: // TCP ��ϢӦ��ʱʱ��  ��λ:s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.TCP_ACK_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n TCP��ϢӦ����: %d s\r\n", JT808Conf_struct.DURATION.TCP_ACK_Dur );
			break;
		case 0x0003: //  TCP ��Ϣ�ش�����
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.TCP_ReSD_Num = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n TCP�ش�����: %d\r\n", JT808Conf_struct.DURATION.TCP_ReSD_Num );
			break;
		case 0x0004: // UDP ��ϢӦ��ʱʱ��  ��λ:s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.UDP_ACK_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n UDPӦ��ʱ: %d\r\n", JT808Conf_struct.DURATION.UDP_ACK_Dur );
			break;
		case 0x0005: //  UDP ��Ϣ�ش�����
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.UDP_ReSD_Num = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n UDP�ش�����: %d\r\n", JT808Conf_struct.DURATION.UDP_ReSD_Num );
			break;
		case 0x0010: //  ��������APN
			if( infolen == 0 )
			{
				break;
			}
			memset( APN_String, 0, sizeof( APN_String ) );
			memcpy( APN_String, (char*)Content, infolen );
			memset( (u8*)SysConf_struct.APN_str, 0, sizeof( APN_String ) );
			memcpy( (u8*)SysConf_struct.APN_str, (char*)Content, infolen );
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );

			DataLink_APN_Set( APN_String, 1 );

			break;
		case 0x0013: //  ����������ַ  IP ������
			memset( reg_in, 0, sizeof( reg_in ) );
			memcpy( reg_in, Content, infolen );
			//----------------------------

			i = str2ip( (char*)reg_in, RemoteIP_main );
			if( i <= 3 )
			{
				rt_kprintf( "\r\n  ����: %s \r\n", reg_in );

				memset( DomainNameStr, 0, sizeof( DomainNameStr ) );
				memset( SysConf_struct.DNSR, 0, sizeof( DomainNameStr ) );
				memcpy( DomainNameStr, (char*)Content, infolen );
				memcpy( SysConf_struct.DNSR, (char*)Content, infolen );
				Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );

				//----- ���� GSM ģ��------
				DataLink_DNSR_Set( SysConf_struct.DNSR, 1 );

				SD_ACKflag.f_CentreCMDack_0001H = 1; // 2 DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�
				break;
			}
			memset( reg_str, 0, sizeof( reg_str ) );
			IP_Str( (char*)reg_str, *( u32* )RemoteIP_main );
			strcat( (char*)reg_str, " :" );
			sprintf( (char*)reg_str + strlen( (const char*)reg_str ), "%u\r\n", RemotePort_main );
			memcpy( SysConf_struct.IP_Main, RemoteIP_main, 4 );
			SysConf_struct.Port_main = RemotePort_main;

			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n ���������������� IP \r\n" );
			rt_kprintf( "\r\n SOCKET :" );
			rt_kprintf( (char*)reg_str );
			//-----------  Below add by Nathan  ----------------------------
			rt_kprintf( "\r\n		   ����IP: %d.%d.%d.%d : %d \r\n", RemoteIP_aux[0], RemoteIP_aux[1], RemoteIP_aux[2], RemoteIP_aux[3], RemotePort_main );

			//-----------  Below add by Nathan  ----------------------------
			DataLink_MainSocket_set( RemoteIP_main, RemotePort_main, 1 );
			//-------------------------------------------------------------

			SD_ACKflag.f_CentreCMDack_0001H = 1;    //DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�

			break;
		case 0x0014:                                // ���ݷ����� APN

			break;
		case 0x0017:                                // ���ݷ�����  IP
			memset( reg_in, 0, sizeof( reg_in ) );
			memcpy( reg_in, Content, infolen );
			//----------------------------
			i = str2ip( (char*)reg_in, RemoteIP_aux );
			if( i <= 3 )
			{
				rt_kprintf( "\r\n  ����aux: %s \r\n", reg_in );
				memset( DomainNameStr_aux, 0, sizeof( DomainNameStr_aux ) );
				memset( SysConf_struct.DNSR_Aux, 0, sizeof( DomainNameStr_aux ) );
				memcpy( DomainNameStr_aux, (char*)Content, infolen );
				memcpy( SysConf_struct.DNSR_Aux, (char*)Content, infolen );
				Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
				//----- ���� GSM ģ��------
				DataLink_DNSR2_Set( SysConf_struct.DNSR_Aux, 1 );

				SD_ACKflag.f_CentreCMDack_0001H = 1; //DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�
				break;
			}
			memset( reg_str, 0, sizeof( reg_str ) );
			IP_Str( (char*)reg_str, *( u32* )RemoteIP_aux );
			strcat( (char*)reg_str, " :" );
			sprintf( (char*)reg_str + strlen( (const char*)reg_str ), "%u\r\n", RemotePort_aux );

			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n �������ñ��÷����� IP \r\n" );
			rt_kprintf( "\r\nUDP SOCKET :" );
			rt_kprintf( (char*)reg_str );
			DataLink_AuxSocket_set( RemoteIP_aux, RemotePort_aux, 1 );
			//-----------  Below add by Nathan  ----------------------------
			rt_kprintf( "\r\n       ����IP: %d.%d.%d.%d : %d \r\n", RemoteIP_aux[0], RemoteIP_aux[1], RemoteIP_aux[2], RemoteIP_aux[3], RemotePort_aux );
			break;
		case 0x0018: //  ������ TCP �˿�
			//----------------------------
			if( infolen != 4 )
			{
				break;
			}
			RemotePort_main = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];

			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n ���������������� PORT \r\n" );
			rt_kprintf( "\r\nUDP SOCKET :" );
			rt_kprintf( (char*)reg_str );
			//-----------  Below add by Nathan  ----------------------------
			DataLink_MainSocket_set( RemoteIP_main, RemotePort_main, 1 );   //-------------------------------------------------------------
			SD_ACKflag.f_CentreCMDack_0001H = 1;                            //DataLink_EndFlag=1; //AT_End(); �ȷ��ؽ���ٹҶ�
			break;
		case 0x0019:                                                        //  ������ UDP �˿�

			if( infolen != 4 )
			{
				break;
			}
			RemotePort_aux = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];

			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n ��������UDP������ PORT \r\n" );
			rt_kprintf( "\r\nUDP SOCKET :" );
			rt_kprintf( (char*)reg_str );
			rt_kprintf( "\r\n		 ����IP: %d.%d.%d.%d : %d \r\n", RemoteIP_aux[0], RemoteIP_aux[1], RemoteIP_aux[2], RemoteIP_aux[3], RemotePort_aux );
			break;
		case 0x0020: //  �㱨����  0 ��ʱ�㱨  1 ����㱨 2 ��ʱ�Ͷ���㱨
			if( infolen != 4 )
			{
				break;
			}
			resualtu32 = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			switch( resualtu32 )
			{
				case 0: rt_kprintf( "\r\n ��ʱ�㱨 \r\n" );
					break;
				case 1: rt_kprintf( "\r\n ����㱨 \r\n" );
					break;
				case 2: rt_kprintf( "\r\n ��ʱ�Ͷ���㱨\r\n" );
					break;
				default:
					break;
			}
			break;
		case 0x0021:    //  λ�û㱨����  0 ����ACC�ϱ�  1 ����ACC�͵�¼״̬�ϱ�

			break;
		//--------

		case 0x0022:    //  ��ʻԱδ��¼ �㱨ʱ���� ��λ:s    >0
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.NoDrvLogin_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��ʻԱδ��¼�㱨���: %d\r\n", JT808Conf_struct.DURATION.NoDrvLogin_Dur );
			break;
		case 0x0027: //  ����ʱ�㱨ʱ��������λ s  >0
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.Sleep_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ���߻㱨ʱ����: %d \r\n", JT808Conf_struct.DURATION.Sleep_Dur );
			break;
		case 0x0028: //  ��������ʱ�㱨ʱ����  ��λ s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.Emegence_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��������ʱ����: %d \r\n", JT808Conf_struct.DURATION.Emegence_Dur );
			break;
		case 0x0029: //  ȱʡʱ��㱨���  ��λ s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.Default_Dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ȱʡ�㱨ʱ����: %d \r\n", JT808Conf_struct.DURATION.Default_Dur );
			break;
		//---------

		case 0x002C: //  ȱʡ����㱨���  ��λ ��
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DISTANCE.Defalut_DistDelta = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ȱʡ����㱨���: %d m\r\n", JT808Conf_struct.DISTANCE.Defalut_DistDelta );
			break;
		case 0x002D: //  ��ʻԱδ��¼�㱨������ ��λ ��
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DISTANCE.NoDrvLogin_Dist = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��ʻԱδ��¼�㱨����: %d m\r\n", JT808Conf_struct.DISTANCE.NoDrvLogin_Dist );
			break;
		case 0x002E: //  ����ʱ�㱨������  ��λ ��
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DISTANCE.Sleep_Dist = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ����ʱ�����ϱ����: %d m\r\n", JT808Conf_struct.DISTANCE.Sleep_Dist );
			break;
		case 0x002F: //  ��������ʱ�㱨������  ��λ ��
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DISTANCE.Emergen_Dist = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��������ʱ�����ϱ����: %d m\r\n", JT808Conf_struct.DISTANCE.Emergen_Dist );
			break;
		case 0x0030: //  �յ㲹���Ƕ� , <180
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.DURATION.SD_Delta_maxAngle = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n �յ㲹���Ƕ�: %d ��\r\n", JT808Conf_struct.DISTANCE.Emergen_Dist );
			break;
		case 0x0040: //   ���ƽ̨�绰����
			if( infolen == 0 )
			{
				break;
			}
			i = strlen( (const char*)JT808Conf_struct.LISTEN_Num );
			rt_kprintf( "\r\n old: %s \r\n", JT808Conf_struct.LISTEN_Num );

			memset( JT808Conf_struct.LISTEN_Num, 0, sizeof( JT808Conf_struct.LISTEN_Num ) );
			memcpy( JT808Conf_struct.LISTEN_Num, Content, infolen );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n new: %s \r\n", JT808Conf_struct.LISTEN_Num );

			//CallState=CallState_rdytoDialLis;  // ׼����ʼ�����������
			rt_kprintf( "\r\n ���ü��ƽ̨����: %s \r\n", JT808Conf_struct.LISTEN_Num );

			break;
		case 0x0041: //   ��λ�绰���룬�ɲ��ô˵绰���벦���ն˵绰���ն˸�λ
			if( infolen == 0 )
			{
				break;
			}
			memset( reg_str, 0, sizeof( reg_str ) );
			memcpy( reg_str, Content, infolen );
			rt_kprintf( "\r\n ��λ�绰���� %s \r\n", reg_str );
			break;
		case 0x0042:    //   �ָ��������õ绰���ɲ��øõ绰�������ն˻ָ���������

			break;
		case 0x0045:    //  �ն˵绰�������� 0 �Զ�����  1 ACC ON�Զ����� OFFʱ�ֶ�����

			break;
		case 0x0046:    //  ÿ��ͨ���ʱ�� ����λ  ��

			break;
		case 0x0047:    //  �����ͨ��ʱ�䣬��λ  ��

			break;
		case 0x0048:    //  �����绰����
			if( infolen == 0 )
			{
				break;
			}
			memset( JT808Conf_struct.LISTEN_Num, 0, sizeof( JT808Conf_struct.LISTEN_Num ) );
			memcpy( JT808Conf_struct.LISTEN_Num, Content, infolen );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			CallState = CallState_rdytoDialLis; // ׼����ʼ�����������
			rt_kprintf( "\r\n ���������������: %s \r\n", JT808Conf_struct.LISTEN_Num );
			break;

		//----------
		case 0x0050:                            //  ���������֣� ��λ����Ϣ�б�����־���Ӧ����ӦλΪ1ʱ����������---

			if( infolen != 4 )
			{
				break;
			}
			resualtu32 = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			rt_kprintf( "\r\n ����������: %x \r\n", resualtu32 );
			break;
		case 0x0052:    //  �������տ��أ� �뱨����־��Ӧ��λ1ʱ������

			break;
		case 0x0053:    //  �������մ洢    �뱨����־��Ӧ��λ1ʱ�����մ洢 ����ʵʱ�ϴ�

			break;
		case 0x0054:    //  �ؼ���־        �뱨����־��Ӧ��λ1  Ϊ�ؼ�����

			break;
		//---------

		case 0x0055:    //  ����ٶ�   ��λ   ǧ��ÿСʱ
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.Speed_warn_MAX = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			memset( reg_str, 0, sizeof( reg_str ) );
			memcpy( reg_str, &JT808Conf_struct.Speed_warn_MAX, 4 );
			memcpy( reg_str + 4, &JT808Conf_struct.Spd_Exd_LimitSeconds, 4 );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ����ٶ�: %d km/h \r\n", JT808Conf_struct.Speed_warn_MAX );
			Spd_ExpInit( );
			break;
		case 0x0056: //  ���ٳ���ʱ��    ��λ s
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.Spd_Exd_LimitSeconds = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			memset( reg_str, 0, sizeof( reg_str ) );
			memcpy( reg_str, &JT808Conf_struct.Speed_warn_MAX, 4 );
			memcpy( reg_str + 4, &JT808Conf_struct.Spd_Exd_LimitSeconds, 4 );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��ʱ����ʱ��: %d s \r\n", JT808Conf_struct.Spd_Exd_LimitSeconds );
			Spd_ExpInit( );
			break;
		case 0x0057:                    //  ������ʻʱ������ ��λ  s
			if( infolen != 4 )
			{
				break;
			}
			TiredConf_struct.TiredDoor.Door_DrvKeepingSec = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( tired_config, 0, (u8*)&TiredConf_struct, sizeof( TiredConf_struct ) );
			Warn_Status[3] &= ~0x04;    //BIT(2)	�Ӵ�ƣ�ͼ�ʻ����
			TIRED_Drive_Init( );        // ���ƣ�ͼ�ʻ״̬
			rt_kprintf( "\r\n ������ʻʱ������: %d s \r\n", TiredConf_struct.TiredDoor.Door_DrvKeepingSec );
			break;
		case 0x0058:                    //  �����ۼƼ�ʻʱ������  ��λ  s
			if( infolen != 4 )
			{
				break;
			}
			TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( tired_config, 0, (u8*)&TiredConf_struct, sizeof( TiredConf_struct ) );
			TiredConf_struct.Tired_drive.ACC_ONstate_counter	= 0;
			TiredConf_struct.Tired_drive.ACC_Offstate_counter	= 0;
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n �����ۼƼ�ʻʱ��: %d s \r\n", TiredConf_struct.TiredDoor.Door_DayAccumlateDrvSec );
			break;
		case 0x0059: //  ��С��Ϣʱ��  ��λ s
			if( infolen != 4 )
			{
				break;
			}
			TiredConf_struct.TiredDoor.Door_MinSleepSec = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( tired_config, 0, (u8*)&TiredConf_struct, sizeof( TiredConf_struct ) );
			rt_kprintf( "\r\n ��С��Ϣʱ��: %d s \r\n", TiredConf_struct.TiredDoor.Door_MinSleepSec );
			break;
		case 0x005A: //  �ͣ��ʱ��   ��λ s
			if( infolen != 4 )
			{
				break;
			}
			TiredConf_struct.TiredDoor.Door_MaxParkingSec = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( tired_config, 0, (u8*)&TiredConf_struct, sizeof( TiredConf_struct ) );
			TiredConf_struct.TiredDoor.Parking_currentcnt	= 0;
			Warn_Status[1]									&= ~0x08;   // �����ʱ����
			rt_kprintf( "\r\n �ͣ��ʱ��: %d s \r\n", TiredConf_struct.TiredDoor.Door_MaxParkingSec );
			break;
		//---------
		case  0x0070:                                                   //  ͼ��/��Ƶ����  1-10  1 ���

			break;
		case  0x0071:                                                   //  ����  0-255

			break;
		case  0x0072:                                                   //  �Աȶ�  0-127

			break;
		case  0x0073:                                                   // ���Ͷ�  0-127

			break;
		case  0x0074:                                                   // ɫ��   0-255

			break;
		//---------
		case  0x0080:                                                   // ������̱����   1/10 km

			break;
		case  0x0081:                                                   // �������ڵ�ʡ��ID
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.Vechicle_Info.Dev_ProvinceID = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��������ʡ��ID: 0x%X \r\n", JT808Conf_struct.Vechicle_Info.Dev_ProvinceID );
			break;
		case  0x0082: // ������������ID
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.Vechicle_Info.Dev_ProvinceID = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ������������ID: 0x%X \r\n", JT808Conf_struct.Vechicle_Info.Dev_ProvinceID );
			break;
		case  0x0083: // ������ͨ�����Ű䷢�Ļ���������
			if( infolen < 4 )
			{
				break;
			}
			memset( JT808Conf_struct.Vechicle_Info.Vech_Num, 0, sizeof( JT808Conf_struct.Vechicle_Info.Vech_Num ) );
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_Num, Content, infolen );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��������ʻ֤��: %s  \r\n", JT808Conf_struct.Vechicle_Info.Vech_Num );
			break;
		case  0x0084: // ������ɫ  ���չ��ҹ涨
			if( infolen != 1 )
			{
				break;
			}
			JT808Conf_struct.Vechicle_Info.Dev_Color = Content[0];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ������ɫ: %d  \r\n", JT808Conf_struct.Vechicle_Info.Dev_Color );
			break;
		//--------------- BD  ����----------------------------------
		case  0x001A: //  IC�� ��������������
			memset( reg_in, 0, sizeof( reg_in ) );
			memcpy( reg_in, Content, infolen );
			//----------------------------

			i = str2ip( (char*)reg_in, SysConf_struct.BD_IC_main_IP );
			if( i <= 3 )
			{
				rt_kprintf( "\r\n IC  �� ����: %s \r\n", reg_in );

				memset( SysConf_struct.BD_IC_DNSR, 0, sizeof( SysConf_struct.BD_IC_DNSR ) );
				memcpy( SysConf_struct.BD_IC_DNSR, (char*)Content, infolen );
				Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
				break;
			}
			memset( reg_str, 0, sizeof( reg_str ) );
			IP_Str( (char*)reg_str, *( u32* )SysConf_struct.BD_IC_main_IP );
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n IC  ��IP: %d.%d.%d.%d \r\n", SysConf_struct.BD_IC_main_IP[0], SysConf_struct.BD_IC_main_IP[1], SysConf_struct.BD_IC_main_IP[2], SysConf_struct.BD_IC_main_IP[3] );
			break;
		case  0x001B:   // IC ��  ��TCP�˿�
			SysConf_struct.BD_IC_TCP_port = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n IC  TCP Port: %d \r\n", SysConf_struct.BD_IC_TCP_port );
			//  IC ������
			DataLink_IC_Socket_set( SysConf_struct.BD_IC_main_IP, SysConf_struct.BD_IC_TCP_port, 0 );
			break;
		case  0x001C:   // IC��   ��UDP �˿�
			SysConf_struct.BD_IC_UDP_port = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n IC  UDP Port : %d \r\n", SysConf_struct.BD_IC_UDP_port );
			break;
		case  0x001D:   // IC �����÷������Ͷ˿�
			memset( reg_in, 0, sizeof( reg_in ) );
			memcpy( reg_in, Content, infolen );
			//----------------------------
			i = str2ip( (char*)reg_in, SysConf_struct.BD_IC_Aux_IP );
			if( i <= 3 )
			{
				rt_kprintf( "\r\n IC  �� ����: %s \r\n", reg_in );

				memset( SysConf_struct.BD_IC_DNSR_Aux, 0, sizeof( SysConf_struct.BD_IC_DNSR_Aux ) );
				memcpy( SysConf_struct.BD_IC_DNSR_Aux, (char*)Content, infolen );
				Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
				break;
			}
			memset( reg_str, 0, sizeof( reg_str ) );
			IP_Str( (char*)reg_str, *( u32* )SysConf_struct.BD_IC_Aux_IP );
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			rt_kprintf( "\r\n IC  ����IP: %d.%d.%d.%d \r\n", SysConf_struct.BD_IC_Aux_IP[0], SysConf_struct.BD_IC_Aux_IP[1], SysConf_struct.BD_IC_Aux_IP[2], SysConf_struct.BD_IC_Aux_IP[3] );

			break;
		case   0x0031: //  ����Χ���뾶(�Ƿ�λ����ֵ)
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.BD_CycleRadius_DoorValue = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ����Χ���뾶(�Ƿ��ƶ���ֵ): %d m\r\n", JT808Conf_struct.BD_CycleRadius_DoorValue );
			break;
		case   0x005B: // ���ٱ���Ԥ����ֵ   1/10 KM/h
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.BD_MaxSpd_preWarnValue = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ���ٱ���Ԥ����ֵ: %d x 0.1km/h\r\n", JT808Conf_struct.BD_MaxSpd_preWarnValue );

			break;
		case   0x005C: // ƣ�ͼ�ʻ��ֵ  ��λ:s
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.BD_TiredDrv_preWarnValue = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ƣ�ͼ�ʻ��ֵ: %d s\r\n", JT808Conf_struct.BD_TiredDrv_preWarnValue );
			break;
		case   0x005D: // ��ײ������������
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.BD_Collision_Setting = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��ײ������������ : %x \r\n", JT808Conf_struct.BD_MaxSpd_preWarnValue );
			break;
		case   0x005E: // �෭������������   Ĭ��30��
			if( infolen != 2 )
			{
				break;
			}
			JT808Conf_struct.BD_Laydown_Setting = ( Content[0] << 8 ) + Content[1];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n�෭������������: %x r\n", JT808Conf_struct.BD_Laydown_Setting );
			break;
		//---  CAMERA
		case   0x0064: //  ��ʱ���տ���
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.BD_CameraTakeByTime_Settings = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n ��ʱ���տ���: %X\r\n", JT808Conf_struct.BD_CameraTakeByTime_Settings );
			break;
		case   0x0065: //  ���������տ���
			if( infolen != 4 )
			{
				break;
			}
			JT808Conf_struct.BD_CameraTakeByDistance_Settings = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			rt_kprintf( "\r\n  ���������տ���: %X\r\n", JT808Conf_struct.BD_CameraTakeByDistance_Settings );
			break;
		//--- GNSS
		case    0x0090:                         // GNSS ��λģʽ

			JT808Conf_struct.BD_EXT.GNSS_Mode = Content[0];
			rt_kprintf( "\r\n  GNSS Value= 0x%2X	\r\n", Content[0] );
			switch( JT808Conf_struct.BD_EXT.GNSS_Mode )
			{
				case 0x01:                      // �� GPS ��λģʽ
					gps_mode( "2" );
					Car_Status[1]	&= ~0x0C;   // clear bit3 bit2      1100
					Car_Status[1]	|= 0x04;    // Gps mode   0100
					break;
				case  0x02:                     //  ��BD2 ��λģʽ
					gps_mode( "1" );
					Car_Status[1]	&= ~0x0C;   // clear bit3 bit2
					Car_Status[1]	|= 0x08;    // BD mode	1000
					break;
				case  0x03:                     //  BD2+GPS ��λģʽ
					gps_mode( "3" );
					Car_Status[1]	&= ~0x0C;   // clear bit3 bit2
					Car_Status[1]	|= 0x0C;    // BD+GPS  mode	1100
					break;
			}
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			break;
		case    0x0091:                         // GNSS ������
			JT808Conf_struct.BD_EXT.GNSS_Baud = Content[0];
			switch( JT808Conf_struct.BD_EXT.GNSS_Baud )
			{
				case 0x00:                      //  4800
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 4800;
					//rt_thread_delay(5);
					//gps_write("$PCAS01,0*1C\r\n",14);
					//rt_thread_delay(5);
					// gps_baud( 4800 );
					break;
				case 0x01:                      //9600   --default
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 9600;
					//rt_thread_delay(5);
					//gps_write("$PCAS01,1*1D\r\n",14);
					//rt_thread_delay(5);
					//gps_baud( 9600 );
					break;
				case  0x02:                     // 19200
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 19200;
					//rt_thread_delay(5);
					//gps_write("$PCAS01,2*1E\r\n",14);
					//rt_thread_delay(5);
					//gps_baud( 19200 );
					break;
				case  0x03:                     //  38400
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 38400;
					//rt_thread_delay(5);
					//gps_write("$PCAS01,3*1F\r\n",14);
					//rt_thread_delay(5);
					//gps_baud( 38400 );
					break;
				case  0x04:                     // 57600
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 57600;
					//rt_thread_delay(5);
					//gps_write("$PCAS01,4*18\r\n",14);
					//rt_thread_delay(5);
					//gps_baud( 57600 );
					break;
				case   0x05:                    // 115200
					JT808Conf_struct.BD_EXT.GNSS_Baud_Value = 115200;
					//rt_thread_delay(5);
					// gps_write("$PCAS01,5*19\r\n",14);
					//rt_thread_delay(5);
					// gps_baud( 115200 );
					break;
			}
			//---UART_GPS_Init(baud);  //   �޸Ĵ��ڲ�����
			rt_thread_delay( 20 );
			// BD_EXT_Write();
			rt_kprintf( "\r\n ��������GNSS ������:  %d s\r\n", JT808Conf_struct.BD_EXT.GNSS_Baud_Value );

			break;
		case    0x0092:     // GNSS ģ����ϸ��λ�������Ƶ��

			break;
		case    0x0093:     //  GNSS ģ����ϸ��λ���ݲɼ�Ƶ��  1

			break;
		case    0x0094:     // GNSS ģ����ϸ��λ�����ϴ���ʽ

			break;
		case    0x0095:     //  GNSS ģ����ϸ�ϴ�����

			break;
		//----CAN--
		case      0x0100:   //  CAN  ����ͨ�� 1  �ɼ����              0   : ��ʾ���ɼ�
			if( infolen != 4 )
			{
				break;
			}
			CAN_trans.can1_sample_dur	= ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			Can_RXnum					= 0;
			Can_sdnum					= 0;
			Can_same					= 0;
			break;
		case    0x0101: //  CAN  ����ͨ�� 1 �ϴ�ʱ����    0 :  ��ʾ���ϴ�
			if( infolen != 2 )
			{
				break;
			}
			CAN_trans.can1_trans_dur = ( Content[0] << 8 ) + Content[1];

			break;
		case    0x0102: //  CAN  ����ͨ�� 2  �ɼ����              0   : ��ʾ���ɼ�
			if( infolen != 4 )
			{
				break;
			}
			CAN_trans.can2_sample_dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			break;
		case    0x0103: //  CAN  ����ͨ�� 2 �ϴ�ʱ����    0 :  ��ʾ���ϴ�
			if( infolen != 2 )
			{
				break;
			}
			CAN_trans.can2_trans_dur = ( Content[0] << 8 ) + Content[1];
			break;
		case    0x0110: //  CAN ����ID �����ɼ�����
			CAN_trans.canid_2_NotGetID = ( ( Content[4] & 0x1F ) << 24 ) + ( Content[5] << 16 ) + ( Content[6] << 8 ) + Content[7];
			rt_kprintf( "\r\n���ɼ�ID  0x0110= %08X\r\n", CAN_trans.canid_2_NotGetID );

			break;
		case    0x0111: //  CAN ����ID �����ɼ����� ����
			if( infolen != 8 )
			{
				break;
			}
			OutPrint_HEX( "0x0111", Content, 8 );
			memcpy( CAN_trans.canid_1, Content, 8 );
			memset( CAN_trans.canid_1_Rxbuf, 0, sizeof( CAN_trans.canid_1_Rxbuf ) );
			CAN_trans.canid_1_RxWr		= 0;    // clear  write
			CAN_trans.canid_timer		= 0;
			CAN_trans.canid_0705_sdFlag = 0;

			//------ ������ֵ --------
			CAN_trans.canid_1_sample_dur = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			if( Content[4] & 0x40 )             // bit 30
			{
				CAN_trans.canid_1_ext_state = 1;
			} else
			{
				CAN_trans.canid_1_ext_state = 0;
			}
			CAN_trans.canid_1_Filter_ID = ( ( Content[4] & 0x1F ) << 24 ) + ( Content[5] << 16 ) + ( Content[6] << 8 ) + Content[7];

			rt_kprintf( "\r\n FilterID=%08X, EXTstate: %d   can1_samle=%d ms   canid_1_sample_dur=%dms    Trans_dur=%d s\r\n", CAN_trans.canid_1_Filter_ID, CAN_trans.canid_1_ext_state, CAN_trans.can1_sample_dur, CAN_trans.canid_1_sample_dur, CAN_trans.can1_trans_dur );

			break;
		default:
			return false;
	}

	return true;
}

//--------------------------------------------------------------------
u8  CentreSet_subService_8105H( u32 Control_ID, u8 infolen, u8 *Content )
{
	switch( Control_ID )
	{
		case 1:                                         //  ������������  ����֮����÷ֺŷָ�   ָ���ʽ����:


			/*
			   URL ��ַ���������ƣ������û������������룻��ַ��TCP�˿ڣ�UDP�˿ڣ�������ID; Ӳ���汾���̼��汾�����ӵ�ָ��������ָ���Ƿ�����ʱ�ޣ�
			   ��ĳ����������ֵ����ſ�
			 */
			rt_kprintf( "\r\n �������� \r\n" );
			rt_kprintf( "\r\n ����: %s\r\n", Content );
			break;
		case 2:                                         // �����ն�����ָ��������


			/*
			   ���ӿ��ƣ����ƽ̨��Ȩ�룻���ŵ����ƣ� �����û������������룻��ַ��TCP�˿ڣ�UDP�˿ڣ����ӵ�ָ��������ʱ��
			   ��ÿ����������ֵ����ſ�
			 */
			rt_kprintf( "\r\n �ն˿�������ָ��������\r\n" );
			rt_kprintf( "\r\n ����: %s\r\n", Content );
			break;
		case 3:                                         //  �ն˹ػ�
			SD_ACKflag.f_CentreCMDack_0001H = 5;
			rt_kprintf( "\r\n �ն˹ػ� \r\n" );
			break;
		case 4:                                         //  �ն˸�λ
			SD_ACKflag.f_CentreCMDack_0001H = 3;
			rt_kprintf( "\r\n �ն˸�λ \r\n" );
			break;
		case 5:                                         //   �ն˻ָ���������

			if( SysConf_struct.Version_ID == SYSID )    //  check  wether need  update  or not
			{
				SysConf_struct.Version_ID = SYSID + 1;
				Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
				Systerm_Reset_counter	= Max_SystemCounter;
				ISP_resetFlag			= 2;            //   ����Զ�������������Ƹ�λϵͳ
			}
			rt_kprintf( "\r\n �ָ��������� \r\n" );
			break;
		case 6:                                         //   �ر�����ͨ��
			SD_ACKflag.f_CentreCMDack_0001H = 5;
			rt_kprintf( "\r\n �ر�����ͨ�� \r\n" );
			break;
		case 7:                                         //   �ر���������ͨ��
			SD_ACKflag.f_CentreCMDack_0001H = 5;
			rt_kprintf( "\r\n �ر�����ͨ�� \r\n" );
			break;
		default:
			return false;
	}
	return true;
}

//-------------------------------------------------------------------
void CenterSet_subService_8701H( u8 cmd, u8*Instr )
{
	switch( cmd )
	{
		case 0x81:                                                                  //	  �������� ��ʻԱ����  ��ʻ֤����
			memset( JT808Conf_struct.Driver_Info.DriverCard_ID, 0, 18 );
			//  ��ʻԱ����û�д��� 3���ֽ�
			memcpy( JT808Conf_struct.Driver_Info.DriveCode, Instr, 3 );
			memcpy( JT808Conf_struct.Driver_Info.DriverCard_ID, Instr + 3, 18 );    //ֻҪ��ʻ֤����
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			break;

		case 0x82:                                                                  //	  �������ó��ƺ�
			memset( (u8*)&JT808Conf_struct.Vechicle_Info, 0, sizeof( JT808Conf_struct.Vechicle_Info ) );

			memcpy( JT808Conf_struct.Vechicle_Info.Vech_VIN, Instr, 17 );
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_Num, Instr + 17, 12 );
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_Type, Instr + 29, 12 );

			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			break;
		case 0xC2:                                                                                                      //���ü�¼��ʱ��
			// ûɶ�ã������ظ����У�����GPSУ׼�͹���

			break;

		case 0xC3:                                                                                                      //�����ٶ�����ϵ��������ϵ����
			JT808Conf_struct.Vech_Character_Value = (u32)( Instr[0] << 16 ) + (u32)( Instr[1] << 8 ) + (u32)Instr[2];   // ����ϵ��  �ٶ�����ϵ��

			JT808Conf_struct.DF_K_adjustState	= 0;
			ModuleStatus						&= ~Status_Pcheck;
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			break;
		default:
			break;
	}
}

//-----------------------------------------------
u8  CentreSet_subService_FF01H( u32 SubID, u8 infolen, u8 *Content )
{
	u32 baud = 0;
	u8	u3_Txstr[40];
	u8	Destu3_Txstr[40];
	u8	len = 0;
	u8	i	= 0;

	rt_kprintf( "\r\n    �յ���չ�ն��������� SubID=%X \r\n", SubID );

	switch( SubID )
	{
		case 0x0003: //  BNSS NMEA ���������
			if( infolen != 4 )
			{
				break;
			}
			BD_EXT.BD_OutputFreq = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			switch( BD_EXT.BD_OutputFreq )
			{
				case 0x00:
					baud = 500;
					gps_write( "$PCAS02,500*1A\r\n", 16 );
					rt_thread_delay( 5 );
					break;
				case 0x01:
					baud = 1000;
					gps_write( "$PCAS02,1000*2E\r\n", 16 );
					rt_thread_delay( 5 );
					break;
				case 0x02:
					baud = 2000;
					gps_write( "$PCAS02,2000*2D\r\n", 16 );
					rt_thread_delay( 5 );
					break;
				case  0x03:
					baud = 3000;
					gps_write( "$PCAS02,3000*2C\r\n", 16 );
					rt_thread_delay( 5 );
					break;
				case  0x04:
					baud = 4000;
					gps_write( "$PCAS02,4000*2B\r\n", 16 );
					rt_thread_delay( 5 );
					break;
			}
			// BD_EXT_Write();
			rt_kprintf( "\r\n  GNSS �������: %dms\r\n", baud );
			break;
		case 0x0004: // GNSS �ɼ�NMEA ����Ƶ��
			if( infolen != 4 )
			{
				break;
			}
			BD_EXT.BD_SampleFrea = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			rt_kprintf( "\r\n GNSS �ɼ�Ƶ��: %d s\r\n", BD_EXT.BD_SampleFrea );
			break;
		case 0x0005: //  CAN 1  ��������
			if( infolen != 4 )
			{
				break;
			}
			BD_EXT.CAN_1_Mode = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			CAN_App_Init( );
			//BD_EXT_Write();
			rt_kprintf( "\r\n ��������CAN1 : 0x%08X\r\n", BD_EXT.CAN_1_Mode );
			break;
		case 0x0006: //  CAN2   ��������    ----- Ҫ��С��ͨ��
			if( infolen != 4 )
			{
				break;
			}
			BD_EXT.CAN_2_Mode = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			// BD_EXT_Write();
			memset( u3_Txstr, 0, sizeof( u3_Txstr ) );
			memset( Destu3_Txstr, 0, sizeof( Destu3_Txstr ) );
			u3_Txstr[0] = 0x7E; // ͷ
			u3_Txstr[1] = 0x33; //  CAN ���
			u3_Txstr[2] = 0x01; //  ��������
			u3_Txstr[3] = ( BD_EXT.CAN_2_Mode >> 24 );
			u3_Txstr[4] = ( BD_EXT.CAN_2_Mode >> 16 );
			u3_Txstr[5] = ( BD_EXT.CAN_2_Mode >> 8 );
			u3_Txstr[6] = ( BD_EXT.CAN_2_Mode );
			u3_Txstr[7] = 0x7E;

			Destu3_Txstr[0]			= 0x7E;
			len						= Protocol_808_Encode( Destu3_Txstr + 1, u3_Txstr + 1, 6 );
			Destu3_Txstr[len + 1]	= 0x7E;

			rt_kprintf( "\r\n U3_orginal:" );
			for( i = 0; i < 8; i++ )
			{
				rt_kprintf( "% 02X", u3_Txstr[i] );
			}
			rt_kprintf( "\r\nU3_len=%d \r\n", len );
			for( i = 0; i < len + 2; i++ )
			{
				rt_kprintf( "% 02X", Destu3_Txstr[i] );
			}
			// U3_PutData(Destu3_Txstr,len+2);  //  ���͸�����
			rt_device_write( &Device_CAN2, 0, Destu3_Txstr, len + 2 );
			rt_kprintf( "\r\n ��������CAN2 : 0x%08X\r\n", BD_EXT.CAN_2_Mode );
			break;
		case 0x0007: //  ��ײ��������
			if( infolen != 4 )
			{
				break;
			}
			BD_EXT.Collision_Check = ( Content[0] << 24 ) + ( Content[1] << 16 ) + ( Content[2] << 8 ) + Content[3];
			//BD_EXT_Write();
			memset( u3_Txstr, 0, sizeof( u3_Txstr ) );
			memset( Destu3_Txstr, 0, sizeof( Destu3_Txstr ) );
			u3_Txstr[0] = 0x7E; // ͷ
			u3_Txstr[1] = 0x32; //  CAN ���
			u3_Txstr[2] = 0x01; //  ��������
			u3_Txstr[3] = ( BD_EXT.Collision_Check >> 24 );
			u3_Txstr[4] = ( BD_EXT.Collision_Check >> 16 );
			u3_Txstr[5] = ( BD_EXT.Collision_Check >> 8 );
			u3_Txstr[6] = BD_EXT.Collision_Check;
			u3_Txstr[7] = 0x7E;

			Destu3_Txstr[0]			= 0x7E;
			len						= Protocol_808_Encode( Destu3_Txstr + 1, u3_Txstr + 1, 6 );
			Destu3_Txstr[len + 1]	= 0x7E;

			//   ����������ײ����
			mma8451_config( (uint16_t)( BD_EXT.Collision_Check >> 16 ), (uint16_t)( BD_EXT.Collision_Check ) );
			rt_kprintf( "\r\n ����������ײ����: 0x%08X\r\n", BD_EXT.Collision_Check );;
			break;

		default:
			return false;
	}
	rt_thread_delay( 20 );
	return true;
}

//--------------------------------------------------
u8  CentreSet_subService_FF03H( u32 SubID, u8 infolen, u8 *Content )
{
	u8	i = 0;
	u8	u3_Txstr[40];
	u8	Destu3_Txstr[40];
	u8	len = 0;

	rt_kprintf( "\r\n    �յ���չ�ն˲�������1 ���� SubID=%X \r\n", SubID );

	switch( SubID )
	{
		case 0x0001: // CAN ID ����
			if( infolen < 4 )
			{
				break;
			}
			// Content[0];  //  ��������
			//Content[1] ;//����Ϣ�а���CAN ID ������
			//------------------------------------
			//Content[2];  //  CAN ID ��ID
			if( Content[3] & 0x80 )                                                                                 //  CAN ID   ������
			{
				BD_EXT.CAN_2_ID = ( Content[4] << 24 ) + ( Content[5] << 16 ) + ( Content[6] << 8 ) + Content[7];   //
				if( Content[3] & 0x40 )
				{
					BD_EXT.CAN_2_Type = 1;                                                                          // ��׼֡
				}else
				{
					BD_EXT.CAN_2_Type = 0;
				}
				BD_EXT.CAN_2_TransDuration = ( Content[9] << 8 ) + Content[10];                                     // 0   ��ʾֹͣ

				//     ���и���С��ͨ�ŵĴ���
				memset( Destu3_Txstr, 0, sizeof( Destu3_Txstr ) );
				u3_Txstr[0]		= 0x7E;                                                                             // ͷ
				u3_Txstr[1]		= 0x33;                                                                             //  CAN ���
				u3_Txstr[2]		= 0x02;                                                                             //  ��������
				u3_Txstr[3]		= Content[3];                                                                       // ����
				u3_Txstr[4]		= Content[4];                                                                       //  ID
				u3_Txstr[5]		= Content[5];
				u3_Txstr[6]		= Content[6];
				u3_Txstr[7]		= Content[7];
				u3_Txstr[8]		= Content[8];                                                                       // ID
				u3_Txstr[9]		= Content[9];                                                                       //
				u3_Txstr[10]	= Content[10];
				u3_Txstr[11]	= 0x7E;

				Destu3_Txstr[0]			= 0x7E;
				len						= Protocol_808_Encode( Destu3_Txstr + 1, u3_Txstr + 1, 10 );
				Destu3_Txstr[len + 1]	= 0x7E;

				rt_kprintf( "\r\nU3_len=%d   :\r\n", len );
				for( i = 0; i < len + 2; i++ )
				{
					rt_kprintf( "% 02X", Destu3_Txstr[i] );
				}

				// U3_PutData(Destu3_Txstr,len+2);  //  ���͸�����
				rt_device_write( &Device_CAN2, 0, Destu3_Txstr, len + 2 );
				rt_kprintf( "\r\n ��������CAN2 : 0x%08X   Dur: %d s\r\n", BD_EXT.CAN_2_ID, BD_EXT.CAN_2_TransDuration );
				//----------------------------------
			}else
			{
				BD_EXT.CAN_1_ID = ( Content[4] << 24 ) + ( Content[5] << 16 ) + ( Content[6] << 8 ) + Content[7];   //
				if( Content[3] & 0x40 )
				{
					BD_EXT.CAN_1_Type = 1;                                                                          // ��׼֡
				}else
				{
					BD_EXT.CAN_1_Type = 0;
				}
				BD_EXT.CAN_1_TransDuration = ( Content[9] << 8 ) + Content[10];                                     // 0   ��ʾֹͣ

				rt_kprintf( "\r\n ��������CAN1 : 0x%08X   Dur: %d s\r\n", BD_EXT.CAN_1_ID, BD_EXT.CAN_1_TransDuration );
			}
			BD_EXT_Write( );
			break;
		case 0x0002:                                                                                                // �ı���Ϣ��־����
			                                                                                                        // Content[0]; // û��
			                                                                                                        //if((Content[1]==0)||(Content[1]==1))  //  �������Ͷ�����ʾ��
		{
			memset( TextInfo.TEXT_Content, 0, sizeof( TextInfo.TEXT_Content ) );
			DwinLCD.TxInfolen = AsciiToGb( TextInfo.TEXT_Content, infolen - 2, Content + 2 );
			//memcpy(TextInfo.TEXT_Content,Content+2,infolen-2);
			TextInfo.TEXT_SD_FLAG = 1;                                                                              // �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||
			rt_kprintf( "\r\n  CAN �ı�:  " );
			for( i = 0; i < DwinLCD.TxInfolen; i++ )
			{
				rt_kprintf( "%c", TextInfo.TEXT_Content[i] );
			}
			rt_kprintf( "\r\n " );
		}

#ifdef LCD_5inch
			//======  ��Ϣ������Ļ����ʾ
			memset( DwinLCD.TXT_content, 0, sizeof( DwinLCD.TXT_content ) );
			DwinLCD.TXT_contentLen	= AsciiToGb( DwinLCD.TXT_content, infolen - 2, Content + 2 );
			DwinLCD.Type			= LCD_SDTXT;

#endif

			break;
		default:
			return false;
	}
	return true;
}

//-------------------------------------------
void  Media_Start_Init( u8 MdType, u8 MdCodeType )
{
	MediaObj.Media_Type				= MdType;       //	ָ����ǰ�����ý�������   0  ��ʾͼƬ
	MediaObj.Media_CodeType			= MdCodeType;   //  ��ý������ʽ   0  ��ʾJPEG	��ʽ
	MediaObj.SD_media_Flag			= 1;            //  �ö�ý���¼���Ϣ���ͱ�־λ  ����ʼͼƬ����
	MediaObj.SD_Eventstate			= 1;            //  ��ʼ���ڷ���״̬
	MediaObj.RSD_State				= 0;
	MediaObj.RSD_Timer				= 0;
	MediaObj.RSD_total				= 0;
	MediaObj.RSD_Reader				= 0;
	MediaObj.SD_Data_Flag			= 0;
	MediaObj.Media_transmittingFlag = 0;
	//------------  add  for debuging   --------------
	// Media_Clear_State();  //  clear


	/*  if(0==MediaObj.Media_Type)
	   {
	   Photo_sdState.photo_sending=enable;
	   Photo_sdState.SD_packetNum=1; // ��һ����ʼ
	   rt_kprintf("\r\n ��ʼ�ϴ���Ƭ! ....\r\n");
	   }
	 */
	//----------------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Media_Clear_State( void )
{
	// �����Meia Type
	MediaObj.MaxSd_counter			= 0;
	MediaObj.SD_Eventstate			= 0;
	MediaObj.SD_timer				= 0;
	MediaObj.SD_media_Flag			= 0;
	MediaObj.SD_Data_Flag			= 0;
	MediaObj.RSD_State				= 0;
	MediaObj.RSD_Timer				= 0;
	MediaObj.RSD_total				= 0;
	MediaObj.RSD_Reader				= 0;
	MediaObj.Media_transmittingFlag = 0;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  Media_Timer( void )
{
	if( 1 == MediaObj.SD_Eventstate )
	{
		MediaObj.SD_timer++;
		if( MediaObj.SD_timer > 6 )
		{
			MediaObj.SD_timer		= 0;
			MediaObj.SD_media_Flag	= 1;
			MediaObj.MaxSd_counter++;
			if( MediaObj.MaxSd_counter > 5 )
			{
				Media_Clear_State( );
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Media_RSdMode_Timer( void )
{
    //   ��ý���б��ش� 
	if( ( 1 == MediaObj.RSD_State ) )
	{
		MediaObj.RSD_Timer++;
		if( MediaObj.RSD_Timer > 35 )
		{
			MediaObj.RSD_Timer		= 0;
			MediaObj.SD_Data_Flag	= 1;    // ���ش����Ͷ�ý����Ϣ��־λ
			switch( MediaObj.Media_Type )   //   ͼƬ�ش�����
			{
				case 0:                     //  ͼ��
					Photo_sdState.SD_packetNum	= MediaObj.Media_ReSdList[MediaObj.RSD_Reader]; 
					Photo_sdState.SD_flag		= 1;
					break;
				case 1:                     // ��Ƶ
					Sound_sdState.SD_packetNum	= MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
					Sound_sdState.SD_flag		= 1;
					break;
				case 2:                     // ��Ƶ
					Video_sdState.SD_packetNum	= MediaObj.Media_ReSdList[MediaObj.RSD_Reader];
					Video_sdState.SD_flag		= 1;
					break;
				default:
					break;
			}
			//-----  �ش��б� ���� -----------
			MediaObj.RSD_Reader++;
			// rt_kprintf("\r\n	  MediaObj.RSD_Reader++  =%d\r\n",MediaObj.RSD_Reader);
			if( MediaObj.RSD_Reader == MediaObj.RSD_total )
			{
				MediaObj.RSD_State = 2; //  ��λ�ȴ�״̬���ȴ��������ٷ��ش�ָ��
			}
		}
	}else
	if( 2 == MediaObj.RSD_State )
	{
		MediaObj.RSD_Timer++;
		if( MediaObj.RSD_Timer > 140 )  //   ���״̬һֱ�ڵȴ��ҳ���30s�����״̬
		{
			switch( MediaObj.Media_Type )
			{
				case 0:                 // ͼ��
					Photo_send_end( );  // �����ϴ�����

					break;
				case 1:                 // ��Ƶ
					Sound_send_end( );
					break;
				case 2:                 // ��Ƶ
					Video_send_end( );
					break;
				default:
					break;
			}
			Media_Clear_State( );
			rt_kprintf( "\r\n ��Ϣ�ش���ʱ����! \r\n" );

			//Check_MultiTakeResult_b4Trans();  // ��·����ͷ����״̬���
		}
	}

    //     ��¼���б��ش� ��ʱ����
    
	if( ( 1 == Recode_Obj.RSD_State ) )
	{
		Recode_Obj.RSD_Timer++;
		if( Recode_Obj.RSD_Timer > 35 )
		{
			Recode_Obj.RSD_Timer		= 0;
            Recode_Obj.SD_Data_Flag	= 1; 

            Recode_Obj.Current_pkt_num	= Recode_Obj.Media_ReSdList[Recode_Obj.RSD_Reader];
			//  ����ش��б��һ��Ϊ1  ��ô������1 ����ֱ������
			if(Recode_Obj.Current_pkt_num==1)
			 {
			    Recode_Obj.RSD_Reader++;
				rt_kprintf( "\r\n --- �ش��б��һ��Ϊ��1 �� ��jump ������ \r\n" );
			 }
			//-----  �ش��б� ���� -----------
			Recode_Obj.RSD_Reader++;
			// rt_kprintf("\r\n	  MediaObj.RSD_Reader++  =%d\r\n",MediaObj.RSD_Reader);
			if( Recode_Obj.RSD_Reader > Recode_Obj.RSD_total )
			{
				Recode_Obj.RSD_State = 2; //  ��λ�ȴ�״̬���ȴ��������ٷ��ش�ָ��
			}
		}
	}	    
	if( 2 == Recode_Obj.RSD_State )
	{
		Recode_Obj.RSD_Timer++;
		if( Recode_Obj.RSD_Timer > 140 )  //   ���״̬һֱ�ڵȴ��ҳ���30s�����״̬
		{
		   Recorder_init(0);
           rt_kprintf( "\r\n ��¼����Ϣ�ش���ʱ����! \r\n" );
	       if(Recode_Obj.Transmit_running==1)
        	{
        	  Rcorder_Recover(); 
              rt_kprintf( "\r\n ˳��ִ����δ���  current=%d  total=%d\r\n",Recode_Obj.Current_pkt_num,Recode_Obj.Total_pkt_num);
        	}
		}
	}

	
}

#ifdef   MEDIA
//-------- photo send -------------------------------------
void Photo_send_start( u16 Numpic )
{
//  UINT ByteRead;
	// FIL FileCurrent;

	rt_kprintf( "   \r\n  Photo_send_start =%d \r\n", Numpic );
	Photo_sdState.photo_sending		= disable;
	Photo_sdState.SD_flag			= 0;
	Photo_sdState.SD_packetNum		= 1; // ��1 ��ʼ
	Photo_sdState.Exeption_timer	= 0;

	if( Camera_Number == 1 )
	{
		PicFileSize = Api_DFdirectory_Query( camera_1, 1 );
	} else
	if( Camera_Number == 2 )
	{
		PicFileSize = Api_DFdirectory_Query( camera_2, 1 );
	} else
	if( Camera_Number == 3 )
	{
		PicFileSize = Api_DFdirectory_Query( camera_3, 1 );
	} else
	if( Camera_Number == 4 )
	{
		PicFileSize = Api_DFdirectory_Query( camera_4, 1 );
	}

	//	 DF_ReadFlash(pic_current_page, 0,PictureName, 23);
	//      Camera_Number=PictureName[18];
	//      memcpy((u8*)&PicFileSize,PictureName+19,4);
	//pic_current_page++; //ָ���ļ�����
	//pic_PageIn_offset=0;// ͼƬ��ȡҳ��ƫ�Ƶ�ַ
	// rt_kprintf("\r\n    open Pic =%s",PictureName);

	if( PicFileSize % 512 )
	{
		Photo_sdState.Total_packetNum = PicFileSize / 512 + 1;
	} else
	{
		Photo_sdState.Total_packetNum = PicFileSize / 512;
	}

	rt_kprintf( "\r\n    Camera %d  ReadpicStart total :%d ��Pagesize: %d Bytes\r\n\r\n", Camera_Number, Photo_sdState.Total_packetNum, PicFileSize );
	if( ( Camera_Number == 0 ) || ( Photo_sdState.Total_packetNum == 0 ) )
	{
		Photo_send_end( ); // clear  state
		rt_kprintf( "\r\n  ͼƬ�ܰ���Ϊ�� ������ͷ���Ϊ0 ����������ʧ�ܵ����� \r\n" );
	}

	// -------  MultiMedia Related --------
	Media_Start_Init( 0, 0 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Sound_send_start( void )
{
	u8	sound_name[20];
	u16 i;
//  u8  oldstate=0;
//  u16 i2,j;
	// u8  WrieEnd=0,LeftLen=0;

	Sound_sdState.photo_sending		= disable;
	Sound_sdState.SD_flag			= 0;
	Sound_sdState.SD_packetNum		= 1; // ��1 ��ʼ
	Sound_sdState.Exeption_timer	= 0;

	//---  Speex_Init();	 // speachX ��ʼ��
	// 1. �������µ��ļ�


/*  memset((u8*)&MediaIndex,0,sizeof(MediaIndex));
   for(i=0;i<8;i++)
   {
   Api_RecordNum_Read(voice_index, i+1, (u8*)&MediaIndex, sizeof(MediaIndex));
        if(MediaIndex.Effective_Flag==1)
        {
            break;
   }
   }
   if(MediaIndex.Effective_Flag)
   {
   rt_kprintf("\r\n ����filename:%s\r\n",MediaIndex.FileName);
   }
   else
   {
      rt_kprintf("\r\n û���Ѵ洢����Ƶ�ļ� \r\n");
      return false	;
   }

 */

	//	2.	����wav �ļ�

	//	3. �ļ���С
	// file name
	memset( sound_name, 0, sizeof( sound_name ) );
	DF_ReadFlash( SoundStart_offdet, 4, sound_name, 20 );
	SrcFileSize = Api_DFdirectory_Query( voice, 1 );
	//  Sound_sdState.Total_packetNum=(SrcFileSize/512); // ÿ��100���ֽ�
	if( SrcFileSize % 512 )
	{
		Sound_sdState.Total_packetNum = SrcFileSize / 512 + 1;
	} else
	{
		Sound_sdState.Total_packetNum = SrcFileSize / 512;
	}
	rt_kprintf( "\r\n	�ļ���: %s��С: %d Bytes  totalpacketnum=%d \r\n", sound_name, SrcFileSize, Sound_sdState.Total_packetNum );

	// -------  MultiMedia Related --------
	//  Media_Start_Init(1,5); // ��Ƶ  wav ��ʽ   0:JPEG ;   1: TIF ;   2:MP3;  3:WAV  4: WMV  ��������
	//    5   amr
	Media_Start_Init( 1, 99 ); // ��word�ĵ�Ҫ�� ���ڷ�mp3  �� wav��ʵ��99
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  MP3_send_start( void )
{
	u8 mp3_name[13];
	mp3_sendstate					= 1;
	Sound_sdState.photo_sending		= disable;
	Sound_sdState.SD_flag			= 0;
	Sound_sdState.SD_packetNum		= 1; // ��1 ��ʼ
	Sound_sdState.Exeption_timer	= 0;

	memset( mp3_name, 0, sizeof( mp3_name ) );
	memcpy( mp3_name, "ch12.mp3", 8 );

	if( mp3_fsize % 512 )
	{
		Sound_sdState.Total_packetNum = mp3_fsize / 512 + 1;
	} else
	{
		Sound_sdState.Total_packetNum = mp3_fsize / 512;
	}

	rt_kprintf( "\r\n  mp3�ļ�����:%s    �ļ���С  %d Bytes  \r\n", mp3_name, mp3_fsize );

	// -------  MultiMedia Related --------
	Media_Start_Init( 1, 2 ); // ��Ƶ  wav ��ʽ
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  Video_send_start( void )
{
	u8 video_name[13];

	wmv_sendstate					= 1;
	Video_sdState.photo_sending		= disable;
	Video_sdState.SD_flag			= 0;
	Video_sdState.SD_packetNum		= 1; // ��1 ��ʼ
	Video_sdState.Exeption_timer	= 0;

	memset( video_name, 0, sizeof( video_name ) );
	memcpy( video_name, "ch1.wmv", 7 );

	if( wmv_fsize % 512 )
	{
		Video_sdState.Total_packetNum = wmv_fsize / 512 + 1;
	} else
	{
		Video_sdState.Total_packetNum = wmv_fsize / 512;
	}

	rt_kprintf( "\r\n  wmv�ļ�����:%s    �ļ���С  %d Bytes  \r\n", video_name, wmv_fsize );

	// -------  MultiMedia Related --------
	Media_Start_Init( 2, 4 ); // ��Ƶ  wav ��ʽ
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8   DrvRecoder_send_start( void )
{
	DrvRecoder_sdState.photo_sending	= disable;
	DrvRecoder_sdState.SD_flag			= 0;
	DrvRecoder_sdState.SD_packetNum		= 1;    // ��1 ��ʼ
	DrvRecoder_sdState.Exeption_timer	= 0;

	Media_Start_Init( 3, 0 );                   // �г���¼��
	return true;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  BlindZoneData_send_start( void )
{
	BlindZone_sdState.photo_sending		= disable;
	BlindZone_sdState.SD_flag			= 0;
	BlindZone_sdState.SD_packetNum		= 1;    // ��1 ��ʼ
	BlindZone_sdState.Exeption_timer	= 0;

       BlindZone_sdState.Total_packetNum=BLIND_NUM/MQ_PKNUM;

	MediaObj.Media_Type = 4;                    // ä��

	// Media_Start_Init(4,0); // ä��
	return true;
}

//==========================================================
void Photo_send_end( void )
{
	Photo_sdState.photo_sending		= 0;
	Photo_sdState.SD_flag			= 0;
	Photo_sdState.SD_packetNum		= 0;
	Photo_sdState.Total_packetNum	= 0;
	Photo_sdState.Exeption_timer	= 0;
	MediaObj.Media_transmittingFlag = 0; // clear
	Media_Clear_State( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Sound_send_end( void )
{
	Sound_sdState.photo_sending		= 0;
	Sound_sdState.SD_flag			= 0;
	Sound_sdState.SD_packetNum		= 0;
	Sound_sdState.Total_packetNum	= 0;
	Sound_sdState.Exeption_timer	= 0;
	MediaObj.Media_transmittingFlag = 0;    // clear
	mp3_sendstate					= 0;
	VocREC.running					= 0;    // clear
	Media_Clear_State( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Video_send_end( void )
{
	Video_sdState.photo_sending		= 0;
	Video_sdState.SD_flag			= 0;
	Video_sdState.SD_packetNum		= 0;
	Video_sdState.Total_packetNum	= 0;
	Video_sdState.Exeption_timer	= 0;
	MediaObj.Media_transmittingFlag = 0; // clear
	wmv_sendstate					= 0;
	Media_Clear_State( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void   DrvRecoder_send_end( void )
{
	DrvRecoder_sdState.photo_sending	= disable;
	DrvRecoder_sdState.SD_flag			= 0;
	DrvRecoder_sdState.SD_packetNum		= 1;    // ��1 ��ʼ
	DrvRecoder_sdState.Exeption_timer	= 0;

	MediaObj.Media_transmittingFlag = 0;        // clear
	Media_Clear_State( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  BlindZoneData_send_end( void )
{
	BlindZone_sdState.photo_sending		= disable;
	BlindZone_sdState.SD_flag			= 0;
	BlindZone_sdState.SD_packetNum		= 1;                                        // ��1 ��ʼ
	BlindZone_sdState.Exeption_timer	= 0;

	MediaObj.Media_transmittingFlag = 0;                                            // clear
	Media_Clear_State( );
}

//=============================================
void Video_Timer( void )
{
	if( ( Video_sdState.photo_sending == enable ) && ( 2 == MediaObj.Media_Type ) ) // ��Ƶ
	{
		if( ( Video_sdState.SD_packetNum <= Video_sdState.Total_packetNum + 1 ) && ( 2 != MediaObj.RSD_State ) )
		{                                                                           //  һ�¶�ʱ����	��˳���͹�������	 ��   �յ��ش���ʼ����Ч
			Video_sdState.Data_SD_counter++;
			if( Video_sdState.Data_SD_counter > 40 )
			{
				Video_sdState.Data_SD_counter	= 0;
				Video_sdState.SD_flag			= 1;
				MediaObj.SD_Data_Flag			= 1;
				Video_sdState.Exeption_timer	= 0;
				//rt_kprintf("\r\n Video send Flag \r\n");
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Sound_Timer( void )
{
	if( ( Sound_sdState.photo_sending == enable ) && ( 1 == MediaObj.Media_Type ) ) // ��Ƶ
	{
		if( ( Sound_sdState.SD_packetNum <= Sound_sdState.Total_packetNum + 1 ) && ( 2 != MediaObj.RSD_State ) )
		{                                                                           //  һ�¶�ʱ����	��˳���͹�������	 ��   �յ��ش���ʼ����Ч
			Sound_sdState.Data_SD_counter++;
			if( Sound_sdState.Data_SD_counter > 35 )
			{
				Sound_sdState.Data_SD_counter	= 0;
				Sound_sdState.Exeption_timer	= 0;
				Sound_sdState.SD_flag			= 1;
				MediaObj.SD_Data_Flag			= 1;

				//rt_kprintf("\r\n Sound  Transmit set Flag \r\n");
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Photo_Timer( void )
{
	if( ( Photo_sdState.photo_sending == enable ) && ( 0 == MediaObj.Media_Type ) )
	{
		if( ( Photo_sdState.SD_packetNum <= Photo_sdState.Total_packetNum + 1 ) && ( 2 != MediaObj.RSD_State ) )
		{                                               //  һ�¶�ʱ����   ��˳���͹�������   ��   �յ��ش���ʼ����Ч
			Photo_sdState.Data_SD_counter++;
			if( Photo_sdState.Data_SD_counter > 35 )    //40   12
			{
				Photo_sdState.Data_SD_counter	= 0;
				Photo_sdState.Exeption_timer	= 0;
				Photo_sdState.SD_flag			= 1;
				MediaObj.SD_Data_Flag			= 1;
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void DrvRecoder_Timer( void )
{
	if( ( DrvRecoder_sdState.photo_sending == enable ) && ( 3 == MediaObj.Media_Type ) )
	{
		if( ( DrvRecoder_sdState.SD_packetNum <= DrvRecoder_sdState.Total_packetNum + 1 ) && ( 3 != MediaObj.RSD_State ) )
		{                                                   //  һ�¶�ʱ����   ��˳���͹�������   ��   �յ��ش���ʼ����Ч
			DrvRecoder_sdState.Data_SD_counter++;
			if( DrvRecoder_sdState.Data_SD_counter > 35 )   //40   12
			{
				DrvRecoder_sdState.Data_SD_counter	= 0;
				DrvRecoder_sdState.Exeption_timer	= 0;
				DrvRecoder_sdState.SD_flag			= 1;
				MediaObj.SD_Data_Flag				= 1;
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void BlindZone_Timer( void )
{
	if( ( BlindZone_sdState.photo_sending == enable ) && ( 4 == MediaObj.Media_Type ) )
	{
		if( ( BlindZone_sdState.SD_packetNum <= BlindZone_sdState.Total_packetNum + 1 ) && ( 4 != MediaObj.RSD_State ) )
		{                                                   //  һ�¶�ʱ����   ��˳���͹�������   ��   �յ��ش���ʼ����Ч
			BlindZone_sdState.Data_SD_counter++;
			if( BlindZone_sdState.Data_SD_counter > 35 )    //40   12
			{
				BlindZone_sdState.Data_SD_counter	= 0;
				BlindZone_sdState.Exeption_timer	= 0;
				BlindZone_sdState.SD_flag			= 1;
				MediaObj.SD_Data_Flag				= 1;
			}
		}
	}
}

//=============================================
void Meida_Trans_Exception( void )
{
	u8 resualt = 0;

	if( Photo_sdState.photo_sending == enable )
	{
		if( Photo_sdState.Exeption_timer++ > 50 )
		{
			Photo_send_end( );
			resualt = 1;
		}
	}else
	if( Sound_sdState.photo_sending == enable )
	{
		if( Sound_sdState.Exeption_timer++ > 50 )
		{
			Sound_send_end( );
			resualt = 2;
		}
	}else
	if( Video_sdState.photo_sending == enable )
	{
		if( Video_sdState.Exeption_timer++ > 50 )
		{
			Video_send_end( );
			resualt = 2;
		}
	}else
	if( DrvRecoder_sdState.photo_sending == enable )
	{
		if( DrvRecoder_sdState.Exeption_timer++ > 50 )
		{
			DrvRecoder_send_end( );
			resualt = 2;
		}
	}else
	if( BlindZone_sdState.photo_sending == enable )
	{
		if( BlindZone_sdState.Exeption_timer++ > 50 )
		{
			BlindZoneData_send_end( );
			resualt = 2;
		}
	}

	if( resualt )
	{
		rt_kprintf( "\r\n   Media  Trans  Timeout  resualt: %d\r\n", resualt );
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Media_Timer_Service( void )
{
	//----------------------------------
	if( DataLink_Status( ) )
	{
		if( Photo_sdState.photo_sending == enable )
		{
			Photo_Timer( );
		} else
		if( Sound_sdState.photo_sending == enable )
		{
			Sound_Timer( );
		} else
		if( Video_sdState.photo_sending == enable )
		{
			Video_Timer( );
		} else
		if( DrvRecoder_sdState.photo_sending == enable )
		{
			DrvRecoder_Timer( );
		} else
		if( BlindZone_sdState.photo_sending == enable )
		{
			BlindZone_Timer( );
		}

		Media_RSdMode_Timer( );
	}
}

#endif
//------------------------------------------------------------
void DataTrans_Init( void )
{
	DataTrans.Data_RxLen	= 0;
	DataTrans.Data_TxLen	= 0;
	DataTrans.Tx_Wr			= 0;
	memset( DataTrans.DataRx, 0, sizeof( (const char*)DataTrans.DataRx ) );
	memset( DataTrans.Data_Tx, 0, sizeof( (const char*)DataTrans.Data_Tx ) );
}

//------------------------------------------------------------
void DoorCameraInit( void )
{
	DoorOpen.currentState	= 0;
	DoorOpen.BakState		= 0;
}

//-----------------------------------------------------------
void Spd_ExpInit( void )
{
	speed_Exd.current_maxSpd	= 0;
	speed_Exd.dur_seconds		= 0;
	speed_Exd.excd_status		= 0;
	memset( (char*)( speed_Exd.ex_startTime ), 0, 5 );
	speed_Exd.speed_flag = 0;
}

//-----------------------------------------------------------
void  TCP_RX_Process( u8 LinkNum )  //  ---- 808  ��׼Э��
{
	u16 i		= 0, j = 0;         //,DF_PageAddr;
	u16 infolen = 0, contentlen = 0, msg_len = 0;
	// u8   ireg[5];
	u8	Ack_Resualt		= 1;
	u16 Ack_CMDid_8001	= 0;
	u16 Ack_Terminal_ID_8001=0;     // �ն˷��ص���ˮ��  
	u8	Total_ParaNum	= 0;        // �������ò�������
	u8	Process_Resualt = 0;        //  bit ��ʾ   bit0 ��ʾ 1  bit 1 ��ʾ2
	u8	ContentRdAdd	= 0;        // ��ǰ��ȡ���ĵ�ַ
	u8	SubInfolen		= 0;        // ����Ϣ����
	u8	Reg_buf[50];
	//u8   CheckResualt=0;
	u32 reg_u32 = 0;
	//u16  Devide_8003_packet_1stID=0; //  �ְ��ϴ���һ�� ID 
	  u8  devide_value=0; 
	//----------------      �г���¼��808 Э�� ���մ���   --------------------------

	//  0.  Decode
	Protocol_808_Decode( );
	//  1.  fliter head
	if( UDP_HEX_Rx[0] != 0x7e )                                 //   ����ͷ
	{
		return;
	}
	//  2.  check Centre Ack
	Centre_CmdID	= ( UDP_HEX_Rx[1] << 8 ) + UDP_HEX_Rx[2];   // ���յ�������ϢID
	Centre_FloatID	= ( UDP_HEX_Rx[11] << 8 ) + UDP_HEX_Rx[12]; // ���յ�������Ϣ��ˮ��

	//  �ְ��ж�
	if( UDP_HEX_Rx[3] & 0x20 )
	{                                                           //  �ְ��ж�
		;
	}

	//  3.   get infolen    ( ����Ϊ��Ϣ��ĳ���)    ���ְ��Ļ�  ��Ϣͷ����Ϊ12 ��������У��ĳ��� =infolen+12
	infolen		= ( u16 )( ( UDP_HEX_Rx[3] & 0x03 ) << 8 ) + ( u16 )UDP_HEX_Rx[4];
	contentlen	= infolen + 12;                                 //  ����У���ֽڵĳ���

	//  4.   Check  Fcs
	FCS_RX_UDP = 0;
	//nop;nop;
	for( i = 0; i < ( UDP_DecodeHex_Len - 3 ); i++ )            //������յ����ݵ�����
	{
		FCS_RX_UDP ^= UDP_HEX_Rx[1 + i];
	}
	//nop;
	// ------- FCS filter -----------------
#if  0                                                          // ��֤ר�ã���ʱ ���ε��ж�У��
	if( UDP_HEX_Rx[UDP_DecodeHex_Len - 2] != FCS_RX_UDP )       //�ж�У����
	{
		rt_kprintf( "\r\n  infolen=%d ", infolen );
		rt_kprintf( "\r\n808Э��У�����	  Caucate %x  ,RX  %x\r\n", FCS_RX_UDP, UDP_HEX_Rx[UDP_DecodeHex_Len - 2] );
		//-----------------  memset  -------------------------------------
		memset( UDP_HEX_Rx, 0, sizeof( UDP_HEX_Rx ) );
		UDP_hexRx_len = 0;
		return;
	}
#endif
	//  else
	// rt_kprintf("\r\n 808Э��У����ȷ	  Caucate %x  ,RX  %x\r\n",FCS_RX_UDP,UDP_HEX_Rx[UDP_DecodeHex_Len-2]);

	//   5 .  Classify  Process
	rt_kprintf( "\r\n           CentreCMD = 0x%X  \r\n", Centre_CmdID );    // add for  debug

	switch( Centre_CmdID )
	{
		case  0x8001:                                                       //ƽ̨ͨ��Ӧ��
			// ��û�зְ�����Ļ�  ��Ϣͷ��12  ��0��ʼ�����12���ֽ�����Ϣ�������

			//  13 14  ��Ӧ���ն���Ϣ��ˮ��
			Ack_Terminal_ID_8001=( UDP_HEX_Rx[13] << 8 ) + UDP_HEX_Rx[14];
			//  15 16  ��Ӧ�ն˵���Ϣ
			Ack_CMDid_8001 = ( UDP_HEX_Rx[15] << 8 ) + UDP_HEX_Rx[16];

			switch( Ack_CMDid_8001 )                                        // �ж϶�Ӧ�ն���Ϣ��ID�����ִ���
			{
				case 0x0200:                                                //  ��Ӧλ����Ϣ��Ӧ��
					//  rt_kprintf( "\r\n  0200-1");
					//----- �ж�ȷ���Ƿ�ɹ�
					//  if(UDP_HEX_Rx[17]!=0x00)
					//	{     rt_kprintf( "\r\n  0200-2"); break;}

					//----- app msg ----------
					if( ( AppQue.sd_enable_flag == 2 ) && ( app_que_enable == 1 ) )
					{
						AppQue.sd_enable_flag = 0;
						rt_kprintf( "\r\n packet= %d  app_msg-ack", AppQue.read_num );
						AppQue.read_num++;
						if( AppQue.read_num >= 420 )
						{
							app_que_enable = 0;
							app_queenable( "0" );
							rt_kprintf( "\r\n DWJD over  =>return normal state\r\n" );
						}
					}
					//----------------------------------------
					if( Warn_Status[1] & 0x10 )         // �������򱨾�
					{
						InOut_Object.TYPE		= 0;    //Բ������
						InOut_Object.ID			= 0;    //  ID
						InOut_Object.InOutState = 0;    //  ������
						Warn_Status[1]			&= ~0x10;
						rt_kprintf( "\r\n����--- ����----�������!\r\n" );
					}

#ifndef   NEW_8203
					//--------------------------------
					if( Warn_Status[3] & 0x01 )         //��������
					{
						StatusReg_WARN_Clear( );
						f_Exigent_warning	= 0;
						warn_flag			= 0;
						Send_warn_times		= 0;
						StatusReg_WARN_Clear( );
						rt_kprintf( "\r\n���������յ�Ӧ�𣬵����!\r\n" );
					}
					if( Warn_Status[3] & 0x08 )         //Σ�ձ���-BD
					{
						Warn_Status[3] &= ~0x08;
					}

					//------------------------------------
					if( Warn_Status[1] & 0x10 )         // �������򱨾�
					{
						InOut_Object.TYPE		= 0;    //Բ������
						InOut_Object.ID			= 0;    //  ID
						InOut_Object.InOutState = 0;    //  ������
						Warn_Status[1]			&= ~0x10;
						rt_kprintf( "\r\n����--- ����----�������!\r\n" );
					}


					/* if(Warn_Status[1]&0x20)// ����·�߱���
					                                {
					                   InOut_Object.TYPE=0;//Բ������
					                   InOut_Object.ID=0; //  ID
					                   InOut_Object.InOutState=0;//  ������
					                   Warn_Status[1]&=~0x20;
					       rt_kprintf( "\r\n����-----·��----�������!\r\n");
					                                } */
					if( Warn_Status[1] & 0x40 )         // ·����ʻʱ�䲻�㡢����
					{
						Warn_Status[1] &= ~0x40;
					}
					//-----------------------------------------
					if( Warn_Status[0] & 0x08 )         //�����Ƿ����
					{
						Warn_Status[0] &= ~0x08;
					}
					if( Warn_Status[0] & 0x10 )         //�����Ƿ�λ��
					{
						Warn_Status[0] &= ~0x10;
					}
					if( Warn_Status[0] & 0x80 )         //�Ƿ����ű���(�ն�δ��������ʱ�����жϷǷ�����)
					{
						Warn_Status[0] &= ~0x80;
					}
					if( Warn_Status[0] & 0x60 )
					{
						Warn_Status[0] &= ~0x60;        // 2 ��bit    ��ײ�Ͳ෭
					}
#endif
					//------------------------------------
					rt_kprintf( "\r\nCentre ACK!\r\n" );
					
					//-------------------------------------------------------------------
					Api_cycle_Update();

					//--------------  ��ý���ϴ����  --------------
					if( MediaObj.Media_transmittingFlag == 1 ) // clear
					{
						MediaObj.Media_transmittingFlag = 2;
						if( Duomeiti_sdFlag == 1 )
						{
							Duomeiti_sdFlag = 0;
							Media_Clear_State( );
							Photo_send_end( );
							Sound_send_end( );
							Video_send_end( );
							rt_kprintf( "\r\n  �ֶ��ϱ���ý���ϴ�����\r\n" );
						}
						rt_kprintf( "\r\n  ��ý����Ϣǰ�Ķ�ý�巢����� \r\n" );
					}

					break;
				case 0x0002:                                            //  ��������Ӧ��
					//  �����н����  ---
					JT808Conf_struct.DURATION.TCP_ACK_DurCnter	= 0;    //clear
					JT808Conf_struct.DURATION.TCP_SD_state		= 0;    //clear
					rt_kprintf( "\r\n  Centre  Heart ACK!\r\n" );
					break;
				case 0x0101:                                            //  �ն�ע��Ӧ��
					if( 0 == UDP_HEX_Rx[17] )
					{                                                   // ע���ɹ�
						memset( Reg_buf, 0, sizeof( Reg_buf ) );
						memcpy( Reg_buf, JT808Conf_struct.ConfirmCode, 20 );
						JT808Conf_struct.Regsiter_Status	= 0;
						Reg_buf[20]							= JT808Conf_struct.Regsiter_Status;
						Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
						rt_kprintf( "\r\n  �ն�ע���ɹ�!  \r\n" );
					}

					break;
				case 0x0102:                            //  �ն˼�Ȩ

					rt_kprintf( "\r\n �յ���Ȩ���: %x \r\n", UDP_HEX_Rx[17] );
					if( 0 == UDP_HEX_Rx[17] )
					{                                   // ��Ȩ�ɹ�
						DEV_Login.Operate_enable = 2;   // ��Ȩ���
						if( DataLink_Status( ) )
						{
							DataLinkOK_Process( );
						}
						rt_kprintf( "\r\n  �ն˼�Ȩ�ɹ�!  \r\n" );
					}
					break;
				case 0x0800:                // ��ý���¼���Ϣ�ϴ�
					rt_kprintf( "\r\n ��ý���¼���Ϣ�ϴ���Ӧ! \r\n" );
					Media_Clear_State( );   //  clear

					if( 0 == MediaObj.Media_Type )
					{
						MediaObj.Media_transmittingFlag = 1;
						PositionSD_Enable( );
						Current_UDP_sd = 1;

						Photo_sdState.photo_sending = enable;
						Photo_sdState.SD_packetNum	= 1;    // ��һ����ʼ
						PositionSD_Enable( );               //   ʹ���ϱ�
						rt_kprintf( "\r\n ��ʼ�ϴ���Ƭ! ....\r\n" );
					}else
					if( 1 == MediaObj.Media_Type )
					{
						MediaObj.Media_transmittingFlag = 1;

						Sound_sdState.photo_sending = enable;
						Sound_sdState.SD_packetNum	= 1;    // ��һ����ʼ
						PositionSD_Enable( );               //   ʹ���ϱ�
						Current_UDP_sd = 1;
						rt_kprintf( "\r\n ��ʼ�ϴ���Ƶ! ....\r\n" );
					}else
					if( 2 == MediaObj.Media_Type )
					{
						MediaObj.Media_transmittingFlag = 1;
						PositionSD_Enable( );               //   ʹ���ϱ�
						Current_UDP_sd				= 1;
						Video_sdState.photo_sending = enable;
						Video_sdState.SD_packetNum	= 1;    // ��һ����ʼ
						rt_kprintf( "\r\n ��ʼ�ϴ���Ƶ! ....\r\n" );
					}
					break;
				case 0x0702:
					rt_kprintf( "\r\n  ��ʻԱ��Ϣ�ϱ�---����Ӧ��!  \r\n" );

					break;
				case 0x0701:
					rt_kprintf( "\r\n  �����˵��ϱ�---����Ӧ��!  \r\n" );

					break;
				case 0x0705:    //
					rt_kprintf( "\r\n can-ack" );
					break;
				case 0x0704:    // ����Ӧ��
					rt_kprintf( "\r\n MQ packet= %d   0704H-ack", MQ_TrueUse.PacketNum );
					                   //  ���ش洢
                                          if(MangQU.Sd_flag==2)
										  { 	  
										       MangQU.Sd_flag=0;
											   MangQU.Sd_timer=0;
											   MangQU.Sd_timer=0;
											   
											   MangQU.PacketNum++;
											   if(MangQU.PacketNum>=(BLIND_NUM/MQ_PKNUM)) 
									  	    	{
									  	    	   MangQU.PacketNum=0;
												   MangQU.Enable_SD_state=0; 
												   rt_kprintf("\r\nReturn Normal\r\n");  
									  	    	}	
                                          }
										//  ���� �洢
										    if((MQ_TrueUse.Sd_flag==2)&&(MQ_TrueUse.Enable_SD_state==1)) 
										  { 	  
										       MQ_TrueUse.Sd_flag=0; 
											   MQ_TrueUse.Sd_timer=0;
											   MQ_TrueUse.Sd_timer=0;
											   
											   MQ_TrueUse.PacketNum++;
											   if(MQ_TrueUse.PacketNum>CurrentTotal)  
									  	    	{
									  	    	   MQ_TrueUse.PacketNum=0;
												   MQ_TrueUse.Enable_SD_state=0; 
												   // ÿ��Ӧ��洢��ؼ�¼��Ŀ
												   DF_Write_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd);  
												   rt_kprintf("\r\nTrue Return Normal\r\n");  
									  	    	}	
                                          }
							           break; 		   

				default:
					break; 
			}

			//-------------------------------
			break;
		case  0x8100: //  ������Ķ��ն�ע����Ϣ��Ӧ��
			//-----------------------------------------------------------
			switch( UDP_HEX_Rx[15] )
			{
				case 0: rt_kprintf( "\r\n   ----ע��ɹ�\r\n" );
					memset( JT808Conf_struct.ConfirmCode, 0, sizeof( JT808Conf_struct.ConfirmCode ) );
					memcpy( JT808Conf_struct.ConfirmCode, UDP_HEX_Rx + 16, infolen - 3 );

					memset( Reg_buf, 0, sizeof( Reg_buf ) );
					memcpy( Reg_buf, JT808Conf_struct.ConfirmCode, 20 );
					JT808Conf_struct.Regsiter_Status	= 1;
					Reg_buf[20]							= JT808Conf_struct.Regsiter_Status;
					Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
					rt_kprintf( "��Ȩ��: %s\r\n		   ��Ȩ�볤��: %d\r\n", JT808Conf_struct.ConfirmCode, strlen( (const char*)JT808Conf_struct.ConfirmCode ) );
					//-------- ��ʼ��Ȩ ------
					DEV_Login.Operate_enable = 1;
					break;
				case 1: rt_kprintf( "\r\n   ----�����ѱ�ע��\r\n" );
					break;
				case 2: rt_kprintf( "\r\n   ----���ݿ����޸ó���\r\n" );
					break;
				case 3: rt_kprintf( "\r\n   ----�ն��ѱ�ע��\r\n" );
					if( 0 == JT808Conf_struct.Regsiter_Status )
					{
						;                               //JT808Conf_struct.Regsiter_Status=2;  // not  1
						                                //DEV_regist.DeRegst_sd=1;
					}else
					if( 1 == JT808Conf_struct.Regsiter_Status )
					{
						DEV_Login.Operate_enable = 1;   //��ʼ��Ȩ
					}
					break;
				case 4: rt_kprintf( "\r\n   ----���ݿ����޸��ն�\r\n" );
					break;
			}
			break;
		case  0x8103:                                   //  �����ն˲���
			//  Ack_Resualt=0;
			if( contentlen )
			{                                           // �������������ÿ��ֻ�·�����һ������
				Total_ParaNum = UDP_HEX_Rx[13];         // �������ò�������
				rt_kprintf( "\r\n Set ParametersNum =%d  \r\n", Total_ParaNum );
				//-------------------------------------------------------------------
				ContentRdAdd	= 14;
				Process_Resualt = 0;                    // clear resualt
				for( i = 0; i < Total_ParaNum; i++ )
				{
					//  ��������DWORD 4 ���ֽ�
					SubCMD_8103H = ( UDP_HEX_Rx[ContentRdAdd] << 24 ) + ( UDP_HEX_Rx[ContentRdAdd + 1] << 16 ) + ( UDP_HEX_Rx[ContentRdAdd + 2] << 8 ) + UDP_HEX_Rx[ContentRdAdd + 3];
					//  ����Ϣ����
					SubInfolen = UDP_HEX_Rx[ContentRdAdd + 4];
					//  ��������Ϣ ������óɹ�����ӦBit λ��Ϊ 1 ���򱣳� 0
					if( CentreSet_subService_8103H( SubCMD_8103H, SubInfolen, UDP_HEX_Rx + ContentRdAdd + 5 ) )
					{
						Process_Resualt |= ( 0x01 << i );
					}
					//  �ƶ�ƫ�Ƶ�ַ
					ContentRdAdd += 5 + UDP_HEX_Rx[ContentRdAdd + 4]; // ƫ���±�
				}

				//--------------�ж����е����ý��  ---------------


				/* for(i=0;i<Total_ParaNum;i++)
				   {
				     if(!((Process_Resualt>>0)&0x01))
				   {
				          Ack_Resualt=1; //  1  ��ʾʧ��
				          break;
				   }
				   if(i==(Total_ParaNum-1))  //  ���õ����һ��ȷ�ϳɹ�
				   Ack_Resualt=0;  //  �ɹ�/ȷ��
				   }*/
				Ack_Resualt = 0;
			}

			//-------------------------------------------------------------------
			if( SD_ACKflag.f_CentreCMDack_0001H == 2 )
			{
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}else
			if( SD_ACKflag.f_CentreCMDack_0001H == 0 )
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			rt_kprintf( "\r\n  Set Device !\r\n" );

			break;
		case  0x8104:                           //  ��ѯ�ն˲���
			SD_ACKflag.f_SettingPram_0104H = 1; // ����ʲô���ݻظ�ͳһ���
			rt_kprintf( "\r\n  ���Ĳ�ѯ�ն˲��� !\r\n" );
			break;
		case  0x8105:                           // �ն˿���
			// Ack_Resualt=0;
			rt_kprintf( "\r\ny  �ն˿��� -1!\r\n" );
			if( contentlen )
			{                                   // �������������ÿ��ֻ�·�����һ������
				Total_ParaNum = UDP_HEX_Rx[13]; //  �ն˿���������
				rt_kprintf( "\r\n Set ParametersNum =%d  \r\n", Total_ParaNum );
				//-------------------------------------------------------------------
				if( CentreSet_subService_8105H( Total_ParaNum, contentlen - 1, UDP_HEX_Rx + 14 ) )
				{
					Ack_Resualt = 0;            // ���سɹ�
				}
			}

			//-------------------------------------------------------------------
			Ack_Resualt = 0;
			if( SD_ACKflag.f_CentreCMDack_0001H == 0 )
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			rt_kprintf( "\r\ny  �ն˿��� -1!\r\n" );

			break;
		case  0x8201:   // λ����Ϣ��ѯ    λ����Ϣ��ѯ��Ϣ��Ϊ��
			SD_ACKflag.f_CurrentPosition_0201H = 1;
			rt_kprintf( "\r\n  λ����Ϣ��ѯ !\r\n" );
			break;
		case  0x8202:   // ��ʱλ�ø��ٿ���
			Ack_Resualt = 0;

			//  13 14  ʱ����
			JT808Conf_struct.RT_LOCK.Lock_Dur = ( UDP_HEX_Rx[13] << 8 ) + UDP_HEX_Rx[14];
			//  15 16  17 18 ��Ӧ�ն˵���Ϣ
			JT808Conf_struct.RT_LOCK.Lock_KeepDur = ( UDP_HEX_Rx[15] << 24 ) + ( UDP_HEX_Rx[16] << 16 ) + ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];

			JT808Conf_struct.RT_LOCK.Lock_state			= 1;                                        // Enable Flag
			JT808Conf_struct.RT_LOCK.Lock_KeepCnter		= 0;                                        //  ���ּ�����
			Current_SD_Duration							= JT808Conf_struct.RT_LOCK.Lock_KeepDur;    //���ķ��ͼ��
			JT808Conf_struct.SD_MODE.DUR_TOTALMODE		= 1;                                        // ���¶�ʱ���״̬λ
			JT808Conf_struct.SD_MODE.Dur_DefaultMode	= 1;
			//  ��������
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			rt_kprintf( "\r\n  ��ʱλ�ø��ٿ���!\r\n" );
			break;
		case  0x8300:                       //  �ı���Ϣ�·�
			Ack_Resualt			= 0;
			TextInfo.TEXT_FLAG	= UDP_HEX_Rx[13];
			if( TextInfo.TEXT_FLAG & 0x09 ) // ����Ƿ��TTS�ն�  ������Ҳ��TTS����
			{
				Dev_Voice.CMD_Type = '2';
				memset( Dev_Voice.Play_info, 0, sizeof( Dev_Voice.Play_info ) );
				memcpy( Dev_Voice.Play_info, UDP_HEX_Rx + 14, infolen - 1 );
				Dev_Voice.info_sdFlag = 1;

				//#ifdef LCD_5inch
				//DwinLCD.Type=LCD_SDTXT;
				// memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content));
				//DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);
				//#endif
				//  TTS
				TTS_Get_Data( UDP_HEX_Rx + 14, infolen - 1 );
			}
			if( ( TextInfo.TEXT_FLAG & 0x04 ) || ( TextInfo.TEXT_FLAG & 0x01 ) )    // ����Ƿ���ն���ʾ��
			{
				memset( TextInfo.TEXT_Content, 0, sizeof( TextInfo.TEXT_Content ) );
				memcpy( TextInfo.TEXT_Content, UDP_HEX_Rx + 14, infolen - 1 );
				TextInfo.TEXT_SD_FLAG = 1;                                          // �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||

				//========================================
				TextInforCounter++;
				rt_kprintf( "\r\nд���յ��ĵ� %d ����Ϣ,��Ϣ����=%d,��Ϣ:%s", TextInforCounter, infolen - 1, TextInfo.TEXT_Content );
				TEXTMSG_Write( TextInforCounter, 1, infolen - 1, TextInfo.TEXT_Content );
				//========================================
			}

#ifdef LCD_5inch
			//======  ��Ϣ������Ļ����ʾ
			DwinLCD.Type = LCD_SDTXT;
			memset( DwinLCD.TXT_content, 0, sizeof( DwinLCD.TXT_content ) );
			DwinLCD.TXT_contentLen = AsciiToGb( DwinLCD.TXT_content, infolen - 1, UDP_HEX_Rx + 14 );
#endif

			//------- ���� ----
			//  if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			rt_kprintf( "\r\n �ı���Ϣ: %s\r\n", TextInfo.TEXT_Content );
			break;
		case  0x8301:                                   //  �¼�����
			if( contentlen )
			{
				//--- ��������--
				switch( UDP_HEX_Rx[13] )
				{
					case 0:                             //  ɾ���ն����������¼���������󲻴�����ַ�
						Event_Init( 1 );
						rt_kprintf( "\r\n ɾ�������¼�\r\n" );
						break;
					case 1:                             // �����¼�
						if( UDP_HEX_Rx[13] == 1 )
						{
							rt_kprintf( "\r\n �����¼�\r\n" );
						}
					//break;
					case 2:                             // ׷���¼�
						if( UDP_HEX_Rx[13] == 2 )
						{
							rt_kprintf( "\r\n ׷���¼�\r\n" );
						}
					//break;
					case 3:                             // �޸��¼�
						if( UDP_HEX_Rx[13] == 3 )
						{
							rt_kprintf( "\r\n �޸��¼�\r\n" );
						}
					//break;
					case 4:                             // ɾ���ض��¼�
						if( UDP_HEX_Rx[13] == 4 )
						{
							rt_kprintf( "\r\n ɾ���ص��¼�\r\n" );
						}
						Total_ParaNum = UDP_HEX_Rx[14]; // �������ò�������
						rt_kprintf( "\r\n Set ParametersNum(�¼���Ϣ������) =%d  \r\n", Total_ParaNum );
						if( Total_ParaNum == 1 )
						{
							if( ( UDP_HEX_Rx[15] > 8 ) || ( UDP_HEX_Rx[15] == 0 ) )
							{
								EventObj.Event_ID = 0;
							} else
							{
								EventObj.Event_ID = UDP_HEX_Rx[15];
							}

							EventObj.Event_Len = UDP_HEX_Rx[16];
							memset( EventObj.Event_Str, 0, sizeof( EventObj.Event_Str ) );
							memcpy( EventObj.Event_Str, UDP_HEX_Rx + 17, EventObj.Event_Len );
							EventObj.Event_Effective = 1;
							Api_RecordNum_Write( event_808, EventObj.Event_ID, (u8*)&EventObj, sizeof( EventObj ) );
							rt_kprintf( "\r\n(�¼�����1)ID=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", EventObj.Event_ID, EventObj.Event_Len, EventObj.Event_Effective, EventObj.Event_Str );
						}else
						{
							EventObj.Event_ID	= UDP_HEX_Rx[15];
							EventObj.Event_Len	= UDP_HEX_Rx[16];
							memset( EventObj.Event_Str, 0, sizeof( EventObj.Event_Str ) );
							memcpy( EventObj.Event_Str, UDP_HEX_Rx + 17, EventObj.Event_Len );
							EventObj.Event_Effective = 1;
							Api_RecordNum_Write( event_808, EventObj.Event_ID, (u8*)&EventObj, sizeof( EventObj ) );
							rt_kprintf( "\r\n(�¼�����2)ID=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", EventObj.Event_ID, EventObj.Event_Len, EventObj.Event_Effective, EventObj.Event_Str );
							//
							EventObj.Event_ID	= UDP_HEX_Rx[17 + EventObj.Event_Len];
							EventObj.Event_Len	= UDP_HEX_Rx[18 + EventObj.Event_Len];
							memset( EventObj.Event_Str, 0, sizeof( EventObj.Event_Str ) );
							memcpy( EventObj.Event_Str, UDP_HEX_Rx + 19, EventObj.Event_Len );
							EventObj.Event_Effective = 1;
							Api_RecordNum_Write( event_808, EventObj.Event_ID, (u8*)&EventObj, sizeof( EventObj ) );
							rt_kprintf( "\r\n(�¼�����2)ID=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", EventObj.Event_ID, EventObj.Event_Len, EventObj.Event_Effective, EventObj.Event_Str );
						}
						break;
					default:
						break;
				}

				//---------���� -------
				// if(SD_ACKflag.f_CentreCMDack_0001H==0) // һ��ظ�
				{
					SD_ACKflag.f_CentreCMDack_0001H		= 1;
					Ack_Resualt							= 0;
					SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
				}


				/*
				   if(SD_ACKflag.f_CurrentEventACK_0301H==0)
				   {
				   SD_ACKflag.f_CurrentEventACK_0301H=1;
				   }
				 */
			}

			break;
		case  0x8302: // �����·�
			// if(UDP_HEX_Rx[13]&0x08)  // ����־�Ƿ����ʾ�ն�
			rt_kprintf( "\r\n  �����·����� \r\n" );
			{
				ASK_Centre.ASK_infolen = UDP_HEX_Rx[14];
				memset( ASK_Centre.ASK_info, 0, sizeof( ASK_Centre.ASK_info ) );
				memcpy( ASK_Centre.ASK_info, UDP_HEX_Rx + 15, ASK_Centre.ASK_infolen );
				rt_kprintf( "\r\n  ����: %s \r\n", ASK_Centre.ASK_info );
				memset( ASK_Centre.ASK_answer, 0, sizeof( ASK_Centre.ASK_answer ) );
				memcpy( ASK_Centre.ASK_answer, UDP_HEX_Rx + 15 + ASK_Centre.ASK_infolen, infolen - 2 - ASK_Centre.ASK_infolen );

				ASK_Centre.ASK_SdFlag		= 1;                // ||||||||||||||||||||||||||||||||||
				ASK_Centre.ASK_floatID		= Centre_FloatID;   // ���� FloatID
				ASK_Centre.ASK_disp_Enable	= 1;
				rt_kprintf( "\r\n ����Answer:%s\r\n", ASK_Centre.ASK_answer + 3 );

				Api_RecordNum_Write( ask_quesstion, 1, (u8*)&ASK_Centre, sizeof( ASK_Centre ) );
			}

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}

			break;
		case  0x8303:                               //  ��Ϣ�㲥�˵�����
			                                        //--- ��������--
			switch( UDP_HEX_Rx[13] )
			{
				case 0:                             //  ɾ���ն�����������Ϣ
					MSG_BroadCast_Init( 1 );
					rt_kprintf( "\r\n ɾ����Ϣ\r\n" );
					break;
				case 1:                             // ���²˵�
					if( UDP_HEX_Rx[13] == 1 )
					{
						rt_kprintf( "\r\n ���²˵�\r\n" );
					}
				//break;
				case 2:                             // ׷�Ӳ˵�
					if( UDP_HEX_Rx[13] == 2 )
					{
						rt_kprintf( "\r\n ׷�Ӳ˵�\r\n" );
					}
				//break;
				case 3:                             // �޸Ĳ˵�
					if( UDP_HEX_Rx[13] == 3 )
					{
						rt_kprintf( "\r\n �޸Ĳ˵�\r\n" );
					}
					Total_ParaNum = UDP_HEX_Rx[14]; // ��Ϣ������
					rt_kprintf( "\r\n Set ParametersNum =%d(��Ϣ������)\r\n", Total_ParaNum );
					if( Total_ParaNum == 1 )        //׷��1����Ϣ
					{
						if( ( UDP_HEX_Rx[15] > 8 ) || ( UDP_HEX_Rx[15] == 0 ) )
						{
							MSG_BroadCast_Obj.INFO_TYPE = 0;
						} else
						{
							MSG_BroadCast_Obj.INFO_TYPE = UDP_HEX_Rx[15];
							MSG_BroadCast_Obj.INFO_LEN	= ( UDP_HEX_Rx[16] << 8 ) + UDP_HEX_Rx[17];
							memset( MSG_BroadCast_Obj.INFO_STR, 0, sizeof( MSG_BroadCast_Obj.INFO_STR ) );
							memcpy( MSG_BroadCast_Obj.INFO_STR, UDP_HEX_Rx + 18, MSG_BroadCast_Obj.INFO_LEN );
							MSG_BroadCast_Obj.INFO_Effective	= 1;
							MSG_BroadCast_Obj.INFO_PlyCancel	= 1;
							Api_RecordNum_Write( msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj, sizeof( MSG_BroadCast_Obj ) );
							rt_kprintf( "\r\n(��Ϣ�㲥����1)TYPE=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", MSG_BroadCast_Obj.INFO_TYPE, MSG_BroadCast_Obj.INFO_LEN, MSG_BroadCast_Obj.INFO_Effective, MSG_BroadCast_Obj.INFO_STR );
						}
					}else
					{
						if( ( UDP_HEX_Rx[15] > 8 ) || ( UDP_HEX_Rx[15] == 0 ) )
						{
							MSG_BroadCast_Obj.INFO_TYPE = 0;
							rt_kprintf( "\r\n(��Ϣ�㲥����2) type=%d", UDP_HEX_Rx[15] );
						}else
						{
							MSG_BroadCast_Obj.INFO_TYPE = UDP_HEX_Rx[15];
							MSG_BroadCast_Obj.INFO_LEN	= ( UDP_HEX_Rx[16] << 8 ) + UDP_HEX_Rx[17];
							msg_len						= MSG_BroadCast_Obj.INFO_LEN;
							memset( MSG_BroadCast_Obj.INFO_STR, 0, sizeof( MSG_BroadCast_Obj.INFO_STR ) );
							memcpy( MSG_BroadCast_Obj.INFO_STR, UDP_HEX_Rx + 18, msg_len );
							MSG_BroadCast_Obj.INFO_Effective	= 1;
							MSG_BroadCast_Obj.INFO_PlyCancel	= 1;
							Api_RecordNum_Write( msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj, sizeof( MSG_BroadCast_Obj ) );
							rt_kprintf( "\r\n(��Ϣ�㲥����2)TYPE=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", MSG_BroadCast_Obj.INFO_TYPE, MSG_BroadCast_Obj.INFO_LEN, MSG_BroadCast_Obj.INFO_Effective, MSG_BroadCast_Obj.INFO_STR );
						}
						if( ( UDP_HEX_Rx[18 + msg_len] > 8 ) || ( UDP_HEX_Rx[18 + msg_len] == 0 ) )
						{
							MSG_BroadCast_Obj.INFO_TYPE = 0;
							rt_kprintf( "\r\n(��Ϣ�㲥����2) type=%d", UDP_HEX_Rx[18 + msg_len] );
						}else
						{
							MSG_BroadCast_Obj.INFO_TYPE = UDP_HEX_Rx[18 + msg_len];
							MSG_BroadCast_Obj.INFO_LEN	= ( UDP_HEX_Rx[19 + msg_len] << 8 ) + UDP_HEX_Rx[20 + msg_len];
							memset( MSG_BroadCast_Obj.INFO_STR, 0, sizeof( MSG_BroadCast_Obj.INFO_STR ) );
							memcpy( MSG_BroadCast_Obj.INFO_STR, UDP_HEX_Rx + 21 + msg_len, MSG_BroadCast_Obj.INFO_LEN );
							MSG_BroadCast_Obj.INFO_Effective	= 1;
							MSG_BroadCast_Obj.INFO_PlyCancel	= 1;
							Api_RecordNum_Write( msg_broadcast, MSG_BroadCast_Obj.INFO_TYPE, (u8*)&MSG_BroadCast_Obj, sizeof( MSG_BroadCast_Obj ) );
							rt_kprintf( "\r\n(��Ϣ�㲥����2)TYPE=%d,Len=%d,��Ч��=%d(1��Ч)����:%s\r\n", MSG_BroadCast_Obj.INFO_TYPE, MSG_BroadCast_Obj.INFO_LEN, MSG_BroadCast_Obj.INFO_Effective, MSG_BroadCast_Obj.INFO_STR );
						}
					}
					break;
				default:
					break;
			}

			//---------���� -------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0) // һ��ظ�
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}


			/*
			     if(SD_ACKflag.f_MsgBroadCast_0303H==0)
			   {
			   SD_ACKflag.f_MsgBroadCast_0303H=1;
			     }
			 */
			break;
		case  0x8304:                                       //  ��Ϣ����
			Ack_Resualt					= 0;
			MSG_BroadCast_Obj.INFO_TYPE = UDP_HEX_Rx[13];   //  ��Ϣ����
			MSG_BroadCast_Obj.INFO_LEN	= ( UDP_HEX_Rx[14] << 8 ) + UDP_HEX_Rx[15];
			memset( MSG_BroadCast_Obj.INFO_STR, 0, sizeof( MSG_BroadCast_Obj.INFO_STR ) );
			memcpy( MSG_BroadCast_Obj.INFO_STR, UDP_HEX_Rx + 16, infolen - 3 );

			MSG_BroadCast_Obj.INFO_SDFlag = 1;              // ||||||||||||||||||||||||||||||||||
			//------------------------------


			/*  Dev_Voice.CMD_Type='2';
			   memset(Dev_Voice.Play_info,0,sizeof(Dev_Voice.Play_info));
			                        memcpy(Dev_Voice.Play_info,UDP_HEX_Rx+16,infolen-3);
			   Dev_Voice.info_sdFlag=1;
			 */
			rt_kprintf( "\r\n ��Ϣ��������:%s\r\n", Dev_Voice.Play_info );

			// --------  ���͸��ı���Ϣ  --------------
			memset( TextInfo.TEXT_Content, 0, sizeof( TextInfo.TEXT_Content ) );
			memcpy( TextInfo.TEXT_Content, UDP_HEX_Rx + 16, infolen - 3 );
			TextInfo.TEXT_SD_FLAG = 1; // �÷��͸���ʾ����־λ	// ||||||||||||||||||||||||||||||||||

			//------- ���� ----
			//  if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8400:                   //  �绰�ز�

			if( infolen == 0 )
			{
				break;
			}
			if( 0 == UDP_HEX_Rx[13] )   // ��ͨͨ��
			{
				Speak_ON; rt_kprintf( "\r\n   �绰�ز�-->��ͨͨ��\r\n" );
			}else
			if( 1 == UDP_HEX_Rx[13] )   //  ����
			{
				Speak_OFF; rt_kprintf( "\r\n   �绰�ز�-->����" );
			}else
			{
				break;
			}
			memset( JT808Conf_struct.LISTEN_Num, 0, sizeof( JT808Conf_struct.LISTEN_Num ) );
			memcpy( JT808Conf_struct.LISTEN_Num, UDP_HEX_Rx + 14, infolen - 1 );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			CallState = CallState_rdytoDialLis; // ׼����ʼ�����������

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				SD_ACKflag.f_CentreCMDack_resualt	= 0;
			}
			break;
		case  0x8401:                                           //   ���õ绰��

			switch( UDP_HEX_Rx[13] )
			{
				case 0:                                         //  ɾ���ն�����������Ϣ
					// PhoneBook_Init(1);
					rt_kprintf( "\r\n ɾ���绰��\r\n" );
					break;
				case 1:                                         // ���²˵�
					if( UDP_HEX_Rx[13] == 1 )
					{
						rt_kprintf( "\r\n ���µ绰��\r\n" );
					}
				case 3:                                         // �޸Ĳ˵�
					if( UDP_HEX_Rx[13] == 3 )
					{
						rt_kprintf( "\r\n �޸ĵ绰��\r\n" );
					}
					Rx_PhoneBOOK.CALL_TYPE	= UDP_HEX_Rx[15];   // ��־ ���������
					Rx_PhoneBOOK.NumLen		= UDP_HEX_Rx[16];
					memset( Rx_PhoneBOOK.NumberStr, 0, sizeof( Rx_PhoneBOOK.NumberStr ) );
					memcpy( Rx_PhoneBOOK.NumberStr, UDP_HEX_Rx + 17, Rx_PhoneBOOK.NumLen );
					Rx_PhoneBOOK.UserLen = UDP_HEX_Rx[17 + Rx_PhoneBOOK.NumLen];
					memset( Rx_PhoneBOOK.UserStr, 0, sizeof( Rx_PhoneBOOK.UserStr ) );
					memcpy( Rx_PhoneBOOK.UserStr, UDP_HEX_Rx + 18 + Rx_PhoneBOOK.NumLen, Rx_PhoneBOOK.UserLen );

					for( i = 0; i < 8; i++ )
					{
						PhoneBook.CALL_TYPE = 2;                //���Ͷ���Ϊ���
						PhoneBook.NumLen	= 0;                // ���볤��
						memset( PhoneBook.NumberStr, 0, sizeof( PhoneBook.NumberStr ) );
						PhoneBook.UserLen = 0;
						memset( PhoneBook.UserStr, 0, sizeof( PhoneBook.UserStr ) );
						Api_RecordNum_Read( phonebook, i + 1, (u8*)&PhoneBook, sizeof( PhoneBook ) );
						if( strncmp( (char*)PhoneBook.UserStr, (const char*)Rx_PhoneBOOK.UserStr, Rx_PhoneBOOK.UserLen ) == 0 )
						{                                       // �ҵ���ͬ���ֵİ���ǰ��ɾ�����µĴ���
							Api_RecordNum_Write( phonebook, i + 1, (u8*)&Rx_PhoneBOOK, sizeof( Rx_PhoneBOOK ) );
							break;                              // ����for
						}
					}
					break;
				case 2:                                         // ׷�Ӳ˵�
					if( UDP_HEX_Rx[13] == 2 )
					{
						rt_kprintf( "\r\n ׷�ӵ绰��\r\n" );
					}
					Rx_PhoneBOOK.CALL_TYPE	= UDP_HEX_Rx[15];   // ��־ ���������
					Rx_PhoneBOOK.NumLen		= UDP_HEX_Rx[16];
					memset( Rx_PhoneBOOK.NumberStr, 0, sizeof( Rx_PhoneBOOK.NumberStr ) );
					memcpy( Rx_PhoneBOOK.NumberStr, UDP_HEX_Rx + 17, Rx_PhoneBOOK.NumLen );
					Rx_PhoneBOOK.UserLen = UDP_HEX_Rx[17 + Rx_PhoneBOOK.NumLen];
					memset( Rx_PhoneBOOK.UserStr, 0, sizeof( Rx_PhoneBOOK.UserStr ) );
					Rx_PhoneBOOK.Effective_Flag = 1;            // ��Ч��־λ
					memcpy( Rx_PhoneBOOK.UserStr, UDP_HEX_Rx + 18 + Rx_PhoneBOOK.NumLen, Rx_PhoneBOOK.UserLen );
					Api_RecordNum_Read( phonebook, UDP_HEX_Rx[14], (u8*)&Rx_PhoneBOOK, sizeof( Rx_PhoneBOOK ) );
					rt_kprintf( "\r\n Name:%s\r\n", Rx_PhoneBOOK.UserStr );
					rt_kprintf( "\r\n Number:%s\r\n", Rx_PhoneBOOK.NumberStr );
					break;
				default:
					break;
			}

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}

			break;
		case  0x8500:                                       //  ��������
			Vech_Control.Control_Flag = UDP_HEX_Rx[13];
			if( UDP_HEX_Rx[13] & 0x01 )
			{                                               // ���ż���       bit 12
				Car_Status[2] |= 0x10;                      // ��Ҫ���Ƽ̵���
				rt_kprintf( "\r\n  �������� \r\n" );
			}else
			{                                               // ���Ž���
				Car_Status[2] &= ~0x10;                     // ��Ҫ���Ƽ̵���
				rt_kprintf( "\r\n  �������� \r\n" );
			}
			Vech_Control.ACK_SD_Flag = 1;
			break;
		case  0x8600:                                       //  ����Բ������
			rt_kprintf( "\r\n  ����Բ������ \r\n" );
			if( UDP_HEX_Rx[14] == 1 )                       //  ����֧������һ������
			{
				switch( UDP_HEX_Rx[13] )
				{
					case 1:                                 // ׷������
						for( i = 0; i < 8; i++ )
						{
							memset( (u8*)&Rail_Cycle, 0, sizeof( Rail_Cycle ) );
							Api_RecordNum_Write( Rail_cycle, Rail_Cycle.Area_ID, (u8*)&Rail_Cycle, sizeof( Rail_Cycle ) );
                            Rails_Routline_Read(); 
							if( Rail_Cycle.Area_attribute ) // �ҳ���δʹ�õ�
							{
								break;
							}
						}
						if( 8 == i )                        //  ��������ˣ���ô�� 0
						{
							i = 0;
						}

					case 0:                                 // ��������
					case 2:                                 // �޸�����
						memset( (u8*)&Rail_Cycle, 0, sizeof( Rail_Cycle ) );
						Rail_Cycle.Area_ID			= ( UDP_HEX_Rx[15] << 24 ) + ( UDP_HEX_Rx[16] << 16 ) + ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];
						Rail_Cycle.Area_attribute	= ( UDP_HEX_Rx[19] << 8 ) + UDP_HEX_Rx[20];
						Rail_Cycle.Center_Latitude	= ( UDP_HEX_Rx[21] << 24 ) + ( UDP_HEX_Rx[22] << 16 ) + ( UDP_HEX_Rx[23] << 8 ) + UDP_HEX_Rx[24];
						Rail_Cycle.Center_Longitude = ( UDP_HEX_Rx[25] << 24 ) + ( UDP_HEX_Rx[26] << 16 ) + ( UDP_HEX_Rx[27] << 8 ) + UDP_HEX_Rx[28];
						Rail_Cycle.Radius			= ( UDP_HEX_Rx[29] << 24 ) + ( UDP_HEX_Rx[30] << 16 ) + ( UDP_HEX_Rx[31] << 8 ) + UDP_HEX_Rx[32];
						memcpy( Rail_Cycle.StartTimeBCD, UDP_HEX_Rx + 33, 6 );
						memcpy( Rail_Cycle.EndTimeBCD, UDP_HEX_Rx + 39, 6 );
						Rail_Cycle.MaxSpd			= ( UDP_HEX_Rx[45] << 8 ) + UDP_HEX_Rx[46];
						Rail_Cycle.KeepDur			= UDP_HEX_Rx[47];
						Rail_Cycle.Effective_flag	= 1;

						if( ( Rail_Cycle.Area_ID > 8 ) || ( Rail_Cycle.Area_ID == 0 ) )
						{
							Rail_Cycle.Area_ID = 1;
						}
						Api_RecordNum_Write( Rail_cycle, Rail_Cycle.Area_ID, (u8*)&Rail_Cycle, sizeof( Rail_Cycle ) );
                        Rails_Routline_Read(); 
						break;
					default:
						break;
				}
			}
			//------- ���� ----
			//   if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8601:                                                   //  ɾ��Բ������
			rt_kprintf( "\r\n  ɾ��Բ������ \r\n" );
			if( 0 == UDP_HEX_Rx[13] )                                   // ������
			{
				RailCycle_Init( );                                      // ɾ����������
			}else
			{
				memset( (u8*)&Rail_Cycle, 0, sizeof( Rail_Cycle ) );    //  clear all  first
				for( i = 0; i < UDP_HEX_Rx[13]; i++ )
				{
					Rail_Cycle.Area_ID = ( UDP_HEX_Rx[14 + i] << 24 ) + ( UDP_HEX_Rx[15 + i] << 16 ) + ( UDP_HEX_Rx[16 + i] << 8 ) + UDP_HEX_Rx[17 + i];
					if( ( Rail_Cycle.Area_ID > 8 ) || ( Rail_Cycle.Area_ID == 0 ) )
					{
						Rail_Cycle.Area_ID = 1;
					}
					Rail_Cycle.Effective_flag = 0;                                                                  // clear
					Api_RecordNum_Write( Rail_cycle, Rail_Cycle.Area_ID, (u8*)&Rail_Cycle, sizeof( Rail_Cycle ) );  // ɾ����Ӧ��Χ��
                    Rails_Routline_Read(); 
				}
			}

			//----------------
			//   if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8602:                   //  ���þ�������
			rt_kprintf( "\r\n  ���þ������� \r\n" );
			if( UDP_HEX_Rx[14] == 1 )   //  ����֧������һ������
			{
				switch( UDP_HEX_Rx[13] )
				{
					case 1:             // ׷������
						rt_kprintf( "\r\n  ׷��Χ�� " );
					case 0:             // ��������
					case 2:             // �޸�����
						memset( (u8*)&Rail_Rectangle, 0, sizeof( Rail_Rectangle ) );
						Rail_Rectangle.Area_ID				= ( UDP_HEX_Rx[15] << 24 ) + ( UDP_HEX_Rx[16] << 16 ) + ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];
						Rail_Rectangle.Area_attribute		= ( UDP_HEX_Rx[19] << 8 ) + UDP_HEX_Rx[20];
						Rail_Rectangle.LeftUp_Latitude		= ( UDP_HEX_Rx[21] << 24 ) + ( UDP_HEX_Rx[22] << 16 ) + ( UDP_HEX_Rx[23] << 8 ) + UDP_HEX_Rx[24];
						Rail_Rectangle.LeftUp_Longitude		= ( UDP_HEX_Rx[25] << 24 ) + ( UDP_HEX_Rx[26] << 16 ) + ( UDP_HEX_Rx[27] << 8 ) + UDP_HEX_Rx[28];
						Rail_Rectangle.RightDown_Latitude	= ( UDP_HEX_Rx[29] << 24 ) + ( UDP_HEX_Rx[30] << 16 ) + ( UDP_HEX_Rx[31] << 8 ) + UDP_HEX_Rx[32];
						Rail_Rectangle.RightDown_Longitude	= ( UDP_HEX_Rx[33] << 24 ) + ( UDP_HEX_Rx[34] << 16 ) + ( UDP_HEX_Rx[35] << 8 ) + UDP_HEX_Rx[36];
						memcpy( Rail_Rectangle.StartTimeBCD, UDP_HEX_Rx + 37, 6 );
						memcpy( Rail_Rectangle.EndTimeBCD, UDP_HEX_Rx + 43, 6 );
						Rail_Rectangle.MaxSpd			= ( UDP_HEX_Rx[49] << 8 ) + UDP_HEX_Rx[50];
						Rail_Rectangle.KeepDur			= UDP_HEX_Rx[51];
						Rail_Rectangle.Effective_flag	= 1;

						if( ( Rail_Rectangle.Area_ID > 8 ) || ( Rail_Rectangle.Area_ID == 0 ) )
						{
							Rail_Rectangle.Area_ID = 1;
						}
						Api_RecordNum_Write( Rail_rect, Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle, sizeof( Rail_Rectangle ) );
                        Rails_Routline_Read(); 
						rt_kprintf( "\r\n   ��������  ����Χ�� leftLati=%d leftlongi=%d rightLati=%d rightLong=%d \r\n", Rail_Rectangle.LeftUp_Latitude, Rail_Rectangle.LeftUp_Longitude, Rail_Rectangle.RightDown_Latitude, Rail_Rectangle.RightDown_Longitude );

						if( Rail_Rectangle.Area_attribute & 0x4000 )
						{
							rt_kprintf( "\r\n   Χ�� attribute=0x%4X  id=%d\r\n", Rail_Rectangle.Area_attribute, Rail_Rectangle.Area_ID );
						}

						break;
					default:
						break;
				}
			}
			//----------------
			//if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8603:                                                           //  ɾ����������
			rt_kprintf( "\r\n  ɾ���������� \r\n" );
			if( 0 == UDP_HEX_Rx[13] )                                           // ������
			{
				RailRect_Init( );                                               // ɾ����������
                Rails_Routline_Read(); 
			}else
			{
				memset( (u8*)&Rail_Rectangle, 0, sizeof( Rail_Rectangle ) );    //  clear all  first
				for( i = 0; i < UDP_HEX_Rx[13]; i++ )
				{
					Rail_Rectangle.Area_ID = ( UDP_HEX_Rx[14 + i] << 24 ) + ( UDP_HEX_Rx[15 + i] << 16 ) + ( UDP_HEX_Rx[16 + i] << 8 ) + UDP_HEX_Rx[17 + i];
					if( ( Rail_Rectangle.Area_ID > 8 ) || ( Rail_Rectangle.Area_ID == 0 ) )
					{
						Rail_Rectangle.Area_ID = 1;
					}
					Rail_Rectangle.Effective_flag = 0;
					Api_RecordNum_Write( Rail_rect, Rail_Rectangle.Area_ID, (u8*)&Rail_Rectangle, sizeof( Rail_Rectangle ) ); // ɾ����Ӧ��Χ��
                    Rails_Routline_Read(); 
				}
			}

			//----------------
			//  if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8604:                   //  ���������
			rt_kprintf( "\r\n  ���ö�������� \r\n" );
			if( UDP_HEX_Rx[14] == 1 )   //  ����֧������һ������
			{
				switch( UDP_HEX_Rx[13] )
				{
					case 1:             // ׷������
					case 0:             // ��������
					case 2:             // �޸�����
						memset( (u8*)&Rail_Polygen, 0, sizeof( Rail_Polygen ) );
						Rail_Polygen.Area_ID		= ( UDP_HEX_Rx[15] << 24 ) + ( UDP_HEX_Rx[16] << 16 ) + ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];
						Rail_Polygen.Area_attribute = ( UDP_HEX_Rx[19] << 8 ) + UDP_HEX_Rx[20];
						memcpy( Rail_Polygen.StartTimeBCD, UDP_HEX_Rx + 20, 6 );
						memcpy( Rail_Polygen.EndTimeBCD, UDP_HEX_Rx + 26, 6 );
						Rail_Polygen.MaxSpd				= ( UDP_HEX_Rx[32] << 8 ) + UDP_HEX_Rx[33];
						Rail_Polygen.KeepDur			= UDP_HEX_Rx[34];
						Rail_Polygen.Acme_Num			= UDP_HEX_Rx[35];
						Rail_Polygen.Acme1_Latitude		= ( UDP_HEX_Rx[36] << 24 ) + ( UDP_HEX_Rx[37] << 16 ) + ( UDP_HEX_Rx[38] << 8 ) + UDP_HEX_Rx[39];
						Rail_Polygen.Acme1_Longitude	= ( UDP_HEX_Rx[40] << 24 ) + ( UDP_HEX_Rx[41] << 16 ) + ( UDP_HEX_Rx[42] << 8 ) + UDP_HEX_Rx[43];
						Rail_Polygen.Acme2_Latitude		= ( UDP_HEX_Rx[44] << 24 ) + ( UDP_HEX_Rx[45] << 16 ) + ( UDP_HEX_Rx[46] << 8 ) + UDP_HEX_Rx[47];
						Rail_Polygen.Acme2_Longitude	= ( UDP_HEX_Rx[48] << 24 ) + ( UDP_HEX_Rx[49] << 16 ) + ( UDP_HEX_Rx[50] << 8 ) + UDP_HEX_Rx[51];
						Rail_Polygen.Acme3_Latitude		= ( UDP_HEX_Rx[52] << 24 ) + ( UDP_HEX_Rx[53] << 16 ) + ( UDP_HEX_Rx[54] << 8 ) + UDP_HEX_Rx[55];
						Rail_Polygen.Acme3_Longitude	= ( UDP_HEX_Rx[56] << 24 ) + ( UDP_HEX_Rx[57] << 16 ) + ( UDP_HEX_Rx[58] << 8 ) + UDP_HEX_Rx[59];

						if( ( Rail_Polygen.Area_ID > 8 ) || ( Rail_Polygen.Area_ID == 0 ) )
						{
							Rail_Polygen.Area_ID = 1;
						}
						Rail_Polygen.Effective_flag = 1;
						Api_RecordNum_Write( Rail_polygen, Rail_Polygen.Area_ID, (u8*)&Rail_Polygen, sizeof( Rail_Polygen ) );
						break;
					default:
						break;
				}
			}

			//----------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8605:                                                       //  ɾ���������
			rt_kprintf( "\r\n  ɾ����������� \r\n" );
			if( 0 == UDP_HEX_Rx[13] )                                       // ������
			{
				RailPolygen_Init( );                                        // ɾ����������
			}else
			{
				memset( (u8*)&Rail_Polygen, 0, sizeof( Rail_Polygen ) );    //  clear all  first
				for( i = 0; i < UDP_HEX_Rx[13]; i++ )
				{
					Rail_Polygen.Area_ID = ( UDP_HEX_Rx[14 + i] << 24 ) + ( UDP_HEX_Rx[15 + i] << 16 ) + ( UDP_HEX_Rx[16 + i] << 8 ) + UDP_HEX_Rx[17 + i];
					if( ( Rail_Polygen.Area_ID > 8 ) || ( Rail_Polygen.Area_ID == 0 ) )
					{
						Rail_Polygen.Area_ID = 1;
					}
					Rail_Polygen.Effective_flag = 0;
					Api_RecordNum_Write( Rail_polygen, Rail_Polygen.Area_ID, (u8*)&Rail_Polygen, sizeof( Rail_Polygen ) );
				}
			}

			//----------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8606:                                           //  ����·��
			rt_kprintf( "\r\n  ����·�� \r\n" );
			memset( (u8*)&ROUTE_Obj, 0, sizeof( ROUTE_Obj ) );  //  clear all  first
			ROUTE_Obj.Route_ID			= ( UDP_HEX_Rx[13] << 24 ) + ( UDP_HEX_Rx[14] << 16 ) + ( UDP_HEX_Rx[15] << 8 ) + UDP_HEX_Rx[16];
			ROUTE_Obj.Route_attribute	= ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];
			memcpy( ROUTE_Obj.StartTimeBCD, UDP_HEX_Rx + 19, 6 );
			memcpy( ROUTE_Obj.EndTimeBCD, UDP_HEX_Rx + 25, 6 );
			ROUTE_Obj.Points_Num = ( UDP_HEX_Rx[31] << 8 ) + UDP_HEX_Rx[32];
			rt_kprintf( "\r\n ROUTE_Obj.ID:  %d  \r\n ", ROUTE_Obj.Route_ID );
			rt_kprintf( "\r\n ROUTE_Obj.attribute:  %04X  \r\n ", ROUTE_Obj.Route_attribute );
			rt_kprintf( "\r\n ROUTE_Obj.Points_Num:  %d  \r\n ", ROUTE_Obj.Points_Num );
			//if(ROUTE_Obj.Points_Num<3)       // cheat  mask
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;

				line_warn_enable = 1;
				rt_kprintf( "\r\n  line_enable start \r\n" );

				break;
			}
			reg_u32 = 33;
			for( i = 0; i < 6; i++ ) // �յ���Ŀ
			{
				// if((infolen+32)<reg_u32)
				//	 break;

				ROUTE_Obj.RoutePoints[i].POINT_ID	= ( UDP_HEX_Rx[reg_u32] << 24 ) + ( UDP_HEX_Rx[reg_u32 + 1] << 16 ) + ( UDP_HEX_Rx[reg_u32 + 2] << 8 ) + UDP_HEX_Rx[reg_u32 + 3];
				reg_u32								+= 4;
				rt_kprintf( "\r\n PointID=%08x\r\n", ROUTE_Obj.RoutePoints[i].POINT_ID );
				ROUTE_Obj.RoutePoints[i].Line_ID	= ( UDP_HEX_Rx[reg_u32] << 24 ) + ( UDP_HEX_Rx[reg_u32 + 1] << 16 ) + ( UDP_HEX_Rx[reg_u32 + 2] << 8 ) + UDP_HEX_Rx[reg_u32 + 3];
				reg_u32								+= 4;
				rt_kprintf( "\r\n LineID=%08x\r\n", ROUTE_Obj.RoutePoints[i].Line_ID );
				ROUTE_Obj.RoutePoints[i].POINT_Latitude = ( UDP_HEX_Rx[reg_u32] << 24 ) + ( UDP_HEX_Rx[reg_u32 + 1] << 16 ) + ( UDP_HEX_Rx[reg_u32 + 2] << 8 ) + UDP_HEX_Rx[reg_u32 + 3];
				reg_u32									+= 4;
				rt_kprintf( "\r\n LatiID=%08x\r\n", ROUTE_Obj.RoutePoints[i].POINT_Latitude );
				ROUTE_Obj.RoutePoints[i].POINT_Longitude	= ( UDP_HEX_Rx[reg_u32] << 24 ) + ( UDP_HEX_Rx[reg_u32 + 1] << 16 ) + ( UDP_HEX_Rx[reg_u32 + 2] << 8 ) + UDP_HEX_Rx[reg_u32 + 3];
				reg_u32										+= 4;
				rt_kprintf( "\r\n LongID=%08x\r\n", ROUTE_Obj.RoutePoints[i].POINT_Longitude );
				ROUTE_Obj.RoutePoints[i].Width = UDP_HEX_Rx[reg_u32++];
				rt_kprintf( "\r\n Width=%02x\r\n", ROUTE_Obj.RoutePoints[i].Width );
				ROUTE_Obj.RoutePoints[i].Atribute = UDP_HEX_Rx[reg_u32++];
				rt_kprintf( "\r\n atrit=%02x\r\n\r\n", ROUTE_Obj.RoutePoints[i].Atribute );
				if( ROUTE_Obj.RoutePoints[i].Atribute == 0 )
				{
					;
				} else
				if( ROUTE_Obj.RoutePoints[i].Atribute == 1 )
				{
					ROUTE_Obj.RoutePoints[i].MaxSpd = ( UDP_HEX_Rx[reg_u32++] << 8 ) + UDP_HEX_Rx[reg_u32++];
				} else
				{
					ROUTE_Obj.RoutePoints[i].TooLongValue	= ( UDP_HEX_Rx[reg_u32++] << 8 ) + UDP_HEX_Rx[reg_u32++];
					ROUTE_Obj.RoutePoints[i].TooLessValue	= ( UDP_HEX_Rx[reg_u32++] << 8 ) + UDP_HEX_Rx[reg_u32++];
					ROUTE_Obj.RoutePoints[i].MaxSpd			= ( UDP_HEX_Rx[reg_u32++] << 8 ) + UDP_HEX_Rx[reg_u32++];
					ROUTE_Obj.RoutePoints[i].KeepDur		= ( UDP_HEX_Rx[reg_u32++] << 8 ) + UDP_HEX_Rx[reg_u32++];
				}
			}

			if( ( ROUTE_Obj.Route_ID > Route_Mum ) || ( ROUTE_Obj.Route_ID == 0 ) )
			{
				ROUTE_Obj.Route_ID = 1;
			}
			ROUTE_Obj.Effective_flag = 1;
			Api_RecordNum_Write( route_line, ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj, sizeof( ROUTE_Obj ) ); // ɾ����Ӧ��Χ��

			//----------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8607:                                               //  ɾ��·��
			rt_kprintf( "\r\n  ɾ��·�� \r\n" );
			if( 0 == UDP_HEX_Rx[13] )                               // ������
			{
				RouteLine_Init( );                                  // ɾ����������
			}else
			{
				memset( (u8*)&ROUTE_Obj, 0, sizeof( ROUTE_Obj ) );  //  clear all  first
				for( i = 0; i < UDP_HEX_Rx[13]; i++ )
				{
					ROUTE_Obj.Route_ID = ( UDP_HEX_Rx[14 + i] << 24 ) + ( UDP_HEX_Rx[15 + i] << 16 ) + ( UDP_HEX_Rx[16 + i] << 8 ) + UDP_HEX_Rx[17 + i];
					if( ( ROUTE_Obj.Route_ID > Route_Mum ) || ( ROUTE_Obj.Route_ID == 0 ) )
					{
						ROUTE_Obj.Route_ID = 1;
					}
					ROUTE_Obj.Effective_flag = 0;
					Api_RecordNum_Write( route_line, ROUTE_Obj.Route_ID, (u8*)&ROUTE_Obj, sizeof( ROUTE_Obj ) ); // ɾ����Ӧ��Χ��
				}
			}

			//----------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8700: //  �г���¼�����ݲɼ�����


			/*
			                           �г���¼�Ƿ��ͳ�����ʼ
			 */
			rt_kprintf( "\r\n  ��¼�ǲɼ����� \r\n" );
			Recode_Obj.Float_ID = Centre_FloatID;
			
			Recode_Obj.RSD_end=0;// clear
			//Recode_Obj.CMD=UDP_HEX_Rx[13];
			//    Stuff  Hardly
			switch( UDP_HEX_Rx[13] )
			{
				case 0x00: Recode_Obj.CMD	= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					break;
				//--------- Lagre  block -------


				/*
				      cmd         maxnum
				        15H	  2
				        12H	  10
				   11H	  10
				   10H	  100
				   09H	  360
				   08H	  576

				 */
				case  0x08:  Recode_Obj.Total_pkt_num	= 720;
					Recode_Obj.Current_pkt_num			= 1;
					Recode_Obj.Bak_current_num=1;
					Recode_Obj.Transmit_running=1;  // enable transmit
					Recode_Obj.Devide_Flag				= 1;
					//--------------------------------
					Recode_Obj.CMD			= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					MediaObj.Media_Type		= 3; //��ʻ��¼�� ֻ�������������ID ʱ������
					break;

				case  0x09:  Recode_Obj.Total_pkt_num	= 360;  //333----720    666--360
					Recode_Obj.Current_pkt_num			= 1; //0
					Recode_Obj.Devide_Flag				= 1;
					Recode_Obj.Bak_current_num=0;
					Recode_Obj.Transmit_running=1;  // enable transmit
					//--------------------------------
					Recode_Obj.CMD			= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					MediaObj.Media_Type		= 3; //��ʻ��¼��
					break;
				case  0x10:
					Recode_Obj.Total_pkt_num	= 100;
					Recode_Obj.Current_pkt_num	= 1;
					Recode_Obj.Devide_Flag		= 1;
					Recode_Obj.Bak_current_num=1;
					Recode_Obj.Transmit_running=1;  // enable transmit
					//--------------------------------
					Recode_Obj.CMD			= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					Recode_Obj.fcs				= 0;
					MediaObj.Media_Type		= 3; //��ʻ��¼��
					break;
				case  0x11:
					Recode_Obj.Total_pkt_num	= 100;
					Recode_Obj.Current_pkt_num	= 1;
					Recode_Obj.Devide_Flag		= 1;
					Recode_Obj.Bak_current_num=1;
					Recode_Obj.Transmit_running=1;  // enable transmit
					//------------------------------------
					Recode_Obj.CMD				= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag		= 1;
					Recode_Obj.CountStep		= 1;
					Recode_Obj.fcs				= 0;
					MediaObj.Media_Type			= 3; //��ʻ��¼��
					break;
				case  0x12:
					Recode_Obj.Total_pkt_num	= 10;
					Recode_Obj.Current_pkt_num	= 1;
					Recode_Obj.Devide_Flag		= 1;
					Recode_Obj.Bak_current_num=1;
					Recode_Obj.Transmit_running=1;	// enable transmit

					//------------------------------------
					Recode_Obj.CMD				= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag		= 1;
					Recode_Obj.CountStep		= 1;
					Recode_Obj.fcs				= 0;
					MediaObj.Media_Type			= 3; //��ʻ��¼��
					break;
				case  0x13:
					//--------------------------------
					Recode_Obj.CMD			= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					break;
				case  0x14:
					//--------------------------------
					Recode_Obj.CMD			= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag = 1;
					Recode_Obj.CountStep	= 1;
					break;
				case  0x15:
					Recode_Obj.Current_pkt_num	= 1;
					Recode_Obj.Devide_Flag		= 1;
					Recode_Obj.Bak_current_num=1;
					Recode_Obj.Transmit_running=1;  // enable transmit
					Recode_Obj.CMD				= UDP_HEX_Rx[13];
					Recode_Obj.SD_Data_Flag		= 1;
					Recode_Obj.CountStep		= 1;
					Recode_Obj.Total_pkt_num	= 10;
					Recode_Obj.fcs				= 0;
					MediaObj.Media_Type			= 3;                        //��ʻ��¼��
					break;
				default:
					break;
			}

			break;
		case  0x8701:                                                       //  ��ʻ��¼�ǲ����´�����
			rt_kprintf( "\r\n  ��¼�ǲ����´� \r\n" );
			Recode_Obj.Float_ID		= Centre_FloatID;
			Recode_Obj.CMD			= UDP_HEX_Rx[13];
			Recode_Obj.SD_Data_Flag = 1;
			CenterSet_subService_8701H( Recode_Obj.CMD, UDP_HEX_Rx + 14 );  //����2B���Ⱥ�1 ������
			break;
		case  0x8800:                                                       //   ��ý�������ϴ�Ӧ��
			if( infolen == 5 )
			{                                                               //  �ж��Ƿ����ش�ID�б����û�����ʾ���Ľ������!
				switch( MediaObj.Media_Type )
				{
					case 0:                                                 // ͼ��
						Photo_send_end( );                                  // �����ϴ�����
						rt_kprintf( "\r\n ͼ�������! \r\n" );
						//------------��·���մ���  -------
						// CheckResualt=Check_MultiTakeResult_b4Trans();

						break;
					case 1:                                                 // ��Ƶ
						Sound_send_end( );
						rt_kprintf( "\r\n ��Ƶ�������! \r\n" );
						break;
					case 2:                                                 // ��Ƶ
						Video_send_end( );
						rt_kprintf( "\r\n ��Ƶ�������! \r\n" );
						break;
					default:
						break;
				}
				// if(CheckResualt==0)
				Media_Clear_State( );
			}
			else
			{                                               //  �ش���ID �б�
				if( UDP_HEX_Rx[17] != 0 )
				{
					MediaObj.RSD_State	= 1;                //   �����ش�״̬
					MediaObj.RSD_Timer	= 0;                //   ����ش���ʱ��
					MediaObj.RSD_Reader = 0;
					MediaObj.RSD_total	= UDP_HEX_Rx[17];   // �ش�������
					//   ��ȡ�ش��б�
					j = 0;
					for( i = 0; i < MediaObj.RSD_total; i++ )
					{
						MediaObj.Media_ReSdList[i]	= ( UDP_HEX_Rx[18 + j] << 8 ) + UDP_HEX_Rx[19 + j];
						j							+= 2;
					}
					rt_kprintf( "\r\n  8003 �ش��б�Total=%d: ", MediaObj.RSD_total );
					for( i = 0; i < MediaObj.RSD_total; i++ )
					{
						rt_kprintf( "%d,", MediaObj.Media_ReSdList[i] );
					}
					rt_kprintf( "\r\n" );
				}
			}

			break;
		case  0x8801:                                                               //    ����ͷ��������

			Camera_Obj.Channel_ID		= UDP_HEX_Rx[13];                           //   ͨ��
			Camera_Obj.Operate_state	= UDP_HEX_Rx[18];                           //   �Ƿ񱣴��־λ
			//----------------------------------

			if( ( Camera_Take_Enable( ) ) && ( Photo_sdState.photo_sending == 0 ) ) //ͼƬ�����в�����
			{
				Camera_Number = UDP_HEX_Rx[13];
				if( ( Camera_Number > Max_CameraNum ) && ( Camera_Number < 1 ) )
				{
					break;
				}

				Start_Camera( Camera_Number );  //��ʼ����
				SingleCamera_TakeRetry = 0;     // clear
			}
			rt_kprintf( "\r\n   ���ļ�ʱ����  Camera: %d    \r\n", Camera_Number );

			if( UDP_HEX_Rx[18] == 0x01 )        // ���ղ��ϴ�
			{
				Camera_Take_not_trans = 1;
				rt_kprintf( "\r\n   ���ղ��ϴ�\r\n" );
			}else
			{
				Camera_Take_not_trans = 0;
			}

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)   //mask below  use   0805H
			//		 {
			//		 SD_ACKflag.f_CentreCMDack_0001H=1;
			//	 Ack_Resualt=0;
			// SD_ACKflag.f_CentreCMDack_resualt=Ack_Resualt;
			//	 }
			break;
		case  0x8802:   //    �洢��ý�����ݼ���
			SD_ACKflag.f_QueryEventCode = UDP_HEX_Rx[15];
			switch( UDP_HEX_Rx[13] )
			{
				case 0: // ͼ��
					SD_ACKflag.f_MediaIndexACK_0802H = 1;
					rt_kprintf( "\r\n  ���Ĳ�ѯͼ������ \r\n" );
					break;
				case 1: //  ��Ƶ
					SD_ACKflag.f_MediaIndexACK_0802H = 2;
					rt_kprintf( "\r\n  ���Ĳ�ѯ��Ƶ���� \r\n" );
				case 2: //  ��Ƶ
					SD_ACKflag.f_MediaIndexACK_0802H = 3;
				default:
					break;
			}

			break;
		case  0x8803:   //    �洢��ý�������ϴ�����
			rt_kprintf( "\r\n ��ý�������ϴ�\r\n" );
			switch( UDP_HEX_Rx[13] )
			{
				case 0: // ͼ��
					rt_kprintf( "\r\n   �ϴ�����ͼƬ\r\n" );
					break;
				case 1: //  ��Ƶ
					MP3_send_start( );
					rt_kprintf( "\r\n  �ϴ�������Ƶ \r\n" );
					break;
				case 2: //  ��Ƶ
					    // Video_send_start();
					    // MP3_send_start();
					rt_kprintf( "\r\n  �ϴ�������Ƶ  �������� ����Ƶ\r\n" );
					break;
				default:
					break;
			}

			//----------------------------------------------------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8804:       //    ¼����ʼ����

			//#if  1
			switch( UDP_HEX_Rx[13] )
			{
				case 0x00:  // ͣ¼��
					// VOC_REC_STOP(void);
					//   VOC_REC_Stop();


					/*
					                               Dev_Voice.CMD_Type='0';
					    Dev_Voice.info_sdFlag=0;
					    Dev_Voice.Voice_FileOperateFlag=0;
					    Dev_Voice.Centre_RecordFlag=0; // �������¼����־λ
					    if(TF_Card_Status()==1)
					     {
					       //memset(Dev_Voice.Voice_Reg,0,512);//clear left
					       //edit_file(Dev_Voice.FileName,Dev_Voice.Voice_Reg,512); //д��Ϣ��TF
					       //Dev_Voice.Voice_FileSize+=500; //add
					       rt_kprintf("\r\n ---------   �����ļ�������ַ	  VoiceFileSize %d Bytes  \r\n",Dev_Voice.Voice_FileSize);
					     }
					   Api_DFdirectory_Write(voice, Dev_Voice.Voice_Reg,500);
					                            Dev_Voice.Voice_FileSize+=500;
					                            Sound_SaveEnd();  */
					break;
				case 0x01:  // ��ʼ¼��

					VOC_REC_Start(1); 


					/*
					          if(MMedia2_Flag==0)
					           {
					                                 MP3_send_start();
					      rt_kprintf("\r\n  �ϴ�������Ƶ \r\n");
					      MMedia2_Flag=1;
					      break;
					           }
					                              Dev_Voice.Rec_Dur=(UDP_HEX_Rx[14]<<8)+UDP_HEX_Rx[15];
					   Dev_Voice.SaveOrNotFlag=UDP_HEX_Rx[16];

					                              // ------------   ¼���ļ��� -----------
					                              if(TF_Card_Status()==1)
					                           {
					                               memset(Dev_Voice.FileName,0,sizeof(Dev_Voice.FileName));
					    sprintf((char*)Dev_Voice.FileName,"%d%d%d%d.spx",time_now.day,time_now.hour,time_now.min,time_now.sec);
					                              // creat_file(Dev_Voice.FileName); //�����ļ�����
					                               Sound_SaveStart();
					    rt_kprintf("\r\n			  ����¼���ļ�����: %s \r\n",Dev_Voice.FileName);
					    Save_MediaIndex(1,Dev_Voice.FileName,0,0);
					                               }
					                              Dev_Voice.Centre_RecordFlag=1;//���Ŀ�ʼ¼����־λ

					   Dev_Voice.CMD_Type='1';
					   Dev_Voice.info_sdFlag=2;
					   Dev_Voice.Voice_FileSize=0; //clear size
					   Dev_Voice.Voice_FileOperateFlag=1;
					   MMedia2_Flag=0;
					 */
					break;
			}

			//------------------------------
			//if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8900:                   //    ��������͸��
			switch( UDP_HEX_Rx[13] )    // ͸����Ϣ����  // BD
			{
				case  0x41:             // ����1  ͸����Ϣ

					break;
				case  0x42:             //  ����2 ͸����Ϣ

					break;
				case  0x0B:             //  IC  �� ��Ϣ
					memcpy( IC_MOD.IC_Rx, UDP_HEX_Rx + 14, infolen - 1 );
					rt_kprintf( "\r\n IC ��͸��len=%dBytes  RX:", infolen - 1 );
					for( i = 0; i < infolen - 1; i++ )
					{
						rt_kprintf( "%2X ", UDP_HEX_Rx[14 + i] );
					}
					rt_kprintf( "\r\n" );
					//------ ֱ�ӷ��͸�IC ��ģ��-----
					Reg_buf[0] = 0x00;
					memcpy( Reg_buf + 1, IC_MOD.IC_Rx, infolen - 1 );
					DeviceData_Encode_Send( 0x0B, 0x40, Reg_buf, infolen );
					return;
			}
			//---------------------------------------------------
			if( LinkNum == 0 )
			{
				DataTrans.TYPE = UDP_HEX_Rx[13];
				memset( DataTrans.DataRx, 0, sizeof( DataTrans.DataRx ) );
				memcpy( DataTrans.DataRx, UDP_HEX_Rx + 14, infolen - 1 );
				DataTrans.Data_RxLen = infolen - 1;

				//--------- �͸�С��Ļ----------
				memset( TextInfo.TEXT_Content, 0, sizeof( TextInfo.TEXT_Content ) );
				AsciiToGb( TextInfo.TEXT_Content, infolen - 1, UDP_HEX_Rx + 14 );
				TextInfo.TEXT_SD_FLAG = 1; // �÷��͸���ʾ����־λ  // ||||||||||||||||||||||||||||||||||

				//========================================

#ifdef LCD_5inch

				DwinLCD.Type = LCD_SDTXT;
				memset( DwinLCD.TXT_content, 0, sizeof( DwinLCD.TXT_content ) );
				DwinLCD.TXT_contentLen = AsciiToGb( DwinLCD.TXT_content, infolen - 1, UDP_HEX_Rx + 14 );

#endif
				// if(SD_ACKflag.f_CentreCMDack_0001H==0)
				{
					SD_ACKflag.f_CentreCMDack_0001H		= 1;
					Ack_Resualt							= 0;
					SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
				}
			}
			break;
		case  0x8A00: //    ƽ̨RSA��Կ

			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
		{
			SD_ACKflag.f_CentreCMDack_0001H		= 1;
			Ack_Resualt							= 0;
			SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
		}
		break;
		case  0x8106:                                                                   //BD--8.11 ��ѯָ���ն˲���
			Setting_Qry.Num_pram	= UDP_HEX_Rx[13];                                   // ����
			reg_u32					= 0;                                                // �������±������
			for( i = 0; i < ( infolen - 1 ) / 2; i++ )
			{
				Detach_PKG.List_Resend[reg_u32] = ( UDP_HEX_Rx[16 + 2 * reg_u32] << 8 ) + UDP_HEX_Rx[17 + 2 * reg_u32 + 1];
				reg_u32++;
			}

			SD_ACKflag.f_SettingPram_0104H = 2;                                         // ���Ĳ�ѯָ������
			break;
		case  0x8107:                                                                   // BD--8.14 ��ѯ�ն�����
			SD_ACKflag.f_BD_DeviceAttribute_8107 = 1;                                   // ��Ϣ��Ϊ��
			break;
		case   0x8108:                                                                  //BD--8.16 �·��ն�������   Զ������  (��Ҫ)

			//   1.  �ְ��ж�bit λ
			if( UDP_HEX_Rx[3] & 0x20 )
			{                                                                           //  �ְ��ж�
				BD_ISP.Total_PacketNum		= ( UDP_HEX_Rx[13] << 8 ) + UDP_HEX_Rx[14]; // �ܰ���
				BD_ISP.CurrentPacket_Num	= ( UDP_HEX_Rx[15] << 8 ) + UDP_HEX_Rx[16]; // ��ǰ����Ŵ�1 ��ʼ
			}else
			{
				//------------   ACK   Flag -----------------
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			//   2.  ���
			if( BD_ISP.CurrentPacket_Num == 1 )
			{
				BD_ISP.Update_Type = UDP_HEX_Rx[17]; //����������
				//----  Debug ------
				switch( BD_ISP.Update_Type )
				{
					case  0:
						rt_kprintf( "\r\n ��������: �ն�\r\n" );
						break;
					case  12:
						rt_kprintf( "\r\n ��������: IC ��������\r\n" );
						break;
					case  52:
						rt_kprintf( "\r\n ��������: ����ģ��\r\n" );
						break;
				}
				//----------------------
				memcpy( BD_ISP.ProductID, UDP_HEX_Rx + 18, 5 );                                                                                 // ������ID
				BD_ISP.Version_len = UDP_HEX_Rx[23];                                                                                            // �汾�ų���
				memcpy( BD_ISP.VersionStr, UDP_HEX_Rx + 20, BD_ISP.Version_len );                                                               // ����汾��
				i					= 24 + BD_ISP.Version_len;
				BD_ISP.Content_len	= ( UDP_HEX_Rx[i] << 24 ) + ( UDP_HEX_Rx[i + 1] << 16 ) + ( UDP_HEX_Rx[i + 2] << 8 ) + UDP_HEX_Rx[i + 3];   // ����������
				i					+= 4;
				rt_kprintf( "\r\n ����������:%d Bytes\r\n", BD_ISP.Content_len );

				infolen = infolen - 11 - BD_ISP.Version_len;                                                                                    // infolen

				BD_ISP.PacketRX_wr	= 0;                                                                                                        // clear
				BD_ISP.ISP_running	= 1;
			}
			//--------  ����������  -------------
			if( infolen )                                                                                                                       //  ���������Ժ������������ͬ�Ĵ洢����д
			{
				memcpy( BD_ISP.ContentData, UDP_HEX_Rx + i + 4, infolen );                                                                      // ����������
				BD_ISP.PacketRX_wr += infolen;
			}
			//------------   ACK   Flag -----------------
			SD_ACKflag.f_CentreCMDack_0001H		= 1;
			Ack_Resualt							= 0;
			SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			//------------------------------------------------------------------------
			break;
		case   0x8203:                                                                                  //BD--8.22  �˹�ȷ�ϱ�����Ϣ
			HumanConfirmWarn.Warn_FloatID		= ( UDP_HEX_Rx[13] << 8 ) + UDP_HEX_Rx[14];
			HumanConfirmWarn.ConfirmWarnType	= ( UDP_HEX_Rx[15] << 24 ) + ( UDP_HEX_Rx[16] << 16 ) + ( UDP_HEX_Rx[17] << 8 ) + UDP_HEX_Rx[18];
			if( HumanConfirmWarn.Warn_FloatID == 0x00 )
			{                                                                                           //  ���Ϊ0 ��ʾ������Ϣ
				Warn_Status[3]	&= ~0x01;                                                               // bit 0
				Warn_Status[3]	&= ~0x08;                                                               // bit3
				Warn_Status[1]	&= ~0x10;                                                               // bit20 �������򱨾�
				Warn_Status[1]	&= ~0x20;                                                               // bit 21 ������·����
				Warn_Status[1]	&= ~0x40;                                                               // bit 22  ·����ʻʱ�䲻�㱨��
				Warn_Status[0]	&= ~0x08;                                                               // bit 27  ȷ�ϳ����Ƿ���𱨾�
				Warn_Status[0]	&= ~0x10;                                                               // bit 28  ȷ�ϳ����Ƿ�����
				Warn_Status[0]	&= ~0x80;                                                               //bit 31  �Ƿ����� (�ն˲���������ʱ�����ſ�����Ч)
			}else
			{
				//--------------------------------
				if( ( Warn_Status[3] & 0x01 ) && ( 0x00000001 & HumanConfirmWarn.ConfirmWarnType ) )    //��������
				{
					StatusReg_WARN_Clear( );
					f_Exigent_warning	= 0;
					warn_flag			= 0;
					Send_warn_times		= 0;
					StatusReg_WARN_Clear( );
					rt_kprintf( "\r\n���������յ�Ӧ�𣬵����!\r\n" );
				}
				if( ( Warn_Status[3] & 0x08 ) && ( 0x00000008 & HumanConfirmWarn.ConfirmWarnType ) )    //Σ�ձ���-BD
				{
					Warn_Status[3] &= ~0x08;
				}

				//------------------------------------
				if( ( Warn_Status[1] & 0x10 ) && ( 0x00001000 & HumanConfirmWarn.ConfirmWarnType ) )    // �������򱨾�
				{
					InOut_Object.TYPE		= 0;                                                        //Բ������
					InOut_Object.ID			= 0;                                                        //  ID
					InOut_Object.InOutState = 0;                                                        //  ������
					Warn_Status[1]			&= ~0x10;
				}
				if( ( Warn_Status[1] & 0x20 ) && ( 0x00002000 & HumanConfirmWarn.ConfirmWarnType ) )    // ����·�߱���
				{
					InOut_Object.TYPE		= 0;                                                        //Բ������
					InOut_Object.ID			= 0;                                                        //  ID
					InOut_Object.InOutState = 0;                                                        //  ������
					Warn_Status[1]			&= ~0x20;
				}
				if( ( Warn_Status[1] & 0x40 ) && ( 0x00004000 & HumanConfirmWarn.ConfirmWarnType ) )    // ·����ʻʱ�䲻�㡢����
				{
					Warn_Status[1] &= ~0x40;
				}
				//-----------------------------------------
				if( ( Warn_Status[0] & 0x08 ) && ( 0x08000000 & HumanConfirmWarn.ConfirmWarnType ) )    //�����Ƿ����
				{
					Warn_Status[0] &= ~0x08;
				}
				if( ( Warn_Status[0] & 0x10 ) && ( 0x10000000 & HumanConfirmWarn.ConfirmWarnType ) )    //�����Ƿ�λ��
				{
					Warn_Status[0] &= ~0x10;
				}
				if( ( Warn_Status[0] & 0x80 ) && ( 0x80000000 & HumanConfirmWarn.ConfirmWarnType ) )    //�Ƿ����ű���(�ն�δ��������ʱ�����жϷǷ�����)
				{
					Warn_Status[0] &= ~0x80;
				}
				//------------------------------------
			}

			//---------  ����ȷ�Ϻ��ͣ��������Ͷ�λ��
			PositionSD_Enable( );
			Current_UDP_sd = 1;
			break;
		case   0x8702:                              // BD-8.47-  �ϱ���ʻԱ�����Ϣ����
			SD_ACKflag.f_DriverInfoSD_0702H = 1;    // ��Ϣ��Ϊ��
			break;
		case  0x8805:                               //BD--8.55    �����洢��ý�����ݼ����ϴ�����	---- ����Э��Ҫ��

			reg_u32 = ( UDP_HEX_Rx[13] << 24 ) + ( UDP_HEX_Rx[14] << 16 ) + ( UDP_HEX_Rx[15] << 8 ) + UDP_HEX_Rx[16];
			rt_kprintf( "\r\n�����洢��ý�����ݼ����ϴ� MeidiaID=%d ɾ����־: %d ", reg_u32, UDP_HEX_Rx[17] );

			Camera_Number = 1;
			Photo_send_start( Camera_Number );      //�ڲ��Ƕ�·���յ����������Ϳ����ϴ���
			//------------------------------
			// if(SD_ACKflag.f_CentreCMDack_0001H==0)
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			break;
		case  0x8003:                                //    �����ְ�����

			    //Devide_8003_packet_1stID=( UDP_HEX_Rx[13] << 8 ) + UDP_HEX_Rx[14]; //���ֽ���ǰ���ں� 
			                                                                      // ����ͨ�����ֽ��ж�

  		       devide_value=0;
			   switch(UDP_HEX_Rx[13]) //���������·�����ˮ���ж�
			   {
				   case 0x80:
				   	         Recode_Obj.CMD=0x08;
							 Recode_Obj.Total_pkt_num	= 720;
							Recode_Obj.Devide_Flag				= 1;
							MediaObj.Media_Type		= 3; //��ʻ��¼�� ֻ�������������ID ʱ������
							  break;
				   case 0x90: 
				   	        Recode_Obj.CMD=0x09;
							Recode_Obj.Total_pkt_num	= 720;
							Recode_Obj.Devide_Flag				= 1;					
							MediaObj.Media_Type		= 3; //��ʻ��¼��
							  break;
				   case 0xA0:
				   	        Recode_Obj.CMD=0x10;
							Recode_Obj.Total_pkt_num	= 100;
							Recode_Obj.Devide_Flag				= 1;					
							MediaObj.Media_Type		= 3; //��ʻ��¼��
							  break;
				   case 0xB0:
				   	         Recode_Obj.CMD=0x11;
							 Recode_Obj.Total_pkt_num	= 100;
					        Recode_Obj.Devide_Flag				= 1;					
							MediaObj.Media_Type		= 3; //��ʻ��¼��
							  break;
				   case 0xC0:
				   	        Recode_Obj.CMD=0x12;
							Recode_Obj.Total_pkt_num	= 10;						
					        Recode_Obj.Devide_Flag				= 1;					
							MediaObj.Media_Type		= 3; //��ʻ��¼��
							  break;
							  break;
				   case  0xD0:
				   	        Recode_Obj.CMD=0x15;
							Recode_Obj.Total_pkt_num	= 10;						
					        Recode_Obj.Devide_Flag				= 1;					
							MediaObj.Media_Type		= 3; //��ʻ��¼��
							  break;
							  break;  
				   default:
                              devide_value=1; // ��ý��ְ�����ʽ
					          break;
                 }

                 


				  if(0==devide_value)
				  	{  // ��¼�Ƿְ�����
				  		//   ��ȡ�ش��б�
						if( Recode_Obj.RSD_end==1)
						 {
						    rt_kprintf( "\r\n ֮ǰ�б���Ϊ0  �����ڲ��账���ˡ�\r\n" );
							Recorder_init(1);// clear all state
							Recode_Obj.RSD_end=1; 
							break;
						 }	
						
				  	if(Recode_Obj.CountStep)
				       	Recode_Obj.RSD_State	= 3;                //   �����ش�״̬
				    else
                        Recode_Obj.RSD_State	= 1;                //   �����ش�״̬
					
						Recode_Obj.RSD_Timer	= 0;                //   ����ش���ʱ��
						Recode_Obj.RSD_Reader = 0;
						Recode_Obj.RSD_total	= UDP_HEX_Rx[15];   // �ش�������

					
						j = 0;
						for( i = 0; i < Recode_Obj.RSD_total; i++ )
						{
							Recode_Obj.Media_ReSdList[i]	= ( UDP_HEX_Rx[16 + j] << 8 ) + UDP_HEX_Rx[17 + j];
							j							+= 2;
						}
						rt_kprintf( "\r\n ��¼��  8003 �ش��б�Total=%d: ", Recode_Obj.RSD_total );  
							for( i = 0; i < Recode_Obj.RSD_total; i++ )
							{
								rt_kprintf( "%d,", Recode_Obj.Media_ReSdList[i] ); 
							}
							rt_kprintf( "\r\n" );

                         if(Recode_Obj.RSD_total==0)
						 	  Recode_Obj.RSD_end=1;//enable
				  	}
                   else
		            {  //  ��ý��ְ�  
		       
					MediaObj.RSD_State	= 1;                //   �����ش�״̬
					MediaObj.RSD_Timer	= 0;                //   ����ش���ʱ��
					MediaObj.RSD_Reader = 0;
					MediaObj.RSD_total	= UDP_HEX_Rx[15];   // �ش�������

					//   ��ȡ�ش��б�
					j = 0;
					for( i = 0; i < MediaObj.RSD_total; i++ )
					{
						MediaObj.Media_ReSdList[i]	= ( UDP_HEX_Rx[16 + j] << 8 ) + UDP_HEX_Rx[17 + j];
						j							+= 2;
					}
					rt_kprintf( "\r\n  8003 �ش��б�Total=%d: ", MediaObj.RSD_total );   
					if( MediaObj.RSD_total<=100)
					{
						for( i = 0; i < MediaObj.RSD_total; i++ )
						{
							rt_kprintf( "%d,", MediaObj.Media_ReSdList[i] );
						}
						rt_kprintf( "\r\n" );
			         }
		           }
			//==================================================================
           /*      �б��ش���ҪӦ�� 
                     if(Recode_Obj.RSD_total)  // �ش��б���Ϊ0 ������ 
			{
				SD_ACKflag.f_CentreCMDack_0001H		= 1;
				Ack_Resualt							= 0;
				SD_ACKflag.f_CentreCMDack_resualt	= Ack_Resualt;
			}
			*/

			break;
		//--------  BD  add  Over  --------------------------------
		default:
			break;
	}

	//-----------------  memset  -------------------------------------
	//memset(UDP_HEX_Rx, 0, sizeof(UDP_HEX_Rx));
	//UDP_hexRx_len= 0;
	return;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Time2BCD( u8 *dest )
{
	dest[0] = ( ( time_now.year / 10 ) << 4 ) + ( time_now.year % 10 );
	dest[1] = ( ( time_now.month / 10 ) << 4 ) + ( time_now.month % 10 );
	dest[2] = ( ( time_now.day / 10 ) << 4 ) + ( time_now.day % 10 );
	dest[3] = ( ( time_now.hour / 10 ) << 4 ) + ( time_now.hour % 10 );
	dest[4] = ( ( time_now.min / 10 ) << 4 ) + ( time_now.min % 10 );
	dest[5] = ( ( time_now.sec / 10 ) << 4 ) + ( time_now.sec % 10 );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void AvrgSpd_MintProcess( u8 hour, u8 min, u8 sec )
{
	//------- ϵͳ����ʱ����   �� ��־λΪ0 �������ն�Ϊ 0��������ϵͳ�ո���������ʱ��ȡ��ǰʱ��ѵ�ǰʱ���BAK------
	if( ( 0 == Avrgspd_Mint.saveFlag ) && ( Avrgspd_Mint.datetime_Bak[0] == 0 ) && ( Avrgspd_Mint.datetime_Bak[1] == 0 ) && ( Avrgspd_Mint.datetime_Bak[2] == 0 ) )
	{
		Time2BCD( Avrgspd_Mint.datetime );
		memcpy( Avrgspd_Mint.datetime_Bak, Avrgspd_Mint.datetime, 6 );              //����ǰʱ���BAK
		avgspd_Mint_Wr = time_now.min;                                              //���Ӽ����±�	 5-----4
	}else
	{                                                                               // ������ϵͳ������ʼ
		if( min == 0 )                                                              //  ������дΪ��0 ���Ա�����ʱ��ȥ�洢��1Сʱ������
		{
			avgspd_Mint_Wr = 0;                                                     //(xj)������Ϊ0����0
			if( sec == 2 )                                                          //�洢��һСʱÿ���ӵ�ƽ���ٶ� ==2����ΪҪ��֤��59�����ٶ��Ѿ����洢����Ӧ�ļĴ�����
			{
				memcpy( Avrgspd_Mint.datetime_Bak, Avrgspd_Mint.datetime, 6 );      //����ǰʱ���BAK
				Avrgspd_Mint.datetime_Bak[4]	= 0;                                // ����Ϊ0
				Avrgspd_Mint.datetime_Bak[5]	= 0;                                // ��ҲΪ0
				Avrgspd_Mint.saveFlag			= 1;                                //ʹ�ܴ洢      ��NandFlag  ��־λ
				Time2BCD( Avrgspd_Mint.datetime );                                  //��ȡ�µ�ʱ��
			}
			if( sec == 8 )                                                          //�洢���  ���ʱ���±�
			{
				avgspd_Mint_Wr = 0;
			}
		}
	}
	if( sec == 0 )                                                                  // ��Ϊ0ʱ�洢��1���ӵ�����
	{
		//-----------------------------------------------------------------------------------------
		//avgspd_Mint_Wr=(Avrgspd_Sec.datetime_Bak[4]>>4)*10+(0x0f&Avrgspd_Sec.datetime_Bak[4]);
		if( AspdCounter )
		{
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr] = PerMinSpdTotal / AspdCounter;    //�ڼ����ӵ�ƽ���ٶ�
		}else
		{
			Avrgspd_Mint.avgrspd[avgspd_Mint_Wr] = 0;
		}
		//---------------------------------------------------------------------------------------------
		// rt_kprintf("\r\n	  %2X:%2X  ÿ���ӵ�ƽ���ٶ�  avrgSpd=%d \r\n",Avrgspd_Sec.datetime_Bak[3],Avrgspd_Sec.datetime_Bak[4], Avrgspd_Mint.avgrspd[avgspd_Mint_Wr]);
		//--------------------------------------------------
		PerMinSpdTotal	= 0;                    // clear
		AspdCounter		= 0;                    // clear
	}else
	{
		if( UDP_dataPacket_flag == 0x02 )
		{
			AspdCounter++;
			PerMinSpdTotal += GPS_speed / 10;   // ֻҪ��ȷ�� km/h   ����Ҫ�� 10
		}
	}
}

//==================================================================================================
// ���Ĳ��� :   �������г���¼�����Э�� �� ��¼A
//==================================================================================================

//  1.  �����������������


/*
   u8 In_Type  :   ��������
   u8* InStr   :   �����ַ���
   u8 TransType:   ���䷽ʽ   ����  ��  GPRS

 */
void Device_Data( u8 In_Type, u8 TransType )
{
	u8	UpReg[500];
	u16 Up_wr	= 0, SregLen = 0, Greglen = 0, Swr = 0; //,Gwr=0; // S:serial  G: GPRS
	u8	Sfcs	= 0, Gfcs = 0;
	u16 i		= 0;
//  u16  packet_len=0;
	u8	Reg[70];
	u32 regdis		= 0, reg2 = 0;
	u8	QueryRecNum = 0;

	memset( UpReg, 0, 500 );
	Up_wr = 0;

	if( TransType )                                                                     //-------  GPRS  ����� ����Э��ͷ
	{
		memcpy( UpReg + Up_wr, "*GB", 3 );                                              // GPRS ��ʼͷ
		Up_wr += 3;

		UpReg[Up_wr++]	= 0x00;                                                         // SIM ����   ���������ֽ���д0x00
		UpReg[Up_wr++]	= 0x00;
		memcpy( UpReg + Up_wr, SIM_code, 6 );
		Up_wr += 6;

		Greglen = Up_wr;                                                                // ����
		Up_wr	+= 2;

		UpReg[Up_wr++] = 0x00;                                                          //��Ϣ��� Ĭ�� 0x00

		UpReg[Up_wr++] = 0x20;                                                          //����  bit5 bit4 10 ѡ��Ӧ������Э��

		UpReg[Up_wr++] = 0xF0;                                                          // ������  ���ߴ���RS232����

		//   ��������������
	}
	//---------------��д A Э��ͷ ------------------------
	Swr				= Up_wr;                                                            // reg save
	UpReg[Up_wr++]	= 0xAA;                                                             // ��ʼͷ
	UpReg[Up_wr++]	= 0x75;
	//---------------�������ͷ�����д����------------------
	switch( In_Type )
	{
		//---------------- �ϴ�������  -------------------------------------
		case  A_Up_DrvInfo:                                                             //  ��ǰ��ʻ����Ϣ
			UpReg[Up_wr++] = In_Type;                                                   //������

			SregLen			= 0x00;                                                     // ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                                                     // Hi
			UpReg[Up_wr++]	= 39;                                                       // Lo

			UpReg[Up_wr++] = 0x00;                                                      // ������

			memcpy( UpReg + Up_wr, JT808Conf_struct.Driver_Info.DriverCard_ID, 18 );    //��Ϣ����
			Up_wr += 18;
			memcpy( UpReg + Up_wr, JT808Conf_struct.Driver_Info.DriveName, 21 );
			Up_wr += 21;

			break;
		case  A_Up_RTC:                                                                 //  �ɼ���¼�ǵ�ʵʱʱ��
			UpReg[Up_wr++] = In_Type;                                                   //������

			SregLen			= 0x00;                                                     // ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                                                     // Hi
			UpReg[Up_wr++]	= 6;                                                        // Lo

			UpReg[Up_wr++] = 0x00;                                                      // ������

			Time2BCD( UpReg + Up_wr );                                                  //��Ϣ����
			Up_wr += 6;
			break;
		case  A_Up_Dist:                                                                //  �ɼ����
			UpReg[Up_wr++] = In_Type;                                                   //������

			SregLen			= 0x00;                                                     // ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                                                     // Hi
			UpReg[Up_wr++]	= 16;                                                       // Lo

			UpReg[Up_wr++] = 0x00;                                                      // ������
			//   ��Ϣ����
			Time2BCD( UpReg + Up_wr );
			Up_wr += 6;
			memcpy( UpReg + Up_wr, (u8*)JT808Conf_struct.FirstSetupDate, 6 );
			Up_wr			+= 6;
			regdis			= JT808Conf_struct.Distance_m_u32 / 100;                    //��λ0.1km
			reg2			= regdis / 10000000;
			UpReg[Up_wr++]	= ( reg2 << 4 ) + ( ( regdis % 10000000 ) / 1000000 );
			UpReg[Up_wr++]	= ( ( regdis % 1000000 / 100000 ) << 4 ) + ( regdis % 100000 / 10000 );
			UpReg[Up_wr++]	= ( ( regdis % 10000 / 1000 ) << 4 ) + ( regdis % 1000 / 100 );
			UpReg[Up_wr++]	= ( ( regdis % 100 / 10 ) << 4 ) + ( regdis % 10 );

		case  A_Up_PLUS:                                                                //  �ɼ���¼���ٶ�����ϵ��
			UpReg[Up_wr++] = In_Type;                                                   //������

			SregLen			= 0x00;                                                     // ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                                                     // Hi
			UpReg[Up_wr++]	= 10;                                                       // Lo

			UpReg[Up_wr++] = 0x00;                                                      // ������

			//  ��Ϣ����
			Time2BCD( UpReg + Up_wr );
			Up_wr			+= 6;
			UpReg[Up_wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value << 24 );
			UpReg[Up_wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value << 16 );
			UpReg[Up_wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value << 8 );
			UpReg[Up_wr++]	= (u8)( JT808Conf_struct.Vech_Character_Value );

			break;
		case  A_Up_VechInfo:                                                        //  ������Ϣ
			UpReg[Up_wr++] = In_Type;                                               //������

			SregLen			= 0x00;                                                 // ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                                                 // Hi
			UpReg[Up_wr++]	= 41;                                                   // Lo

			UpReg[Up_wr++] = 0x00;                                                  // ������

			memcpy( UpReg + Up_wr, JT808Conf_struct.Vechicle_Info.Vech_VIN, 17 );   //��Ϣ����
			Up_wr += 17;
			memcpy( UpReg + Up_wr, JT808Conf_struct.Vechicle_Info.Vech_Num, 12 );
			Up_wr += 12;
			memcpy( UpReg + Up_wr, JT808Conf_struct.Vechicle_Info.Vech_Type, 12 );
			Up_wr += 12;

			break;
		case  A_Up_AvrgMin:                                         //  ÿ����ƽ���ٶȼ�¼      // Ĭ����д����7���Ӽ�¼
			UpReg[Up_wr++] = In_Type;                               //������

			SregLen			= 455;                                  // ��Ϣ����
			UpReg[Up_wr++]	= (u8)( SregLen >> 8 );                 // Hi
			UpReg[Up_wr++]	= (u8)SregLen;                          // Lo	65x7

			UpReg[Up_wr++] = 0x00;                                  // ������
			//-----------  ��Ϣ����  --------------
			//----------------------
			QueryRecNum = Api_DFdirectory_Query( spdpermin, 0 );    //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
			if( QueryRecNum > 7 )
			{
				QueryRecNum = 7;
			}

			SregLen				= QueryRecNum * 65;                 // ��д��Ϣ����
			UpReg[Up_wr - 3]	= (u8)( SregLen >> 8 );             // Hi
			UpReg[Up_wr - 2]	= (u8)SregLen;                      // Lo    65x7

			for( i = 0; i < QueryRecNum; i++ )                      // �����´���ȡ�洢��д
			{
				Api_DFdirectory_Read( spdpermin, Reg, 70, 0, i );   // ��new-->old  ��ȡ
				memcpy( UpReg + Up_wr, Reg + 5, 60 );               // ֻ��д�ٶ�
				Up_wr += 65;
			}
			//------------------------------

			break;
		case  A_Up_Tired:                                           //  ƣ�ͼ�ʻ��¼
			UpReg[Up_wr++] = In_Type;                               //������

			SregLen			= 180;                                  // ��Ϣ����
			UpReg[Up_wr++]	= (u8)( SregLen >> 8 );                 // Hi
			UpReg[Up_wr++]	= (u8)SregLen;                          // Lo	30x6

			UpReg[Up_wr++] = 0x00;                                  // ������

			//----------------------------------
			QueryRecNum = Api_DFdirectory_Query( tired_warn, 0 );   //��ѯ��ǰƣ�ͼ�ʻ��¼��Ŀ
			if( QueryRecNum > 6 )
			{
				QueryRecNum = 6;
			}

			SregLen				= 30 * QueryRecNum;                 // ��Ϣ����
			UpReg[Up_wr - 3]	= (u8)( SregLen >> 8 );             // Hi
			UpReg[Up_wr - 2]	= (u8)SregLen;                      // Lo	65x7

			for( i = 0; i < QueryRecNum; i++ )                      // �����´���ȡ�洢��д
			{
				Api_DFdirectory_Read( tired_warn, Reg, 31, 0, i );  // ��new-->old  ��ȡ
				memcpy( UpReg + Up_wr, Reg, 30 );
				Up_wr += 30;
			}
			//----------------------------------

			break;
		//------------------------ �´������������ ---------------------
		case  A_Dn_DrvInfo: //  ���ó�����Ϣ

		//-----------------------------------------------------------------------


		/*  memset(JT808Conf_struct.Vechicle_Info.Vech_VIN,0,18);
		   memset(JT808Conf_struct.Vechicle_Info.Vech_Num,0,13);
		   memset(JT808Conf_struct.Vechicle_Info.Vech_Type,0,13);

		   memcpy(JT808Conf_struct.Vechicle_Info.Vech_VIN,InStr,17);
		   memcpy(JT808Conf_struct.Vechicle_Info.Vech_Num,InStr+17,12);
		   memcpy(JT808Conf_struct.Vechicle_Info.Vech_Type,InStr+29,12);

		   Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
		 */
		// break;

		case  A_Dn_RTC:                                     //  ���ü�¼��ʱ��

			UpReg[Up_wr++] = In_Type;                       //������

			// ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                         // Hi
			UpReg[Up_wr++]	= 0;                            // Lo   20x8

			UpReg[Up_wr++] = 0x00;                          // ������

			break;
		case  A_Dn_Plus:                                    //  �����ٶ�����ϵ��

			//	 JT808Conf_struct.Vech_Character_Value=((u32)(*InStr)<<24)+((u32)(*InStr+1)<<16)+((u32)(*InStr+2)<<8)+(u32)(*InStr+3); // ����ϵ��	�ٶ�����ϵ��
			//	 Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));
			//-------------------------------------------------------------
			UpReg[Up_wr++] = In_Type;                       //������
			// ��Ϣ����
			UpReg[Up_wr++]	= 0x00;                         // Hi
			UpReg[Up_wr++]	= 0;                            // Lo   20x8
			UpReg[Up_wr++]	= 0x00;                         // ������
			break;
		default:
			rt_kprintf( "Error:   Device Type Error! \r\n" );
			return;
	}
	//---------------  ��д���� A Э��  Serial Data   У��λ  -------------------------------------
	Sfcs = 0;                                               //	����SУ�� ��OxAA ��ʼ
	for( i = Swr; i < Up_wr; i++ )
	{
		Sfcs ^= UpReg[i];
	}
	UpReg[Up_wr++] = Sfcs;                                  // ��дFCS
	//-----------------------------------------------------------------------------
	if( TransType )                                         //-------ͨ��  GPRS
	{
		//----stuff Ginfolen ----
		UpReg[Greglen]		= (u8)( ( Up_wr - 2 ) << 8 );   //���Ȳ���ͷ
		UpReg[Greglen + 1]	= (u8)( Up_wr - 2 );

		Gfcs = 0;                                           //  ����ӵ绰���뿪ʼ��У��ǰ���ݵ�����  G Э��У��
		for( i = 3; i < Up_wr; i++ )
		{
			Gfcs ^= UpReg[i];
		}

		UpReg[Up_wr++] = Gfcs;                              // ��дGУ��λ

		UpReg[Up_wr++]	= 0x0D;                             // G Э��β
		UpReg[Up_wr++]	= 0x0A;
		//=================================================================================================
		//---------ͨ�� GPRS ����  --------------
		GPRS_infoWr_Tx = 0;                                 //--------------  clear  ---------
		//-----------------------------------------------------------------------
		memcpy( GPRS_info + GPRS_infoWr_Tx, UpReg, Up_wr ); // �������ݵ����ͻ�����
		GPRS_infoWr_Tx += Up_wr;
		//-----------  Add for Debug ---------------------------------
#if  0
		memset( UDP_AsciiTx, 0, sizeof( UDP_AsciiTx ) );
		strcat( (char*)UDP_AsciiTx, "AT%IPSENDX=1,\"" );
		packet_len		= 14;                               //strlen((const char*)UDP_AsciiTx);
		UDP_AsciiTx_len = HextoAscii( GPRS_info, GPRS_infoWr_Tx, UDP_AsciiTx + packet_len );

		packet_len += UDP_AsciiTx_len;
		strcat( (char*)UDP_AsciiTx, "\"\r\n" );

		�Ժ���˵��
		if( DispContent == 2 )
		{
			Uart1_PutData( (uint8_t*)UDP_AsciiTx, packet_len + 3 );
		}
		GSM_PutData( (uint8_t*)UDP_AsciiTx, packet_len + 3 );
		rt_kprintf( "\r\n	SEND DriveRecord -UP-Data! \r\n");
#endif
	}else
	{   //-----  ͨ���������
		for( i = 0; i < Up_wr; i++ )
		{
			rt_kprintf( "%c", UpReg[i] );
		}
	}
}

//------------------------------------------------------------------
void  Process_GPRSIN_DeviceData( u8 *instr, u16 infolen )
{
	u8	fcs = 0;
	u16 i	= 0;

	//   caculate  and   check fcs
	for( i = 0; i < infolen - 1; i++ )
	{
		fcs ^= instr[i];
	}

	if( fcs != instr[infolen - 1] )
	{
		return;
	}
	//  classify  cmd
	switch( instr[2] )                                          // AAH 75H CMD
	{
		//  ����
		case 0x00:                                              // �ɼ��г���¼��ִ�б�׼�汾��
			Adata_ACKflag.A_Flag__Up_Ver_00H = 0xff;
			break;
		case 0x01:                                              // �ɼ���ǰ��ʻ����Ϣ
			Adata_ACKflag.A_Flag_Up_DrvInfo_01H = instr[2];
			break;
		case 0x02:                                              // �ɼ���¼�ǵ�ʵʱʱ��
			Adata_ACKflag.A_Flag_Up_RTC_02H = instr[2];
			break;
		case 0x03:                                              // �ɼ���ʻ���
			Adata_ACKflag.A_Flag_Up_Dist_03H = instr[2];
			break;
		case 0x04:                                              // �ɼ���¼���ٶ�����ϵ��
			Adata_ACKflag.A_Flag_Up_PLUS_04H = instr[2];
			break;
		case 0x06:                                              // �ɼ�������Ϣ
			Adata_ACKflag.A_Flag_Up_VechInfo_06H = instr[2];
			break;
		case 0x08:                                              // �ɼ���¼��״̬�ź�������Ϣ
			Adata_ACKflag.A_Flag_Up_SetInfo_08H = instr[2];
			break;
		case 0x16:                                              // �ɼ���¼��Ψһ���
			Adata_ACKflag.A_Flag_Up_DevID_16H = instr[2];
			break;
		case 0x09:                                              // �ɼ�ָ����ÿ����ƽ���ٶȼ�¼
			Adata_ACKflag.A_Flag_Up_AvrgSec_09H = instr[2];     // ����ʼ����ʱ��
			break;
		case 0x05:                                              // �ɼ�ָ����ÿ����ƽ���ٶȼ�¼
			Adata_ACKflag.A_Flag_Up_AvrgMin_05H = instr[2];     // ����ʼ����ʱ��
			break;
		case 0x13:                                              // �ɼ�ָ����λ����Ϣ��¼
			Adata_ACKflag.A_Flag_Up_Posit_13H = instr[2];
			break;
		case 0x07:                                              // �ɼ��¹��ɵ��¼
			Adata_ACKflag.A_Flag_Up_Doubt_07H = instr[2];       // ����ʼ����ʱ��
			break;
		case 0x11:                                              // �ɼ�ָ����ƣ�ͼ�ʻ��¼
			Adata_ACKflag.A_Flag_Up_Tired_11H = instr[2];       // ����ʼ����ʱ��
			break;
		case 0x10:                                              // �ɼ�ָ���ĵ�¼�˳���¼
			Adata_ACKflag.A_Flag_Up_LogIn_10H = instr[2];       // ����ʼ����ʱ��
			break;
		case 0x14:                                              // �ɼ�ָ���ļ�¼���ⲿ�����¼
			Adata_ACKflag.A_Flag_Up_Powercut_14H = instr[2];    // ����ʼ����ʱ��
			break;
		case 0x15:                                              // �ɼ�ָ���ļ�¼�ǲ����޸ļ�¼
			Adata_ACKflag.A_Flag_Up_SetMdfy_15H = instr[2];
			break;
		//  ����
		case 0x82:                                              // ���ó�����Ϣ
			memset( JT808Conf_struct.Vechicle_Info.Vech_VIN, 0, sizeof( JT808Conf_struct.Vechicle_Info.Vech_VIN ) );
			memset( JT808Conf_struct.Vechicle_Info.Vech_Num, 0, sizeof( JT808Conf_struct.Vechicle_Info.Vech_Num ) );
			memset( JT808Conf_struct.Vechicle_Info.Vech_Type, 0, sizeof( JT808Conf_struct.Vechicle_Info.Vech_Type ) );

			//-----------------------------------------------------------------------
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_VIN, instr, 17 );
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_Num, instr + 17, 12 );
			memcpy( JT808Conf_struct.Vechicle_Info.Vech_Type, instr + 29, 12 );

			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );

			Adata_ACKflag.A_Flag_Dn_DrvInfo_82H = instr[2];

			Settingchg_Status				= 0x82;                                                                                                 //���ó�����Ϣ
			NandsaveFlg.Setting_SaveFlag	= 1;                                                                                                    //�洢�����޸ļ�¼
		case 0x83:                                                                                                                                  // ���ó��ΰ�װ����
			memcpy( JT808Conf_struct.FirstSetupDate, instr, 6 );
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			Adata_ACKflag.A_Flag_Dn_SetupDate_83H	= instr[2];
			Settingchg_Status						= 0x83;                                                                                         //���ó�����Ϣ
			NandsaveFlg.Setting_SaveFlag			= 1;                                                                                            //�洢�����޸ļ�¼
			break;
		case 0x84:                                                                                                                                  // ����״̬����Ϣ
			Settingchg_Status				= 0x84;                                                                                                 //���ó�����Ϣ
			NandsaveFlg.Setting_SaveFlag	= 1;                                                                                                    //�洢�����޸ļ�¼
			break;
		case 0xc2:                                                                                                                                  // ���ü�¼��ʱ��
			Adata_ACKflag.A_Flag_Dn_RTC_C2H = instr[2];
			Settingchg_Status				= 0xc2;                                                                                                 //���ó�����Ϣ
			NandsaveFlg.Setting_SaveFlag	= 1;                                                                                                    //�洢�����޸ļ�¼
			break;
		case 0xc3:                                                                                                                                  // ���ü�¼���ٶ�����ϵ��
			JT808Conf_struct.Vech_Character_Value = (u32)( instr[0] << 24 ) + (u32)( instr[1] << 16 ) + (u32)( instr[2] << 8 ) + (u32)( instr[3] ); // ����ϵ��	�ٶ�����ϵ��
			Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
			Adata_ACKflag.A_Flag_Dn_Plus_C3H	= instr[2];
			Settingchg_Status					= 0xc3;                                                                                             //���ó�����Ϣ
			NandsaveFlg.Setting_SaveFlag		= 1;                                                                                                //�洢�����޸ļ�¼
			break;
		default:

			break;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8 Send_Device_Data( void )  //  ���г���¼�Ƿ�������
{
	u8 j = 0, i = 0, reg = 0;;

	j = sizeof( Adata_ACKflag );
	for( i = 0; i < j; i++ )
	{
		reg = ( *( (u8*)&Adata_ACKflag + i ) );
		if( reg )
		{
			if( reg != 0xff )
			{
				Device_Data( reg, 1 );
			}else
			{
				Device_Data( 0x00, 1 );             // �ɼ��г���¼��ִ�б�׼�汾�� ����Ϊ0x00 ��λʱ��д0xFF,���Ե�������
			}
			( *( (u8*)&Adata_ACKflag + i ) ) = 0;   // clear flag
			return true;
		}
	}
	return false;
}

#if 0


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8 RecordSerial_output_Str( const char *fmt, ... )
{
	u8		regstr[100], fcs = 0;
	u16		reglen = 0, i = 0;

	va_list args;
	va_start( args, fmt );
	memset( regstr, 0, sizeof( regstr ) );
	regstr[0]	= 0x55;                 // Э��ͷ
	regstr[1]	= 0x7A;
	regstr[2]	= 0xFE;                 // ������ ����Ԥ�������ֱ�ʾ�������
	//  3,4 Ϊ�����ֽ������д
	regstr[5] = 0x00;                   // ������  0x00

	reglen = vsprintf( (char*)regstr + 6, fmt, args );
	va_end( args );
	regstr[3]	= (u8)( reglen >> 8 );  // ��д����  ������Ϊ��Ϣ���ݵĳ���
	regstr[4]	= (u8)reglen;

	reglen	+= 6;
	fcs		= 0;
	for( i = 0; i < reglen; i++ )
	{
		fcs ^= regstr[i];
	}
	regstr[reglen] = fcs;
	reglen++;
	for( i = 0; i < reglen; i++ )
	{
		rt_kprintf( "%c", regstr[i] );
	}

	return 1;
}

#endif


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void SpeedWarnJudge( void )                     //  �ٶȱ����ж�
{
	//--------  �ٶȱ���  -------
	if( JT808Conf_struct.Speed_warn_MAX > 0 )   //> 0
	{
		//----- GPS  ��ʱ�ٶ�	0.1 km/h  ---------
		if( GPS_speed > ( JT808Conf_struct.Speed_warn_MAX * 10 ) )
		// if( DebugSpd > ( JT808Conf_struct.Speed_warn_MAX*10) )
		{
			speed_Exd.dur_seconds++;
			if( speed_Exd.dur_seconds > JT808Conf_struct.Spd_Exd_LimitSeconds )
			{
				speed_Exd.dur_seconds = 0;
				if( speed_Exd.speed_flag != 1 )
				{
					speed_Exd.speed_flag = 1;
					//PositionsSD_Enable();	  //  �ر�GPS ��Ϣ

					StatusReg_SPD_WARN( );                  //  ���ٱ���״̬
					rt_kprintf( "\r\n  ���ٱ���\r\n" );
					TTS_play( " ���ٱ�������" );
				}
				//---------------------------------------------
				Time2BCD( speed_Exd.ex_startTime );         //��¼���ٱ�����ʼʱ��
				if( speed_Exd.current_maxSpd < GPS_speed )  //������ٶ�
				{
					speed_Exd.current_maxSpd = GPS_speed;
				}
				speed_Exd.excd_status = 1;
				speed_Exd.dur_seconds++;

				//----------------------------------------------
			}

			if( speed_Exd.excd_status == 1 )                // ʹ��flag ��ʼ��ʱ
			{
				speed_Exd.dur_seconds++;
				if( speed_Exd.current_maxSpd < GPS_speed )  //������ٶ�
				{
					speed_Exd.current_maxSpd = GPS_speed;
				}
			}
		}else
		{
			StatusReg_SPD_NORMAL( );                        //  ����ٶȱ���״̬�Ĵ���

			if( speed_Exd.excd_status != 2 )
			{
				StatusReg_SPD_NORMAL( );                    //  ����ٶȱ���״̬�Ĵ���
				speed_Exd.dur_seconds	= 0;
				speed_Exd.speed_flag	= 0;
			}
			//----------------------------------------------
			if( speed_Exd.excd_status == 1 )
			{
				Time2BCD( speed_Exd.ex_endTime );           //��¼���ٱ�������ʱ��
				speed_Exd.excd_status = 2;
			}else
			if( speed_Exd.excd_status == 0 )
			{
				Spd_ExpInit( );
			}
			//----------------------------------------------
		}
	} //------- �ٶȱ��� over	 ---
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u16  Protocol_808_Encode( u8 *Dest, u8 *Src, u16 srclen )
{
	u16 lencnt = 0, destcnt = 0;

	for( lencnt = 0; lencnt < srclen; lencnt++ )
	{
		if( Src[lencnt] == 0x7e )           // 7e ת��
		{
			Dest[destcnt++] = 0x7d;
			Dest[destcnt++] = 0x02;
		}else
		if( Src[lencnt] == 0x7d )           //  7d  ת��
		{
			Dest[destcnt++] = 0x7d;
			Dest[destcnt++] = 0x01;
		}else
		{
			Dest[destcnt++] = Src[lencnt];  // ԭʼ��Ϣ
		}
	}

	return destcnt;                         //����ת���ĳ���
}

//-------------------------------------------------------------------------------
void Protocol_808_Decode( void )            // ����ָ��buffer :  UDP_HEX_Rx
{
	//-----------------------------------
	u16 i = 0;

	// 1.  clear  write_counter
	UDP_DecodeHex_Len = 0;                  //clear DecodeLen

	// 2   decode process
	for( i = 0; i < UDP_hexRx_len; i++ )
	{
		if( ( UDP_HEX_Rx[i] == 0x7d ) && ( UDP_HEX_Rx[i + 1] == 0x02 ) )
		{
			UDP_HEX_Rx[UDP_DecodeHex_Len] = 0x7e;
			i++;
		}else
		if( ( UDP_HEX_Rx[i] == 0x7d ) && ( UDP_HEX_Rx[i + 1] == 0x01 ) )
		{
			UDP_HEX_Rx[UDP_DecodeHex_Len] = 0x7d;
			i++;
		}else
		{
			UDP_HEX_Rx[UDP_DecodeHex_Len] = UDP_HEX_Rx[i];
		}
		UDP_DecodeHex_Len++;
	}
	//  3.  The  End
}

//---------  �յ㲹�����Գ���  ---------------------------
//#if 0


/*
   void Inflexion_Process(void)
   {            //
   u16  once_delta=0;


   Inflexion_Current=GPS_direction;  //  update new
   //-----------------------------------------------------------------------
    if(Inflexion_Current>Inflexion_Bak)   // �����жϴ�С
     {  // ����
            if((Inflexion_Current-Inflexion_Bak)>300)  // �ж��Ƿ��ü�С
   {   //  �����ֵ����300 ��˵����С��
        once_delta=Inflexion_Bak+360-Inflexion_Current;  //�жϲ�ֵ����ֵ
        InflexDelta_Accumulate+=once_delta;
     if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��  �յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
     {
         if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2))  //�ж�֮ǰ�Ƿ�һֱ��С��
          {
                          Inflexion_chgcnter++;
        if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
         {     //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
                                           InflexLarge_or_Small=0;
             Inflexion_chgcnter=0;
             InflexDelta_Accumulate=0;
             PositionSD_Enable=1; // ���͹յ��־λ
             rt_kprintf("\r\n �յ��ϱ� --1\r\n");
         }
        else
          InflexLarge_or_Small=2; // �����С��
          }
      else
       {
          InflexLarge_or_Small=2;  // ���ǵ�һ��С��
          Inflexion_chgcnter=1;
          InflexDelta_Accumulate=once_delta;
       }
     }
     else
     {    //  С�� 15 �������
       InflexLarge_or_Small=0;
       Inflexion_chgcnter=0;
       InflexDelta_Accumulate=0;
     }

            }
   else		// current���������ı�Bak ��
   {
      once_delta=Inflexion_Current-Inflexion_Bak;  //�жϲ�ֵ����ֵ
      InflexDelta_Accumulate+=once_delta;
      if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��  �յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
      {
                if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1)) //�ж�֮ǰ�Ƿ�һֱ����
       {
           Inflexion_chgcnter++;
        if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
         {	 //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
          InflexLarge_or_Small=0;
          Inflexion_chgcnter=0;
          InflexDelta_Accumulate=0;
          PositionSD_Enable(); // ���͹յ��־λ
          rt_kprintf("\r\n �յ��ϱ� --2\r\n");
         }
        else
          InflexLarge_or_Small=1; // ����Ǵ���
                }
       else
        {
                       InflexLarge_or_Small=1;  // ���ǵ�һ�δ���
        Inflexion_chgcnter=1;
        InflexDelta_Accumulate=once_delta;
        }
      }
       else
     {     // С��15�Ⱦ������
       InflexLarge_or_Small=0;
       Inflexion_chgcnter=0;
       InflexDelta_Accumulate=0;
     }

   }
     }
   else
   if(Inflexion_Current<Inflexion_Bak)
   {  // ��С
               if((Inflexion_Bak-Inflexion_Current)>300)  // �ж��Ƿ�������
               { //  �����ֵ����300 ��˵���Ǵ���
                  once_delta=Inflexion_Current+360-Inflexion_Bak;  //�жϲ�ֵ����ֵ
         InflexDelta_Accumulate+=once_delta;
      if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��	�յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
                  {   // ��С�仯�� ��С�� 15
                     if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==1))  //�ж�֮ǰ�Ƿ�һֱ�Ǵ���
          {
                          Inflexion_chgcnter++;
        if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
         {     //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
                                           InflexLarge_or_Small=0;
             Inflexion_chgcnter=0;
             InflexDelta_Accumulate=0;
             PositionSD_Enable(); // ���͹յ��־λ
             rt_kprintf("\r\n �յ��ϱ� --3\r\n");
         }
        else
          InflexLarge_or_Small=1; // ����Ǵ���
          }
      else
       {
          InflexLarge_or_Small=1;  // ���ǵ�һ�δ���
          Inflexion_chgcnter=1;
          InflexDelta_Accumulate=once_delta;
       }

                  }
      else
      {    //  С�� 15 �������
       InflexLarge_or_Small=0;
       Inflexion_chgcnter=0;
       InflexDelta_Accumulate=0;
      }
     }//---------------------------
      else     // current ���������ı�Bak С
      {
      once_delta=Inflexion_Bak-Inflexion_Current;  //�жϲ�ֵ����ֵ
      InflexDelta_Accumulate+=once_delta;
      if((once_delta>=15)&&(once_delta<=60)) // �Ƕ���С�仯�ʲ���С��15��	�յ㲹���ǶȲ�����180 Ҫ������3s  ����ÿ�벻����60
      {
       if((Inflexion_chgcnter>0)&&(InflexLarge_or_Small==2)) //�ж�֮ǰ�Ƿ�һֱС��
       {
        Inflexion_chgcnter++;
        if((InflexDelta_Accumulate>=3)&&(InflexDelta_Accumulate>JT808Conf_struct.DURATION.SD_Delta_maxAngle)&&(InflexDelta_Accumulate<=180))
         {  //Ҫ�����ٳ���3s	�ۼƹսǲ����ǶȲ��ô���180
          InflexLarge_or_Small=0;
          Inflexion_chgcnter=0;
          InflexDelta_Accumulate=0;
          PositionSD_Enable(); // ���͹յ��־λ
          rt_kprintf("\r\n �յ��ϱ� --4\r\n");
         }
        else
         InflexLarge_or_Small=2; // �����С��
       }
       else
        {
        InflexLarge_or_Small=2;  // ���ǵ�һ��С��
        Inflexion_chgcnter=1;
        InflexDelta_Accumulate=once_delta;
        }
      }
       else
     {	  // С��15�Ⱦ������
       InflexLarge_or_Small=0;
       Inflexion_chgcnter=0;
       InflexDelta_Accumulate=0;
     }

      }



   }
   else
   {
    InflexLarge_or_Small=0;
    Inflexion_chgcnter=0;
    InflexDelta_Accumulate=0;
   }

    //--------------------------------------------------------
    Inflexion_Bak=Inflexion_Current; //  throw  old	 to  Bak

   }*/
//#endif

void  Sleep_Mode_ConfigEnter( void )
{
	if( SleepState == 0 )
	{
		SleepCounter++;
		if( SleepCounter > 15 )                                             // ��������
		{
			SleepCounter	= 0;
			SleepState		= 1;
			if( JT808Conf_struct.RT_LOCK.Lock_state != 1 )
			{
				Current_SD_Duration = JT808Conf_struct.DURATION.Sleep_Dur;  // 5����
			}
			//   �޸ļ������������
			PositionSD_Enable( );                                           //����
			Current_UDP_sd = 1;
			memcpy( BakTime, CurrentTime, 3 );                              // update

			//JT808Conf_struct.DURATION.Heart_SDCnter=25;
			//JT808Conf_struct.DURATION.Heart_Dur=320;


			/*   ���ͨҪ�����߲����¼�ʱ  ����
			   memcpy(BakTime,CurrentTime,3); // update  //gps��ʼ��ʱ
			   systemTick_trigGPS_counter=0; // ϵͳ��ʱ ���
			 */
			if( DataLink_Status( ) )
			{
				PositionSD_Enable( ); //  ���߾ͷ���    �ӱ����ͨҪ����������
			}
			rt_kprintf( "\r\n ��������״̬! \r\n" );
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  Sleep_Mode_ConfigExit( void )
{
	if( SleepState == 1 )
	{
		SleepState = 0;
		rt_kprintf( "\r\n ��̨����! \r\n" );
	}
	if( JT808Conf_struct.RT_LOCK.Lock_state != 1 )
	{
		Current_SD_Duration = JT808Conf_struct.DURATION.Default_Dur;
	}
	JT808Conf_struct.DURATION.Heart_Dur = 90;
	SleepState							= 0;
	SleepCounter						= 0;
}

#if 0


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u16 WaveFile_EncodeHeader( u32 inFilesize, u8* DestStr )
{
	u32 Filesize = 0, i = 0;                // Header Len  =44Bytes

	//  1. RIFF
	memcpy( DestStr, "RIFF", 4 );
	i += 4;
	//  2. Wave �ļ� ��С  С��ģʽ
	Filesize = 0x24 + ( inFilesize << 3 );  // ����16 �� 36 wave �ļ���С
	rt_kprintf( "\r\n .wav �ļ���С: %d Rawdata: %d \r\n ", Filesize, ( inFilesize << 3 ) );
	DestStr[i++]	= Filesize;             // LL
	DestStr[i++]	= ( Filesize >> 8 );    //LH
	DestStr[i++]	= ( Filesize >> 16 );   //HL
	DestStr[i++]	= ( Filesize >> 24 );   //HH
	//  3. WAVE string
	memcpy( DestStr + i, "WAVE", 4 );
	i += 4;
	//  4. fmt string
	memcpy( DestStr + i, "fmt ", 4 );
	i += 4;
	//  5. PCM Code
	DestStr[i++]	= 0x10;                 // LL
	DestStr[i++]	= 0x00;                 //LH
	DestStr[i++]	= 0x00;                 //HL
	DestStr[i++]	= 0x00;                 //HH
	//  6. Audio Format  PCM=1
	DestStr[i++]	= 0x01;                 // L
	DestStr[i++]	= 0x00;                 //H
	//  7. NumChannels  ͨ����
	DestStr[i++]	= 0x01;                 // L
	DestStr[i++]	= 0x00;                 //H
	//  8. SampleRate     8000<=>0x00001F40    16000<=>0x00003E80
	DestStr[i++]	= 0x40;                 //0x40; // LL
	DestStr[i++]	= 0x1F;                 //0x1F;//LH
	DestStr[i++]	= 0x00;                 //HL
	DestStr[i++]	= 0x00;                 //HH
	//  9.ByteRate       == SampleRate * NumChannels * BitsPerSample/8  ==8000x1x8/8
	DestStr[i++]	= 0x40;                 //0x40; // LL
	DestStr[i++]	= 0x1F;                 //0x1F;//LH
	DestStr[i++]	= 0x00;                 //HL
	DestStr[i++]	= 0x00;                 //HH

	// 10.BlockAlign    == NumChannels * BitsPerSample/8
	DestStr[i++]	= 0x01;                 //0x02;//0x01; // L
	DestStr[i++]	= 0x00;                 //H
	// 11.BitsPerSample
	DestStr[i++]	= 0x08;                 //0x10;//0x08; // L
	DestStr[i++]	= 0x00;                 //H
	// 12.data string
	memcpy( DestStr + i, "data", 4 );
	i += 4;
	// 13 .datasize
	Filesize		= ( inFilesize << 3 );  // ����16 �� 36 wave �ļ���С
	DestStr[i++]	= Filesize;             // LL
	DestStr[i++]	= ( Filesize >> 8 );    //LH
	DestStr[i++]	= ( Filesize >> 16 );   //HL
	DestStr[i++]	= ( Filesize >> 24 );   //HH

	return i;
}

#endif

//-----------  starttime[6]
u8 CurrentTime_Judge( u8*startTime, u8* endTime )
{
	u32 daycaculate_current		= 0, daycaculate_start = 0, daycaculate_end = 0;
	u32 secondcaculate_current	= 0, secondcaculate_start = 0, secondcaculate_end = 0;

	daycaculate_start		= ( ( startTime[0] >> 4 ) * 10 + ( startTime[0] & 0x0f ) ) * 365 + ( ( startTime[1] >> 4 ) * 10 + ( startTime[1] & 0x0f ) ) * 30 + ( ( startTime[2] >> 4 ) * 10 + ( startTime[2] & 0x0f ) );
	secondcaculate_start	= ( ( startTime[3] >> 4 ) * 10 + ( startTime[3] & 0x0f ) ) * 60 + ( ( startTime[4] >> 4 ) * 10 + ( startTime[4] & 0x0f ) ) * 60 + ( ( startTime[5] >> 4 ) * 10 + ( startTime[5] & 0x0f ) );

	daycaculate_end		= ( ( endTime[0] >> 4 ) * 10 + ( endTime[0] & 0x0f ) ) * 365 + ( ( endTime[1] >> 4 ) * 10 + ( endTime[1] & 0x0f ) ) * 30 + ( ( endTime[2] >> 4 ) * 10 + ( endTime[2] & 0x0f ) );
	secondcaculate_end	= ( ( endTime[3] >> 4 ) * 10 + ( endTime[3] & 0x0f ) ) * 60 + ( ( endTime[4] >> 4 ) * 10 + ( endTime[4] & 0x0f ) ) * 60 + ( ( endTime[5] >> 4 ) * 10 + ( endTime[5] & 0x0f ) );

	daycaculate_current		= ( time_now.year ) * 365 + time_now.month * 30 + time_now.day;
	secondcaculate_current	= time_now.hour * 60 + time_now.min * 60 + time_now.sec;

	if( ( daycaculate_current > daycaculate_start ) && ( daycaculate_current < daycaculate_end ) )
	{
		return true;
	}else
	if( ( secondcaculate_current >= secondcaculate_start ) && ( secondcaculate_current <= secondcaculate_end ) )
	{
		return true;
	}else
	{
		return false;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void CycleRail_Judge( u8* LatiStr, u8* LongiStr )
{
#if 0


	/*
	    γ��û�в�ֵ    1γ��  111km
	    40��γ���� 1����Ϊ  85.3km   (��������)
	 */
	u8	i				= 0;
	u32 Latitude		= 0, Longitude = 0;
	u32 DeltaLatiDis	= 0, DeltaLongiDis = 0, CacuDist = 0;
	u8	InOutState		= 0; //   0 ��ʾ in   1  ��ʾOut

	//  1. get value
	Latitude	= ( LatiStr[0] << 24 ) + ( LatiStr[1] << 16 ) + ( LatiStr[2] << 8 ) + LatiStr[3];
	Longitude	= ( LongiStr[0] << 24 ) + ( LongiStr[1] << 16 ) + ( LongiStr[2] << 8 ) + LongiStr[3];

	for( i = 0; i < 8; i++ )
	{
		InOutState = 0;
		memset( (u8*)&Rail_Cycle, 0, sizeof( Rail_Cycle ) );
		Api_RecordNum_Read( Rail_cycle, i + 1, (u8*)&Rail_Cycle, sizeof( Rail_Cycle ) );
		// rt_kprintf("\r\n\r\n Բ��Χ�� ��Ч״̬:%d  TYPE: %d    atrri=%d  lati: %d  longiti:%d  radicus:%d  maxspd: %d  keepdur:%d \r\n",Rail_Cycle.Effective_flag,Rail_Cycle.Area_ID,Rail_Cycle.Area_attribute,Rail_Cycle.Center_Latitude,Rail_Cycle.Center_Longitude,Rail_Cycle.Radius,Rail_Cycle.MaxSpd,Rail_Cycle.KeepDur);

		if( Rail_Cycle.Effective_flag == 1 )
		{
			DeltaLatiDis = abs( Latitude - Rail_Cycle.Center_Latitude ) / 9;                //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ���

			DeltaLongiDis = abs( Longitude - Rail_Cycle.Center_Longitude ) * 853 / 10000;   // a/1000000*85300=a 853/10000 m

			CacuDist = sqrt( ( DeltaLatiDis * DeltaLatiDis ) + ( DeltaLongiDis * DeltaLongiDis ) );

			rt_kprintf( "\r\n  TemperLati  %d  TemperLongi	%d	  Centerlati %d  center longi %d\r\n", Latitude, Longitude, Rail_Cycle.Center_Latitude, Rail_Cycle.Center_Longitude );
			rt_kprintf( "\r\n  he=%d heng=%d   shu=%d   juli=%d\r\n", abs( Longitude - Rail_Cycle.Center_Longitude ), DeltaLongiDis, DeltaLatiDis, CacuDist );

			if( DeltaLatiDis > Rail_Cycle.Radius )
			{                                                                                   // ���γ�Ⱦ������ �뾶�϶���
				InOutState = 1;
			}else
			{
				DeltaLongiDis = abs( Longitude - Rail_Cycle.Center_Longitude ) * 853 / 10000;   // a/1000000*85300=a 853/10000 m
				if( DeltaLongiDis > Rail_Cycle.Radius )
				{                                                                               // ������Ⱦ�����ڰ뾶�϶���
					InOutState = 1;
				}else //  �������������
				{
					CacuDist = sqrt( ( DeltaLatiDis * DeltaLatiDis ) + ( DeltaLongiDis * DeltaLongiDis ) );
				}
			}

			// 1. �ж�����
			if( Rail_Cycle.Area_attribute & 0x0001 ) //Bit 0 ����ʱ��
			{
				if( CurrentTime_Judge( Rail_Cycle.StartTimeBCD, Rail_Cycle.EndTimeBCD ) == false )
				{
					rt_kprintf( "\r\n ʱ��û�������� \r\n" );
					return;
				}
				//continue;
			}
			if( Rail_Cycle.Area_attribute & 0x0002 )    //Bit 1 ����
			{
				if( GPS_speed > Rail_Cycle.MaxSpd )
				{
					StatusReg_SPD_WARN( );              //  ���ٱ���״̬
					rt_kprintf( "\r\n  �趨Χ�����ٱ���\r\n" );
				}else
				{
					StatusReg_SPD_NORMAL( );
				}
				//continue;
			}
			if( Rail_Cycle.Area_attribute & 0x0004 )                                                    //Bit 2 �����򱨾�����ʻԱ
			{
				//continue;
			}
			if( Rail_Cycle.Area_attribute & 0x0008 )                                                    //Bit 3 �����򱨾���ƽ̨
			{
				if( ( InOutState == 0 ) && ( CacuDist < Rail_Cycle.Radius ) && ( Rail_Cycle.MaxSpd > ( Speed_gps / 10 ) ) )
				{
					if( InOut_Object.Keep_state != 1 )                                                  // ���û��in ����һ��
					{
						Warn_Status[1]			|= 0x10;                                                // �������򱨾�
						InOut_Object.TYPE		= 1;                                                    //Բ������
						InOut_Object.ID			= i;                                                    //  ID
						InOut_Object.InOutState = 0;                                                    //  ������
						rt_kprintf( "\r\n -----Բ�ε���Χ��--�뱨��" );
					}
					break;
				}
				//continue;
			}
			if( Rail_Cycle.Area_attribute & 0x0010 )                                                    //Bit 4 �����򱨾���˾��
			{
				;
				//continue;
			}
			if( ( Rail_Cycle.Area_attribute & 0x0020 ) && ( Rail_Cycle.MaxSpd > ( Speed_gps / 10 ) ) )  //Bit 5 �����򱨾���ƽ̨
			{
				if( ( InOutState == 1 ) || ( CacuDist > Rail_Cycle.Radius ) )
				{
					Warn_Status[1]			|= 0x10;                                                    // �������򱨾�
					InOut_Object.TYPE		= 1;                                                        //Բ������
					InOut_Object.ID			= i;                                                        //  ID
					InOut_Object.InOutState = 1;                                                        //  ������
					rt_kprintf( "\r\n -----Բ�ε���Χ��--������" );
					break;
				}

				//continue;
			}
			if( Rail_Cycle.Area_attribute & 0x0100 )                                                    //bit8  --BD ������
			{
				;                                                                                       // continue
			}
			if( Rail_Cycle.Area_attribute & 0x4000 )                                                    //bit14  --BD  ������ ����ͨ��ģ��
			{
				;                                                                                       // continue
			}
			if( Rail_Cycle.Area_attribute & 0x8000 )                                                    //bit15  --BD  ������ �ɼ���ϸ��Ϣ
			{
				;                                                                                       // continue
			}
		}
	}

#endif
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void RectangleRail_Judge( u8* LatiStr, u8* LongiStr )
{
	u8	i			= 0;
	u32 Latitude	= 0, Longitude = 0;
//	u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
	u8	InOutState = 1; //	 0 ��ʾ in	 1	��ʾOut

	//  1. get value
	Latitude	= ( LatiStr[0] << 24 ) + ( LatiStr[1] << 16 ) + ( LatiStr[2] << 8 ) + LatiStr[3];
	Longitude	= ( LongiStr[0] << 24 ) + ( LongiStr[1] << 16 ) + ( LongiStr[2] << 8 ) + LongiStr[3];

	// rt_kprintf("\r\n  1---TemperLati  %d  TemperLongi	%d	 res %d\r\n",Latitude,Longitude,InOutState);

	for( i = 0; i < 8; i++ )
	{
		InOutState = 1;
		//Api_RecordNum_Read( Rail_rect, i + 1, (u8*)&Rail_Rectangle, sizeof( Rail_Rectangle ) );		
		memcpy((u8*)&Rail_Rectangle,(u8*)&Rail_Rectangle_multi[i],sizeof(Rail_Rectangle));	
		

		if( Rail_Rectangle.Effective_flag == 1 )
		{
			//  rt_kprintf("\r\n\r\n �жϾ�����Χ�� ��Ч:%d ID: %d  atrri=%X  leftlati: %d  leftlongiti:%d    rightLati:%d   rightLongitu: %d	\r\n",Rail_Rectangle.Effective_flag,i+1,Rail_Rectangle.Area_attribute,Rail_Rectangle.LeftUp_Latitude,Rail_Rectangle.LeftUp_Longitude,Rail_Rectangle.RightDown_Latitude,Rail_Rectangle.RightDown_Longitude);
			if( ( Latitude > Rail_Rectangle.RightDown_Latitude ) && ( Latitude < Rail_Rectangle.LeftUp_Latitude ) && ( Longitude > Rail_Rectangle.LeftUp_Longitude ) && ( Longitude < Rail_Rectangle.RightDown_Longitude ) )
			{
				InOutState = 0;
			}

			//rt_kprintf("\r\n  TemperLati  %d  TemperLongi  %d   res %d\r\n",Latitude,Longitude,InOutState);

			// 1. �ж�����
			if( Rail_Rectangle.Area_attribute & 0x0001 )    //Bit 0 ����ʱ��
			{
				//continue;
			}
			if( Rail_Rectangle.Area_attribute & 0x0002 )    //Bit 1 ����
			{
				//continue;
			}
			if( Rail_Rectangle.Area_attribute & 0x0004 )    //Bit 2 �����򱨾�����ʻԱ
			{
				//continue;
			}
			if( Rail_Rectangle.Area_attribute & 0x0008 )    //Bit 3 �����򱨾���ƽ̨
			{
				if( InOutState == 0 )
				{
					if( InOut_Object.Keep_state == 2 )      // ���û��in ����һ��
					{
						Warn_Status[1]			|= 0x10;    // �������򱨾�
						InOut_Object.TYPE		= 2;        //��������
						InOut_Object.ID			= i;        //	ID
						InOut_Object.InOutState = 0;        //  ������
						InOut_Object.Keep_state = 1;        //��Χ����
						PositionSD_Enable( );
						Current_UDP_sd = 1;
						rt_kprintf( "\r\n -----���ε���Χ��--�뱨��" );
						gps_raw( "3" );  // ʹ��ԭʼGPS ������Ϣ�ϴ�
					}
					InOut_Object.Keep_state = 1;            //��Χ����	//
					break;
				}
				//continue;
			}
			if( Rail_Rectangle.Area_attribute & 0x0010 )    //Bit 4 �����򱨾���˾��
			{
				// continue;
			}
			if( Rail_Rectangle.Area_attribute & 0x0020 )    //Bit 5 �����򱨾���ƽ̨
			{
				if( InOutState == 1 )
				{
					if( InOut_Object.Keep_state == 1 )      // ���û��out ����һ��
					{
						Warn_Status[1]			|= 0x10;    // �������򱨾�
						InOut_Object.TYPE		= 2;        //��������
						InOut_Object.ID			= i;        //	ID
						InOut_Object.InOutState = 1;        //  ������
						InOut_Object.Keep_state = 2;        //��Χ����
						PositionSD_Enable( );
						Current_UDP_sd = 1;
						rt_kprintf( "\r\n -----���ε���Χ��--������" );
						gps_raw( "0" );
					}
					InOut_Object.Keep_state = 2;            //��Χ����
					break;
				}
			}
			// continue;
			if( Rail_Rectangle.Area_attribute & 0x4000 )    //Bit 14 �����ر�ͨ��ģ��
			{
				if( InOutState == 0 )
				{
					;                                       //  �ر�ͨ��


					   if( JT808Conf_struct.Close_CommunicateFlag==0)
					           {
					                  close_com();
									  mq_true_enable(2);  //  ʹ��ä�������洢
					                rt_kprintf("\r\n -----���ε���Χ��--�ر�ͨ��");

					           }	
					break;
				}else
				if( InOutState == 1 )
				{
					; // ����ͨ��
					if( JT808Conf_struct.Close_CommunicateFlag == 1 )
					{
						 open_com("0");
						 mq_true_enable(1); // ʹ��ä�������ϱ� 
						rt_kprintf( "\r\n -----���ε���Χ��--����ͨ��" );
					}
					break;
				}
			}
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void RouteLineWarn_judge( u8* LatiStr, u8* LongiStr )
{
	//  u32 Latitude=0,Longitude=0;

	//  1. get value
	// Latitude=(LatiStr[0]<<24)+(LatiStr[1]<<16)+(LatiStr[2]<<8)+LatiStr[3];
	// Longitude=(LongiStr[0]<<24)+(LongiStr[1]<<16)+(LongiStr[2]<<8)+LongiStr[3];

	//13 03 05 10 15 32 A 39587815N116000189E   ��-> ��
	if( ( Temp_Gps_Gprs.Date[0] == 13 ) && ( Temp_Gps_Gprs.Date[1] == 3 ) && ( Temp_Gps_Gprs.Date[2] == 5 ) && \
	    ( Temp_Gps_Gprs.Time[0] == 18 ) && ( Temp_Gps_Gprs.Time[1] == 15 ) && \
	    ( ( Temp_Gps_Gprs.Time[2] == 32 ) || ( Temp_Gps_Gprs.Time[2] == 33 ) ) )
	{
		if( ( Warn_Status[1] & 0x80 ) == 0 )    //  �����ǰû��������ô��ʱ�ϱ�
		{
			PositionSD_Enable( );
			Current_UDP_sd = 1;
			rt_kprintf( "\r\n     ����· !\r\n" );
		}
		Warn_Status[1] |= 0x80;                 // ·��ƫ������
	}

	//13 03 05 10 21 32 A 39578028N116000001E	   ��-> ��
	if( ( Temp_Gps_Gprs.Date[0] == 13 ) && ( Temp_Gps_Gprs.Date[1] == 3 ) && ( Temp_Gps_Gprs.Date[2] == 5 ) && \
	    ( Temp_Gps_Gprs.Time[0] == 18 ) && ( Temp_Gps_Gprs.Time[1] == 21 ) && \
	    ( ( Temp_Gps_Gprs.Time[2] == 32 ) || ( Temp_Gps_Gprs.Time[2] == 33 ) ) )
	{
		if( Warn_Status[1] & 0x80 ) //  �����ǰû��������ô��ʱ�ϱ�
		{
			PositionSD_Enable( );
			Current_UDP_sd = 1;
			rt_kprintf( "\r\n     �� ��· !\r\n" );
		}
		Warn_Status[1] &= ~0x80;    // ·��ƫ������
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void RouteRail_Judge( u8* LatiStr, u8* LongiStr )
{
	/*
	    γ��û�в�ֵ    1γ��  111km
	    40��γ���� 1����Ϊ  85.3km   (��������)
	 */
	u8	i			= 0;
	u8	route_cout	= 0, seg_count = 0, seg_num = 0;
	u32 Latitude	= 0, Longitude = 0;
	// u32 DeltaLatiDis=0,DeltaLongiDis=0,CacuDist=0;
	// u8  InOutState=0;   //   0 ��ʾ in   1  ��ʾOut
	u32 Route_Status	= 0;    // ÿ��bit ��ʾ һ��·�� ƫ��״̬Ĭ��Ϊ0
	u32 Segment_Status	= 0;    //  ��ǰ��·�У���Ӧ�˵�ƫ������� Ĭ��Ϊ0
	u32 Distance		= 0;
//     u8    InAreaJudge=0; //  �ж��Ƿ����ж����� bit 0 ���ȷ�Χ bit  1 γ�ȷ�Χ
	u32 Distance_Array[6];      //�洢������·����С���룬Ĭ���Ǹ�����ֵ

	//  1. get value
	Latitude	= ( LatiStr[0] << 24 ) + ( LatiStr[1] << 16 ) + ( LatiStr[2] << 8 ) + LatiStr[3];
	Longitude	= ( LongiStr[0] << 24 ) + ( LongiStr[1] << 16 ) + ( LongiStr[2] << 8 ) + LongiStr[3];

	// rt_kprintf("\r\n ��ǰ---->  Latitude:   %d     Longitude: %d\r\n",Latitude,Longitude);

	//  2.  Judge
	for( route_cout = 0; route_cout < Route_Mum; route_cout++ )                 // ��ȡ·��
	{
		// 2.1  --------   ��ȡ·��-----------
		memset( (u8*)&ROUTE_Obj, 0, sizeof( ROUTE_Obj ) );                      //  clear all  first
		DF_ReadFlash( DF_Route_Page + route_cout, 0, (u8*)&ROUTE_Obj, sizeof( ROUTE_Obj ) );
		DF_delay_us( 20 );
		//rt_kprintf("\r\n -----> ROUTE_Obj.RouteID:   %d \r\n",ROUTE_Obj.Route_ID);
		// 2.2  -----  �ж��Ƿ���Ч  -------
		if( ( ROUTE_Obj.Effective_flag == 1 ) && ( ROUTE_Obj.Points_Num > 1 ) ) //  �ж��Ƿ���Ч���йյ㣬����Ч������
		{
			// 2.2.0    ��ǰ�ξ��븶��һ�������ֵ
			for( i = 0; i < 6; i++ )
			{
				Distance_Array[i] = ROUTE_DIS_Default;
			}
			// 2.2.1      �������
			seg_num = ROUTE_Obj.Points_Num - 1; // ��·����Ŀ
			//  2.2.2    �ж�·����ÿһ�ε�״̬
			Segment_Status = 0;                 // ������ж�״̬��ÿ����·���¿�ʼһ��
			for( seg_count = 0; seg_count < seg_num; seg_count++ )
			{
				if( ( ROUTE_Obj.RoutePoints[seg_count + 1].POINT_Latitude == 0 ) && ( ROUTE_Obj.RoutePoints[seg_count + 1].POINT_Longitude == 0 ) )
				{
					rt_kprintf( "\r\n  �õ�Ϊ0 ��jump\r\n" );
					continue;
				}
				//----- ��ʼ���������, ��û�������ں�����������ж�
				Distance_Array[seg_count] = Distance_Point2Line( Latitude, Longitude, ROUTE_Obj.RoutePoints[seg_count].POINT_Latitude, ROUTE_Obj.RoutePoints[seg_count].POINT_Longitude, ROUTE_Obj.RoutePoints[seg_count + 1].POINT_Latitude, ROUTE_Obj.RoutePoints[seg_count + 1].POINT_Longitude );
			}
			//=========================================================
			//  2.4 ------  ��ӡ��ʾ���룬�ҳ���С��ֵ----
			Distance = Distance_Array[0]; // ��С����
			for( i = 0; i < 6; i++ )
			{
				if( Distance >= Distance_Array[i] )
				{
					Distance = Distance_Array[i];
				}
				// rt_kprintf("\r\n  Distance[%d]=%d",i,Distance_Array[i]);
			}
			rt_kprintf( "\r\n MinDistance =%d  Width=%d \r\n", Distance, ( ROUTE_Obj.RoutePoints[seg_num].Width >> 1 ) ); //

			if( Distance < ROUTE_DIS_Default )
			{
				//  ---- ��·�ο�����Ա�
				if( Distance > ( ROUTE_Obj.RoutePoints[seg_num].Width >> 1 ) )
				{
					rt_kprintf( "\r\n ·��ƫ��\r\n" );
					Segment_Status |= ( 1 << seg_num ); //  ����Ӧ��bit  ��λ
				}
			}

			//
		}
		// 2.4  ���� 2.2 ����жϵ���·��״̬
		if( Segment_Status )
		{
			Route_Status |= ( 1 << route_cout );    //  ����Ӧ��bit  ��λ
		}
	}
	// 3.  Result
	if( Route_Status )
	{
		if( ( Warn_Status[1] & 0x80 ) == 0 )        //  �����ǰû��������ô��ʱ�ϱ�
		{
			PositionSD_Enable( );
			Current_UDP_sd = 1;
		}

		Warn_Status[1] |= 0x80;                     // ·��ƫ������
		rt_kprintf( "\r\n    ·��ƫ������ !\r\n" );
	}else
	{
		if( Warn_Status[1] & 0x80 )                 //  �����ǰû��������ô��ʱ�ϱ�
		{
			PositionSD_Enable( );
			Current_UDP_sd = 1;
		}

		Warn_Status[1] &= ~0x80;                    // ·��ƫ������
	}
}

//--------  D�㵽ֱ�߾������-------


/*
     P1(x1,y1)   P2(x2,y2)  ,�ѵ�P(x1,y2)��Ϊ����ԭ�㣬��x1=0��y2=0��

     ��ô����P1��P2 ȷ����ֱ�߷���(����ʽ)Ϊ:
             (x-x1)/(x2-x1) =(y-y1)/(y2-y1)                          (1)

    ע:  ��׼ʽֱ�߷���Ϊ AX+BY+C=0;
             ��ôƽ��������һ��P(x0,y0) ��ֱ�ߵľ����ʾΪ
             d=abs(Ax0+By0+C)/sqrt(A^2+B^2)

    ���аѷ���ʽ(1) ת���ɱ�׼ʽΪ:
            (y2-y1)x+(x1-x2)y+x1(y1-y2)+y1(x2-x1)=0;

   ���ڵ�(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
    ����   A=-y1 ,  B=-x2, C=y1x2
    ��ô ֱ�ߵķ���:
                  -y1x-x2y+y1x2=0;  (2)

   =>     d=abs(-y1x0-x2y0+y1x2)/sqrt(y1^2+x2^2)       (3)

         ���� (3)  Ϊ����Ӧ�õĹ�ʽ

        ע:  �ȸ��ݾ�γ���ۺϼ���� x0��y0��x1,y1,x2,y2  ����ֵ��λΪ: ��
   =>  �����ж�:
           ����(2) �������  �� P1(0,y1) , P2(x2,0) ������ֱ֪�ߴ�ֱ������ֱ�߷���
              P1(0,y1) :      x2x-y1y+y1^2=0  (4)
              P2(x2,0) :      x2x-y1y-x2^2=0  (5)

          ��� y1 >=0      ֱ��(4)    ��ֱ��(5)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) <=  0    ��  (5)  >=0
       ��
           ��� y1 <=0      ֱ��(5)    ��ֱ��(4)  ���ϱ�
          ��ô �����߶������ڵ��жϷ�����
                       (4) >=  0    ��  (5)  <=0
   //------------------------------------------------------------------------------------------

       γ��û�в�ֵ    1γ��  111km
       40��γ���� 1����Ϊ  85.3km   (��������)

       X ��Ϊ ����(longitude) ��ֵ
       Y ��Ϊγ�� (latitude)  ��ֵ


   //------------------------------------------------------------------------------------------
 */

u32   Distance_Point2Line( u32 Cur_Lat, u32 Cur_Longi, u32 P1_Lat, u32 P1_Longi, u32 P2_Lat, u32 P2_Longi )
{                                                                           //   ���뵱ǰ�� �����ص㵽����ֱ�ߵľ���
	long	x0			= 0, y0 = 0, Line4_Resualt = 0, Line5_Resualt = 0;  // ��λ: ��
	long	y1			= 0;
	long	x2			= 0;
	long	distance	= 0;
	// long  Rabs=0;
//      long  Rsqrt=0;
	long	DeltaA1 = 0, DeltaA2 = 0, DeltaO1 = 0, DeltaO2 = 0;             //  DeltaA : Latitude     DeltaO:  Longitude
	// u32   Line4_Resualt2=0,Line5_Resualt2=0;
	double	fx0				= 0, fy0 = 0, fy1 = 0, fx2 = 0;
	double	FLine4_Resualt2 = 0, FLine5_Resualt2 = 0, fRabs = 0, fRsqrt = 0;

	// 0.   �ȴ��Ե��ж�
	DeltaA1 = abs( Cur_Lat - P1_Lat );
	DeltaA2 = abs( Cur_Lat - P2_Lat );
	DeltaO1 = abs( Cur_Lat - P1_Longi );
	DeltaO2 = abs( Cur_Lat - P2_Longi );


	/* if((DeltaA1>1000000) &&(DeltaA2>1000000))
	        {
	            rt_kprintf("\r\n  Latitude ��̫��\r\n");
	            return   ROUTE_DIS_Default;
	   }
	   if((DeltaO1>1000000) &&(DeltaO2>1000000))
	        {
	            rt_kprintf("\r\n  Longitude ��̫��\r\n");
	            return   ROUTE_DIS_Default;
	   }
	 */
	// 1.  ��ȡ  P1(0,y1)   P2(x2,0) ,��P(x0,y0)    P(x1,y2)Ϊԭ��  ��x1=0��y2=0��  P1(0,y1) , P2(x2,0)
	x2 = abs( P2_Longi - P1_Longi ); // a/1000000*85300=a 853/10000 m =a x 0.0853
	if( P2_Longi < P1_Longi )
	{
		x2 = 0 - x2;
	}
	fx2 = (double)( (double)x2 / 1000 );
	//rt_kprintf("\r\n P2_L=%d,P1_L=%d   delta=%d \r\n",P2_Longi,P1_Longi,(P2_Longi-P1_Longi));
	// if(P2_Longi
	y1 = abs( P2_Lat - P1_Lat ); //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ���
	if( P2_Lat < P1_Lat )
	{
		y1 = 0 - y1;
	}
	fy1 = (double)( (double)y1 / 1000 );
	//rt_kprintf("\r\n P2_LA=%d,P1_LA=%d   delta=%d \r\n",P2_Lat,P1_Lat,(P2_Lat-P1_Lat));

	//   rt_kprintf("\r\n ��֪��������: P1(0,%d)   P2(%d,0) \r\n", y1,x2);
	//    ��ǰ��
	x0 = abs( Cur_Longi - P1_Longi );
	if( Cur_Longi < P1_Longi )
	{
		x0 = 0 - x0;
	}
	fx0 = (double)( (double)x0 / 1000 );
	//rt_kprintf("\r\n Cur_L=%d,P1_L=%d   delta=%d \r\n",Cur_Longi,P1_Longi,(Cur_Longi-P1_Longi));

	y0 = abs( Cur_Lat - P2_Lat ); //  a/1000000*111000=a/9.009
	if( Cur_Lat < P2_Lat )
	{
		y0 = 0 - y0;
	}
	fy0 = (double)( (double)y0 / 1000 );
	// rt_kprintf("\r\n Cur_La=%d,P2_La=%d   delta=%d \r\n",Cur_Lat,P2_Lat,(Cur_Lat-P2_Lat));
	//   rt_kprintf("\r\n��ǰ������: P0(%d,%d)    \r\n", x0,y0);
	// 2. �ж�y1  �Ĵ�С�� ����� P1(0,y1)   P2(x2,0) ,����ֱ֪�ߵķ��̣����ж�
	//     ��ǰ���Ƿ���·�δ�ֱ��Χ��

	//  2.1   ����ǰ����룬 �� P1(0,y1)   �� ֱ�߷���(4)  ������
	Line4_Resualt	= ( x2 * x0 ) - ( y1 * y0 ) + ( y1 * y1 );
	FLine4_Resualt2 = fx2 * fx0 - fy1 * fy0 + fy1 * fy1;
	//     rt_kprintf("\r\n Line4=x2*x0-y1*y0+y1*y1=(%d)*(%d)-(%d)*(%d)+(%d)*(%d)=%ld     x2*x0=%d    y1*y0=%d   y1*y1=%d  \r\n",x2,x0,y1,y0,y1,y1,Line4_Resualt,x2*x0,y1*y0,y1*y1);
	//     rt_kprintf("\r\n FLine4=fx2*fx0-fy1*fy0+fy1*fy1=(%f)*(%f)-(%f)*(%f)+(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fy1*fy1=%f  \r\n",fx2,fx0,fy1,fy0,fy1,fy1,FLine4_Resualt2,fx2*fx0,fy1*fy0,fy1*fy1);

	//   2.2   ����ǰ����룬 ��P2(x2,0) �� ֱ�߷���(5)  ������
	Line5_Resualt	= ( x2 * x0 ) - y1 * y0 - x2 * x2;
	FLine5_Resualt2 = fx2 * fx0 - fy1 * fy0 - fx2 * fx2;
	//rt_kprintf("\r\n Line5=x2*x0-y1*y0-x2*x2=(%d)*(%d)-(%d)*(%d)-(%d)*(%d)=%ld     Se : %ld   \r\n",x2,x0,y1,y0,x2,x2,Line5_Resualt,Line5_Resualt2);
	//    rt_kprintf("\r\n FLine5=fx2*fx0-fy1*fy0-fx2*fx2=(%f)*(%f)-(%f)*(%f)-(%f)*(%f)=%f      fx2*fx0=%f    fy1*fy0=%f   fx2*fx2=%f  \r\n",fx2,fx0,fy1,fy0,fx2,fx2,FLine5_Resualt2,fx2*fx0,fy1*fy0,fx2*fx2);
	// rt_kprintf("\r\n  Line4_Resualt=%d     Line5_Resualt=%d  \r\n",Line4_Resualt,Line5_Resualt);

	if( fy1 >= 0 )                      //  ֱ��(4) ���Ϸ�
	{
		//   2.3   �ж�����    (4) <=  0    ��  (5)  >=0     // �ж�����ȡ��
		if( ( FLine4_Resualt2 > 0 ) || ( FLine5_Resualt2 < 0 ) )
		{
			return ROUTE_DIS_Default;   //  �������������������ֵ
		}
	} else
	{                                   //  ֱ��(5)
		//   2.4   �ж�����     (4) >=  0    ��  (5)  <=0     // �ж�����ȡ��
		if( ( FLine4_Resualt2 < 0 ) || ( FLine5_Resualt2 > 0 ) )
		{
			return ROUTE_DIS_Default;   //  �������������������ֵ
		}
	}

	rt_kprintf( "\r\n In judge area \r\n" );
	//rt_kprintf("\r\n   Current== Latitude:   %d     Longitude: %d     Point1== Latitude:   %d     Longitude: %d     Point2== Latitude:   %d     Longitude: %d\r\n",Cur_Lat,Cur_Longi,P1_Lat,P1_Longi,P2_Lat,P2_Longi);

	//  3. ����ֵ�����ʵ�ʾ���
#if 0
	x2	= x2 * 0.0853;  // a/1000000*85300=a 853/10000 m =a x 0.0853
	y1	= y1 / 9;       //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ���
	x0	= x0 * 0.0853;
	y0	= y0 / 9;       //  a/1000000*111000=a/9.009
#else
	fx2 = fx2 * 0.0853; // a/1000000*85300=a 853/10000 m =a x 0.0853
	fy1 = fy1 / 9;      //  a/1000000*111000=a/9.009	����һ����õ����� �ٳ��� 111000 �׵õ�ʵ�ʾ���
	fx0 = fx0 * 0.0853;
	fy0 = fy0 / 9;      //  a/1000000*111000=a/9.009
#endif

	//  4. �������
	//Rabs=0-y1*x0-x2*y0+y1*x2;
	// rt_kprintf("\r\n Test -y1*x0=%d -y0*x2=%d  y1*x2=%d   Rabs=%d  \r\n",0-y1*x0,0-y0*x2,0-y1*x2,Rabs);
#if 0
	Rabs	= abs( -y1 * x0 - x2 * y0 + y1 * x2 );
	Rsqrt	= sqrt( y1 * y1 + x2 * x2 );
	// distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2);
	distance = Rabs / Rsqrt;
	// rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);
#else
	fRabs	= abs( -fy1 * fx0 - fx2 * fy0 + fy1 * fx2 );
	fRsqrt	= sqrt( fy1 * fy1 + fx2 * fx2 );
	// distance=abs(-y1*x0-x2*y0-y1*x2)/sqrt(y1*y1+x2*x2);
	distance = (long)( ( fRabs / fRsqrt ) * 1000 );
	// rt_kprintf("\r\n Rabs=%d    Rsqrt=%d   d=%d",Rabs,Rsqrt,distance);
#endif

	return distance;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
unsigned short int CRC16_file( unsigned short int num )
{
#if  0
	unsigned short int	i = 0, j = 0;
	unsigned char		buffer_temp[514];
	memset( buffer_temp, 0, sizeof( buffer_temp ) );
	for( i = 0; i < num; i++ )
	{
		if( i == 0 )                //��һ��
		{
			Last_crc		= 0;    // clear first
			crc_fcs			= 0;
			buffer_temp[0]	= 0;
			buffer_temp[1]	= 51;
			DF_ReadFlash( 51, 0, &buffer_temp[2], PageSIZE );
			WatchDog_Feed( );

			Last_crc = CRC16_1( buffer_temp, 514, 0xffff );
			rt_kprintf( "\r\ni=%d,j=%d,Last_crc=%x", i, j, Last_crc );
		}else if( i == ( num - 1 ) ) //���һ��
		{
			buffer_temp[0]	= 0;
			buffer_temp[1]	= 50;
			DF_ReadFlash( 50, 0, &buffer_temp[2], PageSIZE );
			FileTCB_CRC16	= ( (unsigned short int)buffer_temp[512] << 8 ) + (unsigned short int)buffer_temp[513];
			crc_fcs			= CRC16_1( buffer_temp, 512, Last_crc );
			rt_kprintf( "\r\ni=%d,j=%d,Last_crc=%x ReadCrc=%x ", i, j, crc_fcs, FileTCB_CRC16 );
		}else
		{   // �м�İ�
			j				= i + 51;
			buffer_temp[0]	= (char)( j >> 8 );
			buffer_temp[1]	= (char)j;
			DF_ReadFlash( j, 0, &buffer_temp[2], PageSIZE );
			WatchDog_Feed( );
			Last_crc = CRC16_1( buffer_temp, 514, Last_crc );
			//rt_kprintf("\r\ni=%d,j=%d,Last_crc=%d",i,j,Last_crc);
		}
	}
	rt_kprintf( "\r\n  У���� %x", crc_fcs );
	return crc_fcs;
#endif
	return 1;
}

//-------  JT808  Related   Save  Process---------
void  Save_Status( u8 year, u8 mon, u8 day, u8 hour, u8 min, u8 sec )
{    // �洢�¹��ɵ� ���20s������״̬�֣��͵�ǰ�������Чλ����Ϣ
	u16				wr_add = 3, j = 0, cur = 0;
	u8				FCS;
	u8				regDateTime[6];
	static uint8_t	Statustmp[500];
//		u32  latitude=0,longitude=0;

	Time2BCD( regDateTime );
	//----------------------------------------------------------------------------
	wr_add = 0;
	memcpy( Statustmp, regDateTime, 6 );
	wr_add += 6;
	//-----------------------  Status Register   --------------------------------
	cur = save_sensorCounter;                               //20s���¹��ɵ�
	for( j = 0; j < 100; j++ )
	{
		Statustmp[wr_add++] = Sensor_buf[cur].DOUBTspeed;   //�ٶ�
		Statustmp[wr_add++] = Sensor_buf[cur].DOUBTstatus;  //״̬
		cur++;
		if( cur > 100 )
		{
			cur = 0;
		}
	}
	//---------------------------------------------------------------------------------------
	FCS = 0;
	for( j = 0; j < wr_add; j++ )
	{
		FCS ^= Statustmp[j];
	} //���ϱ����ݵ�����      ����У�� ��Ϣ����Ϊ   206
	  //------------------------------------------------------------------------------
	Statustmp[wr_add++] = FCS;
	//-----------------------------------------------------------------------------
	rt_kprintf( "\r\n �洢�г���¼  Record  write: %d    SaveLen: %d    \r\n", Recorder_write, wr_add );
	Api_DFdirectory_Write( doubt_data, Statustmp, wr_add );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Spd_Exp_Wr( void )
{
	u8	content[40];
	u8	wr_add = 0, i = 0, FCS = 0;

	memset( content, 0, sizeof( content ) );
	memcpy( content + wr_add, JT808Conf_struct.Driver_Info.DriverCard_ID, 18 );
	wr_add += 18;
	memcpy( content + wr_add, speed_Exd.ex_startTime, 6 );
	wr_add += 6;
	memcpy( content + wr_add, speed_Exd.ex_endTime, 6 );
	wr_add				+= 6;
	content[wr_add++]	= speed_Exd.current_maxSpd / 10;

	FCS = 0;
	for( i = 0; i < 32; i++ )
	{
		FCS ^= content[i];
	}                           //���ϱ����ݵ�����
	content[wr_add++] = FCS;    // ��31�ֽ�

	Api_DFdirectory_Write( spd_warn, (u8*)content, 32 );
	//----------- debug -----------------------
	rt_kprintf( "\r\n ���ٱ���  %X-%X-%X %X:%X:%X,MaxSpd=%d\r\n", speed_Exd.ex_endTime[0], speed_Exd.ex_endTime[1], speed_Exd.ex_endTime[2], speed_Exd.ex_endTime[3], speed_Exd.ex_endTime[4], speed_Exd.ex_endTime[5], speed_Exd.current_maxSpd );
	//--------- clear status ----------------------------
	Spd_ExpInit( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  JT808_Related_Save_Process( void )
{
	if( DF_LOCK )
	{
		return;
	}

	if( Dev_Voice.CMD_Type != '1' ) // ¼��ʱ�������´���
	{
		//-------------------------
		//    �洢�ɵ�����
		if( 1 == NandsaveFlg.Doubt_SaveFlag )
		{
			if( sensor_writeOverFlag == 1 )
			{
				time_now = Get_RTC( ); //  RTC  ���
				Save_Status( time_now.year, time_now.month, time_now.day, time_now.hour, time_now.min, time_now.sec );
				NandsaveFlg.Doubt_SaveFlag = 0;
				return;
			}
		}

		//-----------------  ���ٱ��� ----------------------
		if( speed_Exd.excd_status == 2 )
		{
			Spd_Exp_Wr( );
			return;
		}


		/*
		   if(NandsaveFlg.Setting_SaveFlag==1)   // ���� �޸ļ�¼
		   {
		   Save_Common(Settingchg_write,TYPE_SettingChgAdd);
		   Settingchg_write++;
		   if(Settingchg_write>=Max_CommonNum)
		   Settingchg_write=0;
		   DF_Write_RecordAdd(Settingchg_write,Settingchg_read,TYPE_SettingChgAdd);
		   NandsaveFlg.Setting_SaveFlag=0; // clear flag
		   }
		 */
	}
	//	 ��ʱ�洢���
	if( ( Vehicle_RunStatus ) && ( ( Systerm_Reset_counter & 0xff ) == 0xff ) )
	{   //  �����������ʻ�����У�ÿ255 ��洢һ���������
		Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
		return;
	}

	//--------------------------------------------------------------
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8   Time_FastJudge( void )
{
	u32 Date_value1		= 0, Time_value1 = 0;
	u32 Date_valueRTC	= 0, Time_valueRTC = 0;

	Date_value1		= Gps_Gprs.Date[0] * 365 + Gps_Gprs.Date[1] * 30 + Gps_Gprs.Date[2];
	Date_valueRTC	= time_now.year * 3600 + time_now.day * 60 + time_now.day;
	Time_value1		= Gps_Gprs.Time[0] * 365 + Gps_Gprs.Time[1] * 30 + Gps_Gprs.Time[2];
	Time_valueRTC	= time_now.hour * 3600 + time_now.min * 60 + time_now.sec;

	if( Date_value1 > Date_valueRTC )
	{
		rt_kprintf( "\r\n  ���ڳ�ǰ��Kick  OUT  !\r\n" );
		StatusReg_GPS_V( ); //  ����λ
		return false;
	}
	if( ( Time_value1 > Time_valueRTC ) && ( ( Time_value1 - Time_valueRTC ) > 120 ) )
	{
		rt_kprintf( "\r\n  ʱ�䳬ǰ��Kick  OUT  !\r\n" );
		StatusReg_GPS_V( ); // ����λ
		return false;
	}
	return true;
}

/*
    ��ӡ��� HEX ��Ϣ��Descrip : ������Ϣ ��instr :��ӡ��Ϣ�� inlen: ��ӡ����
 */
void OutPrint_HEX( u8 * Descrip, u8 *instr, u16 inlen )
{
	u32 i = 0;
	rt_kprintf( "\r\n %s:", Descrip );
	for( i = 0; i < inlen; i++ )
	{
		rt_kprintf( "%2X ", instr[i] );
	}
	rt_kprintf( "\r\n" );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void Tired_Check( void )
{
	/*if( DispContent == 2 )
	{
		rt_kprintf( "\r\n				�ٶ�: %d Km/h    Debugspd=%d KM/h \r\n", GPS_speed / 10, DebugSpd / 10 );
	}*/
	if( GPS_speed > 60 )                                                                                                        // GPS_speed ��λΪ0.1 km/h  �ٶȴ���6 km/h  ��Ϊ����ʻ
	// if(DebugSpd>60)
	{
		//-------------ƣ�ͼ�ʻ��� -----------------------
		TiredConf_struct.Tired_drive.ACC_ONstate_counter++;                                                                     // ACC �ۼƹ���ʱ��
		if( DispContent == 5 )
		{
			rt_kprintf( "\r\n  ACC ON = %d", TiredConf_struct.Tired_drive.ACC_ONstate_counter );
		}
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter > 3 )                                                              //ACC �� 485 ���迪�磬��������
		{
			Power_485CH1_ON;                                                                                                    // ��һ·485�ĵ�        �ϵ繤��
		}

		TiredConf_struct.Tired_drive.ACC_Offstate_counter = 0;                                                                  // clear
		//------ ͣ����ʱ���  -----------
		TiredConf_struct.TiredDoor.Parking_currentcnt	= 0;                                                                    // Clear  ͣ�� �������� 0
		Warn_Status[1]									&= ~0x08;                                                               // ͣ����ʱ���
		//---------------------------------
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 - 30 ) )   //��ǰ5���ӷ�������ʾע��ƣ�ͼ�ʻ 14100
		{
			TiredConf_struct.Tired_drive.Tgvoice_play = 1;
			TTS_play( "������20���ƣ�ͼ�ʻ����ע�ⰲȫ" );
		}

		Status_TiredwhRst = 1;                                                                                                  // ûƣ��ʱΪ1
		if( TiredConf_struct.Tired_drive.ACC_ONstate_counter >= ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 ) )        //14400 // ������ʻ����4Сʱ��ƣ�ͼ�ʻ
		{
			TiredConf_struct.Tired_drive.Tgvoice_play = 0;

			Status_TiredwhRst								= 2;                                                                // ƣ���˾�Ϊ2��
			TiredConf_struct.Tired_drive.Tireddrv_status	= 1;                                                                // ��¼ƣ�ͼ�ʻ��״̬
			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 ) )    //14400
			{
				//Tired_drive.Tgvoice_play=1;  // ��ʼ����ƣ�ͼ�ʻ������ʾ
				// Tired_drive.voicePly_counter++;
				Time2BCD( TiredConf_struct.Tired_drive.start_time );
				rt_kprintf( "\r\n   ƣ�ͼ�ʻ������! \r\n" );
				TTS_play( "���Ѿ�ƣ�ͼ�ʻ����ע����Ϣ" );
				tts_bro_tired_flag	= 1;
				Warn_Status[3]		|= 0x04; //BIT(2)  ƣ�ͼ�ʻ
				//---- ������ʱ�ϱ�����-------
				PositionSD_Enable( );
				Current_UDP_sd = 1;
				//-------------------------------------
				Rstart_time = 1;
			}
			//--------- ��������ʾ���	�رշ����� ��¼��ƣ�ͼ�ʻ  ------------
			Warn_Status[3]								|= 0x04;                                                                    //BIT(2)  ƣ�ͼ�ʻ
			TiredConf_struct.Tired_drive.Tgvoice_play	= 0;
		}
		TiredConf_struct.Tired_drive.Flag = 1;
	}else
	{
		//------------ ƣ�ͼ�ʻ���  ----------------------
		if( TiredConf_struct.Tired_drive.Flag == 1 )                                                                                //ֻ�д�ACCon�л�����ʱ�������´���������ڹ��ţ�û�б�Ҫ��
		{
			//-- ACC û����Ϣǰ����AccON ��״̬  ---------------
			TiredConf_struct.Tired_drive.ACC_ONstate_counter++;                                                                     // ACC �ۼƹ���ʱ��

			if( DispContent == 5 )
			{
				rt_kprintf( "\r\n	ACC ON 2 = %d, GPS_speed = %d", TiredConf_struct.Tired_drive.ACC_ONstate_counter, GPS_speed );
			}
			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 - 30 ) )   //��ǰ5���ӷ�������ʾע��ƣ�ͼ�ʻ 14100
			{
				TiredConf_struct.Tired_drive.Tgvoice_play = 1;
				TTS_play( "������20���ƣ�ͼ�ʻ����ע�ⰲȫ" );
			}

			if( TiredConf_struct.Tired_drive.ACC_ONstate_counter >= ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 ) )        //14400 // ������ʻ����4Сʱ��ƣ�ͼ�ʻ
			{
				Status_TiredwhRst								= 2;                                                                // ƣ���˾�Ϊ2��
				TiredConf_struct.Tired_drive.Tireddrv_status	= 1;                                                                // ��¼ƣ�ͼ�ʻ��״̬
				if( TiredConf_struct.Tired_drive.ACC_ONstate_counter == ( TiredConf_struct.TiredDoor.Door_DrvKeepingSec + 40 ) )    //14400
				{
					Time2BCD( TiredConf_struct.Tired_drive.start_time );
					rt_kprintf( "\r\n	 �ٶ�С����δ������Ϣ����ʱ�� ƣ�ͼ�ʻ������! \r\n");
					TTS_play( "���Ѿ�ƣ�ͼ�ʻ����ע����Ϣ" );
					tts_bro_tired_flag	= 1;
					Warn_Status[3]		|= 0x04;                                                                                    //BIT(2)  ƣ�ͼ�ʻ
					//---- ������ʱ�ϱ�����-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------
					Rstart_time = 1;
				}
				Warn_Status[3] |= 0x04;                                                                                             //BIT(2)  ƣ�ͼ�ʻ
			}
			//------------------------------------------------------------------------------
			TiredConf_struct.Tired_drive.ACC_Offstate_counter++;
			if( DispContent == 5 )
			{
				rt_kprintf( "\r\n  ACC OFF = %d, GPS_speed = %d", TiredConf_struct.Tired_drive.ACC_Offstate_counter, GPS_speed );
			}
			if( TiredConf_struct.Tired_drive.ACC_Offstate_counter >= TiredConf_struct.TiredDoor.Door_MinSleepSec )  //1200	// ACC ��20������Ϊ��Ϣ
			{
				if( TiredConf_struct.Tired_drive.ACC_Offstate_counter == TiredConf_struct.TiredDoor.Door_MinSleepSec )
				{
					TiredConf_struct.Tired_drive.ACC_Offstate_counter	= 0;
					TiredConf_struct.Tired_drive.ACC_ONstate_counter	= 0;
					Warn_Status[3]										&= ~0x04;                                   //BIT(2)	ƣ�ͼ�ʻ
					if( tts_bro_tired_flag == 1 )
					{
						TTS_play( "����ƣ�ͼ�ʻ�����Ѿ����" );
						tts_bro_tired_flag = 0;
					}
					// Status_TiredwhRst=0;
					// DF_WriteFlashSector(DF_TiredStartTime_Page,0,(u8*)&Status_TiredwhRst,1); //���O��Ҫд��
					//---- ������ʱ�ϱ�����-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------	                          rt_kprintf("\r\n   ACC  ��Ϣ�� \r\n");
					if( Rstart_time == 1 )
					{
						TiredConf_struct.Tired_drive.Flag = 0; ///////
						//Tired_drive.ACC_ONstate_counter=0;

						Rstart_time										= 0;
						TiredConf_struct.Tired_drive.Tireddrv_status	= 2;
						//rt_kprintf("\r\nƣ�ͽ���ʱ��д��");
					}
				}
			}
		}

		//----------  ����ͣ����� ------------------------------------------------------
		if( GPS_speed < 3 )                                                                                         //(GPS_speed==0)  // ͣ����ʱ���ٶ�Ϊ0ʱ��ʼ
		//if(DebugSpd<3)
		{
			TiredConf_struct.TiredDoor.Parking_currentcnt++;
			//if(DispContent==2)
			//  printf("\r\n @#@\r\n");
			if( TiredConf_struct.TiredDoor.Parking_currentcnt >= TiredConf_struct.TiredDoor.Door_MaxParkingSec )    //TiredDoor.Door_MaxParkingSec)
			{                                                                                                       //  ͣ����������ֵ������ͣ����
				Warn_Status[1] |= 0x08;                                                                             // ͣ����ʱ����
				if( TiredConf_struct.TiredDoor.Parking_currentcnt == TiredConf_struct.TiredDoor.Door_MaxParkingSec )
				{
					//---- ������ʱ�ϱ�����-------
					PositionSD_Enable( );
					Current_UDP_sd = 1;
					//-------------------------------------
					TTS_play( " ��ʱͣ������" );
					rt_kprintf( "\r\n ��ʱͣ������! \r\n" );
				}
			}
		}else
		{
			TiredConf_struct.TiredDoor.Parking_currentcnt = 0;
		}
		//--------------------------------------------------------------------------------
	}
}

//E:\work_xj\F4_BD\�����г���¼�ǹ���(��Э��)\2-21 RT-Thread_NewBoard-LCD2-Jiance\bsp\stm32f407vgt6_RecDrv\app_712\lcd\Menu_0_3_Sim.c
void CAN_struct_init( void )
{
	//-------  protocol variables
	CAN_trans.can1_sample_dur	= 0;        // can1 �ɼ����  ms
	CAN_trans.can1_trans_dur	= 0;        // can1  �ϱ���� s
	CAN_trans.can1_enable_get	= 0;        // 500ms

	CAN_trans.can2_sample_dur	= 0;        // can2 �ɼ����  ms
	CAN_trans.can2_trans_dur	= 0;        // can2  �ϱ���� s

	//   u8      canid_1[8];// ԭʼ����
	CAN_trans.canid_1_Filter_ID = 0;        // �����ж���
	//u8      canid_1_Rxbuf[400]; // ����buffer
	CAN_trans.canid_1_RxWr			= 0;    // дbuffer�±�
	CAN_trans.canid_1_SdWr			= 0;
	CAN_trans.canid_1_ext_state		= 0;    // ��չ֡״̬
	CAN_trans.canid_1_sample_dur	= 10;   // ��ID �Ĳɼ����
	CAN_trans.canid_ID_enableGet	= 0;

	//------- system variables
	CAN_trans.canid_timer		= 0;        //��ʱ��
	CAN_trans.canid_0705_sdFlag = 0;        // ���ͱ�־λ

	CAN_trans.nonew_CANdataflag;            //û������������־λ
	CAN_trans.nonew_CANdata_timer;          //  ��ʱ��
	CAN_trans.BAK_WR = 0;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  CAN_send_timer( void )
{
	u16 i = 0, datanum = 0;;
	if( CAN_trans.can1_trans_dur > 0 )
	{
		CAN_trans.canid_timer++;
		// if( CAN_trans.canid_timer>=CAN_trans.can1_trans_dur)
		if( CAN_trans.canid_timer >= 4 )
		{
			CAN_trans.canid_timer = 0;
			//------  �ж���û��������
			if( CAN_trans.canid_1_RxWr )
			{
				datanum = ( CAN_trans.canid_1_RxWr >> 3 );
				memcpy( CAN_trans.canid_1_Sdbuf, CAN_trans.canid_1_Rxbuf, CAN_trans.canid_1_RxWr );
				CAN_trans.canid_1_SdWr = CAN_trans.canid_1_RxWr;
				for( i = 0; i < datanum; i++ )
				{
					CAN_trans.canid_1_ID_SdBUF[i] = CAN_trans.canid_1_ID_RxBUF[i];
				}

				CAN_trans.canid_1_RxWr		= 0; // clear
				CAN_trans.canid_0705_sdFlag = 1;
			}else
			{
				rt_kprintf( "\r\n rx=%d  dur=%ds   CAN send trigger ,no data \r\n", CAN_trans.canid_1_RxWr, CAN_trans.can1_trans_dur );
			}
		}
	}else
	{
		CAN_trans.canid_0705_sdFlag = 0;
		CAN_trans.canid_timer		= 0;
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  CAN_Send_judge( void )
{
	u16 i = 0, datanum = 0;;

	datanum = ( CAN_trans.canid_1_RxWr >> 3 );

	if( datanum >= 35 )
	{
		datanum = ( CAN_trans.canid_1_RxWr >> 3 );
		memcpy( CAN_trans.canid_1_Sdbuf, CAN_trans.canid_1_Rxbuf, CAN_trans.canid_1_RxWr );
		CAN_trans.canid_1_SdWr = CAN_trans.canid_1_RxWr;
		for( i = 0; i < datanum; i++ )
		{
			CAN_trans.canid_1_ID_SdBUF[i] = CAN_trans.canid_1_ID_RxBUF[i];
		}

		CAN_trans.canid_1_RxWr		= 0; // clear
		CAN_trans.canid_0705_sdFlag = 1;
	}
}

//--------------  SMS Msg  Send Related  ------------------------
void SMS_send_init( void )
{
	SMS_send.Msg_sdState	= 0;
	SMS_send.Msg_step		= 0;
	SMS_send.Msg_timer		= 0;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void SMS_send_process( void )
{
	if( SMS_send.Msg_sdState == 1 )
	{
		switch( SMS_send.Msg_step )
		{
			case 1:                         // ��������AT+CMGS="13051953513"    //13602069191
				rt_hw_gsm_output( "AT+CMGS=\"13051953513\"\r\n" );
				SMS_send.Msg_step	= 2;
				SMS_send.Msg_timer	= 0;    // clear  timer   and  start counter
				rt_kprintf( "AT+CMGS=\"13051953513\"\r\n" );
				break;
			case  2:    break;
			case  4:                        // �������� 0x1A ����
				rt_hw_gsm_output( "ProductNum:70420  Type: TW705  DeviceNum:03 " );
				SMS_send.Msg_step = 5;
				rt_kprintf( "ProductNum:70420  Type: TW705  DeviceNum:03 \x1a" );
				break;
			case  7:
				rt_hw_gsm_output( "\x1a" );
				SMS_send.Msg_step = 8;
				break;
			default:
				break;
		}
	}
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  SMS_send_Step( u8 step )
{
	SMS_send.Msg_step = step;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
u8  SMS_send_stateQuery( void )
{
	return SMS_send.Msg_sdState;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  SMS_send_enable( void )
{
	//  Ҫ���Ͷ���
	SMS_send.Msg_sdState	= 1;
	SMS_send.Msg_step		= 1;
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  SMS_send_end( void )
{
	SMS_send_init( );
}

/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void SMS_Send_timer( void )
{
	if( SMS_send.Msg_sdState == 1 )
	{
		SMS_send.Msg_timer++;
		if( SMS_send.Msg_timer >= 10 ) // 2s
		{
			SMS_send.Msg_timer = 0;
			SMS_send_end( );
			rt_kprintf( " \r\n   SMS send timeout !\r\n" );
		}
		//-----------------------------
		if( SMS_send.Msg_step == 6 )
		{
			SMS_send.Msg_step = 7;
		}

		if( SMS_send.Msg_step == 5 )
		{
			SMS_send.Msg_step = 6;
		}

		if( SMS_send.Msg_step == 3 )
		{
			SMS_send.Msg_step = 4;
		}

		if( SMS_send.Msg_step == 2 )
		{
			rt_hw_gsm_output( "\r\n" );
			SMS_send.Msg_step = 3;
		}
		//-----------------------------
	}
}

//--------------------------------------------------------------------------
int use_gpsSpd( void )
{
	JT808Conf_struct.Speed_GetType		= 0;
	JT808Conf_struct.DF_K_adjustState	= 0; // clear  Flag
	ModuleStatus						&= ~Status_Pcheck;

	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
	rt_kprintf( " \r\n  ------------------------���ô�GPS��ȡ�ٶ�  Speed_GetType= %d", JT808Conf_struct.Speed_GetType );
	return 1;
}

FINSH_FUNCTION_EXPORT( use_gpsSpd, usegpsspd );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
int use_sensorspd( void )
{
	JT808Conf_struct.Speed_GetType		= 1;
	JT808Conf_struct.DF_K_adjustState	= 1; // clear  Flag
	ModuleStatus						|= Status_Pcheck;
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
	rt_kprintf( " \r\n  ------------------------���ô�Sensor��ȡ�ٶ�	Speed_GetType= %d", JT808Conf_struct.Speed_GetType );
	return 1;
}

FINSH_FUNCTION_EXPORT( use_sensorspd, sensorSpd );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  Init_original( void )
{
	if( SysConf_struct.Version_ID == SYSID )    //  check  wether need  update  or not
	{
		SysConf_struct.Version_ID = SYSID + 1;
		Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
		Systerm_Reset_counter	= Max_SystemCounter;
		ISP_resetFlag			= 2;            //   ����Զ�������������Ƹ�λϵͳ
	}
	rt_kprintf( "\r\n �ֶ��ָ��������� \r\n" );
}

FINSH_FUNCTION_EXPORT( Init_original, Init_original );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  apn_set( u8 * Content )
{
	u8 infolen = 0;

	infolen = strlen( Content );
	memset( APN_String, 0, sizeof( APN_String ) );
	memcpy( APN_String, (char*)Content, infolen );
	memset( (u8*)SysConf_struct.APN_str, 0, sizeof( APN_String ) );
	memcpy( (u8*)SysConf_struct.APN_str, (char*)Content, infolen );
	Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
	DataLink_APN_Set( APN_String, 1 );
	rt_kprintf( "\r\n �ֶ�����APN \r\n" );
}

FINSH_FUNCTION_EXPORT( apn_set, Set APN_String );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  dur( u8 *content )
{
	sscanf( content, "%d", (u32*)&Current_SD_Duration );
	rt_kprintf( "\r\n �ֶ������ϱ�ʱ���� %d s\r\n", Current_SD_Duration );

	JT808Conf_struct.DURATION.Default_Dur = Current_SD_Duration;
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
}

FINSH_FUNCTION_EXPORT( dur, Set APN_String );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void idset( u8 *ID )
{
	u8 len = 0;

	if( strlen( ID ) != 12 )
	{
		rt_kprintf( "\r\nID  ���Ȳ���\r\n" ); return;
	}

	memcpy( (char*)JT808Conf_struct.Vechicle_Info.Vech_sim, ID, 12 );
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );

	memcpy( (char*)IMSI_CODE + 3, (char*)JT808Conf_struct.Vechicle_Info.Vech_sim, 12 );
	IMSI_Convert_SIMCODE( ); //  translate
	rt_kprintf( "\r\n ID=%s\r\n", ID );
}

FINSH_FUNCTION_EXPORT( idset, idset );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void recoverset( void )
{
	FirstRun_Config_Write( ); // ��߸����� SYSID
}

FINSH_FUNCTION_EXPORT( recoverset, recoverset );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void chepai( u8 *instr )
{
	memset( JT808Conf_struct.Vechicle_Info.Vech_Num, 0, sizeof( JT808Conf_struct.Vechicle_Info.Vech_Num ) );
	memcpy( JT808Conf_struct.Vechicle_Info.Vech_Num, instr, 8 );
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
}

FINSH_FUNCTION_EXPORT( chepai, chepai );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  localtrig( u8 *instr )
{
	local_trig = instr[0] - 0x30;
	if( local_trig )
	{
		GPS_getfirst = 1; local_timer = 0;
	}else
	{
		GPS_getfirst = 0; local_timer = 0;
	}
	rt_kprintf( "\r\n Enable local  time=%d  \r\n", local_trig );
}

FINSH_FUNCTION_EXPORT( localtrig, localtrig );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  current( void )
{
	PositionSD_Enable( );
	Current_UDP_sd = 1;
	memcpy( BakTime, CurrentTime, 3 ); // update

	//  rt_kprintf("\r\n---> tim1  %d  \r\n",TIM1_Timer_Counter);
	//TIM1_Timer_Counter=0;
}

FINSH_FUNCTION_EXPORT( current, current );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void txt_play( u8 *instr )
{
	u16 len = strlen( instr );

	//--------------  ����----
	Dev_Voice.CMD_Type = '2';
	memset( Dev_Voice.Play_info, 0, sizeof( Dev_Voice.Play_info ) );
	memcpy( Dev_Voice.Play_info, instr, len );
	Dev_Voice.info_sdFlag = 1;

	//#ifdef LCD_5inch
	//DwinLCD.Type=LCD_SDTXT;
	// memset(DwinLCD.TXT_content,0,sizeof(DwinLCD.TXT_content));
	//DwinLCD.TXT_contentLen=AsciiToGb(DwinLCD.TXT_content,infolen-1,UDP_HEX_Rx+14);
	//#endif
	//  TTS
	TTS_Get_Data( instr, len );

#ifdef LCD_5inch
	//======  ��Ϣ������Ļ����ʾ
	DwinLCD.Type = LCD_SDTXT;
	memset( DwinLCD.TXT_content, 0, sizeof( DwinLCD.TXT_content ) );
	DwinLCD.TXT_contentLen = AsciiToGb( DwinLCD.TXT_content, len, instr );
#endif
}

FINSH_FUNCTION_EXPORT( txt_play, txt_play );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void ICack( void )
{
	u8	i			= 0;
	u8	ackreg[24]	= { 0x00, 0x01, 0xf1, 0xd8, 0x37, 0x25, 0x61, 0xe8, 0x30, 0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xc6, 0x39 };

	memcpy( IC_MOD.IC_Rx, ackreg, 24 );
	rt_kprintf( "\r\n IC ��͸��len=%dBytes  RX:", 24 );
	for( i = 0; i < 24; i++ )
	{
		rt_kprintf( "%2X ", ackreg[i] );
	}
	rt_kprintf( "\r\n" );
	//------ ֱ�ӷ��͸�IC ��ģ��-----
	DeviceData_Encode_Send( 0x0B, 0x40, ackreg, 24 ); 
}

FINSH_FUNCTION_EXPORT( ICack, ICack ); 


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  msg_send( void )
{
	SMS_send_enable( );
	rt_kprintf( "\r\n   �յ�ʹ�ܷ��Ͷ���\r\n" );
}

FINSH_FUNCTION_EXPORT( msg_send, msg_send );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void close_com( void )
{
	JT808Conf_struct.Close_CommunicateFlag = 1;
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
	Close_DataLink( );
	rt_kprintf( "\r\nclose com\r\n" );
}

FINSH_FUNCTION_EXPORT( close_com, close_com );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void open_com( u8 *intstr )
{
	rt_kprintf( "\r\n open com\r\n" );
	redial( );
	JT808Conf_struct.Close_CommunicateFlag = 0;
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );

	if( intstr[0] == '0' )
	{
		Mangqu_Init( );
		MangQU.Enable_SD_state = 0;
	}else
	if( intstr[0] == '1' )
	{
		MangQU.Enable_SD_state = 1;
		// MangQU.PacketNum=0;
		MangQU.PacketNum = 0;
		BlindZoneData_send_start( );
	}
}

FINSH_FUNCTION_EXPORT( open_com, open_com[1 | 0] );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void app_queenable( u8* instr )
{
	if( instr[0] == '0' )
	{
		app_que_enable	= 0;
		stop_current	= 0;
		//----- init-----------------
		AppQue.write_num		= 0;
		AppQue.read_num			= 0;
		AppQue.sd_enable_flag	= 0;
		AppQue.abnormal_counter = 0;
		AppQue.re_send_flag		= 0;
		AppQue.send_timer		= 0;
		//-------------------------
		rt_kprintf( "\r\n app que disable\r\n" );
	}
	if( instr[0] == '1' )
	{
		app_que_enable = 1;
		//----- init-----------------
		AppQue.write_num		= 0;
		AppQue.read_num			= 0;
		AppQue.sd_enable_flag	= 0;
		AppQue.abnormal_counter = 0;
		AppQue.re_send_flag		= 0;
		AppQue.send_timer		= 0;
		//-------------------------
		rt_kprintf( "\r\n app que enable\r\n" );
	}
}

FINSH_FUNCTION_EXPORT( app_queenable, app_queenable );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void   stop_normal( u8 *instr )
{
	if( instr[0] == '0' )
	{
		stopNormal				= 0;
		AppQue.sd_enable_flag	= 0;
		rt_kprintf( "\r\n  stop_normal disable\r\n" );
	}
	if( instr[0] == '1' )
	{
		stopNormal				= 1;
		AppQue.sd_enable_flag	= 0;
		rt_kprintf( "\r\n stop_normal que enable -only  pos\r\n" );
	}
	if( instr[0] == '2' )
	{
		stopNormal = 2;
		rt_kprintf( "\r\n stop_normal que enable-forbid all\r\n" );
	}
}

FINSH_FUNCTION_EXPORT( stop_normal, stop_normal[1 | 0] );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void  link_ic( char *str )
{
	u8			i = 0;
	u8			reg_str[80];

	uint8_t		ip[4];
	uint16_t	port;

	if( strlen( (const char*)str ) )
	{
		i = str2ipport( (char*)str, ip, &port );
		if( i == 5 )
		{
			memcpy( (char*)SysConf_struct.BD_IC_main_IP, ip, 4 );
			SysConf_struct.BD_IC_TCP_port = port;
			Api_Config_write( config, ID_CONF_SYS, (u8*)&SysConf_struct, sizeof( SysConf_struct ) );
			DataLink_IC_Socket_set( ip, port, 0 );
		}
	}

	Hand_Login		= 0;
	Enable_IClink	= 1;
	TCP2_ready_dial = 1;
	rt_kprintf( "   \r\n    Handle  link  IC  \r\n" );
}

FINSH_FUNCTION_EXPORT( link_ic, link_ic );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void cansd_last( void )
{
	u16 i = 0, datanum = 0;;

	datanum = ( CAN_trans.canid_1_RxWr >> 3 );
	memcpy( CAN_trans.canid_1_Sdbuf, CAN_trans.canid_1_Rxbuf, CAN_trans.canid_1_RxWr );
	CAN_trans.canid_1_SdWr = CAN_trans.canid_1_RxWr;
	for( i = 0; i < datanum; i++ )
	{
		CAN_trans.canid_1_ID_SdBUF[i] = CAN_trans.canid_1_ID_RxBUF[i];
	}

	CAN_trans.canid_1_RxWr		= 0; // clear
	CAN_trans.canid_0705_sdFlag = 1;

	rt_kprintf( "   \r\n    send link last \r\n" );
}

FINSH_FUNCTION_EXPORT( cansd_last, cansd_last );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void stopcurrent( u8 *instr )
{
	if( instr[0] == '0' )
	{
		stop_current = 0; rt_kprintf( "\r\n  enable current \r\n" );
	}

	if( instr[0] == '1' )
	{
		stop_current = 1; rt_kprintf( "\r\n stop current \r\n" );
	}
}

FINSH_FUNCTION_EXPORT( stopcurrent, stopcurrent[1 | 0] );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void sim_0700( uint8_t cmd )
{
	Recode_Obj.SD_Data_Flag = 1;
	Recode_Obj.CountStep	= 1;
	Recode_Obj.CMD			= cmd;
	if( ( cmd == 0x13 ) || ( cmd == 0x14 ) )
	{
		Recode_Obj.Devide_Flag = 0;
		Stuff_RecoderACK_0700H( Packet_Normal );
	}else
	{
		Recode_Obj.Devide_Flag = 1;
		Stuff_RecoderACK_0700H( Packet_Divide );
	}
	Recode_Obj.SD_Data_Flag = 0;
}


FINSH_FUNCTION_EXPORT( sim_0700, sim_0700 );


void password(u8 value)
{
  rt_kprintf("\r\n password(%d)",value);
  JT808Conf_struct.password_flag=value;     // clear  first flag		
  Api_Config_Recwrite_Large(jt808,0,(u8*)&JT808Conf_struct,sizeof(JT808Conf_struct));    

}
FINSH_FUNCTION_EXPORT( password, password ); 

void buzzer_onoff(u8 in) 
{
   
   GPIO_InitTypeDef GPIO_InitStructure;
   
     if(0==in)
    {  
     GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 			//ָ����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;		//ģʽΪ����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//Ƶ��Ϊ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;		//�����Ա��ʡ����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  
	  
    }
	  
   if(1==in)
    {
	  //-----------------  hardware  0x101    5   Beep -----------------
	/*�����ýṹ���еĲ��ֳ�Ա����������£��û�Ӧ�����ȵ��ú���PPP_SturcInit(..)
	����ʼ������PPP_InitStructure��Ȼ�����޸�������Ҫ�޸ĵĳ�Ա���������Ա�֤����
	��Ա��ֵ����Ϊȱʡֵ������ȷ���롣
	 */
	
	GPIO_StructInit(&GPIO_InitStructure);
	
	/*����GPIOA_Pin_5����ΪTIM2_Channel1 PWM���*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 			//ָ����������
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;		//ģʽ����Ϊ���ã�
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//Ƶ��Ϊ����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;		//��������PWM������Ӱ��
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2); //����GPIOA_Pin1ΪTIM2_Ch2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_TIM2); //����GPIOA_Pin5ΪTIM2_Ch1, 
	} 


}
FINSH_FUNCTION_EXPORT(buzzer_onoff, buzzer_onoff[1|0]);    

void end_link(void)
{
   DataLink_EndFlag=1;
   rt_kprintf("\r\n �ֶ��Ҷ�\r\n");
   
}
FINSH_FUNCTION_EXPORT(end_link, end_link);  

void mq_erase(void)
{
  u8  i=0;
				for(i=0;i<10;i++)
				{
	                 WatchDog_Feed(); 
					 SST25V_BlockErase_64KByte(((CycleStart_offset+128*i)<<9));   
			         DF_delay_ms(600);   
				     WatchDog_Feed(); 
				}
				
	cycle_write=0;//6959
	cycle_read=0;	//0
	DF_Write_RecordAdd(cycle_write,cycle_read,TYPE_CycleAdd);   	
	rt_kprintf("\r\n earse_MQ   WR=%d  RD=%d \r\n",cycle_write,cycle_read);    		

}
FINSH_FUNCTION_EXPORT(mq_erase,mq_erase);
	
void mq_cmd(void)
{
   Get_GSM_HexData("7E860200180136013000046AEF01010000006640000262548106BC3E400238A9580717CBC09C7E",78,0);
   OutPrint_HEX("ä��Χ��", GSM_HEX, GSM_HEX_len); 
}
FINSH_FUNCTION_EXPORT(mq_cmd, mq_cmd);

u32 Get_MQ_true_CuurentTotal_packets(void)
{
   u32  delta_reg=0;
        //��ȡ�ܰ���
	        if(cycle_read<cycle_write)
	        {
	           /* delta_reg=cycle_write-cycle_read;
				// �����ܰ���
				if(delta_reg%MQ_PKNUM)
				    CurrentTotal=delta_reg/MQ_PKNUM+1;
				else
					 CurrentTotal=delta_reg/MQ_PKNUM;
			    */
			    if(cycle_write%MQ_PKNUM)
				    CurrentTotal=cycle_write/MQ_PKNUM+1;
				else
					 CurrentTotal=cycle_write/MQ_PKNUM; 

	        }
			else   // write С�� read
			{	    
				// �����ܰ���
			    delta_reg=Max_CycleNum+cycle_write-cycle_read; 
				if(delta_reg%MQ_PKNUM)
				    CurrentTotal=delta_reg/MQ_PKNUM+2;  // ���Ҫ��2��
				else
					 CurrentTotal=delta_reg/MQ_PKNUM+1; 

			}
   return CurrentTotal;
}

void  sequence_sd(u8 value)
{

  BD_send_Init();
  DF_Write_RecordAdd(BDSD.write,BDSD.read,TYPE_BDsdAdd); 
  
  BDSD.Enable_Working=value;   
  		  
  rt_kprintf("\r\n  ��Ϣ���з���ģʽ: %d\r\n",BDSD.Enable_Working);
}
FINSH_FUNCTION_EXPORT(sequence_sd, sequence_sd0:disable1:enable);
	

void mq_true_enable(u8 value)
{
 u32  delta_reg=0;
  switch(value)
  { 
     case 2:
          {
		    mq_erase();
			MQsend_counter=0;
		    MQ_TrueUse.Enable_SD_state=2;
			rt_kprintf("\r\n  MQ_true mode  :enable save !\r\n");
		  }
		 break;
	case 1:  	  
		  {
		    MQ_TrueUse.Enable_SD_state=1; 
			MQ_TrueUse.PacketNum=1;
		     //��ȡ�ܰ���
		    Mq_total_pkg=Get_MQ_true_CuurentTotal_packets();
			rt_kprintf("\r\n  MQ_true mode  :enable send ! write=%d  read=%d  Totalpackets: %d  CurrentTotal=%d\r\n",cycle_write,cycle_read,Mq_total_pkg,CurrentTotal);
			
		  }
	      break;
    case  0:
	default:
		  MQ_TrueUse.Enable_SD_state=0; 
			rt_kprintf("\r\n  MQ_true mode  :enable disable !\r\n"); 
	      break;
  	}		  

}
FINSH_FUNCTION_EXPORT(mq_true_enable, mq_true_enable 0:idle+othervalue 1:send 2:save);
//+++++++++++++++++++++++++++++++++++++++wxg++++++++++��Ȩ+++++++++++
void jianquan(u8 *str)
{
	memset( JT808Conf_struct.ConfirmCode, 0, sizeof( JT808Conf_struct.ConfirmCode ));
	memcpy( JT808Conf_struct.ConfirmCode, str, strlen(str) );	
	JT808Conf_struct.Regsiter_Status	= 1;
	Api_Config_Recwrite_Large( jt808, 0, (u8*)&JT808Conf_struct, sizeof( JT808Conf_struct ) );
	rt_kprintf( "��Ȩ��: %s\r\n		   ��Ȩ�볤��: %d\r\n", str, strlen( (const char*)str ) );
	//-------- ��ʼ��Ȩ ------
	DEV_Login.Operate_enable = 1;
}
FINSH_FUNCTION_EXPORT(jianquan,1);

 void bd_time_set(void)
 	{
       
	   TDateTime   now;
	   
 	  	  now.year=Gps_Gprs.Date[0];
	  now.month=Gps_Gprs.Date[1];
	  now.day=Gps_Gprs.Date[2]; 

	  now.hour=Gps_Gprs.Time[0];
	  now.min=Gps_Gprs.Time[1];
	  now.sec=Gps_Gprs.Time[2];
	  now.week=3;
	  
	 // Set_RTC(now);
	 Device_RTC_set(now);  

 	}
FINSH_FUNCTION_EXPORT(bd_time_set,1); 

void in(void)
{
  close_com();
   mq_true_enable(2);  //  ʹ��ä�������洢
   rt_kprintf("\r\n  Area  In \r\n");
}
FINSH_FUNCTION_EXPORT(in,1); 


void out(void)
{
   open_com("0");
   mq_true_enable(1); // ʹ��ä�������ϱ�  
   rt_kprintf("\r\n  Area  Out \r\n");
}
FINSH_FUNCTION_EXPORT(out,1); 

// C.  Module
