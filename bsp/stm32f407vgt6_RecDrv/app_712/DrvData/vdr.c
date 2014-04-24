/*
   vdr Vichle Driver Record 车辆行驶记录
 */

#include <rtthread.h>
#include <finsh.h>
#include "stm32f4xx.h"

#include <rtdevice.h>
#include <dfs_posix.h>

#include <time.h>

#include "sst25vf.h"


/*
   实现每条记录128字节，便于定位。要做一定的处理。没有使用delta编码

   记录格式

   000--003  yy-mm-dd hh:mm   可以把时间转成UTC格式,便于比较查找
      又可以节省一个字节，无效的时间格式FFFFFFFFFF
   004--056  秒速度记录 速度要求+/- 1kmh
          原先60*8=480bit 现在用7bit保存需要 60*7=420bit=53byte
          7bit的 0b1111111 表示速度无效(gps未定位)
   057--116  状态信息
   117--126  单位分钟位置,参见行车记录仪 GBT19065 首个有效位置

 */


/*
   4MB serial flash 0x400000
 */

/*转换hex到bcd的编码*/
#define HEX_TO_BCD( A ) ( ( ( ( A ) / 10 ) << 4 ) | ( ( A ) % 10 ) )

static rt_thread_t tid_usb_vdr = RT_NULL;

struct _sect_info
{
	uint32_t	addr;               //开始的地址
	uint16_t	bytes_per_block;
	uint8_t		blocks;
} sect_info[7] =
{
	{ 0x00300000, 8192, 24	},      //block_08h_09h		8*1024Bytes	24Block		0x30000	192k	0x30000
	{ 0x00330000, 256,	128 },      //block_10h         234Bytes	100Block	0x8000	32k		0x 8000
	{ 0x00338000, 64,	128 },      //block 11          50Bytes		100Block	0x2000	8k		0x 2000
	{ 0x0033A000, 32,	256 },      //block 12          25Bytes		200block	0x2000	8k		0x 2000
	{ 0x0033C000, 8,	128 },      //block 13          7Bytes		100block	0x400	1k		0x 1000
	{ 0x0033D000, 8,	128 },      //block 14          7Bytes		100block	0x400	1k		0x 1000
	{ 0x0033E000, 256,	16	},      //block 15          133Bytes	10block		0x1000	4k		0x 1000
};

#define VDR_08H_09H_START	0x300000
#define VDR_08H_09H_END		0x32FFFF

#define VDR_10H_START	0x330000    /*256*100= 0x6400*/
#define VDR_10H_END		0x3363FF

#define VDR_11H_START	0x338000
#define VDR_11H_END		0x3398FF    /*64*100=0x1900*/

#define VDR_12H_START	0x33A000
#define VDR_12H_END		0x33B8FF    /*32*200=*/

#define VDR_13H_START	0x33C000
#define VDR_13H_END		0x33C31F    /*128*100*/

#define VDR_14H_START	0x33D000
#define VDR_14H_END		0x33D31F

#define VDR_15H_START	0x33E000
#define VDR_15H_END		0x33E98F


/*基于小时的时间戳,主要是为了比较大小使用
   byte0 year
   byte1 month
   byte2 day
   byte3 hour
 */
typedef unsigned int YMDH_TIME;

typedef struct
{
	uint8_t cmd;

	uint32_t	ymdh_start;
	uint8_t		minute_start;

	uint32_t	ymdh_end;
	uint8_t		minute_end;

	uint32_t	ymdh_curr;
	uint8_t		minute_curr;
	uint32_t	addr;

	uint16_t	blocks;         /*定义每次上传多少个数据块*/
	uint16_t	blocks_remain;  /*当前组织上传包是还需要的的blocks*/
}VDR_CMD;

VDR_CMD		vdr_cmd;

uint8_t		vdr_tx_info[1024];
uint16_t	vtr_tx_len = 0;

/*当前要写入数据的地址,8K边界对齐*/
static uint32_t		vdr_addr_curr_wr	= 0x0;
static YMDH_TIME	vdr_ymdh_curr_wr	= 0;

static uint8_t		fvdr_debug = 1;


/*传递写入文件的信息
   0...3  写入SerialFlash的地址
   4...31 文件名
 */
static uint8_t file_rec[32];

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
static void  SST25V_BufferRead( u8* pBuffer, u32 ReadAddr, u16 NumByteToRead )
{
	u32 i = 0;
	for( i = 0; i < NumByteToRead; i++ )
	{
		*pBuffer = SST25V_ByteRead( ( (u32)ReadAddr + i ) );
		pBuffer++;
	}
	DF_delay_ms( 5 );
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
static unsigned long mymktime( uint32_t year, uint32_t mon, uint32_t day, uint32_t hour, uint32_t min, uint32_t sec )
{
	if( 0 >= (int)( mon -= 2 ) )
	{
		mon		+= 12;
		year	-= 1;
	}
	return ( ( ( (unsigned long)( year / 4 - year / 100 + year / 400 + 367 * mon / 12 + day ) + year * 365 - 719499 ) * 24 + hour * 60 + min ) * 60 + sec );
}

/*
   压缩数据，将8bit转为7bit,并移位
   每到整分钟时，准备好数据  5Byte(yymmddhhmm)+60Byte(8bit Speed) +60Byte( Status)+10Byte(首个速度)=135 bytes
 */
static void compress_data( uint8_t *src, uint8_t *dst )
{
	uint8_t *psrc;
	uint8_t *pdst = dst;
	uint8_t c1, c2, c3, c4, c5, c6, c7, c8;
	uint8_t i;

/*头5个字节不需要了*/
	psrc = src + 5;

	for( i = 0; i < 7; i++ )
	{
		c1		= *psrc++;
		c2		= *psrc++;
		c3		= *psrc++;
		c4		= *psrc++;
		c5		= *psrc++;
		c6		= *psrc++;
		c7		= *psrc++;
		c8		= *psrc++;
		*pdst++ = c1 | ( ( c2 >> 7 ) & 0x01 );
		*pdst++ = ( c2 << 1 ) | ( ( c3 >> 6 ) & 0x03 );
		*pdst++ = ( c3 << 2 ) | ( ( c4 >> 5 ) & 0x07 );
		*pdst++ = ( c4 << 3 ) | ( ( c5 >> 4 ) & 0x0f );
		*pdst++ = ( c5 << 4 ) | ( ( c6 >> 3 ) & 0x1f );
		*pdst++ = ( c6 << 5 ) | ( ( c7 >> 2 ) & 0x3f );
		*pdst++ = ( c7 << 6 ) | ( ( c8 >> 1 ) & 0x7f );
	}

	for( i = 0; i < 74; i++ )
	{
		*pdst++ = *psrc++;
	}
}

/*
   解压缩数据，将7bit转为8bit
 */

static void decompress_data( uint8_t *src, uint8_t *dst )
{
	uint8_t *psrc	= src;
	uint8_t *pdst	= dst;
	uint8_t c1, c2, c3, c4, c5, c6, c7;
	uint8_t i;
/*字节就是7bit 速度压缩后的编码53byte*/
	for( i = 0; i < 7; i++ )
	{
		c1	= *psrc++;
		c2	= *psrc++;
		c3	= *psrc++;
		c4	= *psrc++;
		c5	= *psrc++;
		c6	= *psrc++;
		c7	= *psrc++;

		*pdst++ = c1 & 0xfe;
		*pdst++ = ( ( c1 & 0x01 ) << 7 ) | ( ( c2 >> 1 ) & 0x7e );
		*pdst++ = ( ( c2 & 0x03 ) << 6 ) | ( ( c3 >> 2 ) & 0x3e );
		*pdst++ = ( ( c3 & 0x07 ) << 5 ) | ( ( c4 >> 3 ) & 0x1e );
		*pdst++ = ( ( c4 & 0x0f ) << 4 ) | ( ( c5 >> 4 ) & 0x0e );
		*pdst++ = ( ( c5 & 0x1f ) << 3 ) | ( ( c6 >> 5 ) & 0x06 );
		*pdst++ = ( ( c6 & 0x3f ) << 2 ) | ( ( c7 >> 6 ) & 0x02 );
		*pdst++ = ( ( c7 & 0x7f ) << 1 );
	}
	for( i = 0; i < 74; i++ )
	{
		*pdst++ = *psrc++;
	}
}

#if 1


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
static void dump( uint8_t *info, uint16_t len )
{
	uint16_t	i, j = 0;
	uint8_t		*p = info;
	for( i = 0; i < len; i++ )
	{
		if( j == 0 )
		{
			rt_kprintf( "\r\n>" );
		}
		rt_kprintf( "%02x ", *p++ );
		j++;
		if( j == 16 )
		{
			j = 0;
		}
	}
}

/*
   初始化VDR存储区域，并定位到最大可写入的小时记录地址
   0:只是初始化
   1:格式化
 */
uint32_t vdr_init( uint8_t cmd )
{
	uint8_t		buf[128];
	uint32_t	addr;

	YMDH_TIME	i;
/*遍历所有的小时记录头*/
	for( addr = VDR_08H_09H_START; addr < VDR_08H_09H_END; addr += 8192 )
	{
		rt_kprintf( "\r\nVDR>addr=0x%08x", addr );
		SST25V_BufferRead( buf, addr, 16 );
		i = ( buf[0] << 24 ) | ( buf[1] << 16 ) | ( buf[2] << 8 ) | ( buf[3] );
		if( i == 0xFFFFFFFF )
		{
			continue; /*不是有效的数据头*/
		}
#ifdef STRICT_MATCH
		if( i > vdr_ymdh_curr_wr )
#else
		if( buf[3] > ( vdr_ymdh_curr_wr & 0xff ) )
#endif
		{
			vdr_addr_curr_wr	= addr;
			vdr_ymdh_curr_wr	= i;
		}
	}

	if( vdr_ymdh_curr_wr == 0 ) /*没找到*/
	{
		vdr_addr_curr_wr = VDR_08H_09H_START;
	}
	rt_kprintf( "\r\nvdr_init vdr_ymdh_curr_wr=%08x  vdr_addr_curr_wr=%08x", vdr_ymdh_curr_wr, vdr_addr_curr_wr );
	return 0;
}

FINSH_FUNCTION_EXPORT( vdr_init, init vdr );


/*
   写入1分钟的vdr数据 135字节
   有可能没有写入，但要空出空间,但是长时间没有数据如何处理或空出

   改成固定长度的访问(每小时8KB)

   传递进来的是
    yymmddhhmm	  (5byte)
    60组速度和状态  (120Byte)
    单位分钟位置信息 (10Byte) 取该分钟范围内首个有效的位置信息，否则为7FFFFFFFH；
 */
void vdr_write_minute( uint8_t *vdrinfo )
{
	uint8_t		buf[128];
	uint8_t		*p = vdrinfo;

	YMDH_TIME	h_tm;
	uint32_t	addr;
	uint8_t		year	= *( p + 0 );
	uint8_t		month	= *( p + 1 );
	uint8_t		day		= *( p + 2 );
	uint8_t		hour	= *( p + 3 );
	uint8_t		min		= *( p + 4 );

	h_tm = ( year << 24 ) | ( month << 16 ) | ( day << 8 ) | ( hour );

	if( h_tm != vdr_ymdh_curr_wr ) /*又过了一个小时*/
	{
		/*todo:要不要把剩下的内容写成缺省值*/

		vdr_ymdh_curr_wr	= h_tm;
		vdr_addr_curr_wr	+= 0x1FFF;
		vdr_addr_curr_wr	&= 0xFFFF2000;
		if( vdr_addr_curr_wr >= VDR_08H_09H_END )
		{
			vdr_addr_curr_wr = VDR_08H_09H_START;
		}
		/*清除随后的8k,用作存储,这里没有判首次上电，多搽除了一次，没有关系*/
		SST25V_SectorErase_4KByte( vdr_addr_curr_wr );
		SST25V_SectorErase_4KByte( vdr_addr_curr_wr + 4096 );
		/*写入当前的小时记录字段*/
		SST25V_strWrite( p, vdr_addr_curr_wr, 5 );
	}
	/*转换到分钟的存储位置,第一个128byte是小时时间戳头，后面才开始分钟内秒速度状态*/
	addr = vdr_addr_curr_wr + min * 128 + 128;
	compress_data( vdrinfo, buf );
	SST25V_strWrite( buf, addr, 128 );
}

/*
   减少一小时
 */
static YMDH_TIME vdr_ymdh_decrease( YMDH_TIME tm )
{
	uint8_t y, m, d, h;

	y	= tm >> 24; /*2000--2255年*/
	m	= tm >> 16;
	d	= tm >> 8;
	h	= tm & 0xff;

	if( h == 0 )
	{
		h = 23;
		if( d == 1 )
		{
			switch( m )
			{
				case 1: m = 12; y--; d = 31;
					break;
				case 3: m = 2; d = ( y % 4 == 0 ) ? 29 : 28;
					break;
				case 5:
				case 7:
				case 8:
				case 10:
				case 12: m--; d = 30;
					break;
				case 2:
				case 4:
				case 6:
				case 9:
				case 11:
					m--; d = 31;
					break;
			}
		}else
		{
			d--;
		}
	}else
	{
		h--;
	}
	return ( y << 24 ) | ( m << 16 ) | ( d << 8 ) | h;
}

/*获得单位分钟行驶速度*/
static uint8_t vdr_get_08h_rec( YMDH_TIME tm, uint8_t minute, uint8_t *pout )
{
	static YMDH_TIME	curr_08h_tm = 0; /*当前查找的时刻，判断要不要重新查找*/

	int					j, k;
	YMDH_TIME			tm_find;
	uint32_t			addr;
	uint8_t				buf[128], data[135];
	uint8_t				*p, *pdump;

	addr = vdr_cmd.addr;

	if( curr_08h_tm != tm ) /*重新查找*/
	{
		/*遍历记录区，找到特定的时间，都是整小时的,占用8k空间*/
		for( addr = VDR_08H_09H_START; addr < VDR_08H_09H_END; addr += 8192 )
		{
			SST25V_BufferRead( buf, addr, 16 );
			tm_find = ( buf[0] << 24 ) | ( buf[1] << 16 ) | ( buf[2] << 8 ) | buf[3];
#ifdef STRICT_MATCH
			if( tm_find == tm )
			{
				break;                  /*这是严格判断时间包括年月日*/
			}
#else
			if( ( tm_find & 0xFF ) == ( tm & 0xFF ) )
			{
				break;;                 /*这是不严格判断,只判断小时相等*/
			}
#endif
		}

		if( addr > VDR_08H_09H_END )    /*没有找到*/
		{
			rt_kprintf( "VDR>not find 08h_09h" );
			return 0;
		}else
		{
			rt_kprintf( "VDR>find 08h_09h at 0x%08x", addr );
			vdr_cmd.addr	= addr;
			curr_08h_tm		= tm;
		}
	}

	addr	= vdr_cmd.addr + 128; /*定位到开始的单位分钟内秒速度位置*/
	p		= pout;
	SST25V_BufferRead( buf, addr + ( 128 * minute ), 128 );
	rt_kprintf( "\r\nVDR>addr=%08x\r\n", addr );
	for( j = 0; j < 128; j++ )
	{
		rt_kprintf( "%02x ", buf[j] );
	}
	decompress_data( buf, data );

	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 24 ) );  /*year*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 16 ) );  /*month*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 8 ) );   /*day*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm ) );        /*hour*/
	*p++	= HEX_TO_BCD( minute );                 /*miniute*/
	*p++	= HEX_TO_BCD( 0 );                      /*sec*/

	for( j = 0; j < 60; j++ )
	{
		*p++	= data[j];
		*p++	= data[60 + j];
	}

#if 1  /*调试输出*/
	pdump = pout;
	rt_kprintf( "\r\nVDR>08H\r\n" );
	for( j = 0; j < 6; j++ )
	{
		rt_kprintf( "%02x ", *pdump++ );
	}
	for( j = 0; j < 6; j++ )
	{
		rt_kprintf( "\r\n>" );
		for( k = 0; k < 20; k++ )
		{
			rt_kprintf( "%02x ", *pdump++ );
		}
	}
#endif

	return 126;
}

/*
   由于会多次调用发送数据
   1.放在定时器任务或线程中定时检查
   2.在RECODER中标记。当收到特定的中心应答的流水号后，触发再次获得数据

   首次调用
   start和end 不为空

 */

uint16_t vdr_get_08h( void )
{
	uint8_t		* p, *pdump;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		buf[126];
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint16_t	blocks;

/*外面要调用*/
	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_curr;

	if( tm < vdr_cmd.ymdh_start ) /*判断是否大于开始时刻*/
	{
		rt_kprintf( "\r\nVDR>tm < vdr_cmd.ymdh_start" );
		return 0;
	}
	if( ( tm == vdr_cmd.ymdh_start ) && ( minute < vdr_cmd.minute_start ) )
	{
		return 0;
	}

	blocks		= vdr_cmd.blocks;
	vtr_tx_len	= 0;

	while( 1 )
	{
/*逆序输出，指定结束时间之前的最近1分钟*/
		if( minute == 0 ) /*要回到前一小时*/
		{
			minute	= 59;
			tm		= vdr_ymdh_decrease( tm );
		}else
		{
			minute--;
		}

		i = vdr_get_08h_rec( tm, minute, buf );
		if( i == 0 ) /**/
		{
			rt_kprintf( "\r\nVDR>not find rec" );
			break;
		}
		memcpy( vdr_tx_info + vtr_tx_len, buf, 126 );
		vtr_tx_len += 126;
		blocks--;
		if( blocks == 0 )
		{
			rt_kprintf( "\r\nVDR>blocks==0" );
			break;
		}
	}
	if( vtr_tx_len ) /*有数据要发送*/
	{
		rt_kprintf( "\r\nVDR>要发送08h数据(%d)bytes", vtr_tx_len );
	}
	return vtr_tx_len;
}

/*单位小时位置信息数据块*/
static uint16_t vdr_get_09h_rec( YMDH_TIME tm, uint8_t *pout )
{
	uint8_t		buf[128];
	uint8_t		data[130];

	uint8_t		*p, *pmsg;
	uint16_t	msg_count = 0;  /*当前分钟位置数据的个数*/
	uint32_t	i, j, k;
	uint32_t	addr;
	uint8_t		fFind = 0;      /*是否找到*/
	YMDH_TIME	tm_find;

	/*遍历记录区，找到特定的时间，都是整小时的,占用8k空间*/
	for( addr = VDR_08H_09H_START; addr < VDR_08H_09H_END; addr += 8192 )
	{
		SST25V_BufferRead( buf, addr, 16 );
		tm_find = ( buf[0] << 24 ) | ( buf[1] << 16 ) | ( buf[2] << 8 ) | buf[3];
#ifdef STRICT_MATCH
		if( tm_find == tm )
		{
			break;                  /*这是严格判断时间包括年月日*/
		}
#else
		if( ( tm_find & 0xFF ) == ( tm & 0xFF ) )
		{
			break;;                 /*这是不严格判断,只判断小时相等*/
		}
#endif
	}

	if( addr > VDR_08H_09H_END )    /*没有找到*/
	{
		rt_kprintf( "VDR>not find 09h" );
		return 0;
	}

	/*找到了，填充应答帧并发送*/
	p		= pout;
	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 24 ) );  /*year*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 16 ) );  /*month*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm >> 8 ) );   /*day*/
	*p++	= HEX_TO_BCD( (uint8_t)( tm ) );        /*hour*/
	*p++	= HEX_TO_BCD( 0 );                      /*miniute*/
	*p++	= HEX_TO_BCD( 0 );                      /*sec*/
	addr	+= 128;                                 /*第一个为记录头*/
	for( i = addr; i < addr + ( 60 * 128 ); i += 128 )
	{
		SST25V_BufferRead( buf, i, 128 );
		decompress_data( buf, data );
		for( j = 0; j < 60; j++ )
		{
			if( data[60 + j] & 0x01 )
			{
				break;
			}
		}
		if( j < 60 )
		{
			for( k = 0; k < 10; k++ )   /*位置信息*/
			{
				*p++ = data[120 + k];
			}
			*p++ = data[j];             /*速度*/
		}else
		{
			memcpy( p, "\x7F\xFF\xFF\xFF\x7F\xFF\xFF\xFF\x00\x00\x00", 11 );
			p += 11;
		}
	}
	rt_kprintf( "\r\nVDR_09H\r\n" );
	p = pout;
	for( i = 0; i < 6; i++ )
	{
		rt_kprintf( "%02x ", *p++ );
	}
	for( i = 0; i < 60; i++ )
	{
		rt_kprintf( "\r\n>" );
		for( j = 0; j < 11; j++ )
		{
			//rt_kprintf( "%02x ", vdr_tx_info[i * 11 + j + 6] );
			rt_kprintf( "%02x ", *p++ );
		}
	}
	return 666;
}

/*
   传递进参数
   参见GBT19056-2013  表A.15
   返回 要发送的数据长度
   填充Recoder_Obj结构体

 */
uint32_t vdr_get_09h( void )
{
	uint8_t		buf[666];
	YMDH_TIME	tm;

	tm = vdr_cmd.ymdh_curr;

	if( vdr_cmd.blocks_remain == 0 )
	{
		rt_kprintf( "\r\nVDR>09h no block to send" );
		return 0;
	}
	if( vdr_cmd.ymdh_curr < vdr_cmd.ymdh_start )
	{
		rt_kprintf( "\r\nVDR>ymdh_start reach" );
		return 0;
	}

	/*1次只发送1个block*/

	if( vdr_get_09h_rec( tm, buf ) )
	{
		vtr_tx_len = 0;
		memcpy( vdr_tx_info, buf, 666 );
		rt_kprintf( "\r\nVDR>send 09h" );
		tm					= vdr_ymdh_decrease( tm );
		vdr_cmd.ymdh_curr	= tm;
		vdr_cmd.blocks_remain--;
		return 666;
	}
	return 0;
}

/*
   事故疑点 Accident Point
 */

static uint8_t vdr_get_10h( void )
{
	uint8_t		* p, *pdump;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		buf[128];
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	blocks = vdr_cmd.blocks_remain;
	if( blocks == 0 )
	{
		return 0;
	}

	/*遍历记录区，找到特定的时间，都是整小时的,占用256空间*/
	p	= vdr_tx_info;
	i	= 0;
	for( addr = VDR_10H_START; addr <= VDR_10H_END; addr += 256 )
	{
		SST25V_BufferRead( buf, addr, 234 );

		while( blocks )
		{
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
static uint8_t vdr_get_11h( void )
{
	uint8_t		* p, *pdump;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		buf[128];
	uint8_t		data[130];
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_end;
	blocks	= vdr_cmd.blocks;

	/*遍历记录区，找到特定的时间，都是整小时的,占用256空间*/
	for( addr = VDR_11H_START; addr < VDR_11H_END; addr += 64 )
	{
		SST25V_BufferRead( buf, addr, 16 );
		tm_find = ( buf[0] << 24 ) | ( buf[1] << 16 ) | ( buf[2] << 8 ) | buf[3];
#ifdef STRICT_MATCH
		if( tm == tm_find )
		{
			return addr;        /*这是严格判断时间包括年月日*/
		}
#else
		if( ( tm & 0xFF ) == ( tm_find & 0xFF ) )
		{
			return addr;        /*这是不严格判断,只判断小时相等*/
		}
#endif
	}

	if( addr > VDR_11H_END )    /*没有找到*/
	{
		rt_kprintf( "VDR>not find 11h" );
		return 0;
	}
}

/*
   外部供电记录
   都存在4k的记录里，1次读出
 */
static uint8_t vdr_get_12h( void )
{
	uint8_t		* p, *pbuf;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_end;
	blocks	= vdr_cmd.blocks;

	pbuf = rt_malloc( 4096 );
	if( pbuf == RT_NULL )
	{
		rt_kprintf( "\r\nVDR>%s malloc error", __func__ );
		return 0;
	}

	SST25V_BufferRead( pbuf, VDR_12H_START, 4096 );

	for( i = 0; i < 100; i++ )  /*100条 32byte*/
	{
		tm_find = ( *( pbuf + i * 32 ) << 24 ) | ( *( pbuf + i * 32 + 1 ) << 16 ) | ( *( pbuf + i * 32 + 2 ) << 8 ) | *( pbuf + i * 32 + 3 );
#ifdef STRICT_MATCH
		if( tm == tm_find )
		{
			break;;             /*这是严格判断时间包括年月日*/
		}
#else
		if( ( tm & 0xFF ) == ( tm_find & 0xFF ) )
		{
			break;              /*这是不严格判断,只判断小时相等*/
		}
#endif
	}

	if( addr > VDR_12H_END )    /*没有找到*/
	{
		rt_kprintf( "VDR>not find 12h" );
		return 0;
	}

	rt_free( pbuf );
}

/*都存在4k的记录里，1次读出*/

static uint8_t vdr_get_13h( void )
{
	uint8_t		* p, *pbuf;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_end;
	blocks	= vdr_cmd.blocks_remain;

	pbuf = rt_malloc( 4096 );
	if( pbuf == RT_NULL )
	{
		rt_kprintf( "\r\nVDR>%s malloc error", __func__ );
		return 0;
	}

	SST25V_BufferRead( pbuf, VDR_12H_START, 4096 );

	for( i = 0; i < 100; i++ ) /*100条 32byte*/
	{
		tm_find = ( *( pbuf + i * 32 ) << 24 ) | ( *( pbuf + i * 32 + 1 ) << 16 ) | ( *( pbuf + i * 32 + 2 ) << 8 ) | *( pbuf + i * 32 + 3 );
	}

	rt_free( pbuf );
}

/*都存在4k的记录里，1次读出*/

static uint8_t vdr_get_14h( void )
{
	uint8_t		*pbuf;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_end;
	blocks	= vdr_cmd.blocks;

	pbuf = rt_malloc( 4096 );
	if( pbuf == RT_NULL )
	{
		rt_kprintf( "\r\nVDR>%s malloc error", __func__ );
		return 0;
	}

	SST25V_BufferRead( pbuf, VDR_12H_START, 4096 );

	for( i = 0; i < 100; i++ )  /*100条 32byte*/
	{
		tm_find = ( *( pbuf + i * 32 ) << 24 ) | ( *( pbuf + i * 32 + 1 ) << 16 ) | ( *( pbuf + i * 32 + 2 ) << 8 ) | *( pbuf + i * 32 + 3 );
#ifdef STRICT_MATCH
		if( tm == tm_find )
		{
			break;;             /*这是严格判断时间包括年月日*/
		}
#else
		if( ( tm & 0xFF ) == ( tm_find & 0xFF ) )
		{
			break;              /*这是不严格判断,只判断小时相等*/
		}
#endif
	}

	if( addr > VDR_12H_END )    /*没有找到*/
	{
		rt_kprintf( "VDR>not find 12h" );
		return 0;
	}

	rt_free( pbuf );
}

/*都存在4k的记录里，1次读出*/

static uint8_t vdr_get_15h( void )
{
	uint8_t		* p, *pbuf;
	uint32_t	addr;
	int			i, j, k;
	uint8_t		minute;
	YMDH_TIME	tm, tm_find;
	uint32_t	blocks;

	tm		= vdr_cmd.ymdh_curr;
	minute	= vdr_cmd.minute_end;
	blocks	= vdr_cmd.blocks;

	pbuf = rt_malloc( 4096 );
	if( pbuf == RT_NULL )
	{
		rt_kprintf( "\r\nVDR>%s malloc error", __func__ );
		return 0;
	}

	SST25V_BufferRead( pbuf, VDR_12H_START, 4096 );

	rt_free( pbuf );
}

/*
   接收vdr命令,天津平台看到有粘包的情况
   通过 Recoder_obj 返回
 */
uint8_t vdr_rx( uint8_t * info, uint16_t count )
{
	uint8_t		*p = info;
	uint8_t		cmd;
	uint16_t	len;
	uint8_t		dummy;
	uint16_t	blocks;
	uint8_t		i, j, k;

	rt_kprintf( "\r\n vdr_rx %d bytes\r\n", count );
	for( i = 0; i < count; i++ )
	{
		rt_kprintf( "%02x ", *p++ );
	}

	if( *p++ != 0xAA )
	{
		return 1;
	}
	if( *p++ != 0x75 )
	{
		return 2;
	}
	cmd		= *p++;
	len		= ( *p++ << 8 );
	len		|= *p++;
	dummy	= *p++;

	vdr_cmd.ymdh_start = 0;
	for( k = 0; k < 4; k++ )
	{
		i					= *p++;
		j					= ( i >> 4 ) * 10 + ( i & 0x0f );
		vdr_cmd.ymdh_start	<<= 8;
	}
	p					+= 2;
	vdr_cmd.ymdh_end	= 0;
	for( k = 0; k < 4; k++ )
	{
		i					= *p++;
		j					= ( i >> 4 ) * 10 + ( i & 0x0f );
		vdr_cmd.ymdh_end	<<= 8;
	}
	p				+= 2;
	blocks			= ( *p++ << 8 );
	blocks			|= *p;
	vdr_cmd.blocks	= blocks;

	rt_kprintf( "\r\nVDR> rx start:%08x end:%08x block:%d", vdr_cmd.ymdh_start, vdr_cmd.ymdh_end, vdr_cmd.blocks );

	return 0;
}

/*更新行车记录仪数据*/
void thread_usb_vdr( void* parameter )
{
	int			i, res;
	int			count = 0;
	uint8_t		buf[520];
	uint32_t	addr;
	int			fd = -1;
	uint8_t		*p;
/*查找U盘*/
	if( rt_device_find( "udisk" ) == RT_NULL ) /*没有找到*/
	{
		rt_kprintf( "\r\nVDR>没有UDISK" );
		return;
	}

/*查找指定文件*/

	p		= (uint8_t*)parameter;
	addr	= *p++ << 24;
	addr	|= *p++ << 16;
	addr	|= *p++ << 8;
	addr	|= *p++;
	rt_kprintf( "\r\nusb write to %08x", addr );
	strcpy( (char*)buf, "/udisk/" );
	strcat( (char*)buf, p );


/*
   addr=0x00300000;
   strcpy(buf,"/udisk/vdr.bin");
 */
	fd = open( buf, O_RDONLY, 0 );
	if( fd >= 0 )
	{
		rt_kprintf( "\r\nVDR>读取文件" );
	} else
	{
		rt_kprintf( "\r\nVDR>文件不存在" );
		return;
	}
/*写入*/

	while( 1 )
	{
		WatchDog_Feed( );
		rt_thread_delay( RT_TICK_PER_SECOND );
		res = read( fd, buf, 512 );
		if( res < 0 )
		{
			rt_kprintf( "\r\nVDR>读取文件出错，res=%x", res );
			goto end_upgrade_usb_1;
		}else
		{
			if( res == 0 )
			{
				rt_kprintf( "\r\nVDR>写入完成 %d byte", count );
				goto end_upgrade_usb_1;
			}
			/*写入数据*/
			count += res;
			if( ( addr & 0x0FFF ) == 0 )
			{
				SST25V_SectorErase_4KByte( addr );
				//sst25_erase_4k
			}

			for( i = 0; i < 512; i++ )
			{
				SST25V_ByteWrite( buf[i], addr );
				addr++;
			}

			//sst25_write_through( add, uint8_t *p, uint16_t len)
			rt_kprintf( "\r\nVDR>写入 %d byte", count );
			if( res < 512 )
			{
				rt_kprintf( "\r\nVDR>写入完成 %d byte", count );
				goto end_upgrade_usb_1;
			}
		}
	}
end_upgrade_usb_1:
	if( fd >= 0 )
	{
		close( fd );
	}
}

#endif


/*
   调试vdr

   vdr_cmd(0x09,"12:34:56","13:11:03",1)

 */
uint8_t vdr_start( uint8_t cmd, char *start, char *end, uint16_t blocks )
{
	int h, m, s;
	fvdr_debug = 1;

	sscanf( start, "%d:%d:%d", &h, &m, &s );
	vdr_cmd.ymdh_start		= h;
	vdr_cmd.minute_start	= m;

	sscanf( end, "%d:%d:%d", &h, &m, &s );
	vdr_cmd.ymdh_end	= h;
	vdr_cmd.minute_end	= m;

	vdr_cmd.blocks			= blocks;
	vdr_cmd.blocks_remain	= blocks;

	vdr_cmd.ymdh_curr = vdr_cmd.ymdh_end;

	vdr_cmd.cmd = cmd;

	switch( cmd )
	{
		case 0x08: vdr_cmd.addr = 0x00300000; vdr_get_08h( ); break;
		case 0x09: vdr_get_09h( ); break;
		case 0x10: vdr_get_10h( ); break;
		case 0x11: vdr_get_11h( ); break;
		case 0x12: vdr_get_12h( ); break;
		case 0x13: vdr_get_13h( ); break;
		case 0x14: vdr_get_14h( ); break;
		case 0x15: vdr_get_15h( ); break;
	}
}

FINSH_FUNCTION_EXPORT( vdr_start, debug vdr );


/***********************************************************
* Function:
* Description:
* Input:
* Input:
* Output:
* Return:
* Others:
***********************************************************/
void vdr_import( uint32_t addr, char* filename )
{
	file_rec[0] = addr >> 24;
	file_rec[1] = addr >> 16;
	file_rec[2] = addr >> 8;
	file_rec[3] = addr & 0xFF;
	strcpy( file_rec + 4, filename );

	tid_usb_vdr = rt_thread_create( "usb_vdr", thread_usb_vdr, (void*)&file_rec, 1024, 9, 5 );
	if( tid_usb_vdr != RT_NULL )
	{
		rt_kprintf( "线程创建成功" );
		rt_thread_startup( tid_usb_vdr );
	}else
	{
		rt_kprintf( "线程创建失败" );
	}
}

FINSH_FUNCTION_EXPORT( vdr_import, import vdr );

//#define DBG_VDR  

#ifdef DBG_VDR
static uint8_t testbuf[1000];
#endif


/*
   行驶速度记录
   不管他，自己填数据，自己报
   单条126字节每分钟
   48*60=2880包

   每5条 组一包  630字节
   每6条 组一包  756字节
 */

uint8_t get_08h( uint8_t *pout,u16 packet_in )
{   //  2014 年 2 月份
	static uint8_t	month_08 = 2, day_08_const = 27; // 起始日期
	u32  minute_total=0;
	uint8_t day_08, hour_08 = 0, min_08 = 0;
	static uint32_t addr_08		= VDR_08H_09H_START;
	uint32_t count_08	= 0;
	uint8_t			buf[128], data[135];
	uint8_t			*p = RT_NULL, *pdump;
	int				i, j, k;
#ifdef DBG_VDR
	p = testbuf;
#else
	p = pout;
#endif
    // 根据输入的包序号获取相应的信息
     // 1. 将日期转换为分钟
     minute_total=day_08_const*24*60-(packet_in-1)*4;  //      2-28  00:00 

	 min_08=minute_total%60;// 分钟
	 day_08=minute_total/1440;//  天 每天1440 分钟
	 hour_08=(minute_total-day_08*1440)/60; // 小时
	 

    count_08=(packet_in-1)*4; // 每组起始

	for( i = 0; i < 4; i++ )//5
	{
		WatchDog_Feed();
		addr_08 = 0x00300000 + hour_08 * 8192 + min_08 * 128 + 128;
		if( addr_08 >= VDR_08H_09H_END )
		{
			addr_08 = VDR_08H_09H_START + 128;
		}

		SST25V_BufferRead( buf, addr_08, 128 );
		decompress_data( buf, data );
		buf[0]	= HEX_TO_BCD( 14 );         /*year*/
		buf[1]	= HEX_TO_BCD( month_08 );   /*month*/
		buf[2]	= HEX_TO_BCD( day_08 );     /*day*/
		buf[3]	= HEX_TO_BCD( hour_08 );    /*hour*/
		buf[4]	= HEX_TO_BCD( min_08 );     /*miniute*/
		buf[5]	= HEX_TO_BCD( 0 );          /*sec*/

		for( j = 0; j < 60; j++ )
		{
			buf[j * 2 + 6]	= data[j];
			buf[j * 2 + 7]	= data[60 + j];
		}
		memcpy( pout + i * 126, buf, 126 ); 
		
		count_08++;

		
		//rt_kprintf( "\r\nVDR>08H(%d) 14-%02d-%02d %02d:%02d \r\n", count_08, month_08, day_08, hour_08, min_08 );
		if( min_08 == 0 )
		{
			min_08 = 59;
			if( hour_08 == 0 )
			{
				day_08--;
				hour_08 = 24;
			}
			hour_08--;
		}else
		{
			min_08--;
		}

		/*调试输出*/
		pdump = buf;
#ifdef  DBG_VDR
		for( j = 0; j < 6; j++ )
		{
			rt_kprintf( "%02x ", *pdump++ );
		}
		for( j = 0; j < 6; j++ )
		{
			rt_kprintf( "\r\n>" );
			for( k = 0; k < 20; k++ )
			{
				rt_kprintf( "%02x ", *pdump++ );
			}
		}
#endif
		//addr_08 += 128;
	}
	return 126 * 4;//6
}

FINSH_FUNCTION_EXPORT( get_08h, get_08 );


/*
   位置信息
   360小时   ，每小时666字节
 */

uint8_t get_09h( uint8_t *pout,u16 packet_in )
{   //  2014 年 2 月份
    static uint8_t	month_09 = 2, day_09_const = 27, hour_09_const = 23;  //true 
    uint8_t day_09 = 0, hour_09 =0;  //true 
	//static uint8_t	month_09 = 4, day_09 =18, hour_09 = 6;//half change
    u32  hour_total=0;
	static uint32_t addr_09		= VDR_08H_09H_START;
	static uint32_t count_09	= 0;
	uint8_t			buf[128], data[135];
	uint8_t			*p = RT_NULL;
	int				i, j, k;
	 //  39.906678 =           116.192092   +
	u32 lati_ini=0x16D5B47,longi_ini=0x424C537;  
	u32 lati=0,longi=0;
	u8  count=0; 
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

     
	 count_09=packet_in; // 每组起始 
   //  将初始日期转换成小时
     hour_total=day_09_const*24+hour_09_const-count_09; 
 
    day_09=hour_total/24;
	hour_09=hour_total%24; 
	
  
	addr_09 = 0x00300000 + hour_09 * 8192 + 128;                /*定位到小时的第一分钟数据*/
	if( addr_09 >= VDR_08H_09H_END )
	{
		addr_09 = VDR_08H_09H_START + 128;
	}

	*p++	= HEX_TO_BCD( 14 );                                 /*year*/
	*p++	= HEX_TO_BCD( month_09 );                           /*month*/
	*p++	= HEX_TO_BCD( day_09 );                             /*day*/
	*p++	= HEX_TO_BCD( hour_09 );                            /*hour*/
	*p++	= HEX_TO_BCD( 0 );                                  /*miniute*/
	*p++	= HEX_TO_BCD( 0 );                                  /*sec*/

    count=0;
	for( i = addr_09; i < addr_09 + ( 60 * 128 ); i += 128 )    /*读出60个分钟数据*/
	{
		WatchDog_Feed();
		SST25V_BufferRead( buf, i, 128 );
		decompress_data( buf, data );
		for( j = 0; j < 60; j++ )
		{
			if( data[60 + j] & 0x01 )                           /*看是不是有速度*/
			{
				break;
			}
		}
		if( j < 60 )
		{
		      /*位置信息*/
		    /*Note: 市界的地理坐标为：北纬39”26’至41”03’，东经115”25’至 117”30’。
		       	 115”25’=0x0420ABD0                                    117”30’=0x0433BEA0
                             39度26分=2366分=0x016905E0                               41”03’=0x0177D2F0
                             高度30-40 米

                               1度 <=>  111 公里     0.0001 分 =0.18 米         每秒8个单位是1.44米 即速度是5.2KM/H 
                                                                                                             每分钟行驶 1.46x60=90米==500   每分钟增加500
                                                                                                             longi 加 lati不动
                     
                             这里经纬做一下替换 海拔随机
                           note2:    10个字节的组成   
                                           4 个字节  经度
                                           4 个字节纬度
                                           2 个字节海拔
			  */
		  #if 0	    // mask  is  before
			for( k = 0; k < 10; k++ )                        
			{
	

			     *p++ = data[120 + k]; 
			  
			}
          #endif  
             

		  
		  #if 1
		     //  longitude
		    if(packet_in%2)
		     { 
		        longi=longi_ini+count*6000;  // 每分钟走100米<=>    500 个单位
		        
				lati=lati_ini+count*6000;// lati 
		     }
		    else
			 {
			    longi=longi_ini+(62-count)*6000;   
				lati=lati_ini+(62-count)*6000;  // lati       
		     }
		     
             *p++ = (u8)(longi>>24);
		     *p++ = (u8)(longi>>16);
			 *p++ =(u8)(longi>>8);
		     *p++ =(u8)(longi); 
			   //  lati
             *p++ = (u8)(lati>>24);  
		     *p++ = (u8)(lati>>16);
			 *p++ = (u8)(lati>>8); 
		     *p++ = (u8)(lati);
		        //  Height
             *p++ =0x00;
		     *p++ =30+(rt_tick_get()%5); 
		   #endif	 
			 //---------------------------------------------------------------------------- 
			*p++ =5+(rt_tick_get()%10);// data[j];                                     /*速度*/
		}
		else
		{
			memcpy( p, "\x7F\xFF\xFF\xFF\x7F\xFF\xFF\xFF\x00\x00\x00", 11 );
			p += 11;
		}
		count++;
	}
	rt_kprintf( "\r\nVDR>09H(%d) 14-%02d-%02d %02d \r\n", count_09, month_09, day_09, hour_09 );

	if( hour_09 == 0 )
	{
		day_09--;
		hour_09 = 24;
	}
	hour_09--;

#ifdef  DBG_VDR

	p = pout;
	for( i = 0; i < 6; i++ )
	{
		rt_kprintf( "%02x ", *p++ );
	}
	for( i = 0; i < 60; i++ )
	{
		rt_kprintf( "\r\n>" );
		for( j = 0; j < 11; j++ )
		{
			rt_kprintf( "%02x ", *p++ );
		}
	}
#endif
	return 666;
}

FINSH_FUNCTION_EXPORT( get_09h, get_09 );


/*
   事故疑点
   234Byte
   共100个
 */
uint8_t get_10h( uint8_t *pout )
{
	static uint32_t addr_10 = VDR_10H_START;
	uint8_t			buf[240];
	uint8_t			*p;
	uint32_t		i;
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

	if( addr_10 > VDR_10H_END )
	{
		addr_10 = VDR_10H_START;
	}
		WatchDog_Feed();
	SST25V_BufferRead( buf, addr_10, 234 );
	//-------------------------------------------------- 
	// 年月替换一下	 14 年2 月份
	buf[0]=0x14;  
	buf[1]=0x03;  

    //  状态线有变化
    for(i=0;i<80;i=i+2)
       buf[25+i]= buf[25+i]&(rt_tick_get()%7);   



	// 替换经纬度
	buf[224]=0x04;  // longi
	buf[225]=0x24+(rt_tick_get()%6); 
	buf[226]=buf[226]+(rt_tick_get()%3)*3;

	buf[228]=0x01;  //lati
	buf[229]=0x6D+rt_tick_get()%3;  
	buf[230]=buf[230]+3*(rt_tick_get()%5); 

	buf[232]=0x00;  // height
	buf[233]=30+(rt_tick_get()%5);  
	//--------------------------------------------------
	memcpy( p, buf, 234 );
	addr_10 += 256;

#ifdef  DBG_VDR
	rt_kprintf( "\r\nVDR>10H" ); 
	p = pout;
	for( i = 0; i < 234; i++ )
	{
		if( i % 8 == 0 )
		{
			rt_kprintf( "\r\n" );
		}
		rt_kprintf( "%02x ", *p++ );
	}
#endif
	return 234;
}

FINSH_FUNCTION_EXPORT( get_10h, get_10 );


/*超时驾驶记录
   50Bytes   100条
 */

uint8_t get_11h( uint8_t *pout,u16 packet_in ) 
{
	static uint8_t	month_11 = 2, day_11_const = 27; // 起始日期	
	u32  minute_total=0;
	static uint32_t addr_11 = VDR_11H_START;
	uint8_t			buf[64];
	uint8_t			*p = RT_NULL;
	uint32_t		i, j;	
	uint8_t day_11, hour_11 = 0, min_11 = 0;
	u8  value_reg=0;


#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

	if( addr_11 > VDR_11H_END )
	{
		addr_11 = VDR_11H_START;
	}
		WatchDog_Feed();
	for( j = 0; j < 1; j++ )
	{
		SST25V_BufferRead( buf, addr_11, 50 );
		//-------------------------------------------------- 
	// 年月替换一下	 14 年2 月份
	/*
	        

             
	*/
	//开始时间年月
	buf[18]=0x14;   
	buf[19]=0x02; //month

	minute_total=day_11_const*24*60-(packet_in)*241-packet_in*30;  //	  2-28	00:00   241+30= 

	min_11=minute_total%60;// 分钟
	day_11=minute_total/1440;//  天 每天1440 分钟
	hour_11=(minute_total-day_11*1440)/60; // 小时

	buf[20]= HEX_TO_BCD(day_11);// day
	buf[21]= HEX_TO_BCD(hour_11);// hour
	buf[22]= HEX_TO_BCD(min_11);// min

	//                         
    // 结束时间年月
	buf[24]=0x14;  
    buf[25]=0x02;
    
	minute_total=minute_total+241; 

	
	min_11=minute_total%60;// 分钟
	day_11=minute_total/1440;//  天 每天1440 分钟
	hour_11=(minute_total-day_11*1440)/60; // 小时

	buf[26]= HEX_TO_BCD(day_11);// day
	buf[27]= HEX_TO_BCD(hour_11);// hour
	buf[28]= HEX_TO_BCD(min_11);// min

	// 开始longi
    buf[30]=0x04;  
	buf[31]=0x25+(rt_tick_get()%2);   

	// 开始 laiti
    buf[34]=0x01;  
	buf[35]=0x6D-(rt_tick_get()%3); 

	//  开始高程
    buf[38]=0x00;
    buf[39]=30+(rt_tick_get()%5); 

	// 结束longi
    buf[40]=0x04;  
	buf[41]=0x27; 

	// 结束 laiti
	buf[44]=0x01;  
    buf[45]=0x6D;

	//  结束高程
	buf[48]=0x00;
    buf[49]=30+(rt_tick_get()%5); 
	
	//--------------------------------------------------
		for(i=0;i<50;i++)		/*平台不认转义。SHIT....*/
		{
			if(buf[i]==0x7d) buf[i]=0x7C;
			if(buf[i]==0x7E) buf[i]=0x7F;
		}
		memcpy( p + j * 50, buf, 50 );
		addr_11 += 64;
	}

#ifdef  DBG_VDR
	rt_kprintf( "\r\nVDR>11H @0x%08x", addr_11 );
	p = pout;
	for( i = 0; i < 500; i++ )
	{
		//WatchDog_Feed( );
		if( i % 8 == 0 )
		{
			rt_kprintf( "\r\n" );
		}
		rt_kprintf( "%02x ", *p++ );
	}
#endif
	return 50;
}

FINSH_FUNCTION_EXPORT( get_11h, get_11 );


/*驾驶员身份登录
   25Bytes   200条
 */

uint8_t get_12h( uint8_t *pout, u16 packet_in ) 
{
    
	static uint8_t	month_11 = 2, day_11_const = 26; // 起始日期		
	u32  minute_total=0,reg_min=0;
	static uint32_t addr_12 = VDR_12H_START;
	uint8_t			buf[26];
	uint8_t			*p;
	uint32_t		i, j; 
	
	uint8_t day_12, hour_12 = 0, min_12 = 0;
	u8  value_reg=0;
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

	if( addr_12 > VDR_12H_END )
	{
		addr_12 = VDR_12H_START;
	}

    reg_min=day_11_const*24*60-(packet_in-1)*2401; 
	for( j = 0; j < 20; j++ )
	{
		WatchDog_Feed();
		SST25V_BufferRead( buf, addr_12, 25 );
		//--------------------------------------------------
	// 年月替换一下	 14 年2 月份

	buf[0]=0x14;   
	buf[1]=0x02; //month

	minute_total=reg_min-(j)*120;  //	  每2小时一条   

	min_12=minute_total%60;// 分钟
	day_12=minute_total/1440;//  天 每天1440 分钟
	hour_12=(minute_total-day_12*1440)/60; // 小时 

	buf[2]= HEX_TO_BCD(day_12);// day
	buf[3]= HEX_TO_BCD(hour_12);// hour
	buf[4]= HEX_TO_BCD(min_12);// min
	//--------------------------------------------------
		memcpy( p + j * 25, buf, 25 );
		addr_12 += 32;
	}
#ifdef DBG_VDR
	rt_kprintf( "\r\nVDR>12H @0x%08x", addr_12 );
	p = pout;
	for( i = 0; i < 500; i++ )
	{
		if( i % 8 == 0 )
		{
			rt_kprintf( "\r\n" );
		}
		rt_kprintf( "%02x ", *p++ );
	}
#endif
	return 500;
}

FINSH_FUNCTION_EXPORT( get_12h, get_12 );


/*外部供电记录
   7字节 100条**/
uint8_t get_13h( uint8_t *pout )
{
	static uint32_t addr_13 = VDR_13H_START;
	uint8_t			buf[8];
	uint8_t			*p;
	uint32_t		i, j;
	u8  day=0;
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

	if( addr_13 > VDR_13H_END )
	{
		addr_13 = VDR_13H_START;
	}
	for( j = 0; j < 100; j++ )
	{
		WatchDog_Feed();
		SST25V_BufferRead( buf, addr_13, 8 );
		//--------------------------------------------------
	// 年月替换一下	 14 年2 月份
	buf[0]=0x14;  
	if(j<13)	
	  buf[1]=0x03;
	else
	{ 
	    buf[1]=0x02;	 
	 	day=(buf[2]>>4)*10+(buf[2]&0x0F); 
	 	if(day>=5)
	 		day=day-2;
	 	buf[2]=HEX_TO_BCD(day);
	}
	//--------------------------------------------------
		memcpy( p + j * 7, buf, 7 );

		addr_13 += 8;
	}
#if 0
	rt_kprintf( "\r\nVDR>13H @0x%08x\r\n", addr_13 );
	p = pout;
	for( i = 0; i < 700; i++ )
	{
		if( i % 7 == 0 )
		{
			rt_kprintf( "\r\n" );
		}
		rt_kprintf( "%02x ", *p++ );
	}
#endif

	return 700;
}

FINSH_FUNCTION_EXPORT( get_13h, get_13 );


/*记录仪参数
   7字节 100条*/
uint8_t get_14h( uint8_t *pout )
{
	static uint32_t addr_14 = VDR_14H_START;
	uint8_t			buf[8];
	uint8_t			*p;
	uint32_t		i, j;
	u8 day=0;
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;
	if( addr_14 > VDR_14H_END )
	{
		addr_14 = VDR_14H_START;
	}
	for( j = 0; j < 100; j++ )
	{
		WatchDog_Feed();
		SST25V_BufferRead( buf, addr_14, 8 );
		//--------------------------------------------------
	// 年月替换一下	 14 年2 月份
	buf[0]=0x14;  
	buf[1]=0x02; 
	day=(buf[2]>>4)*10+(buf[2]&0x0F);
	day=day+12;
	buf[2]=HEX_TO_BCD(day); 
	//--------------------------------------------------
		memcpy( p + j * 7, buf, 7 );
		addr_14 += 8;
	}

#ifdef  DBG_VDR
	rt_kprintf( "\r\nVDR>14H @0x%08x\r\n", addr_14 );
	p = pout;
	for( i = 0; i < 700; i++ )
	{
		if( i % 7 == 0 )
		{
			rt_kprintf( "\r\n" );
		}
		rt_kprintf( "%02x ", *p++ );
	}
#endif
	return 700;
}

FINSH_FUNCTION_EXPORT( get_14h, get_14 );


/*
   速度状态日志
   133Byte
   共10个
 */
uint8_t get_15h( uint8_t *pout ,u16 packet_in ) 
{
	static uint32_t addr_15 = VDR_15H_START;
	uint8_t			buf[140];
	uint8_t			*p;
	uint32_t		i, j;
	u8  value_reg=0;
	u8  day=0;   
#ifdef DBG_VDR
	pout = testbuf;
#endif
	p = pout;

	if( addr_15 > VDR_15H_END )
	{
		addr_15 = VDR_15H_START;
	}
		WatchDog_Feed();
	for( i = 0; i < 1; i++ )
	{
		SST25V_BufferRead( buf, addr_15, 134 );
		//--------------------------------------------------
	// 年月替换一下	 14 年2 月份
	if(packet_in%2) 
	    buf[0]=0x02; // 异常 情况下是5  分钟  
	    
	    
	buf[1]=0x14;   // 开始时间
   if(packet_in<=4)	
	  buf[2]=0x03;
	else
	{ 
	    buf[2]=0x02;	 
	 	day=(buf[3]>>4)*10+(buf[3]&0x0F); 
	 	if(day>=5)
	 		day=day-2;
	 	buf[3]=HEX_TO_BCD(day);
	}
	
     // 判断分钟
        value_reg=((buf[5]>>4)*10)+(buf[5]&0x0F);
	    if(value_reg>=55)
		{
		   value_reg=value_reg-5;
		   buf[5]=((value_reg/10)<<4)+(value_reg%10);  
	    }

	buf[7]=0x14;   // 结束时间
	buf[8]=buf[2];  
	buf[9]=buf[3]; 

    value_reg=value_reg+5;
	buf[11]=((value_reg/10)<<4)+(value_reg%10);// 5 分钟  
	//--------------------------------------------------
     value_reg=13; // 第一个速度字段

	 for(j=0;j<60;j++)  // 速度要大于40
	 	{
	 	  if(buf[value_reg]<40)		 	
	 	     buf[value_reg]=40+(rt_tick_get()%4);     
		  value_reg++;
		  buf[value_reg++]=40;
	 	}
     //  
	
		memcpy( p + i * 133, buf, 133 );

#ifdef  DBG_VDR
		rt_kprintf( "\r\nVDR>15H @0x%08x", addr_15 );
		p = pout;
		for( i = 0; i < 133; i++ )
		{
			if( i % 8 == 0 )
			{
				rt_kprintf( "\r\n" );
			}
			rt_kprintf( "%02x ", *p++ );
		}
#endif
		addr_15 += 256;
	}
	return 133;
//	return 133 * 5;
}

FINSH_FUNCTION_EXPORT( get_15h, get_15 ); 

/************************************** The End Of File **************************************/
