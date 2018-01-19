/****************************************Copyright (c)****************************************************
**               Copyright � 2003~2009 Shenzhen uCdragon Technology Co.,Ltd. All Rights Reserved 
**
**                                 http://www.ucdragon.cn
**
**      �����������Ƽ����޹�˾���ṩ�����з�������ּ��Э���ͻ����ٲ�Ʒ���з����ȣ��ڷ�����������ṩ
**  ���κγ����ĵ������Խ����������֧�ֵ����Ϻ���Ϣ���������ο����ͻ���Ȩ��ʹ�û����вο��޸ģ�����˾��
**  �ṩ�κε������ԡ��ɿ��Եȱ�֤�����ڿͻ�ʹ�ù��������κ�ԭ����ɵ��ر�ġ�żȻ�Ļ��ӵ���ʧ������˾��
**  �е��κ����Ρ�
**                                                                        �������������Ƽ����޹�˾
********************************************************************************************************/
#include "w25x32.h"

#define     SSEL_EN         (0 << 16)
#define     SSEL_DIS        (1 << 16)
#define     EOT_EN          (1 << 20)
#define     EOT_DIS         (0 << 20)
#define     EOF_EN          (1 << 21)
#define     EOF_DIS         (0 << 21)
#define     RXIGNORE_EN     (1 << 22)
#define     RXIGNORE_DIS    (0 << 22)
#define     FLEN(n)         (((n) - 1) << 24)


/*********************************************************************************************************
** ��������: SendRecv_Start
** �������ܣ�һ�δ�����ʼ�ĵ��ֽ�֡���������
** �������: �������Ϸ��ͳ�������
** �������: �������Ͻ��յ�������
** �� �� ֵ����
*********************************************************************************************************/
uint8_t SendRecv_Start (uint8_t ucData)
{
    while (!(LPC_SPI1->STAT & (1 << 1)));                               /* �ȴ�����׼������             */
    LPC_SPI1->TXDATCTL = FLEN(8) | EOF_EN | SSEL_EN | ucData;           /* 8 λ�����俪ʼ��֡����       */    
    
    while (!(LPC_SPI1->STAT & (1 << 0)));                               /* �ȴ������������             */
    ucData = LPC_SPI1->RXDAT;                                           /* ��������                     */
    
    return ucData;
}

/*********************************************************************************************************
** ��������: SendRecv_Stop
** �������ܣ�һ�δ�������ĵ��ֽ�֡���������
** �������: �������Ϸ��ͳ�������
** �������: �������Ͻ��յ�������
** �� �� ֵ����
*********************************************************************************************************/
uint8_t SendRecv_Stop (uint8_t ucData)
{
    while (!(LPC_SPI1->STAT & (1 << 1)));                               /* �ȴ�����׼������             */
    LPC_SPI1->TXDATCTL = FLEN(8) | EOT_EN | ucData;                     /* 8 λ���������               */    
    
    while (!(LPC_SPI1->STAT & (1 << 0)));                               /* �ȴ������������             */
    ucData = LPC_SPI1->RXDAT;                                           /* ��������                     */
    
    return ucData;
}
/*********************************************************************************************************
** ��������: SendRecv_Byte
** �������ܣ�һ�δ����ڲ��ĵ��ֽ�֡���������
** �������: �������Ϸ��ͳ�������
** �������: �������Ͻ��յ�������
** �� �� ֵ����
*********************************************************************************************************/
uint8_t Send_Byte (uint8_t ucData)
{
    while (!(LPC_SPI1->STAT & (1 << 1)));                               /* �ȴ�����׼������             */
    LPC_SPI1->TXDATCTL = FLEN(8) | EOF_EN | ucData;                     /* 8 λ��֡����                 */    
    
    while (!(LPC_SPI1->STAT & (1 << 0)));                               /* �ȴ������������             */
    ucData = LPC_SPI1->RXDAT;                                           /* ��������                     */
    
    return ucData;
}
/*********************************************************************************************************
** ��������: flash_read_status
** ��������: ��ȡоƬ״̬�Ĵ�����ֵ
** �������: ��
** �������: ״̬�Ĵ�����ֵ
** �� �� ֵ: ״̬�Ĵ�����ֵ
*********************************************************************************************************/
uint8_t flash_read_status ( void )
{
	uint8_t status;

	SendRecv_Start(0x05);					        
	status = SendRecv_Stop(0xff);				        
	return status;											    /*���ؼĴ��� Reg 1's ֵ		*/
}

/*********************************************************************************************************
** ��������: flash_write_enable
** ��������: оƬ�Ķ�ʹ��
** �������: ��
** �������: ��
** �� �� ֵ: ��
*********************************************************************************************************/
static void flash_write_enable (void)
{
	while ((flash_read_status() & 0x01) != 0x00);                         /* �ȴ�оƬ����         */
	SendRecv_Stop(0x06);	
	while ((flash_read_status() & 0x03) != 0x02);                         /* �ȴ��������  */
}

/*********************************************************************************************************
** ��������: flash_read_id
** ��������: ��ȡоƬ��ID��
** �������: IDType ��ȡID���ͣ�Manu_ID,Dev_ID,Jedec_ID
** �������: IDcode оƬID
** �� �� ֵ: IDcode оƬID
*********************************************************************************************************/
uint32_t flash_read_id (uint8_t IDType)
{
	uint32_t IDcode,temp = 0;

	if (IDType == Jedec_ID) {
		SendRecv_Start(0x9F);                                       /* ��ʼ�����Ͷ�ID����(9Fh)      */
		temp = (temp | Send_Byte(0xFF)) << 8;                       /* �������ݵ� 1 ���ֽ�          */
		temp = (temp | Send_Byte(0xFF)) << 8;                       /* �������ݵ� 2 ���ֽ�          */
		temp = (temp | SendRecv_Stop(0xFF));                        /* �������ݵ� 3 ���ֽڣ�����    */
		IDcode = temp;
		return (IDcode);
	}else{
	SendRecv_Start(0x90);
	Send_Byte(0x00);
	Send_Byte(0x00);
	Send_Byte(0x00);
  IDcode = (Send_Byte(0xff) << 8) | SendRecv_Stop(0xff);
	return IDcode; 	
	}		
}

/*********************************************************************************************************
** ��������: flash_read_data
** ��������: ��ȡоƬ������
** �������: RAddr    -- ��ʼ��ȡ�ĵ�ַ
** �������: buf      -- ��Ŷ��������ݻ���
             RLength  -- ��ȡ���ݵĳ���
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_read_data (uint32_t RAddr, uint8_t *buf, uint32_t RLength)
{
	uint8_t Temp;
	uint32_t i;

	if (RLength == 0)
	{
		return 0;
	}

	/*
	 *	Check the state register. If it's busy , wait until it's free
	 */
	while(1)														
	{														
		Temp = flash_read_status( );								
		Temp &= 0x01;											
		if(Temp == 0x00)									
			break;									
		for(i=0; i<10; i++);						
	}

	SendRecv_Start(0x03);                                   /* ��ʼ�����Ͷ�����(03h)        */
	Send_Byte((RAddr & 0xFF0000) >> 16);                    /* ���͵�ַ��Ϣ:�õ�ַΪ3���ֽ� */
	Send_Byte((RAddr & 0x00FF00) >> 8);                    
	Send_Byte((RAddr & 0x0000FF));
	for (i=0; i<RLength; i++)
	{
		if(i==(RLength-1))                                     /* �ж϶�ȡ���ǲ������һ���ֽ�*/
			buf[i] = SendRecv_Stop(0xff);                        /* �ǵĻ��������ζ�ȡ*/
		else                                                   /* �����ǵĻ�������ȡ*/
			buf[i] = Send_Byte(0xff);
	}									
	return 1;
}


/*********************************************************************************************************
** ��������: flash_write_sector
** ��������: ��һ��������д��ָ�����ȵ�����
** �������: RAddr   -- ��ʼд��ĵ�ַ
** �������: buf     -- Ҫд������ݻ���
             RLength -- д�����ݵĳ��� 0-4095
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_write_sector (uint32_t WAddr, uint8_t *buf, uint32_t WLength)
{
	uint32_t i;

	if (WLength == 0)
	{
		return 0;
	}

	flash_write_enable();												         /* дʹ�� */

	SendRecv_Start(0x02);                                /* ��ʼ������д����(02h)     */
	Send_Byte((WAddr & 0xFF0000) >> 16);
	Send_Byte((WAddr & 0x00FF00) >> 8);
	Send_Byte((WAddr & 0x0000FF));
	for (i=0; i<WLength; i++)
	{
		if(i==(WLength-1))                                 /* �ж�д����ǲ������һ���ֽ�*/
		SendRecv_Stop(buf[i]);                             /* �ǵĻ���������д��*/
		else
		Send_Byte(buf[i]);                                 /* �����ǵĻ�����д��*/
	}
	while ((flash_read_status() & 0x01) != 0x00);

	return 1;
}

/*********************************************************************************************************
** ��������: flash_write_data
** ��������: д��ָ�����ȵ����ݣ�������һ������
** �������: RAddr   -- ��ʼд��ĵ�ַ
** �������: buf     -- Ҫд������ݻ���
             RLength -- д�����ݵĳ��� 0-
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��

*********************************************************************************************************/							
uint8_t flash_write_data (uint32_t WAddr, uint8_t *buf, uint32_t WLength)
{
	uint32_t dealer, remainder;
	uint32_t i, addr, len = 0;

	if (WLength == 0)
	{
		return 0;
	}

	remainder = WAddr % W25X32_PAGE_SIZE;

	/*
	 * Write the data not enough to one page memory
	 */
	if (remainder != 0)
	{
		len = W25X32_PAGE_SIZE - remainder;
		if (len < WLength)
		{
			flash_write_sector(WAddr, buf, len);
		} else
		{
			flash_write_sector(WAddr, buf, WLength);
			return 1;
		}
	}
	
	/*
	 * Calculate the rest data, then write several packets with whole page memory
	 */
	remainder = (WLength - len) % W25X32_PAGE_SIZE;
	dealer    = (WLength - len) / W25X32_PAGE_SIZE;
	for (i=0; i<dealer; i++)
	{
		addr = len + (i * W25X32_PAGE_SIZE);
		flash_write_sector(WAddr+addr, (uint8_t *)&buf[addr], W25X32_PAGE_SIZE);
	}
	
	/*
	 * Write the last data that not enough to one page memory
	 */
	if (remainder != 0)
	{
		addr = len + (i * W25X32_PAGE_SIZE);
		flash_write_sector(WAddr+addr, (uint8_t *)&buf[addr], remainder);
	}												
	
	return 1;
}


/*********************************************************************************************************
** ��������: flash_whole_erase
** ��������: ��������оƬ������
** �������: ��
** �������: ��
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_whole_erase( void )
{
	flash_write_enable();												    /* Write enable                 */
	
	SendRecv_Stop(0x60);
	while ((flash_read_status() & 0x01) != 0x00);	                        /* Wait for the flash free      */
    
	return 1;
}

/*********************************************************************************************************
** ��������: flash_block_erase
** ��������: ʹ�ÿ����оƬ�ϵ�����
** �������: addr ��ʼ�����ĵ�ַ
** �������: ��
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_block_erase (uint32_t addr)
{
	flash_write_enable();												    /* Write enable                 */
	
	SendRecv_Start(0xD8);
	Send_Byte((addr & 0xFF0000) >> 16);
	Send_Byte((addr & 0x00FF00) >> 8);
	SendRecv_Stop(addr & 0x0000FF);
	
	while ((flash_read_status() & 0x01) != 0x00);					        /* Wait for the flash free      */
    
	return 1;
}

/*********************************************************************************************************
** ��������: flash_sector_erase
** ��������: ʹ����������оƬ�ϵ�����
** �������: addr ��ʼ�����ĵ�ַ
** �������: ��
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_sector_erase (uint32_t addr)
{
	flash_write_enable();												    /* Write enable                 */
	
	SendRecv_Start(0x20);
	Send_Byte((addr & 0xFF0000) >> 16);
	Send_Byte((addr & 0x00FF00) >> 8);
	SendRecv_Stop(addr & 0x0000FF);
	
	while ((flash_read_status() & 0x01) != 0x00);							/* Wait for the flash free      */
    
	return 1;
}


/*********************************************************************************************************
** ��������: flash_sel_erases
** ��������: ͬʱ����оƬ�϶������������
** �������: startSec  --��ʼ����������
** �������: endSec    --��������������
** �� �� ֵ: ������� 1--�ɹ���0--ʧ��
*********************************************************************************************************/
uint8_t flash_sel_erases (uint32_t startSec, uint32_t endSec)
{
	uint32_t i;

	for (i=startSec; i<=endSec; i++)
	{
		flash_sector_erase(i * W25X32_SECTOR_SIZE);
	}

	return(1);
}

/*********************************************************************************************************
  End Of File
*********************************************************************************************************/













