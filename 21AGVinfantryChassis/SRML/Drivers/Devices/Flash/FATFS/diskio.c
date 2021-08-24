/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */
#include "../W25Qx.h"
#include "main.h"

/* Definitions of physical drive number for each drive */
#define SPI_FLASH		0	/* Example: Map Ramdisk to physical drive 0 */

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	if(pdrv == SPI_FLASH)
	{
		return RES_OK;
	}
	else{
		return RES_PARERR;
	}
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	if(pdrv == SPI_FLASH)
	{
		if(W25Qx_Init(&hspi1,W25Q128) == 0)
			return RES_OK;
		else
			return RES_PARERR;
	}
	else{
		return RES_PARERR;
	}
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	if(pdrv == SPI_FLASH)
	{
		W25Qx_Read_Sector(buff,sector,count);
		return RES_OK;
	}
	else{
		return RES_PARERR;
	}
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	if(pdrv == SPI_FLASH)
	{
		W25Qx_Write_Sector((uint8_t*)buff,sector,count);
		return RES_OK;
	}
	else{
		return RES_PARERR;
	}
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	if(pdrv == SPI_FLASH)
	{
		switch(cmd)
		{
			case CTRL_SYNC:
				return RES_OK;
			
			case GET_SECTOR_COUNT:
				*(DWORD * )buff = 4096;//W25Q128有4096个大小为4k bytes 的扇区
				return RES_OK;
		
			case GET_SECTOR_SIZE :
				*(WORD * )buff = 4096;//spi flash的扇区大小是 4K Bytes
			 return RES_OK;
				
			case GET_BLOCK_SIZE :
				*(DWORD * )buff = 1;
				return RES_OK;
				
			default:
				return RES_PARERR;
		}
	}
	else{
		return RES_PARERR;
	}
}

DWORD get_fattime()
{
	return 0;
}
