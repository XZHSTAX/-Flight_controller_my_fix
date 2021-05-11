/******************** (C) COPYRIGHT 2018 DY EleTe ********************************
 * 作者    ：徳研电科
 * 官网    ：www.gototi.com
 * 描述    ：板载FLASH存储芯片驱动
**********************************************************************************/
#include "stdio.h"
#include "Drv_w25qxx.h"

typedef struct
{
    uint8_t Manufacturer;               /* Manufacturer ID */
    uint8_t Memory;                     /* Density Code */
    uint8_t Capacity;                   /* Family Code */
    uint8_t rev;

} jedec_id_t;

/* Private define ------------------------------------------------------------*/

//#define W25QXX_DEBUG
#ifdef W25QXX_DEBUG
#define w25qxx_debug(fmt, ...)  printf(fmt, ##__VA_ARGS__)
#else
#define w25qxx_debug(fmt, ...)
#endif

#define JEDEC_MANUFACTURER_ST       0x20
#define JEDEC_MANUFACTURER_MACRONIX 0xC2
#define JEDEC_MANUFACTURER_WINBOND  0xEF

/* JEDEC Device ID: Memory type and Capacity */
#define JEDEC_W25Q16_BV_CL_CV   (0x4015) /* W25Q16BV W25Q16CL W25Q16CV  */
#define JEDEC_W25Q16_DW         (0x6015) /* W25Q16DW  */
#define JEDEC_W25Q32_BV         (0x4016) /* W25Q32BV */
#define JEDEC_W25Q32_DW         (0x6016) /* W25Q32DW */
#define JEDEC_W25Q64_BV_CV      (0x4017) /* W25Q64BV W25Q64CV */
#define JEDEC_W25Q64_DW         (0x4017) /* W25Q64DW */
#define JEDEC_W25Q128_BV        (0x4018) /* W25Q128BV */

#define JEDEC_WRITE_ENABLE           0x06
#define JEDEC_WRITE_DISABLE          0x04
#define JEDEC_READ_STATUS            0x05
#define JEDEC_WRITE_STATUS           0x01
#define JEDEC_READ_DATA              0x03
#define JEDEC_FAST_READ              0x0b
#define JEDEC_DEVICE_ID              0x9F
#define JEDEC_PAGE_WRITE             0x02
#define JEDEC_SECTOR_ERASE           0x20

#define JEDEC_STATUS_BUSY            0x01
#define JEDEC_STATUS_WRITEPROTECT    0x02
#define JEDEC_STATUS_BP0             0x04
#define JEDEC_STATUS_BP1             0x08
#define JEDEC_STATUS_BP2             0x10
#define JEDEC_STATUS_TP              0x20
#define JEDEC_STATUS_SEC             0x40
#define JEDEC_STATUS_SRP0            0x80

#define DUMMY_BYTE     0xFF

#define W25QXX_CS_LOW()      GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0)
#define W25QXX_CS_HIGH()     GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, GPIO_PIN_4)

static flash_info_t flash_info;
static uint8_t Flash_SendByte ( uint8_t byte );
static void Flash_WaitForEnd ( void );
static uint8_t Flash_ReadID ( jedec_id_t *id );
static void Flash_WriteEnable ( void );

static uint8_t Flash_SendByte ( uint8_t byte )
{
    uint32_t getData = 0x00;

    /* Send the data from the SSI-1 Master */
    MAP_SSIDataPut(SSI1_BASE, byte);
     
    /* Wait for the data to be transmitted out of the SSI0 by checking on
     * the busy status from the SSI controller*/
    while(MAP_SSIBusy(SSI1_BASE));
    
    MAP_SSIDataGet(SSI1_BASE, &getData);
    
    return getData;
}

void Flash_Init ( void )
{
    jedec_id_t flash_id;
    
    MAP_SysCtlPeripheralDisable(SYSCTL_PERIPH_SSI1);
    MAP_SysCtlPeripheralReset(SYSCTL_PERIPH_SSI1);
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_SSI1)))
    {
    }
    
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB)))
    {
    }
    MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!(MAP_SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE)))
    {
    }
    
    MAP_GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    MAP_GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    MAP_GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    
    MAP_GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);
    MAP_GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
    
      //Polarity Phase       Mode
      //  0       0   SSI_FRF_MOTO_MODE_0
      //  0       1   SSI_FRF_MOTO_MODE_1
      //  1       0   SSI_FRF_MOTO_MODE_2
      //  1       1   SSI_FRF_MOTO_MODE_3
    MAP_SSIConfigSetExpClk(SSI1_BASE, g_ui32SysClock, SSI_FRF_MOTO_MODE_0,
                           SSI_MODE_MASTER, (g_ui32SysClock/12), 8);
    
    MAP_SSIEnable(SSI1_BASE);
    
    W25QXX_CS_HIGH();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "0xff " instruction */
    Flash_SendByte ( DUMMY_BYTE );
    
    W25QXX_CS_HIGH();
    /* read flash id */
    Flash_ReadID ( &flash_id );
    
    if ( flash_id.Manufacturer == JEDEC_MANUFACTURER_WINBOND )
    {
        w25qxx_debug ( "W25Q128 ID = 0x%02X\r\n", flash_id.Manufacturer );
        flash_info.sector_size = 4096;                              /* Page Erase (4096 Bytes) */
        if ( flash_id.Capacity == ( JEDEC_W25Q128_BV & 0xff ) )
        {
            w25qxx_debug ( "W25Q128_BV detection\r\n" );
            flash_info.sector_count = 4096;                         /* 128Mbit / 8 / 4096 = 4096 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q64_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q64_DW or W25Q64_BV or W25Q64_CV detection\r\n" );
            flash_info.sector_count = 2048;                       /* 64Mbit / 8 / 4096 = 2048 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q32_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q32_DW or W25Q32_BV detection\r\n" );
            flash_info.sector_count = 1024;                       /* 32Mbit / 8 / 4096 = 1024 */
        }
        else if ( flash_id.Capacity == ( JEDEC_W25Q16_DW & 0xff ) )
        {
            w25qxx_debug ( "W25Q16_DW or W25Q16_BV detection\r\n" );
            flash_info.sector_count = 512;                       /* 16Mbit / 8 / 4096 = 512 */
        }
        else
        {
            w25qxx_debug ( "error flash capacity\r\n" );
            flash_info.sector_count = 0;
        }

        flash_info.capacity = flash_info.sector_size * flash_info.sector_count;
    }
    else
    {
        w25qxx_debug ( "Unknow Manufacturer ID!%02X\r\n", flash_id.Manufacturer );
        flash_info.initialized = 0;
        return ;
    }

    flash_info.initialized = 1;
}

static void Flash_WriteEnable ( void )
{
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Write Enable instruction */
    Flash_SendByte ( JEDEC_WRITE_ENABLE );
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
}

//Erases the specified FLASH sector.
void Flash_SectorErase ( uint32_t address, uint8_t state )
{
    Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send Sector Erase instruction */
    Flash_SendByte ( JEDEC_SECTOR_ERASE );
    /* Send SectorAddr high nibble address byte */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send SectorAddr medium nibble address byte */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send SectorAddr low nibble address byte */
    Flash_SendByte ( address & 0xFF );
    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    if ( state )
    {
        Flash_WaitForEnd();
    }
}

/**
  * @brief  Writes more than one byte to the FLASH with a single WRITE
  *         cycle(Page WRITE sequence). The number of byte can't exceed
  *         the FLASH page size.
  * @param pBuffer : pointer to the buffer  containing the data to be
  *                  written to the FLASH.
  * @param WriteAddr : FLASH's internal address to write to.
  * @param NumByteToWrite : number of bytes to write to the FLASH,
  *                       must be equal or less than "SPI_FLASH_PageSize" value.
  * @retval : None
  */
void Flash_PageWrite ( uint32_t address, uint8_t* buffer,  uint32_t lenght )
{
    Flash_WriteEnable();
    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();
    /* Send "Write to Memory " instruction */
    Flash_SendByte ( JEDEC_PAGE_WRITE );
    /* Send WriteAddr high nibble address byte to write to */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send WriteAddr medium nibble address byte to write to */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send WriteAddr low nibble address byte to write to */
    Flash_SendByte ( address & 0xFF );

    /* while there is data to be written on the FLASH */
    while ( lenght-- )
    {
        /* Send the current byte */
        Flash_SendByte ( *buffer );
        /* Point on the next byte to be written */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    /* Wait the end of Flash writing */
    Flash_WaitForEnd();
}

/**
  * @brief  Reads a block of data from the FLASH.
  * @param buffer : pointer to the buffer that receives the data read
  *                  from the FLASH.
  * @param address : FLASH's internal address to read from.
  * @param lenght : number of bytes to read from the FLASH.
  * @retval : None
  */
void Flash_PageRead ( uint32_t address, uint8_t* buffer,  uint32_t lenght )
{

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "Read from Memory " instruction */
    Flash_SendByte ( JEDEC_READ_DATA );

    /* Send ReadAddr high nibble address byte to read from */
    Flash_SendByte ( ( address & 0xFF0000 ) >> 16 );
    /* Send ReadAddr medium nibble address byte to read from */
    Flash_SendByte ( ( address & 0xFF00 ) >> 8 );
    /* Send ReadAddr low nibble address byte to read from */
    Flash_SendByte ( address & 0xFF );

    while ( lenght-- ) /* while there is data to be read */
    {
        /* Read a byte from the FLASH */
        *buffer = Flash_SendByte ( DUMMY_BYTE );
        /* Point to the next location where the byte read will be saved */
        buffer++;
    }

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();
}

//Reads FLASH identification.
uint8_t Flash_ReadID ( jedec_id_t *id )
{
    uint8_t *recv_buffer = ( uint8_t* ) id;

    /* Select the FLASH: Chip Select low */
    W25QXX_CS_LOW();

    /* Send "RDID " instruction */
    Flash_SendByte ( JEDEC_DEVICE_ID );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Read a byte from the FLASH */
    *recv_buffer++ = Flash_SendByte ( DUMMY_BYTE );

    /* Deselect the FLASH: Chip Select high */
    W25QXX_CS_HIGH();

    return id->Manufacturer;
}

static void Flash_WaitForEnd ( void )
{
    u8 FLASH_Status = 0;

    /* Loop as long as the memory is busy with a write cycle */
    do
    {
        /* Select the FLASH: Chip Select low */
        W25QXX_CS_LOW();
        /* Send "Read Status Register" instruction */
        Flash_SendByte ( JEDEC_READ_STATUS );
        /* Send a dummy byte to generate the clock needed by the FLASH
        and put the value of the status register in FLASH_Status variable */
        FLASH_Status = Flash_SendByte ( DUMMY_BYTE );
        /* Deselect the FLASH: Chip Select high */
        W25QXX_CS_HIGH();
    }
    while ( FLASH_Status & JEDEC_STATUS_BUSY );
}

void Flash_SectorsRead ( uint32_t address, uint8_t *buffer, uint16_t count )
{
    uint16_t i = 0;

    for ( i = 0; i < count; i++ )
    {
//		  page=flash_info.sector_size/256;
//		  while(page--)
//			{
        Flash_PageRead ( address, buffer, flash_info.sector_size );
        buffer += flash_info.sector_size;
        address += flash_info.sector_size;
//			}
    }
}

void Flash_SectorsWrite ( uint32_t address, uint8_t *buffer, uint16_t count )
{
    uint16_t i = 0, page = flash_info.sector_size / 256;
    Flash_WriteEnable();
    for ( i = 0; i < count; i++ )
    {
        Flash_SectorErase ( address, 1 );
        page = flash_info.sector_size / 256;
        while ( page-- )
        {
            Flash_PageWrite ( address, buffer, 256 );
            buffer += 256;
            address += 256;
        }
    }
    //Flash_WriteDisable();
}

flash_info_t *Flash_GetInfo ( void )
{
    return &flash_info;
}

/******************* (C) COPYRIGHT 2018 DY EleTe *****END OF FILE************/
