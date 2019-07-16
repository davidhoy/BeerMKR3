/**
 ******************************************************************************
 * @file    spi_flash_platform.c
 * @author  William Xu
 * @version V1.0.0
 * @date    16-Sep-2014
 * @brief   This file provides all the headers of flash operation functions.
 ******************************************************************************
 *  UNPUBLISHED PROPRIETARY SOURCE CODE
 *  Copyright (c) 2016 MXCHIP Inc.
 *
 *  The contents of this file may not be disclosed to third parties, copied or
 *  duplicated in any form, in whole or in part, without the prior written
 *  permission of MXCHIP Corporation.
 ******************************************************************************
 */

#include "spi_flash_platform_interface.h"
//#include "mico_platform.h"
#define UNUSED_PARAMETER(x)
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "main.h"

#if defined ( USE_MICO_SPI_FLASH )

extern const mico_spi_device_t mico_spi_flash;

int sflash_platform_init ( /*@shared@*/ void* peripheral_id, /*@out@*/ void** platform_peripheral_out )
{
    UNUSED_PARAMETER( peripheral_id );  /* Unused due to single SPI Flash */

    if ( kNoErr != MicoSpiInitialize( &mico_spi_flash ) )
    {
        /*@-mustdefine@*/ /* Lint: failed - do not define platform peripheral */
        return -1;
        /*@+mustdefine@*/
    }

    if( platform_peripheral_out != NULL)
      *platform_peripheral_out = NULL;
    
    return 0;
}


extern int sflash_platform_send_recv ( const void* platform_peripheral, /*@in@*/ /*@out@*/ sflash_platform_message_segment_t* segments, unsigned int num_segments  )
{
    UNUSED_PARAMETER( platform_peripheral );

    if ( kNoErr != MicoSpiTransfer( &mico_spi_flash, (mico_spi_message_segment_t*) segments, (uint16_t) num_segments ) )
    {
        return -1;
    }

    return 0;
}

int sflash_platform_deinit( void )
{
    if ( kNoErr != MicoSpiFinalize( &mico_spi_flash ) )
    {
        /*@-mustdefine@*/ /* Lint: failed - do not define platform peripheral */
        return -1;
        /*@+mustdefine@*/
    }

    return 0;
}

#else
extern SPI_HandleTypeDef hspi1;


int sflash_platform_init( /*@shared@*/ void* peripheral_id, /*@out@*/ void** platform_peripheral_out )
{
    UNUSED_PARAMETER( peripheral_id );
    UNUSED_PARAMETER( platform_peripheral_out );

    //HAL_StatusTypeDef status = HAL_SPI_Init(&hspi);
    //return (status == HAL_OK) ? 0 : -1;
    return 0;

}

extern int sflash_platform_send_recv( const void* platform_peripheral, /*@in@*//*@out@*/sflash_platform_message_segment_t* segments, unsigned int num_segments )
{
	UNUSED_PARAMETER( platform_peripheral );

    HAL_StatusTypeDef status = HAL_ERROR;

    HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_RESET);
    for (int i = 0; i < num_segments; i++)
    {
    	sflash_platform_message_segment_t segment = segments[i];
    	if (segment.tx_buffer != NULL && segment.rx_buffer != NULL)
    	{
    		status = HAL_SPI_TransmitReceive_DMA(&hspi1,
    				                             (uint8_t*)segment.tx_buffer,
					                             (uint8_t*)segment.rx_buffer,
												 segment.length);
    	}
    	else if (segment.tx_buffer != NULL)
    	{
    		status = HAL_SPI_Transmit_DMA(&hspi1,
    									  (uint8_t*)segment.tx_buffer,
									      segment.length);
    	}
    	else if (segment.rx_buffer != NULL)
    	{
    		status = HAL_SPI_Receive_DMA(&hspi1,
    								     (uint8_t*)segment.rx_buffer,
									     segment.length);
    	}
    	else
    	{
    		HAL_Delay(1);
    	}
    	if (status != HAL_OK)
    	{
    		break;
    	}
    }
    HAL_GPIO_WritePin(Flash_SPI1_NSS_GPIO_Port, Flash_SPI1_NSS_Pin, GPIO_PIN_SET);
    return (status == HAL_OK) ? 0 : -1;
}

int sflash_platform_deinit( void )
{
	HAL_StatusTypeDef status = HAL_SPI_DeInit(&hspi1);
	return (status == HAL_OK) ? 0 : -1;
}


void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_SPI_TxCpltCallback should be implemented in the user file
   */
}


void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}

void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hspi);
}




#endif
