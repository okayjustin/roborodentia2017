/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"
#include "dma.h"

/* USER CODE BEGIN 0 */
typedef enum {
    I2C_TYPE_TX,
    I2C_TYPE_RX
} I2C_TransferType;

// A single I2C transaction request
typedef struct {
    uint8_t ready;
    uint8_t numBytes;
    uint8_t addr;
    I2C_TransferType type;
    union {
        i2cTxCallback txCallback;
        i2cRxCallback rxCallback;
    };
    void* cParams;
} I2CTransaction;

// List of future transactions
//static I2CTransaction transactionList[I2C_MAX_NUM_TRANSACTIONS];

//// Buffer to hold tx data for future transactions. Treated as a fifo
//static uint8_t i2cTxDataBuffer[I2C_TRANSACTION_TX_BUF_SIZE];
//
//// Buffer to hold rx data for the current transaction
//static uint8_t i2cRxDataBuffer[I2C_TRANSACTION_RX_BUF_SIZE];
//
//// Index into i2cTxDataBuffer
//static uint32_t i2cRxDataBufferIdx;

// Fifo struct used to treat i2cTxDataBuffer as a fifo
//static buffer8_t i2cTxDataFifo;

// Bufffer to hold tx data for the current transaction.
// Data from i2cTxDataBuffer is copied into this buffer
// before the transaction starts
//static uint8_t i2cTxNextTransferBuffer[I2C_TRANSACTION_TX_BUF_SIZE];

// Index into i2cTxNextTransferBuffer
//static uint32_t i2cTxNextTransferIdx;

// The next free transaction number
//static uint32_t freeTransaction;

// The current transaction, or the next transaction that will occur
static volatile uint32_t nextTransaction;

// Flag to indicate whether the i2c is active or not
static volatile uint8_t i2cRunning;

// Flag to indicate if i2c should be serviced
static volatile uint8_t shouldServiceI2C;

// Status of the current transaction
static volatile uint8_t i2cStatus;

// Theory of Operation
//
// This file control the I2C peripheral which talks to
// several slave devices on the control board. For a
// list of all the slave devices, see the control board
// documentation.
//
// Usage:
// 1. Setup the buffer sizes in config.h
// 
// 2. Initialize the I2C peripheral with i2cInit()
//
// 3. Call serviceI2C() in the main while (1) loop.
//    This function must be called between I2C transactions.
//
// 4. To write data to an I2C slave use the i2cAddTxTransaction(...)
//    function.
//
// 5. To read data from an I2C slave use the i2cAddRxTransaction(...)
//    function.
//
// Callbacks:
// After a transaction has been completed a user provided callback
// will be called. This function is called whether or not the transaction
// was successful. Note that it is possible to start a new I2C transaction
// from inside another transaction's callback.
//
// Tx Data Callback:
//  (*i2cTxCallback)(void* parameters, I2CStatus status)
//
// The (parameters) parameter is a void* passed direction from the
// i2cAddTxTransaction function call. This allows the user to use a
// single callback for multiple purposes.
// The (status) parameter indicates the status of the transaction.
// There are three possible status: I2C_ACK, I2C_NACK, I2C_ERR.
// Only I2C_ACK indicates that the transaction was successful. An I2C_NACK
// status means that no slave responded to the address and the I2C_ERR
// status means that some sort of bus error occured during the transaction.
//
// Rx Data Callback:
//  (*i2cRxCallback)(void* parameters,
//                   uint8_t* rxData,
//                   uint32_t numBytes,
//                   I2CStatus status)
//
// See "Tx Data Callback" for information on (parameters) and (status).
// The (rxData) parameter is a pointer to the recieved data. The data
// is only valid during this callback so any data that needs to be kept
// must be copied at this time.
// The (numBytes) parameter contains the number of bytes actually recieved
// by the slave.
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;
DMA_HandleTypeDef hdma_i2c3_rx;
DMA_HandleTypeDef hdma_i2c3_tx;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* I2C3 init function */
void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PC12     ------> I2C2_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  
    /* I2C2 DMA Init */
    /* I2C2_RX Init */
    hdma_i2c2_rx.Instance = DMA1_Stream2;
    hdma_i2c2_rx.Init.Channel = DMA_CHANNEL_7;
    hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c2_rx);

    /* I2C2_TX Init */
    hdma_i2c2_tx.Instance = DMA1_Stream7;
    hdma_i2c2_tx.Init.Channel = DMA_CHANNEL_7;
    hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c2_tx);

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
  /* USER CODE END I2C2_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspInit 0 */

  /* USER CODE END I2C3_MspInit 0 */
  
    /**I2C3 GPIO Configuration    
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C3 clock enable */
    __HAL_RCC_I2C3_CLK_ENABLE();
  
    /* I2C3 DMA Init */
    /* I2C3_RX Init */
    hdma_i2c3_rx.Instance = DMA1_Stream1;
    hdma_i2c3_rx.Init.Channel = DMA_CHANNEL_1;
    hdma_i2c3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_i2c3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_rx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_i2c3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_rx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c3_rx);

    /* I2C3_TX Init */
    hdma_i2c3_tx.Instance = DMA1_Stream4;
    hdma_i2c3_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_i2c3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_i2c3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_i2c3_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_i2c3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_i2c3_tx.Init.Mode = DMA_NORMAL;
    hdma_i2c3_tx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_i2c3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_i2c3_tx) != HAL_OK)
    {
      _Error_Handler(__FILE__, __LINE__);
    }

    __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c3_tx);

    /* I2C3 interrupt Init */
    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspInit 1 */
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
  /* USER CODE END I2C3_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PC12     ------> I2C2_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    /* I2C2 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C3)
  {
  /* USER CODE BEGIN I2C3_MspDeInit 0 */

  /* USER CODE END I2C3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C3_CLK_DISABLE();
  
    /**I2C3 GPIO Configuration    
    PA8     ------> I2C3_SCL
    PB4     ------> I2C3_SDA 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_4);

    /* I2C3 DMA DeInit */
    HAL_DMA_DeInit(i2cHandle->hdmarx);
    HAL_DMA_DeInit(i2cHandle->hdmatx);

    /* I2C3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);
  /* USER CODE BEGIN I2C3_MspDeInit 1 */

  /* USER CODE END I2C3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

// I2C scan command(I2C handle), prints out all I2C addresses that ACK 
void I2C_Scan (I2C_HandleTypeDef *hi2c) {
    printf("Starting I2C scan...\r\n");
    uint8_t addr;
    int i2c_time_out = 10;
    for (addr = 0; addr < 128; addr++){
        if (HAL_I2C_Master_Receive(hi2c, addr << 1 | 1, NULL, 1, i2c_time_out) == HAL_OK){
            printf("Found I2C device at 7-bit address (decimal): %d\r\n", addr);
        }
    }
}

// I2C read command(I2C handle, 7-bit address, data array, number of data bytes) 
void I2C_Read (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t *data, uint16_t size) {
    int i2c_time_out = 10 + size * 1;
    if (HAL_I2C_Master_Receive(hi2c, addr << 1 | 1, data, size, i2c_time_out) != HAL_OK){
        printf("Error in I2C_Read\r\n");
        Error_Handler();
    }

//    while ( HAL_I2C_Master_Receive_DMA(hi2c, addr << 1 | 1 , (uint8_t*)data, size) != HAL_OK){
//        if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF){
//            printf("Error in I2C_Read\r\n");
//            Error_Handler();
//        }
//    }
}

// I2C write command(I2C handle, 7-bit address, data array, number of data bytes) 
void I2C_Write (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t *data, uint16_t size) {
    int i2c_time_out = 10 + size * 1;
    if (HAL_I2C_Master_Transmit(hi2c, addr << 1 | 0, data, size, i2c_time_out) != HAL_OK){
        printf("Error in I2C_Write\r\n");
        Error_Handler();
    }

//    while ( HAL_I2C_Master_Transmit_DMA(hi2c, addr << 1 | 0 , (uint8_t*)data, size) != HAL_OK){
//        if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_AF){
//            printf("Error in I2C_Write\r\n");
//            Error_Handler();
//        }
//    }
}

// I2C write then read command
void I2C_WriteRead (I2C_HandleTypeDef *hi2c, uint8_t addr, uint8_t *wr_data, uint16_t wr_size, uint8_t *rd_data, uint16_t rd_size)
{
    I2C_Write(hi2c, addr, wr_data, wr_size);
    I2C_Read(hi2c, addr, rd_data, rd_size);
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
{
//    uint8_t data[16];
//    data[0] = 0x20;
//    data[1] = 0x0F;
//    I2C_Write(&hi2c1, 0x1E, data, 2);
}

void HAL_I2C_ErrorCallback (I2C_HandleTypeDef *hi2c)
{
    printf("I2C error callback\r\n");
}

void HAL_I2C_AbortCpltCallback (I2C_HandleTypeDef * hi2c)
{
    printf("I2C abort callback\r\n");
}


// Interrupt handler for events
void I2C2_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

// Interrupt handler for errors
void I2C2_ER_IRQHandler()
{
    printf("Error in I2C2_ER_IRQHandler\r\n");
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

// Interrupt handler for events
void I2C3_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(&hi2c3);
}

// Interrupt handler for errors
void I2C3_ER_IRQHandler()
{
    printf("Error in I2C3_ER_IRQHandler\r\n");
    HAL_I2C_ER_IRQHandler(&hi2c3);
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
