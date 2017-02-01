
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"
#include "gpio.h"
#include "dma.h"
#include "config.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

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

// Number of bytes left in the current transaction
static volatile uint8_t i2cBytesLeft;

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

/* I2C1 init function */
void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0x10;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0x11;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* I2C2 init function */
void MX_I2C2_Init(void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0x10;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0x11;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(i2cHandle->Instance==I2C1)
    {
        /* USER CODE BEGIN I2C1_MspInit 0 */
        printf("Initializing I2C1.\r\n");
        /* USER CODE END I2C1_MspInit 0 */

        /**I2C1 GPIO Configuration    
          PB6     ------> I2C1_SCL
          PB7     ------> I2C1_SDA 
          */
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_I2C1_CLK_ENABLE();

        /* Peripheral DMA init*/

        hdma_i2c1_rx.Instance = I2C1_RX_DMA_STREAM;
        hdma_i2c1_rx.Init.Channel = I2C1_RX_DMA_CHANNEL;
        hdma_i2c1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c1_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c1_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_i2c1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_i2c1_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_i2c1_rx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_i2c1_rx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_i2c1_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c1_rx);

        hdma_i2c1_tx.Instance = I2C1_TX_DMA_STREAM;
        hdma_i2c1_tx.Init.Channel = I2C1_TX_DMA_CHANNEL;
        hdma_i2c1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c1_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c1_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c1_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_i2c1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_i2c1_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_i2c1_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_i2c1_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_i2c1_tx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c1_tx);

        /* USER CODE BEGIN I2C1_MspInit 1 */
        HAL_NVIC_SetPriority(I2C1_DMA_TX_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(I2C1_DMA_TX_IRQn);
        HAL_NVIC_SetPriority(I2C1_DMA_RX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(I2C1_DMA_RX_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
        HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
        /* USER CODE END I2C1_MspInit 1 */
    }
    else if(i2cHandle->Instance==I2C2)
    {
        /* USER CODE BEGIN I2C2_MspInit 0 */
        printf("Initializing I2C2.\r\n");
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

        /* Peripheral clock enable */
        __HAL_RCC_I2C2_CLK_ENABLE();

        /* Peripheral DMA init*/

        hdma_i2c2_rx.Instance = I2C2_RX_DMA_STREAM;
        hdma_i2c2_rx.Init.Channel = I2C2_RX_DMA_CHANNEL;
        hdma_i2c2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_i2c2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c2_rx.Init.Mode = DMA_NORMAL;
        hdma_i2c2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_i2c2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_i2c2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_i2c2_rx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_i2c2_rx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_i2c2_rx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(i2cHandle,hdmarx,hdma_i2c2_rx);

        hdma_i2c2_tx.Instance = I2C2_TX_DMA_STREAM;
        hdma_i2c2_tx.Init.Channel = I2C2_TX_DMA_CHANNEL;
        hdma_i2c2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_i2c2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_i2c2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_i2c2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_i2c2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_i2c2_tx.Init.Mode = DMA_NORMAL;
        hdma_i2c2_tx.Init.Priority = DMA_PRIORITY_HIGH;
        hdma_i2c2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_i2c2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_i2c2_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_i2c2_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_i2c2_tx) != HAL_OK)
        {
            Error_Handler();
        }

        __HAL_LINKDMA(i2cHandle,hdmatx,hdma_i2c2_tx);

        /* USER CODE BEGIN I2C2_MspInit 1 */
        HAL_NVIC_SetPriority(I2C2_DMA_TX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(I2C2_DMA_TX_IRQn);
        HAL_NVIC_SetPriority(I2C2_DMA_RX_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(I2C2_DMA_RX_IRQn);
        HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
        HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
        /* USER CODE END I2C2_MspInit 1 */
    }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

    if(i2cHandle->Instance==I2C1)
    {
        /* USER CODE BEGIN I2C1_MspDeInit 0 */

        /* USER CODE END I2C1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_I2C1_CLK_DISABLE();

        /**I2C1 GPIO Configuration    
          PB6     ------> I2C1_SCL
          PB7     ------> I2C1_SDA 
          */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(i2cHandle->hdmarx);
        HAL_DMA_DeInit(i2cHandle->hdmatx);
        /* USER CODE BEGIN I2C1_MspDeInit 1 */
        HAL_NVIC_DisableIRQ(I2C1_DMA_TX_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_DMA_RX_IRQn);
        /* USER CODE END I2C1_MspDeInit 1 */
    }
    else if(i2cHandle->Instance==I2C2)
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

        /* Peripheral DMA DeInit*/
        HAL_DMA_DeInit(i2cHandle->hdmarx);
        HAL_DMA_DeInit(i2cHandle->hdmatx);
        /* USER CODE BEGIN I2C2_MspDeInit 1 */
        HAL_NVIC_DisableIRQ(I2C2_DMA_TX_IRQn);
        HAL_NVIC_DisableIRQ(I2C2_DMA_RX_IRQn);
        /* USER CODE END I2C2_MspDeInit 1 */
    }
} 
uint8_t aTxBuffer[] = {0,0,0,0};

// I2C read command(I2C handle, 7-bit address, data array, number of data bytes) 
void I2C_Read (I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t size)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    while ( HAL_I2C_Master_Receive_DMA(&hi2c1, addr << 1 | 0 , (uint8_t*)data, size) != HAL_OK){
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
            Error_Handler();
        }
    }
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

// I2C write command(I2C handle, 7-bit address, data array, number of data bytes) 
void I2C_Write (I2C_HandleTypeDef *hi2c, uint16_t addr, uint8_t *data, uint16_t size)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    while ( HAL_I2C_Master_Transmit_DMA(&hi2c1, addr << 1 | 0 , (uint8_t*)data, size) != HAL_OK){
        if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF){
            Error_Handler();
        }
    }
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef *hi2c)
{
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef *hi2c)
{
//    uint8_t data[16];
//    data[0] = 0x20;
//    data[1] = 0x0F;
//    I2C_Write(&hi2c1, 0x1E, data, 2);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
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
void I2C1_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

// Interrupt handler for errors
void I2C1_ER_IRQHandler()
{
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

// Interrupt handler for events
void I2C2_EV_IRQHandler()
{
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

// Interrupt handler for errors
void I2C2_ER_IRQHandler()
{
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

