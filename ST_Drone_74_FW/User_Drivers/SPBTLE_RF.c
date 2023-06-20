/*
 * SBTLE_RF.c
 *
 *  Created on: June 12, 2023
 *      Author: Mai Huynh Long Nhan
 */
#include "SPBTLE_RF.h"
#include "gp_timer.h"
#include "debug.h"
#include "hci.h"

#ifdef PRINT_CSV_FORMAT
extern volatile uint32_t ms_counter;
#endif /* PRINT_CSV_FORMAT */

#define HEADER_SIZE 5
#define MAX_BUFFER_SIZE 255
#define TIMEOUT_DURATION 15

SPI_HandleTypeDef SpiHandle;
//GPIO_InitTypeDef GPIO_InitStruct;
//uint32_t HCI_ProcessEvent=1;
/* Private function prototypes -----------------------------------------------*/
static void us150Delay(void);
void set_irq_as_output(void);
void set_irq_as_input(void);

#ifdef PRINT_CSV_FORMAT
/**
* @brief  This function is a utility to print the log time
*          in the format HH:MM:SS:MSS (DK GUI time format)
* @param  None
* @retval None
*/
void print_csv_time(void){
  uint32_t ms = ms_counter;
  PRINT_CSV("%02d:%02d:%02d.%03d", ms/(60*60*1000)%24, ms/(60*1000)%60, (ms/1000)%60, ms%1000);
}
#endif /* PRINT_CSV_FORMAT */
/**
 * @brief  EXTI line detection callback.
 * @param  uint16_t GPIO_Pin Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{  
  switch(GPIO_Pin)
  {
    case BNRG_SPI_EXTI_PIN:
      HCI_Isr();
      HCI_ProcessEvent=1;
    break;
  }
}

/**
* @brief  Writes data to a serial interface.
* @param  data1   :  1st buffer
* @param  data2   :  2nd buffer
* @param  n_bytes1: number of bytes in 1st buffer
* @param  n_bytes2: number of bytes in 2nd buffer
* @retval None
*/
void Hal_Write_Serial(const void* data1, const void* data2, int32_t n_bytes1,
                      int32_t n_bytes2)
{
  struct timer t;
  Timer_Set(&t, CLOCK_SECOND/10);
  
#ifdef PRINT_CSV_FORMAT
  print_csv_time();
  for (int i=0; i<n_bytes1; i++)
  {
    PRINT_CSV(" %02x", ((uint8_t *)data1)[i]);
  }
  for (int i=0; i<n_bytes2; i++)
  {
    PRINT_CSV(" %02x", ((uint8_t *)data2)[i]);
  }
  PRINT_CSV("\n");
#endif
  
  while(1)
  {
    if(BlueNRG_SPI_Write(&SpiHandle, (uint8_t *)data1,(uint8_t *)data2, n_bytes1, n_bytes2)==0) break;
    if(Timer_Expired(&t))
    {
      break;
    }
  }
}

/**
* @brief  Initializes the SPI communication with the BlueNRG
*         Expansion Board.
* @param  None
* @retval None
*/
void BNRG_SPI_Init(void)
{
  SpiHandle.Instance = BNRG_SPI_INSTANCE;
  SpiHandle.Init.Mode = BNRG_SPI_MODE;
  SpiHandle.Init.Direction = BNRG_SPI_DIRECTION;
  SpiHandle.Init.DataSize = BNRG_SPI_DATASIZE;
  SpiHandle.Init.CLKPolarity = BNRG_SPI_CLKPOLARITY;
  SpiHandle.Init.CLKPhase = BNRG_SPI_CLKPHASE;
  SpiHandle.Init.NSS = BNRG_SPI_NSS;
  SpiHandle.Init.FirstBit = BNRG_SPI_FIRSTBIT;
  SpiHandle.Init.TIMode = BNRG_SPI_TIMODE;
  SpiHandle.Init.CRCPolynomial = BNRG_SPI_CRCPOLYNOMIAL;
  SpiHandle.Init.BaudRatePrescaler = BNRG_SPI_BAUDRATEPRESCALER;
  SpiHandle.Init.CRCCalculation = BNRG_SPI_CRCCALCULATION;
  
  HAL_SPI_Init(&SpiHandle);
}

/**
* @brief  Resets the BlueNRG.
* @param  None
* @retval None
*/
void BlueNRG_RST(void)
{
  HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(5);
  HAL_GPIO_WritePin(BNRG_SPI_RESET_PORT, BNRG_SPI_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(5);
}

/**
* @brief  Reports if the BlueNRG has data for the host micro.
* @param  None
* @retval 1 if data are present, 0 otherwise
*/
uint8_t BlueNRG_DataPresent(void)
{
  if (HAL_GPIO_ReadPin(BNRG_SPI_EXTI_PORT, BNRG_SPI_EXTI_PIN) == GPIO_PIN_SET)
    return 1;
  else  
    return 0;
}

/**
* @brief  Activate internal bootloader using pin.
* @param  None
* @retval None
*/
void BlueNRG_HW_Bootloader(void)
{
  Disable_SPI_IRQ();
  set_irq_as_output();
  BlueNRG_RST();
  set_irq_as_input();
  Enable_SPI_IRQ();
}

/**
* @brief  Reads from BlueNRG SPI buffer and store data into local buffer.
* @param  hspi     : SPI handle
* @param  buffer   : Buffer where data from SPI are stored
* @param  buff_size: Buffer size
* @retval int32_t  : Number of read bytes
*/
int32_t BlueNRG_SPI_Read_All(SPI_HandleTypeDef *hspi, uint8_t *buffer,
                             uint8_t buff_size)
{
  uint16_t byte_count;
  uint8_t len = 0;
  uint8_t char_ff = 0xff;
  volatile uint8_t read_char;
  
  uint8_t header_master[HEADER_SIZE] = {0x0b, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[HEADER_SIZE];
  
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
  
  /* Read the header */  
  HAL_SPI_TransmitReceive(hspi, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);
  
  if (header_slave[0] == 0x02)
  {
    /* device is ready */
    byte_count = (header_slave[4]<<8)|header_slave[3];
    if (byte_count > 0)
    {
      
      /* avoid to read more data that size of the buffer */
      if (byte_count > buff_size)
      {
        byte_count = buff_size;
      }
      
      for (len = 0; len < byte_count; len++)
      {
        __disable_irq();
        HAL_SPI_TransmitReceive(hspi, &char_ff, (uint8_t*)&read_char, 1, TIMEOUT_DURATION);
        __enable_irq();
        buffer[len] = read_char;
      }
    }    
  }
  /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  
  // Add a small delay to give time to the BlueNRG to set the IRQ pin low
  // to avoid a useless SPI read at the end of the transaction
  for(volatile int i = 0; i < 2; i++)__NOP();
  
#ifdef PRINT_CSV_FORMAT
  if (len > 0)
  {
    print_csv_time();
    for (int i=0; i<len; i++)
    {
      PRINT_CSV(" %02x", buffer[i]);
    }
    PRINT_CSV("\n");
  }
#endif
  
  return len;   
}

/**
* @brief  Writes data from local buffer to SPI.
* @param  hspi     : SPI handle
* @param  data1    : First data buffer to be written
* @param  data2    : Second data buffer to be written
* @param  Nb_bytes1: Size of first data buffer to be written
* @param  Nb_bytes2: Size of second data buffer to be written
* @retval Number of read bytes
*/
int32_t BlueNRG_SPI_Write(SPI_HandleTypeDef *hspi, uint8_t* data1,
                          uint8_t* data2, uint8_t Nb_bytes1, uint8_t Nb_bytes2)
{  
  int32_t result = 0;
  int32_t spi_fix_enabled = 0;
  
#ifdef ENABLE_SPI_FIX
  spi_fix_enabled = 1;
#endif //ENABLE_SPI_FIX
  
  unsigned char header_master[HEADER_SIZE] = {0x0a, 0x00, 0x00, 0x00, 0x00};
  unsigned char header_slave[HEADER_SIZE]  = {0xaa, 0x00, 0x00, 0x00, 0x00};
  unsigned char read_char_buf[MAX_BUFFER_SIZE];
  
  Disable_SPI_IRQ(); 
  
  /*
  If the SPI_FIX is enabled the IRQ is set in Output mode, then it is pulled
  high and, after a delay of at least 112us, the CS line is asserted and the
  header transmit/receive operations are started.
  After these transmit/receive operations the IRQ is reset in input mode.
  */
  if (spi_fix_enabled)
  {
    set_irq_as_output();
    
    /* Assert CS line after at least 112us */
    us150Delay();
  }
  
  /* CS reset */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_RESET);
  
  /* Exchange header */  
  __disable_irq();
  HAL_SPI_TransmitReceive(hspi, header_master, header_slave, HEADER_SIZE, TIMEOUT_DURATION);
  __enable_irq();
  
  if (spi_fix_enabled)
  {
    set_irq_as_input();
  }
  
  if (header_slave[0] == 0x02)
  {
    /* SPI is ready */
    if (header_slave[1] >= (Nb_bytes1+Nb_bytes2))
    {
      
      /*  Buffer is big enough */
      if (Nb_bytes1 > 0)
      {
        __disable_irq();
        HAL_SPI_TransmitReceive(hspi, data1, read_char_buf, Nb_bytes1, TIMEOUT_DURATION);
        __enable_irq();
      }
      if (Nb_bytes2 > 0)
      {
        __disable_irq();
        HAL_SPI_TransmitReceive(hspi, data2, read_char_buf, Nb_bytes2, TIMEOUT_DURATION);
        __enable_irq();
      }
      
    }
    else
    {
      /* Buffer is too small */
      result = -2;
    }
  }
  else
  {
    /* SPI is not ready */
    result = -1;
  }
  
  /* Release CS line */
  HAL_GPIO_WritePin(BNRG_SPI_CS_PORT, BNRG_SPI_CS_PIN, GPIO_PIN_SET);
  Enable_SPI_IRQ();
  return result;
}

/**
* @brief  Set in Output mode the IRQ.
* @param  None
* @retval None
*/
void set_irq_as_output(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Pull IRQ high */
  GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
  HAL_GPIO_WritePin(BNRG_SPI_IRQ_PORT, BNRG_SPI_IRQ_PIN, GPIO_PIN_SET);
}

/**
* @brief  Set the IRQ in input mode.
* @param  None
* @retval None
*/
void set_irq_as_input(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* IRQ input */  
  GPIO_InitStructure.Pin = BNRG_SPI_IRQ_PIN;
  GPIO_InitStructure.Mode = BNRG_SPI_IRQ_MODE;
  GPIO_InitStructure.Pull = GPIO_PULLDOWN;
  GPIO_InitStructure.Speed = BNRG_SPI_IRQ_SPEED;
  GPIO_InitStructure.Alternate = BNRG_SPI_IRQ_ALTERNATE;    
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pull = BNRG_SPI_IRQ_PULL;
  HAL_GPIO_Init(BNRG_SPI_IRQ_PORT, &GPIO_InitStructure);
}

/**
* @brief  Utility function for delay
* @param  None
* @retval None
* NOTE: TODO: implement with clock-independent function.
*/
static void us150Delay(void)
{
#if SYSCLK_FREQ == 4000000
  for(volatile int i = 0; i < 35; i++)__NOP();
#elif SYSCLK_FREQ == 32000000
  for(volatile int i = 0; i < 420; i++)__NOP();
#elif SYSCLK_FREQ == 80000000
  for(volatile int i = 0; i < 1072; i++)__NOP();
#elif SYSCLK_FREQ == 84000000
  for(volatile int i = 0; i < 1125; i++)__NOP();
#elif SYSCLK_FREQ == 168000000
  for(volatile int i = 0; i < 2250; i++)__NOP();
#else
#error Implement delay function.
#endif    
}

/**
* @brief  Enable SPI IRQ.
* @param  None
* @retval None
*/
void Enable_SPI_IRQ(void)
{
  HAL_NVIC_EnableIRQ(BNRG_SPI_EXTI_IRQn);  
}

/**
* @brief  Disable SPI IRQ.
* @param  None
* @retval None
*/
void Disable_SPI_IRQ(void)
{ 
  HAL_NVIC_DisableIRQ(BNRG_SPI_EXTI_IRQn);
}

/**
* @brief  Clear Pending SPI IRQ.
* @param  None
* @retval None
*/
void Clear_SPI_IRQ(void)
{
  HAL_NVIC_ClearPendingIRQ(BNRG_SPI_EXTI_IRQn);
}

/**
* @brief  Clear EXTI (External Interrupt) line for SPI IRQ.
* @param  None
* @retval None
*/
void Clear_SPI_EXTI_Flag(void)
{  
  __HAL_GPIO_EXTI_CLEAR_IT(BNRG_SPI_EXTI_PIN);  
}

//void BlueNRG_Init(void)
//{
//
//  int ret = 1;
//  uint8_t bdaddr[6];
//  uint8_t  hwVersion=0;
//  uint16_t fwVersion=0;
//  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
//  DrvStatusTypeDef testStatus;
//
//  PRINTF("****** START BLE TESTS ******\r\n");
//  BNRG_SPI_Init();
//
//  bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
//  bdaddr[1] = (STM32_UUID[0]    )&0xFF;
//  bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
//  bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
//  bdaddr[4] = (hwVersion > 0x30) ?
//            ((((0x34-48)*10) + (0x30-48)+100)&0xFF) :
//            ((((0x34-48)*10) + (0x30-48)    )&0xFF) ;
//  bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
//
//  /* Initialize the BlueNRG HCI */
//  HCI_Init();
//
// /* Reset BlueNRG hardware */
//  BlueNRG_RST();
//
//  /* get the BlueNRG HW and FW versions */
//  PRINTF("\r\nReading BlueNRG version ...\r\n");
//  if (getBlueNRGVersion(&hwVersion, &fwVersion)== BLE_STATUS_SUCCESS)
//  {
//     //Reset BlueNRG again otherwise it will fail.
//    BlueNRG_RST();
//
//    PRINTF("GATT Initializzation...\r\n");
//    ret = aci_gatt_init();
//    if(ret)
//    {
//      testStatus = COMPONENT_ERROR;
//      PRINTF("\r\nGATT_Init failed ****\r\n");
//      goto fail;
//    }
//    //Set the GAP INIT like X-NUCLEO-IDB05A1 eval board  since using same SPBTLE_RF module
//    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
//
//    if(ret != BLE_STATUS_SUCCESS)
//    {
//      PRINTF("\r\nGAP_Init failed\r\n");
//      goto fail;
//    }
//    ret = hci_le_set_random_address(bdaddr);
//    const char BoardName[7] = {NAME_BLUEMS};
//    ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
//                                       7/*strlen(BoardName)*/, (uint8_t *)BoardName);
//
//    PRINTF("GAP setting Authentication ....\r\n");
//    ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
//                                       OOB_AUTH_DATA_ABSENT,
//                                       NULL, 7, 16,
//                                       USE_FIXED_PIN_FOR_PAIRING, 123456,
//                                       BONDING);
//    if (ret != BLE_STATUS_SUCCESS)
//    {
//      testStatus = COMPONENT_ERROR;
//       PRINTF("\r\nGAP setting Authentication failed ******\r\n");
//       goto fail;
//    }
//
//    PRINTF("SERVER: BLE Stack Initialized \r\n"
//           "Board HWver=%d, FWver=%d.%d.%c\r\n"
//           "BoardMAC = %x:%x:%x:%x:%x:%x\r\n",
//           hwVersion,
//           fwVersion>>8,
//           (fwVersion>>4)&0xF,
//           (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
//           bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
//
//    /* Set output power level */
//    aci_hal_set_tx_power_level(1,4);    /* -2.1dBm */
//
//    ret = Add_ConsoleW2ST_Service();
//    if(ret == BLE_STATUS_SUCCESS)
//    {
//       PRINTF("Console Service W2ST added successfully\r\n");
//    }
//    else
//    {
//       testStatus = COMPONENT_ERROR;
//       PRINTF("\r\nError while adding Console Service W2ST\r\n");
//    }
//
//    ret = Add_ConfigW2ST_Service();
//    if(ret == BLE_STATUS_SUCCESS)
//       PRINTF("Config  Service W2ST added successfully\r\n");
//    else{
//       testStatus = COMPONENT_ERROR;
//       PRINTF("\r\nError while adding Config Service W2ST\r\n");
//    }
//
//    PRINTF("\r\nAll test passed!\r\n");
//  }
//  else {
//       testStatus = COMPONENT_ERROR;
//       PRINTF("\r\nError in BlueNRG tests. ******\r\n");
//  }
//  PRINTF("****** END BLE TESTS ******\r\n");
//  return;
//
//fail:
//  testStatus = COMPONENT_ERROR;
//  return;
//}

//void Init_BlueNRG_Custom_Services(void)
//{
//	  int ret;
//
//	  ret = Add_HWServW2ST_Service();
//	  if(ret == BLE_STATUS_SUCCESS) {
//	     PRINTF("HW      Service W2ST added successfully\r\n");
//	  } else {
//	     PRINTF("\r\nError while adding HW Service W2ST\r\n");
//	  }
//
//	  ret = Add_ConsoleW2ST_Service();
//	  if(ret == BLE_STATUS_SUCCESS) {
//	     PRINTF("Console Service W2ST added successfully\r\n");
//	  } else {
//	     PRINTF("\r\nError while adding Console Service W2ST\r\n");
//	  }
//
//	  ret = Add_ConfigW2ST_Service();
//	  if(ret == BLE_STATUS_SUCCESS) {
//	     PRINTF("Config  Service W2ST added successfully\r\n");
//	  } else {
//	     PRINTF("\r\nError while adding Config Service W2ST\r\n");
//	  }
//}
