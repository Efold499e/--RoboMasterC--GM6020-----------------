#include "bsp_can.h"

volatile uint8_t CAN1_RxData[8];
volatile uint32_t CAN1_RxId = 0;
volatile uint8_t CAN1_RxDLC = 0;
volatile uint8_t CAN1_RxNewFlag = 0;
moto_info_t motor_info[MOTOR_MAX_NUM];
/**
  * @brief  init can filter, start can, enable can rx interrupt
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
void can_user_init(CAN_HandleTypeDef* hcan)
{
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
    sFilterConfig.SlaveStartFilterBank = 0;

    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
    {
      Error_Handler();
    }
  

  /* 启动 CAN1 并启用 FIFO0 消息到达中断，应用可以根据需要调整通知位 */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    /* 启动失败处理（可改为日志或错误指示） */
    Error_Handler();
  }
  /* 激活 FIFO0 收到消息的中断回调 */
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief  can rx callback, get motor feedback info
  * @param  hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  * @retval None
  */
 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
 {
   if (hcan == &hcan1 || hcan->Instance == CAN1)
   {
     CAN_RxHeaderTypeDef rxHeader;
     uint8_t rxBuf[8];
     if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxBuf) == HAL_OK)
     {
       /* 只拷贝 rxHeader.DLC 字节（但我们的缓冲是 8 字节，先清零后拷贝更稳健） */
       memset((void *)CAN1_RxData, 0, sizeof(CAN1_RxData));
       memcpy((void *)CAN1_RxData, rxBuf, rxHeader.DLC <= 8 ? rxHeader.DLC : 8);
       CAN1_RxId = (rxHeader.IDE == CAN_ID_STD) ? rxHeader.StdId : rxHeader.ExtId;
       CAN1_RxDLC = rxHeader.DLC;
       CAN1_RxNewFlag = 1;
     }
   }
 }

/**
  * @brief  send motor control message through can bus
  * @param  id_range to select can control id 0x1ff or 0x2ff
  * @param  motor voltage 1,2,3,4 or 5,6,7
  * @retval None
  */
void set_motor_voltage(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
  uint32_t            txMailbox;
  tx_header.StdId = (id_range == 1)?(0x1ff):(0x2ff);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data, &txMailbox); 
}
