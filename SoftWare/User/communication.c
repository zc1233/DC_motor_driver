#include "communication.h"
#include "motor_controller.h"
#include "key_id.h"
#include "motor_update.h"

CAN_Message_t RxData;
CAN_Message_t TxData;

CAN_TxHeaderTypeDef TxHeader = 
{
    .IDE = CAN_ID_STD, //标识符类型：标准标识符--CAN_ID_STD
    .RTR = CAN_RTR_DATA, //数据类型：数据帧--CAN_RTR_DATA
    .TransmitGlobalTime = DISABLE, //禁止报文传输时间戳
};

CAN_FilterTypeDef FilterConfig = 
{
    /*配置CAN过滤器*/
    .FilterBank = 0,                      //过滤器组号
    .FilterMode = CAN_FILTERMODE_IDLIST,  //过滤模式：列表模式--CAN_FILTERMODE_IDLIST
    .FilterScale = CAN_FILTERSCALE_16BIT, //过滤器位宽：16位
    .FilterMaskIdHigh = 0x0000,           //MASK
    .FilterMaskIdLow = 0x0000,
    .FilterFIFOAssignment = CAN_RX_FIFO0, //过滤器0关联到FIFO0
    .FilterActivation = ENABLE,           //激活滤波器
    .SlaveStartFilterBank = 14,           //单CAN此参数无意义
};

//过滤器配置
void CAN_Filter_configure(uint8_t id)
{
    HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //禁止CAN接收中断
    FilterConfig.FilterIdHigh = ((id << 5) | (1 << 4)); //过滤器0标识符高16位
    FilterConfig.FilterIdLow = (id << 5); //过滤器0标识符高16位
    
    //过滤器配置
    if (HAL_CAN_ConfigFilter(&hcan, &FilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); //使能CAN接收中断
    HAL_CAN_Start(&hcan); //启动CAN
}

uint8_t HAL_CAN_SendTxMessage(CAN_TxHeaderTypeDef* TxHeader,uint8_t std_id, uint8_t aData[],uint16_t lengh)
{
    uint32_t TxMailBox;
    uint8_t FreeTxMailBoxNum;

    TxHeader->StdId = std_id;//ID
    TxHeader->DLC = lengh;//数据长度

    while(0 == FreeTxMailBoxNum)
    {
        error_flag = 1;
        FreeTxMailBoxNum = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);//查看空闲邮箱数量
    }
    error_flag = 0;

    if (HAL_CAN_AddTxMessage(&hcan, TxHeader, aData, &TxMailBox) != HAL_OK)//判断是否发送成功
    {
        /* Transmission request Error */
        error_flag = 2;
        Error_Handler();
    }
    return 1;
}

//CAN接收中断回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, (uint8_t*)&RxData) != HAL_OK)
    {
        Error_Handler();
        return;
    }
    if (RxHeader.RTR)
    {
        motor.type = RxData.type;
        if (RxData.type == 0)
        {
            speed_controller.target = RxData.speed;
        }
        else if (RxData.type == 1)
        {
            angle_controller.target = RxData.angle;
            angle_controller.pid.max_output = RxData.speed>MOTOR_MAX_SPEED?MOTOR_MAX_SPEED:RxData.speed;
        }
    }
    
    TxData.angle = motor.angle_controller.current;
    TxData.speed = motor.speed_controller.current;
    uint8_t* buffer = (uint8_t*)&TxData;
    HAL_CAN_SendTxMessage(&TxHeader, ID, buffer + 1, sizeof(TxData) - 1);
}

