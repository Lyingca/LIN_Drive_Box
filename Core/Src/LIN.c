//
// Created by 陈骏骏 on 2023/5/3.
//

#include "LIN.h"
#include "usart.h"
#include "key.h"
#include <string.h>

//LIN同步帧字节
uint8_t  SYNC_Frame = 0x55;

//LIN接收数据缓存
uint8_t pLINRxBuff[LIN_RX_MAXSIZE];
//LIN发送数据缓存
uint8_t pLINTxBuff[LIN_TX_MAXSIZE];
//当前测试的电机步长
uint16_t EXV_Test_Step;
//当前测试的循环次数
uint16_t EXV_Test_Cycles;
//发送读取帧的标志位
uint8_t LIN_Read_Flag = DISABLE;
//发送写帧的标志位
uint8_t LIN_Send_Flag = DISABLE;
//保存LIN芯片的信息
struct LIN_Chip_Msg
{
    //读PID
    uint8_t read_PID;
    //写PID
    uint8_t write_PID;
    //电机运动使能
    uint8_t EXV_Move_Enable;
    //初始化请求
    uint8_t EXV_Init_Request;
    //非初始化请求
    uint8_t EXV_Not_Init_Request;
};
//初始化LIN芯片信息
struct LIN_Chip_Msg chip[3] = {
        {LIN_PID_53_0x35,LIN_PID_52_0x34,0xFF,0xFD,0xFC},
        {LIN_PID_55_0x37,LIN_PID_54_0x36,0xFF,0xFD,0xFC},
        {LIN_PID_32_0x20,LIN_PID_16_0x10,0xFF,0xFD,0xFC}
};
//芯片编号
uint8_t chip_Num = 0;
//无限循环的标志位
uint8_t InfiniteLoop = 0;
//电子膨胀阀达到目标步数的计数值
uint8_t EXV_Finished_Count = 0;
uint8_t EXV_Compare_Count = 0x07;

/****************************************************************************************
** 函数名称: LINCheckSum----标准校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 1; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = chm1 / 256;
    ckm = ckm + chm1 % 256;
    ckm = 0xFF - ckm;
    return ckm;
}
/****************************************************************************************
** 函数名称: LINCheckSumEn----增强校验
** 功能描述: 计算并返回LIN校验值
** 参    数:  uint8_t *buf：需要计算的数组
			        uint8_t lens：数组长度
** 返 回 值:   uint8_t ckm: 计算结果
****************************************************************************************/
uint8_t LIN_Check_Sum_En(uint8_t *buf, uint8_t lens)
{
    uint8_t i, ckm = 0;
    uint16_t chm1 = 0;
    for(i = 0; i < lens; i++)
    {
        chm1 += *(buf+i);
    }
    ckm = ~(chm1 % 255);
    return ckm;
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID_Data
** 功能描述: LIN发送数据帧
** 参    数: *buf:数组地址；buf[0]=PID
			       lens:数据长度,不含校验字节
			       CK_Mode: 校验类型增强型LIN_CK_ENHANCED=1：基本LIN_CK_STANDARD=0
             Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID_Data(UART_HandleTypeDef *huart, uint8_t *buf, uint8_t lens, LIN_CK_Mode CK_Mode)
{
    if(CK_Mode == LIN_CK_STANDARD)
    {
        //arr[i] = *(arr + i)
        //计算标准型校验码，不计算PID
        *(buf + lens) = LIN_Check_Sum(buf, LIN_CHECK_STD_NUM);
    }
    else
    {
        //计算增强型校验码,连PID一起校验
        *(buf + lens) = LIN_Check_Sum_En(buf, LIN_CHECK_EN_NUM);
    }

    //发送同步间隔段
    HAL_LIN_SendBreak(huart);
    //发送同步段
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    //发送PID,数据内容和校验
    HAL_UART_Transmit(huart,buf,LIN_TX_MAXSIZE,HAL_MAX_DELAY);
}
/****************************************************************************************
** 函数名称: Lin_Tx_PID
** 功能描述: LIN发送报文头，PID，读取从机状态信息
** 参    数: PID, Timeout (0xffff)不做时间限制
** 返 回 值: 无
****************************************************************************************/
void LIN_Tx_PID(UART_HandleTypeDef *huart, uint8_t PID)
{
    //发送间隔帧
    HAL_LIN_SendBreak(huart);
    //发送同步帧
    HAL_UART_Transmit(huart,&SYNC_Frame,1,HAL_MAX_DELAY);
    HAL_UART_Transmit(huart,&PID,1,HAL_MAX_DELAY);
}

void Data_To_LIN(uint16_t step,uint16_t cycles,uint8_t init_enable)
{
    LIN_Send_Flag = DISABLE;
    uint8_t index = 0;
    EXV_Test_Step = step;
    EXV_Test_Cycles = cycles;

    pLINTxBuff[index++] = chip[chip_Num].write_PID;
    pLINTxBuff[index++] = step & 0xFF;
    pLINTxBuff[index++] = step >> 8;
    pLINTxBuff[index++] = chip[chip_Num].EXV_Move_Enable;
    if(init_enable)
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Init_Request;
    }
    else
    {
        pLINTxBuff[index++] = chip[chip_Num].EXV_Not_Init_Request;
    }
    //剩余的字节数有0xFF填充
    while(index < LIN_TX_MAXSIZE - 1)
    {
        pLINTxBuff[index++] = 0xFF;
    }
    LIN_Send_Flag = ENABLE;
}

void Finished_LIN(uint8_t send,uint8_t read)
{
    LIN_Send_Flag = send;
    LIN_Read_Flag = read;
}

/**
 * 发送LIN数据，包括读取帧和写帧
 */
void Send_LIN_Data()
{
    if(LIN_Send_Flag)
    {
        pLINTxBuff[0] = chip[chip_Num].write_PID;
        LIN_Tx_PID_Data(&huart2,pLINTxBuff,LIN_TX_MAXSIZE - 1,LIN_CK_ENHANCED);
        LIN_Read_Flag = ENABLE;
        HAL_Delay(20);
    }
    if(LIN_Read_Flag)
    {
        LIN_Tx_PID(&huart2, chip[chip_Num].read_PID);
        HAL_Delay(20);
        chip_Num++;
        if(chip_Num == 3)
        {
            chip_Num = 0;
        }
    }
}

/**
 * 循环执行命令
 */
void EXV_Loop_Execution(uint16_t cycles,uint16_t test_step,uint16_t reset_step)
{
    if (cycles)
    {
        int step = 0;
        if (test_step == reset_step)
        {
            step = 0;
            if (!InfiniteLoop)
            {
                currentCycleCount--;
            }
            DisplayCharacter(SECOND_LINE + 5,currentCycleCount,5);
        }
        else
        {
            step = reset_step;
        }
        DisplayChineseCharacter(FOURTH_LINE,"                ", strlen("                "));
        Data_To_LIN(step,currentCycleCount,0);
    }
}

/**
 * 反馈电机信号
 * @param signal
 */
void Feedback_Signal(uint16_t signal)
{

    EXV_Finished_Count = 0;
    EXV_Compare_Count = 0x07;
    if (signal == EXV_RESP_OK)
    {
        EXV_Loop_Execution(EXV_Test_Cycles,EXV_Test_Step,currentStepSize);
    }

}

/*
 * 统计运行到目标步数的电子膨胀阀数量
 */
void CollectCompleteEXVNum()
{
    if(pLINRxBuff[2] == LIN_PID_53_0x35)
    {
        EXV_Finished_Count |= 0x01;
        DisplayChineseCharacter(FOURTH_LINE,"0x34", strlen("0x34"));
        HAL_Delay(100);
    }
    else if(pLINRxBuff[2] == LIN_PID_55_0x37)
    {
        EXV_Finished_Count |= 0x02;
        DisplayChineseCharacter(FOURTH_LINE + 3,"0x36", strlen("0x36"));
        HAL_Delay(100);
    }
    else if(pLINRxBuff[2] == LIN_PID_32_0x20)
    {
        EXV_Finished_Count |= 0x04;
        DisplayChineseCharacter(FOURTH_LINE + 6,"0x10", strlen("0x10"));
        HAL_Delay(100);
    }
}

/**
 * 数据处理函数
 */
void LIN_Data_Process(uint8_t RXLength)
{
    //电机转动步长
    uint16_t EXV_Run_Step = 0;
    //通过校验位-校验数据
    uint8_t ckm = 0;
    //pLINRxBuff + 2表示从接收的第3个数据开始，因为接收数组第1个是同步间隔段，第2个是同步段（0x55）
    ckm = LIN_Check_Sum_En(pLINRxBuff + 2,LIN_CHECK_EN_NUM);
    //检查电机与测试板之间的连接是否正常
    if (RXLength < LIN_RX_MAXSIZE)
    {
        if(pLINRxBuff[2] == LIN_PID_53_0x35)
        {
            EXV_Compare_Count &= 0x06;
        }
        if(pLINRxBuff[2] == LIN_PID_55_0x37)
        {
            EXV_Compare_Count &= 0x05;
        }
        if(pLINRxBuff[2] == LIN_PID_32_0x20)
        {
            EXV_Compare_Count &= 0x03;
        }
//        Feedback_Signal(EXV_RESP_CHIP_ERROR);
    }
    //如果校验不通过，丢弃这帧数据
    else if(ckm != pLINRxBuff[LIN_RX_MAXSIZE - 1] || pLINRxBuff[2] == LIN_PID_52_0x34 || pLINRxBuff[2] == LIN_PID_54_0x36 || pLINRxBuff[2] == LIN_PID_16_0x10)
    {
        //不需要操作
    }
    //解析数据具有优先级：LIN通信故障->电机故障->电压异常->温度异常->电机停止标志->判断步长
    //校验LIN通信故障反馈
    else if((pLINRxBuff[3] & EXV_F_RESP_COMP) == EXV_F_RESP_ERROR)
    {
        Feedback_Signal(EXV_RESP_LIN_COMM_ERROR);
    }
        //检查初始化状态，解决反馈数据中以E2，E3开始的数据帧
    else if((pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_NOT || (pLINRxBuff[3] & EXV_ST_INIT_COMP) == EXV_ST_INIT_PROCESS)
    {
        //不需要操作
    }
    //校验故障状态
    else if((pLINRxBuff[4] & EXV_ST_FAULT_COMP) > 0)
    {
        uint8_t fault_index = pLINRxBuff[4] & EXV_ST_FAULT_COMP;
        switch(fault_index)
        {
            case EXV_ST_FAULT_SHORTED:
                Feedback_Signal(EXV_RESP_MC_SHORT);
                break;
            case EXV_ST_FAULT_OPENLOAD:
                Feedback_Signal(EXV_RESP_MC_LOADOPEN);
                break;
            case EXV_ST_FAULT_OVERTEMP:
                Feedback_Signal(EXV_RESP_SHUTDOWN);
                break;
            case EXV_ST_FAULT_ACTUATORFAULT:
                Feedback_Signal(EXV_RESP_ACTUATOR_FAULT);
                break;
        }
    }
    //校验电压状态
    else if((pLINRxBuff[4] & EXV_ST_VOLTAGE_COMP) > 0)
    {
        uint8_t voltage_index = pLINRxBuff[4] & EXV_ST_VOLTAGE_COMP;
        switch(voltage_index)
        {
            case EXV_ST_VOLTAGE_OVER:
                Feedback_Signal(EXV_RESP_OVER_VOLTAGE);
                break;
            case EXV_ST_VOLTAGE_UNDER:
                Feedback_Signal(EXV_RESP_UNDER_VOLTAGE);
                break;
        }
    }
    //校验温度状态
    else if((pLINRxBuff[4] & EXV_OVERTEMP_COMP) == EXV_OVERTEMP_OVER)
    {
        Feedback_Signal(EXV_RESP_OVERTEMP);
    }
    //电机停止转动
    else if((pLINRxBuff[3] & EXV_ST_RUN_COMP) == EXV_ST_RUN_NOT_MOVE)
    {
        //计算电机转动步长，步长低字节在前高字节在后
        EXV_Run_Step = (pLINRxBuff[6] << 8) | pLINRxBuff[5];
        if(EXV_Run_Step == EXV_Test_Step)
        {
            CollectCompleteEXVNum();
            if(EXV_Finished_Count == EXV_Compare_Count)
            {
                Feedback_Signal(EXV_RESP_OK);
            }
        }
    }

    //这帧数据解析完成，清空接收缓存数据
    memset(pLINRxBuff,0,LIN_RX_MAXSIZE);
}