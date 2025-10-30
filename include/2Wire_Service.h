/*!
 @file:    BIOZ-2Wire.h
 @author:  Neo Xu
 @brief:   **两线 BIOZ 阻抗测量**示例的头文件（由四线版本修改而来）。
 -----------------------------------------------------------------------------

Copyright (c) 2017-2019 Analog Devices, Inc. All Rights Reserved.

This software is proprietary to Analog Devices, Inc. and its licensors.
By using this software you agree to the terms of the associated
Analog Devices Software License Agreement.

*****************************************************************************/

/* 注：为避免歧义，保留了上方版权/许可的英文原文；功能性注释均已翻译为中文。*/

#ifndef _BODYCOMPOSITION_H_
#define _BODYCOMPOSITION_H_
extern "C" {
#include "ad5940.h"
}
#include "stdio.h"
#include "string.h"
#include "math.h"

#define MAXSWEEP_POINTS   100           /* 需要知道需要多少缓冲区来保存 RTIA 校准结果 */

/* 注：本示例将使用 SEQID_0 作为测量序列，使用 SEQID_1 作为初始化序列。
  SEQID_3 用于校准。
*/

typedef struct
{
/* 适用于所有类型应用的通用配置。 */
  BoolFlag bParaChanged;        /* 指示是否需要再次生成序列。AppBIOZInit 会自动清除此标志 */
  uint32_t SeqStartAddr;        /* AD5940 SRAM 中的初始化序列起始地址 */
  uint32_t MaxSeqLen;           /* 限制最大序列长度。 */
  uint32_t SeqStartAddrCal;     /* AD5940 SRAM 中的测量序列起始地址 */
  uint32_t MaxSeqLenCal;
/* 应用相关参数 */
  //BoolFlag bBioElecBoard;     /* 代码对于 BioElec 板和 AD5941Sens1 板是相同的。无需更改 */
  BoolFlag ReDoRtiaCal;         /* 当需要进行校准时，将此标志设置为 bTRUE。 */
  float SysClkFreq;             /* 系统时钟的实际频率 */
  float WuptClkFreq;            /* 唤醒定时器的时钟频率（单位：Hz）。通常为 32kHz。保留在此，以备我们在软件方法中校准时钟 */
  float AdcClkFreq;             /* ADC 时钟的实际频率 */
  uint32_t FifoThresh;           /* FIFO 阈值。应为 N*2 */
  float BIOZODR;                 /* 单位：Hz。ODR 决定了唤醒定时器（WuptClkFreq）的周期，它将周期性地触发序列发生器。DFT 点数和采样频率决定了最大 ODR。 */
  int32_t NumOfData;            /* 默认为 '-1'。如果希望引擎在获取 NumofData 个数据后停止，请在此处设置该值。否则，将其设置为 '-1'，表示永不停止。 */
  float SinFreq;                /* 激励信号的频率 */
  float RcalVal;                /* Rcal 电阻值（单位：欧姆） */
  uint32_t PwrMod;              /* 控制芯片功耗模式 (LP/HP) */
  float DacVoltPP;              /* 最终激励电压为 DacVoltPP*DAC_PGA*EXCIT_GAIN, DAC_PGA= 1 或 0.2, EXCIT_GAIN=2 或 0.25。DAC 输出电压（单位：mV 峰峰值）。最大值为 800mVpp。峰峰值电压 */
  uint32_t ExcitBufGain;        /* 从 EXCITBUFGAIN_2, EXCITBUFGAIN_0P25 中选择 */
  uint32_t HsDacGain;           /* 从 HSDACGAIN_1, HSDACGAIN_0P2 中选择 */
  uint32_t HsDacUpdateRate;     /* DAC 更新速率为 SystemCLoock/Divider。可用值为 7 到 255。设置为 7 以获得更好性能 */
  uint32_t ADCPgaGain;          /* PGA 增益从 GNPGA_1, GNPGA_1_5, GNPGA_2, GNPGA_4, GNPGA_9 中选择 !!! 我们必须确保信号在 ADC 输入级限制的 +/-1.5V 范围内 */
  uint8_t ADCSinc3Osr;          /* SINC3 OSR 选择。ADCSINC3OSR_2, ADCSINC3OSR_4 */
  uint8_t ADCSinc2Osr;          /* SINC2 OSR 选择。ADCSINC2OSR_22...ADCSINC2OSR_1333 */
  uint32_t HstiaRtiaSel;        /* 使用内部 RTIA，从 RTIA_INT_200, RTIA_INT_1K, RTIA_INT_5K, RTIA_INT_10K, RTIA_INT_20K, RTIA_INT_40K, RTIA_INT_80K, RTIA_INT_160K 中选择 */
  uint32_t CtiaSel;             /* 选择 CTIA（单位 pF），范围 0 到 31pF */

  uint32_t DftNum;              /* DFT 点数 */
  uint32_t DftSrc;              /* DFT 源 */
  BoolFlag HanWinEn;            /* 启用汉宁窗 */

	/* 开关配置 */
  uint32_t DswitchSel;
  uint32_t PswitchSel;
  uint32_t NswitchSel;
  uint32_t TswitchSel;

  /* 扫频功能控制 */
  SoftSweepCfg_Type SweepCfg;
/* 供内部使用的私有变量 */
  float SweepCurrFreq;
  float SweepNextFreq;
  fImpCar_Type RtiaCurrValue;                   /* 当前频率下校准的 Rtia 值 */
  fImpCar_Type RtiaCalTable[MAXSWEEP_POINTS];   /* 校准的 Rtia 值表 */
  float FreqofData;                             /* 最新采样数据的频率 */
  BoolFlag BIOZInited;                          /* 如果程序首次运行，则生成序列命令 */
  SEQInfo_Type InitSeqInfo;
  SEQInfo_Type MeasureSeqInfo;
  BoolFlag StopRequired;          /* FIFO 准备就绪后，停止测量序列 */
  uint32_t FifoDataCount;         /* 计算阻抗已被测量的次数 */
/* 结束 */
}AppBIOZCfg_Type;
extern AppBIOZCfg_Type AppBIOZCfg; 

#define BIOZCTRL_START          0
#define BIOZCTRL_STOPNOW        1
#define BIOZCTRL_STOPSYNC       2
#define BIOZCTRL_GETFREQ        3   /* 从 ISR 返回的数据流中获取当前频点 */
#define BIOZCTRL_SHUTDOWN       4   /* 注：此处的 shutdown 意味着关闭所有功能并将 AFE 置于休眠模式。'SHUT DOWN' 一词仅在此处使用。 */

AD5940Err AppBIOZGetCfg(void *pCfg);
AD5940Err AppBIOZInit(uint32_t *pBuffer, uint32_t BufferSize);
AD5940Err AppBIOZISR(void *pBuff, uint32_t *pCount);
AD5940Err AppBIOZCtrl(int32_t BcmCtrl, void *pPara);
AD5940Err AppBIOZCheckFreq(float freq);
AD5940Err AppBIOZCfg_init(void);
int32_t AD5940PlatformCfg(void);
void AD5940BIOZStructInit(void);
int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount);
#endif