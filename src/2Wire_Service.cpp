#include "2Wire_Service.h"
#include "Arduino.h"
/**
* @note 本示例由 BIOZ 例程修改而来，面向**两线阻抗测量**。
* 默认使用的引脚为 **CE0** 和 **AIN2**。
* 与原 BIOZ 的不同之处在于：将“人体电压测量”替换为“**激励电压测量**”，且该电压**只测量一次**。
*/

/* 应用程序配置结构体。由用户根据模板指定。
这些变量在整个应用程序中都可用。
它包含序列发生器的基本配置和应用相关参数
*/
AppBIOZCfg_Type AppBIOZCfg;

AD5940Err AppBIOZCfg_init(){

  AppBIOZCfg.bParaChanged = bFALSE;
AppBIOZCfg.SeqStartAddr = 0;
AppBIOZCfg.MaxSeqLen = 0;

AppBIOZCfg.SeqStartAddrCal = 0;
AppBIOZCfg.MaxSeqLenCal = 0;

AppBIOZCfg.ReDoRtiaCal = bFALSE;
AppBIOZCfg.SysClkFreq = 16000000.0;
AppBIOZCfg.WuptClkFreq = 32000.0;
AppBIOZCfg.AdcClkFreq = 16000000.0;
AppBIOZCfg.BIOZODR = 20.0; /* 20.0 赫兹 */
AppBIOZCfg.NumOfData = -1;
AppBIOZCfg.RcalVal = 10000.0; /* 10k 欧姆 */

AppBIOZCfg.PwrMod = AFEPWR_LP;
AppBIOZCfg.HstiaRtiaSel = HSTIARTIA_10K;
AppBIOZCfg.CtiaSel = 16;
AppBIOZCfg.ExcitBufGain = EXCITBUFGAIN_2;
AppBIOZCfg.HsDacGain = HSDACGAIN_1;
AppBIOZCfg.HsDacUpdateRate = 7;
AppBIOZCfg.DacVoltPP = 600.0;

AppBIOZCfg.SinFreq = 50000.0; /* 50000 赫兹 */

AppBIOZCfg.ADCPgaGain = ADCPGA_1P5;
AppBIOZCfg.ADCSinc3Osr = ADCSINC3OSR_2;
AppBIOZCfg.ADCSinc2Osr = ADCSINC2OSR_22;

AppBIOZCfg.DftNum = DFTNUM_8192;
AppBIOZCfg.DftSrc = DFTSRC_SINC3;
AppBIOZCfg.HanWinEn = bTRUE;

AppBIOZCfg.SweepCfg.SweepEn = bFALSE;
AppBIOZCfg.SweepCfg.SweepStart = 10000;
AppBIOZCfg.SweepCfg.SweepStop = 150000.0;
AppBIOZCfg.SweepCfg.SweepPoints = 100;
AppBIOZCfg.SweepCfg.SweepLog = bTRUE;
AppBIOZCfg.SweepCfg.SweepIndex = 0;

AppBIOZCfg.FifoThresh = 4; /* 当 SweepEn = bTRUE 时必须为 4 */
AppBIOZCfg.BIOZInited = bFALSE;
AppBIOZCfg.StopRequired = bFALSE;

return AD5940ERR_OK;
}

/**
该函数供上位控制器调用，用于**获取并修改应用参数**（尤其是用户自定义的参数）。
*/
AD5940Err AppBIOZGetCfg(void *pCfg)
{
  if(pCfg){
    *(AppBIOZCfg_Type**)pCfg = &AppBIOZCfg;
    return AD5940ERR_OK;
  }
  return AD5940ERR_PARA;
}

AD5940Err AppBIOZCtrl(int32_t BcmCtrl, void *pPara)
{
  switch (BcmCtrl)
  {
  case BIOZCTRL_START:
    {

      break;
    }
  case BIOZCTRL_STOPNOW:
    {
      if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
        return AD5940ERR_WAKEUP;  /* 唤醒失败 */
      /* 立即停止 Wupt */
      AD5940_WUPTCtrl(bFALSE);
      AD5940_WUPTCtrl(bFALSE);
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Stop Now...\n");
#endif
      break;
    }
  case BIOZCTRL_STOPSYNC:
    {
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Stop SYNC...\n");
#endif
      AppBIOZCfg.StopRequired = bTRUE;
      break;
    }
  case BIOZCTRL_GETFREQ:
    if(pPara)
    {
      if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
        *(float*)pPara = AppBIOZCfg.FreqofData;
      else
        *(float*)pPara = AppBIOZCfg.SinFreq;
    }
    break;
  case BIOZCTRL_SHUTDOWN:
    {
      AppBIOZCtrl(BIOZCTRL_STOPNOW, 0);  /* 如果测量正在运行，则停止它。 */
      /* 关闭那些不受休眠操作自动控制的 LPloop 相关模块 */
      AFERefCfg_Type aferef_cfg;
      LPLoopCfg_Type lp_loop;
      memset(&aferef_cfg, 0, sizeof(aferef_cfg));
      AD5940_REFCfgS(&aferef_cfg);
      memset(&lp_loop, 0, sizeof(lp_loop));
      AD5940_LPLoopCfgS(&lp_loop);
      AD5940_EnterSleepS();  /* 进入休眠模式 */
#ifdef ADI_DEBUG
      ADI_Print("BIOZ Shut down...\n");
#endif
    }
    break;
  default:
    break;
  }
  return AD5940ERR_OK;
}

/* 生成初始化序列 */
static AD5940Err AppBIOZSeqCfgGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  AFERefCfg_Type aferef_cfg;
  HSLoopCfg_Type hs_loop;
  DSPCfg_Type dsp_cfg;
  float sin_freq;

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);

  aferef_cfg.HpBandgapEn = bTRUE;
  aferef_cfg.Hp1V1BuffEn = bTRUE;
  aferef_cfg.Hp1V8BuffEn = bTRUE;
  aferef_cfg.Disc1V1Cap = bFALSE;
  aferef_cfg.Disc1V8Cap = bFALSE;
  aferef_cfg.Hp1V8ThemBuff = bFALSE;
  aferef_cfg.Hp1V8Ilimit = bFALSE;
  aferef_cfg.Lp1V1BuffEn = bFALSE;
  aferef_cfg.Lp1V8BuffEn = bFALSE;

  /* LP 基准控制 - 关闭它们以节省功耗*/
  aferef_cfg.LpBandgapEn = bTRUE;
  aferef_cfg.LpRefBufEn = bTRUE;
  aferef_cfg.LpRefBoostEn = bFALSE;
  AD5940_REFCfgS(&aferef_cfg);
  hs_loop.HsDacCfg.ExcitBufGain = AppBIOZCfg.ExcitBufGain;
  hs_loop.HsDacCfg.HsDacGain = AppBIOZCfg.HsDacGain;
  hs_loop.HsDacCfg.HsDacUpdateRate = AppBIOZCfg.HsDacUpdateRate;

  hs_loop.HsTiaCfg.DiodeClose = bFALSE;
  hs_loop.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hs_loop.HsTiaCfg.HstiaCtia = AppBIOZCfg.CtiaSel; /* 31pF + 2pF */
  hs_loop.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hs_loop.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hs_loop.HsTiaCfg.HstiaRtiaSel = AppBIOZCfg.HstiaRtiaSel;

  hs_loop.SWMatCfg.Dswitch = SWD_OPEN;
  hs_loop.SWMatCfg.Pswitch = SWP_PL|SWP_PL2;
  hs_loop.SWMatCfg.Nswitch = SWN_NL|SWN_NL2;
  hs_loop.SWMatCfg.Tswitch = SWT_TRTIA;

  hs_loop.WgCfg.WgType = WGTYPE_SIN;
  hs_loop.WgCfg.GainCalEn = bFALSE;
  hs_loop.WgCfg.OffsetCalEn = bFALSE;
  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBIOZCfg.SweepCfg.SweepIndex = 0;
    AppBIOZCfg.FreqofData = AppBIOZCfg.SweepCfg.SweepStart;
    AppBIOZCfg.SweepCurrFreq = AppBIOZCfg.SweepCfg.SweepStart;
    AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &AppBIOZCfg.SweepNextFreq);
    sin_freq = AppBIOZCfg.SweepCurrFreq;
  }
  else
  {
    sin_freq = AppBIOZCfg.SinFreq;
    AppBIOZCfg.FreqofData = sin_freq;
  }
  hs_loop.WgCfg.SinCfg.SinFreqWord = AD5940_WGFreqWordCal(sin_freq, AppBIOZCfg.SysClkFreq);
  hs_loop.WgCfg.SinCfg.SinAmplitudeWord = (uint32_t)(AppBIOZCfg.DacVoltPP/800.0f*2047 + 0.5f);
  hs_loop.WgCfg.SinCfg.SinOffsetWord = 0;
  hs_loop.WgCfg.SinCfg.SinPhaseWord = 0;
  AD5940_HSLoopCfgS(&hs_loop);

  dsp_cfg.ADCBaseCfg.ADCMuxN = ADCMUXN_HSTIA_N;
  dsp_cfg.ADCBaseCfg.ADCMuxP = ADCMUXP_HSTIA_P;
  dsp_cfg.ADCBaseCfg.ADCPga = AppBIOZCfg.ADCPgaGain;

  memset(&dsp_cfg.ADCDigCompCfg, 0, sizeof(dsp_cfg.ADCDigCompCfg));

  dsp_cfg.ADCFilterCfg.ADCAvgNum = ADCAVGNUM_16;  /* 此设置无关紧要（该功能未启用） */
  dsp_cfg.ADCFilterCfg.ADCRate = ADCRATE_800KHZ;	/* 告诉滤波链当前 ADC 时钟速率 */
  dsp_cfg.ADCFilterCfg.ADCSinc2Osr = AppBIOZCfg.ADCSinc2Osr;
  dsp_cfg.ADCFilterCfg.ADCSinc3Osr = AppBIOZCfg.ADCSinc3Osr;
  dsp_cfg.ADCFilterCfg.BpSinc3 = bFALSE;
  dsp_cfg.ADCFilterCfg.BpNotch = bTRUE;
  dsp_cfg.ADCFilterCfg.Sinc2NotchEnable = bTRUE;
  dsp_cfg.DftCfg.DftNum = AppBIOZCfg.DftNum;
  dsp_cfg.DftCfg.DftSrc = AppBIOZCfg.DftSrc;
  dsp_cfg.DftCfg.HanWinEn = AppBIOZCfg.HanWinEn;

  memset(&dsp_cfg.StatCfg, 0, sizeof(dsp_cfg.StatCfg)); /* 不关心统计数据 */
  AD5940_DSPCfgS(&dsp_cfg);

  /* 启用所有这些。它们在休眠模式下会自动关闭以节省功耗 */
  AD5940_AFECtrlS(AFECTRL_HPREFPWR|AFECTRL_HSTIAPWR|AFECTRL_INAMPPWR|AFECTRL_EXTBUFPWR|\
    AFECTRL_DACREFPWR|AFECTRL_HSDACPWR|\
      AFECTRL_SINC2NOTCH, bTRUE);
  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .

  /* 序列结束。 */
  AD5940_SEQGenInsert(SEQ_STOP()); /* 添加一个额外的命令来禁用初始化序列的序列发生器，因为我们只想让它运行一次。 */

  /* 在此处停止 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列发生器 */
  if(error == AD5940ERR_OK)
  {
    AppBIOZCfg.InitSeqInfo.SeqId = SEQID_1;
    AppBIOZCfg.InitSeqInfo.SeqRamAddr = AppBIOZCfg.SeqStartAddr;
    AppBIOZCfg.InitSeqInfo.pSeqCmd = pSeqCmd;
    AppBIOZCfg.InitSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 SRAM */
    AD5940_SEQCmdWrite(AppBIOZCfg.InitSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 错误 */
  return AD5940ERR_OK;
}

static AD5940Err AppBIOZSeqMeasureGen(void)
{
  AD5940Err error = AD5940ERR_OK;
  uint32_t const *pSeqCmd;
  uint32_t SeqLen;

  uint32_t WaitClks;
  SWMatrixCfg_Type sw_cfg;
  ClksCalInfo_Type clks_cal;

  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = AppBIOZCfg.DftSrc;
  clks_cal.DataCount = 1L<<(AppBIOZCfg.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = AppBIOZCfg.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = AppBIOZCfg.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBIOZCfg.SysClkFreq/AppBIOZCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

  /* 在此处启动序列发生器 */
  AD5940_SEQGenCtrl(bTRUE);

  AD5940_SEQGpioCtrlS(AGPIO_Pin1/*|AGPIO_Pin5|AGPIO_Pin1*/);//GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .

	/* 配置开关矩阵以连接传感器 */
  sw_cfg.Dswitch = AppBIOZCfg.DswitchSel;
  sw_cfg.Pswitch = AppBIOZCfg.PswitchSel;
  sw_cfg.Nswitch = AppBIOZCfg.NswitchSel;
  sw_cfg.Tswitch = AppBIOZCfg.TswitchSel|SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg);

  AD5940_SEQGenInsert(SEQ_WAIT(16*250));
  /* 步骤 1：测量电流 */
  AD5940_ADCMuxCfgS(ADCMUXP_HSTIA_P, ADCMUXN_HSTIA_N);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* 启用波形发生器, ADC 电源 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* 启动 ADC 转换和 DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* 等待第一个数据就绪 */
	AD5940_SEQGenInsert(SEQ_WAIT(1));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* 停止 ADC 转换和 DFT */

  /* 步骤 2：测量电压 */
  AD5940_ADCMuxCfgS(ADCMUXP_VCE0, ADCMUXN_N_NODE);
  AD5940_AFECtrlS(AFECTRL_WG|AFECTRL_ADCPWR, bTRUE);  /* 启用波形发生器, ADC 电源 */
  AD5940_SEQGenInsert(SEQ_WAIT(16*50));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT, bTRUE);  /* 启动 ADC 转换和 DFT */
  AD5940_SEQGenInsert(SEQ_WAIT(WaitClks));  /* 等待第一个数据就绪 */
	AD5940_SEQGenInsert(SEQ_WAIT(1));
  AD5940_AFECtrlS(AFECTRL_ADCCNV|AFECTRL_DFT|AFECTRL_WG|AFECTRL_ADCPWR, bFALSE);  /* 停止 ADC 转换和 DFT */

  sw_cfg.Dswitch = SWD_OPEN;
  sw_cfg.Pswitch = SWP_PL|SWP_PL2;
  sw_cfg.Nswitch = SWN_NL|SWN_NL2;
  sw_cfg.Tswitch = SWT_TRTIA;
  AD5940_SWMatrixCfgS(&sw_cfg); /* 浮空开关 */

  AD5940_SEQGpioCtrlS(0/*AGPIO_Pin6|AGPIO_Pin5|AGPIO_Pin1*/);        //GP6->结束序列, GP5 -> AD8233=关闭, GP1->RLD=关闭 .
  AD5940_EnterSleepS();/* 进入休眠 */
  /* 序列结束。 */
  error = AD5940_SEQGenFetchSeq(&pSeqCmd, &SeqLen);
  AD5940_SEQGenCtrl(bFALSE); /* 停止序列发生器 */

  if(error == AD5940ERR_OK)
  {
    AppBIOZCfg.MeasureSeqInfo.SeqId = SEQID_0;
    AppBIOZCfg.MeasureSeqInfo.SeqRamAddr = AppBIOZCfg.InitSeqInfo.SeqRamAddr + AppBIOZCfg.InitSeqInfo.SeqLen ;
    AppBIOZCfg.MeasureSeqInfo.pSeqCmd = pSeqCmd;
    AppBIOZCfg.MeasureSeqInfo.SeqLen = SeqLen;
    /* 将命令写入 SRAM */
    AD5940_SEQCmdWrite(AppBIOZCfg.MeasureSeqInfo.SeqRamAddr, pSeqCmd, SeqLen);
  }
  else
    return error; /* 错误 */
  return AD5940ERR_OK;
}

static AD5940Err AppBIOZRtiaCal(void)
{
  HSRTIACal_Type hsrtia_cal;
  FreqParams_Type freq_params;

  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    hsrtia_cal.fFreq = AppBIOZCfg.SweepCfg.SweepStart;
    freq_params = AD5940_GetFreqParameters(AppBIOZCfg.SweepCfg.SweepStart);
  }
  else
  {
    hsrtia_cal.fFreq = AppBIOZCfg.SinFreq;
    freq_params = AD5940_GetFreqParameters(AppBIOZCfg.SinFreq);
  }

  if(freq_params.HighPwrMode == bTRUE)
    hsrtia_cal.AdcClkFreq = 32e6;
  else
    hsrtia_cal.AdcClkFreq = 16e6;
  hsrtia_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  hsrtia_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  hsrtia_cal.DftCfg.DftNum = freq_params.DftNum;
  hsrtia_cal.DftCfg.DftSrc = freq_params.DftSrc;
  hsrtia_cal.bPolarResult = bTRUE; /* 我们在这里需要幅度和相位 */
  hsrtia_cal.DftCfg.HanWinEn = AppBIOZCfg.HanWinEn;
  hsrtia_cal.fRcal= AppBIOZCfg.RcalVal;
  hsrtia_cal.HsTiaCfg.DiodeClose = bFALSE;
  hsrtia_cal.HsTiaCfg.HstiaBias = HSTIABIAS_1P1;
  hsrtia_cal.HsTiaCfg.HstiaCtia = AppBIOZCfg.CtiaSel;
  hsrtia_cal.HsTiaCfg.HstiaDeRload = HSTIADERLOAD_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaDeRtia = HSTIADERTIA_OPEN;
  hsrtia_cal.HsTiaCfg.HstiaRtiaSel = AppBIOZCfg.HstiaRtiaSel;
  hsrtia_cal.SysClkFreq = AppBIOZCfg.SysClkFreq;


  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    uint32_t i;
    AppBIOZCfg.SweepCfg.SweepIndex = 0;  /* 复位扫频索引 */
    for(i=0;i<AppBIOZCfg.SweepCfg.SweepPoints;i++)
    {
      AD5940_HSRtiaCal(&hsrtia_cal, &AppBIOZCfg.RtiaCalTable[i]);
#ifdef ADI_DEBUG
      ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppBIOZCfg.RtiaCalTable[i].Real, AppBIOZCfg.RtiaCalTable[i].Image);
#endif
      AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &hsrtia_cal.fFreq);
      freq_params = AD5940_GetFreqParameters(hsrtia_cal.fFreq);

      if(freq_params.HighPwrMode == bTRUE)
      {
        hsrtia_cal.AdcClkFreq = 32e6;
        /* 将时钟切换到 32MHz 振荡器 */
        AD5940_HPModeEn(bTRUE);
      }
      else
      {
        hsrtia_cal.AdcClkFreq = 16e6;
        /* 将时钟切换为 16MHz 振荡器 */
	AD5940_HPModeEn(bFALSE);
      }
      hsrtia_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
      hsrtia_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
      hsrtia_cal.DftCfg.DftNum = freq_params.DftNum;
      hsrtia_cal.DftCfg.DftSrc = freq_params.DftSrc;
    }

    AppBIOZCfg.SweepCfg.SweepIndex = 0;  /* 复位扫频索引 */
    AppBIOZCfg.RtiaCurrValue = AppBIOZCfg.RtiaCalTable[0];
  }
  else
  {
    AD5940_HSRtiaCal(&hsrtia_cal, &AppBIOZCfg.RtiaCurrValue);
#ifdef ADI_DEBUG
    ADI_Print("Freq:%.2f, (%f, %f)Ohm\n", hsrtia_cal.fFreq, AppBIOZCfg.RtiaCurrValue.Real, AppBIOZCfg.RtiaCurrValue.Image);
#endif
  }
  return AD5940ERR_OK;
}

/* 该函数完成应用初始化（参数、序列、FIFO、校准等） */
AD5940Err AppBIOZInit(uint32_t *pBuffer, uint32_t BufferSize)
{
  AD5940Err error = AD5940ERR_OK;
  SEQCfg_Type seq_cfg;
  FIFOCfg_Type fifo_cfg;

  if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  /* 配置序列发生器并停止它 */
  seq_cfg.SeqMemSize = SEQMEMSIZE_2KB;  /* 2kB SRAM 用于序列发生器，其余用于数据 FIFO */
  seq_cfg.SeqBreakEn = bFALSE;
  seq_cfg.SeqIgnoreEn = bFALSE;
  seq_cfg.SeqCntCRCClr = bTRUE;
  seq_cfg.SeqEnable = bFALSE;
  seq_cfg.SeqWrTimer = 0;
  AD5940_SEQCfg(&seq_cfg);

  /* 执行 RTIA 校准 */
  if((AppBIOZCfg.ReDoRtiaCal == bTRUE) || \
    AppBIOZCfg.BIOZInited == bFALSE)  /* 在第一次初始化时执行校准 */
  {
    AppBIOZRtiaCal();
    AppBIOZCfg.ReDoRtiaCal = bFALSE;
  }
  /* 重新配置 FIFO */
  AD5940_FIFOCtrlS(FIFOSRC_DFT, bFALSE);									/* 首先禁用 FIFO */
  fifo_cfg.FIFOEn = bTRUE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;                       /* 4kB 用于 FIFO，其余 2kB 用于序列发生器 */
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = AppBIOZCfg.FifoThresh;              /* DFT 结果。一对用于 RCAL，另一对用于 Rz。一个 DFT 结果包含实部和虚部 */
  AD5940_FIFOCfg(&fifo_cfg);

  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* 启动序列发生器 */
  /* 初始化序列发生器 */
  if((AppBIOZCfg.BIOZInited == bFALSE)||\
    (AppBIOZCfg.bParaChanged == bTRUE))
  {
    if(pBuffer == 0)  return AD5940ERR_PARA;
    if(BufferSize == 0) return AD5940ERR_PARA;
    AD5940_SEQGenInit(pBuffer, BufferSize);

    /* 生成初始化序列 */
    error = AppBIOZSeqCfgGen(); /* 使用 MCU 或序列发生器的应用程序初始化序列 */
    if(error != AD5940ERR_OK) return error;

    /* 生成测量序列 */
    error = AppBIOZSeqMeasureGen();
    if(error != AD5940ERR_OK) return error;

    AppBIOZCfg.bParaChanged = bFALSE; /* 清除此标志，因为我们已经实现了新的配置 */
  }

  /* 初始化序列发生器  */
  AppBIOZCfg.InitSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIOZCfg.InitSeqInfo);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* 启用序列发生器 */
  AD5940_SEQMmrTrig(AppBIOZCfg.InitSeqInfo.SeqId);
  while(AD5940_INTCTestFlag(AFEINTC_1, AFEINTSRC_ENDSEQ) == bFALSE);
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* 测量序列  */
  AppBIOZCfg.MeasureSeqInfo.WriteSRAM = bFALSE;
  AD5940_SEQInfoCfg(&AppBIOZCfg.MeasureSeqInfo);

  AppBIOZCheckFreq(AppBIOZCfg.FreqofData);
  seq_cfg.SeqEnable = bTRUE;
  AD5940_SEQCfg(&seq_cfg);  /* 启用序列发生器，并等待触发 */
  AD5940_ClrMCUIntFlag();   /* 清除之前生成的中断标志 */

  AppBIOZCfg.BIOZInited = bTRUE;  /* BIOZ 应用程序已初始化。 */
  return AD5940ERR_OK;
}

/* 根据正弦波的频率设置最佳滤波器设置 */
AD5940Err AppBIOZCheckFreq(float freq)
{
  ADCFilterCfg_Type filter_cfg;
  DFTCfg_Type dft_cfg;
  HSDACCfg_Type hsdac_cfg;
  uint32_t WaitClks;
  ClksCalInfo_Type clks_cal;
  FreqParams_Type freq_params;
  uint32_t SeqCmdBuff[2];
  uint32_t SRAMAddr = 0;;
  /* 步骤 1：检查频率 */
  freq_params = AD5940_GetFreqParameters(freq);

	/* 设置电源模式 */
  if(freq_params.HighPwrMode == bTRUE)
  {
    /* 更新 HSDAC 更新速率 */
    hsdac_cfg.ExcitBufGain = AppBIOZCfg.ExcitBufGain;
    hsdac_cfg.HsDacGain = AppBIOZCfg.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x7;
    AD5940_HSDacCfgS(&hsdac_cfg);

    /*更新 ADC 速率 */
    filter_cfg.ADCRate = ADCRATE_1P6MHZ;
    AppBIOZCfg.AdcClkFreq = 32e6;

    /* 将时钟切换到 32MHz 振荡器 */
    AD5940_HPModeEn(bTRUE);
  }else
	{
    /* 更新 HSDAC 更新速率 */
    hsdac_cfg.ExcitBufGain = AppBIOZCfg.ExcitBufGain;
    hsdac_cfg.HsDacGain = AppBIOZCfg.HsDacGain;
    hsdac_cfg.HsDacUpdateRate = 0x1B;
    AD5940_HSDacCfgS(&hsdac_cfg);
    /* 更新 ADC 速率 */
    filter_cfg.ADCRate = ADCRATE_800KHZ;
    AppBIOZCfg.AdcClkFreq = 16e6;

		/* 将时钟切换为 16MHz 振荡器 */
    AD5940_HPModeEn(bFALSE);
  }

  /* 步骤 2：调整 ADCFILTERCON 和 DFTCON 以设置最佳的 SINC3、SINC2 和 DFTNUM 设置 */
  filter_cfg.ADCAvgNum = ADCAVGNUM_16;  /* 不关心，因为它被禁用了 */
  filter_cfg.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  filter_cfg.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  filter_cfg.BpSinc3 = bFALSE;
  filter_cfg.BpNotch = bTRUE;
  filter_cfg.Sinc2NotchEnable = bTRUE;
  dft_cfg.DftNum = freq_params.DftNum;
  dft_cfg.DftSrc = freq_params.DftSrc;
  dft_cfg.HanWinEn = AppBIOZCfg.HanWinEn;
  AD5940_ADCFilterCfgS(&filter_cfg);
  AD5940_DFTCfgS(&dft_cfg);

  /* 步骤 3：计算获取结果到 FIFO 所需的时钟，并更新序列发生器等待命令 */
  clks_cal.DataType = DATATYPE_DFT;
  clks_cal.DftSrc = freq_params.DftSrc;
  clks_cal.DataCount = 1L<<(freq_params.DftNum+2); /* 2^(DFTNUMBER+2) */
  clks_cal.ADCSinc2Osr = freq_params.ADCSinc2Osr;
  clks_cal.ADCSinc3Osr = freq_params.ADCSinc3Osr;
  clks_cal.ADCAvgNum = 0;
  clks_cal.RatioSys2AdcClk = AppBIOZCfg.SysClkFreq/AppBIOZCfg.AdcClkFreq;
  AD5940_ClksCalculate(&clks_cal, &WaitClks);

	/* 最大时钟数是 0x3FFFFFFF。如果频率很低，则需要更多时钟 */
	if(WaitClks > 0x3FFFFFFF)
	{
		WaitClks /=2;
		SRAMAddr = AppBIOZCfg.MeasureSeqInfo.SeqRamAddr;
		SeqCmdBuff[0] = SEQ_WAIT(WaitClks);
		AD5940_SEQCmdWrite(SRAMAddr+11, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+12, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+18, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+19, SeqCmdBuff, 1);
	}
	else
	{
		SRAMAddr = AppBIOZCfg.MeasureSeqInfo.SeqRamAddr;
		SeqCmdBuff[0] = SEQ_WAIT(WaitClks);
		AD5940_SEQCmdWrite(SRAMAddr+11, SeqCmdBuff, 1);
		AD5940_SEQCmdWrite(SRAMAddr+18, SeqCmdBuff, 1);
	}

  return AD5940ERR_OK;
}

/* AFE 唤醒时修改寄存器 */
static AD5940Err AppBIOZRegModify(int32_t * const pData, uint32_t *pDataCount)
{
  if(AppBIOZCfg.NumOfData > 0)
  {
    AppBIOZCfg.FifoDataCount += *pDataCount/4;
    if(AppBIOZCfg.FifoDataCount >= AppBIOZCfg.NumOfData)
    {
      AD5940_WUPTCtrl(bFALSE);
      return AD5940ERR_OK;
    }
  }
  if(AppBIOZCfg.StopRequired == bTRUE)
  {
    AD5940_WUPTCtrl(bFALSE);
    return AD5940ERR_OK;
  }
  if(AppBIOZCfg.SweepCfg.SweepEn) /* 需要设置新频率并设置电源模式 */
  {
    AppBIOZCheckFreq(AppBIOZCfg.SweepNextFreq);
    AD5940_WGFreqCtrlS(AppBIOZCfg.SweepNextFreq, AppBIOZCfg.SysClkFreq);
  }
  return AD5940ERR_OK;
}

/* 根据数据类型，在返回给控制器之前进行适当的数据预处理 */
static AD5940Err AppBIOZDataProcess(int32_t * const pData, uint32_t *pDataCount)
{
  uint32_t DataCount = *pDataCount;
  uint32_t ImpResCount = DataCount/4;

  fImpCar_Type * pOut = (fImpCar_Type*)pData;
  iImpCar_Type * pSrcData = (iImpCar_Type*)pData;

  *pDataCount = 0;

  DataCount = (DataCount/4)*4; /* 一个 DFT 结果在 FIFO 中有两个数据，实部和虚部。每次测量有 2 个 DFT 结果，一个用于电压测量，一个用于电流测量 */

  /* 将 DFT 结果转换为 int32_t 类型 */
  for(uint32_t i=0; i<DataCount; i++)
  {
    pData[i] &= 0x3ffff;
    if(pData[i]&(1<<17)) /* 第 17 位是符号位 */
    {
      pData[i] |= 0xfffc0000; /* 数据是 18 位补码，第 17 位是符号位 */
    }
  }
  for(uint32_t i=0; i<ImpResCount; i++)
  {
    fImpCar_Type DftCurr, DftVolt;
    fImpCar_Type res;

    DftCurr.Real = (float)pSrcData[i].Real;
    DftCurr.Image = (float)pSrcData[i].Image;
    DftVolt.Real = (float)pSrcData[i+1].Real;
    DftVolt.Image = (float)pSrcData[i+1].Image;

    DftCurr.Real = -DftCurr.Real;
    DftCurr.Image = -DftCurr.Image;
    DftVolt.Real = DftVolt.Real;
    DftVolt.Image = DftVolt.Image;
    res = AD5940_ComplexDivFloat(&DftCurr, &AppBIOZCfg.RtiaCurrValue);           /* I=Vrtia/Zrtia */
    res = AD5940_ComplexDivFloat(&DftVolt, &res);
    pOut[i] = res;
  }
  *pDataCount = ImpResCount;
  /* 计算下一个频率点 */
  if(AppBIOZCfg.SweepCfg.SweepEn == bTRUE)
  {
    AppBIOZCfg.FreqofData = AppBIOZCfg.SweepCurrFreq;
    AppBIOZCfg.SweepCurrFreq = AppBIOZCfg.SweepNextFreq;
    AppBIOZCfg.RtiaCurrValue = AppBIOZCfg.RtiaCalTable[AppBIOZCfg.SweepCfg.SweepIndex];
    AD5940_SweepNext(&AppBIOZCfg.SweepCfg, &AppBIOZCfg.SweepNextFreq);
  }
  return AD5940ERR_OK;
}

/**

*/
AD5940Err AppBIOZISR(void *pBuff, uint32_t *pCount)
{
  uint32_t BuffCount;
  uint32_t FifoCnt;
  if(AppBIOZCfg.BIOZInited == bFALSE)
    return AD5940ERR_APPERROR;
  if(AD5940_WakeUp(10) > 10)  /* 通过读寄存器唤醒 AFE，最多读取 10 次 */
    return AD5940ERR_WAKEUP;  /* 唤醒失败 */

  AD5940_SleepKeyCtrlS(SLPKEY_LOCK);  /* 不要进入休眠 */
  *pCount = 0;

  if(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bTRUE)
  {
    /* 现在 FIFO 中应该有 4 个数据 */
    FifoCnt = (AD5940_FIFOGetCnt()/4)*4;
    AD5940_FIFORd((uint32_t *)pBuff, FifoCnt);
    AD5940_INTCClrFlag(AFEINTSRC_DATAFIFOTHRESH);
    AppBIOZRegModify(static_cast<int32_t*>(pBuff), &FifoCnt);   /* 如果需要重新配置 AFE，请在 AFE 处于活动状态时在此处执行 */
    AD5940_EnterSleepS();  /* 手动让 AFE 返回休眠以节省功耗 */
    AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK);  /* 允许 AFE 进入休眠模式 */
    /* 进行数据处理 */
    AppBIOZDataProcess((int32_t*)pBuff,&FifoCnt);
    *pCount = FifoCnt;
    return 0;
  }

  return 0;
}

int32_t AD5940PlatformCfg(void)
{
  CLKCfg_Type clk_cfg;
  FIFOCfg_Type fifo_cfg;
  AGPIOCfg_Type gpio_cfg;

  /* 使用硬复位 (由您的glue层处理) */
  AD5940_HWReset();
  /* 平台配置 */
  AD5940_Initialize();
  /* Step1. 配置时钟 */
  clk_cfg.ADCClkDiv = ADCCLKDIV_1;
  clk_cfg.ADCCLkSrc = ADCCLKSRC_XTAL;
  clk_cfg.SysClkDiv = SYSCLKDIV_1;
  clk_cfg.SysClkSrc = SYSCLKSRC_XTAL;
  clk_cfg.HfOSC32MHzMode = bFALSE;
  clk_cfg.HFOSCEn = bFALSE;
  clk_cfg.HFXTALEn = bTRUE;
  clk_cfg.LFOSCEn = bTRUE;
  AD5940_CLKCfg(&clk_cfg);
  
  /* Step2. 配置FIFO */
  fifo_cfg.FIFOEn = bFALSE;
  fifo_cfg.FIFOMode = FIFOMODE_FIFO;
  fifo_cfg.FIFOSize = FIFOSIZE_4KB;
  fifo_cfg.FIFOSrc = FIFOSRC_DFT;
  fifo_cfg.FIFOThresh = 4; // 1次测量 = 1x I + 1x V = 4个数据 (I_real, I_imag, V_real, V_imag)
  AD5940_FIFOCfg(&fifo_cfg);
  fifo_cfg.FIFOEn = bTRUE;  
  AD5940_FIFOCfg(&fifo_cfg);
  
  /* Step3. 中断控制器 */
  // 使能所有中断标志位，以便我们可以轮询它们
  AD5940_INTCCfg(AFEINTC_1, AFEINTSRC_ALLINT, bTRUE); 
  // 使能FIFO阈值标志位
  AD5940_INTCCfg(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH, bTRUE); 
  AD5940_INTCClrFlag(AFEINTSRC_ALLINT);

  /* Step4: GPIO配置 (不再需要中断引脚) */
  gpio_cfg.FuncSet = GP6_SYNC|GP5_SYNC|GP4_SYNC|GP2_TRIG|GP1_SYNC; // 移除了GP0_INT
  gpio_cfg.InputEnSet = AGPIO_Pin2;
  gpio_cfg.OutputEnSet = AGPIO_Pin1|AGPIO_Pin4|AGPIO_Pin5|AGPIO_Pin6; // 移除了AGPIO_Pin0
  gpio_cfg.OutVal = 0;
  gpio_cfg.PullEnSet = 0;

  //AD5940_AGPIOCfg(&gpio_cfg);

  AD5940_SleepKeyCtrlS(SLPKEY_UNLOCK); 
  return 0;
}

void AD5940BIOZStructInit(void)
{
  AppBIOZCfg_Type *pBIOZCfg; 
  AppBIOZGetCfg(&pBIOZCfg);
  
  pBIOZCfg->SeqStartAddr = 0;
  pBIOZCfg->MaxSeqLen = 512; 
  
	pBIOZCfg->SinFreq = 20e3;			/* 20kHz. This value is ignored if SweepEn = bTRUE */
	pBIOZCfg->RcalVal = 1000.0;	/* Value of RCAl on the evaluaiton board */
	pBIOZCfg->HstiaRtiaSel = HSTIARTIA_1K;	
  
	/* Configure Switch matrix */
	pBIOZCfg->DswitchSel = SWD_CE0;
  pBIOZCfg->PswitchSel = SWP_CE0;
  pBIOZCfg->NswitchSel = SWN_AIN1;
  pBIOZCfg->TswitchSel = SWN_AIN1;
	
  /* Configure Sweep Parameters */
  pBIOZCfg->SweepCfg.SweepEn = bTRUE;
  pBIOZCfg->SweepCfg.SweepStart = 1000;
  pBIOZCfg->SweepCfg.SweepStop = 200000.0;
  pBIOZCfg->SweepCfg.SweepPoints = 40;	/* Maximum is 100 */
  pBIOZCfg->SweepCfg.SweepLog = bFALSE;
	
	pBIOZCfg->BIOZODR = 5;         /* ODR(Sample Rate) 5Hz */
	pBIOZCfg->NumOfData = -1;      	/* Never stop until you stop it manually by AppBIOZCtrl() function */
}

int32_t BIOZShowResult(uint32_t *pData, uint32_t DataCount)
{
  float freq;

  fImpCar_Type *pImp = (fImpCar_Type*)pData;
  AppBIOZCtrl(BIOZCTRL_GETFREQ, &freq);

  /*Process data*/
  for(int i=0;i<DataCount;i++)
  {
		Serial.printf("Freq:%.2f ", freq);
    //printf("RzMag: %f Ohm , RzPhase: %f \n",AD5940_ComplexMag(&pImp[i]), AD5940_ComplexPhase(&pImp[i])*180/MATH_PI);
		Serial.printf("Impedance:(Real,Image) = (%f,%f)\n", pImp[i].Real, pImp[i].Image);
  }
  return 0;
}
