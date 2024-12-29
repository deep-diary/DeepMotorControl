/*
 * File: FOC.c
 *
 * Code generated for Simulink model 'FOC'.
 *
 * Model version                  : 1.42
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Sun Sep  8 09:12:00 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "FOC.h"
#include "rtwtypes.h"
#include "arm_math.h"
#include <math.h>
#include <stddef.h>
#define NumBitsPerChar                 8U

/* Exported data definition */

/* Definition for custom storage class: Default */
real32_T CurrentKi;                    /* '<Root>/CurrentKi' */

/* 电流Ki */
real32_T CurrentKp;                    /* '<Root>/CurrentKp' */

/* 电流Kp */
real32_T EposOpen;                     /* '<S129>/scaleOut' */

/* 实际电角度 */
real32_T EposReq;                      /* '<Root>/EposReq' */

/* 实际电角度 */
real32_T Ia;                           /* '<Root>/Ia' */

/* a相电流 */
real32_T IaSim;                        /* '<S156>/Ka' */

/* a相仿真电流 */
real32_T Ialpha;                       /* '<S4>/Signal Conversion' */

/* alpha电流 */
real32_T Ib;                           /* '<Root>/Ib' */

/* b相电流 */
real32_T IbSim;                        /* '<S156>/Kb' */

/* b相仿真电流 */
real32_T Ibeta;                        /* '<S4>/Signal Conversion1' */

/* beta电流 */
real32_T Ic;                           /* '<Root>/Ic' */

/* c相电流 */
real32_T IcSim;                        /* '<S156>/Kc' */

/* c相仿真电流 */
real32_T Id;                           /* D轴电流 */
real32_T IdReq;                        /* '<Root>/IdReq' */

/* D轴电流 */
real32_T Iq;                           /* Q轴电流 */
real32_T IqReq;                        /* '<Root>/IqReq' */

/* Q轴电流 */
boolean_T PhaseOrder;                  /* '<Root>/PhaseOrder' */

/* 电机相序 */
real32_T SpReqPU;                      /* '<Root>/SpReqPU' */

/* 斜波处理后的速度 */
boolean_T SysEnable;                   /* '<Root>/SysEnable' */

/* 使能系统 */
uint16_T U;                            /* '<S3>/Signal Conversion' */

/* PWM U */
uint16_T V;                            /* '<S3>/Signal Conversion1' */

/* PWM V */
real32_T VdReqPU;                      /* '<S2>/Signal Conversion' */

/* 斜波处理后的d轴电压 */
real32_T VqReqPU;                      /* '<S2>/Signal Conversion1' */

/* 斜波处理后的q轴电压 */
uint16_T W;                            /* '<S3>/Signal Conversion2' */

/* PWM W */
uint8_T mode;                          /* '<Root>/mode' */

/* 工作模式 */

/* Block signals and states (default storage) */
DW rtDW;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
static void TwoinputsCRL(real32_T rtu_Ds, real32_T rtu_Qs, real32_T rtu_sin,
  real32_T rtu_cos, real32_T *rty_Alpha, real32_T *rty_Beta);
static void IfActionSubsystem(real32_T rtu_In1, real32_T *rty_Out1);
static void IfActionSubsystem1(real32_T rtu_In1, real32_T *rty_Out1);

#define NOT_USING_NONFINITE_LITERALS   1

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;
static void rt_InitInfAndNaN(size_t realSize);
static boolean_T rtIsInf(real_T value);
static boolean_T rtIsInfF(real32_T value);
static boolean_T rtIsNaN(real_T value);
static boolean_T rtIsNaNF(real32_T value);
typedef struct {
  struct {
    uint32_T wordH;
    uint32_T wordL;
  } words;
} BigEndianIEEEDouble;

typedef struct {
  struct {
    uint32_T wordL;
    uint32_T wordH;
  } words;
} LittleEndianIEEEDouble;

typedef struct {
  union {
    real32_T wordLreal;
    uint32_T wordLuint;
  } wordL;
} IEEESingle;

real_T rtInf;
real_T rtMinusInf;
real_T rtNaN;
real32_T rtInfF;
real32_T rtMinusInfF;
real32_T rtNaNF;
static real_T rtGetInf(void);
static real32_T rtGetInfF(void);
static real_T rtGetMinusInf(void);
static real32_T rtGetMinusInfF(void);
static real_T rtGetNaN(void);
static real32_T rtGetNaNF(void);

/*
 * Initialize the rtInf, rtMinusInf, and rtNaN needed by the
 * generated code. NaN is initialized as non-signaling. Assumes IEEE.
 */
static void rt_InitInfAndNaN(size_t realSize)
{
  (void) (realSize);
  rtNaN = rtGetNaN();
  rtNaNF = rtGetNaNF();
  rtInf = rtGetInf();
  rtInfF = rtGetInfF();
  rtMinusInf = rtGetMinusInf();
  rtMinusInfF = rtGetMinusInfF();
}

/* Test if value is infinite */
static boolean_T rtIsInf(real_T value)
{
  return (boolean_T)((value==rtInf || value==rtMinusInf) ? 1U : 0U);
}

/* Test if single-precision value is infinite */
static boolean_T rtIsInfF(real32_T value)
{
  return (boolean_T)(((value)==rtInfF || (value)==rtMinusInfF) ? 1U : 0U);
}

/* Test if value is not a number */
static boolean_T rtIsNaN(real_T value)
{
  boolean_T result = (boolean_T) 0;
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  if (bitsPerReal == 32U) {
    result = rtIsNaNF((real32_T)value);
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.fltVal = value;
    result = (boolean_T)((tmpVal.bitVal.words.wordH & 0x7FF00000) == 0x7FF00000 &&
                         ( (tmpVal.bitVal.words.wordH & 0x000FFFFF) != 0 ||
                          (tmpVal.bitVal.words.wordL != 0) ));
  }

  return result;
}

/* Test if single-precision value is not a number */
static boolean_T rtIsNaNF(real32_T value)
{
  IEEESingle tmp;
  tmp.wordL.wordLreal = value;
  return (boolean_T)( (tmp.wordL.wordLuint & 0x7F800000) == 0x7F800000 &&
                     (tmp.wordL.wordLuint & 0x007FFFFF) != 0 );
}

/*
 * Initialize rtInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T inf = 0.0;
  if (bitsPerReal == 32U) {
    inf = rtGetInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0x7FF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    inf = tmpVal.fltVal;
  }

  return inf;
}

/*
 * Initialize rtInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetInfF(void)
{
  IEEESingle infF;
  infF.wordL.wordLuint = 0x7F800000U;
  return infF.wordL.wordLreal;
}

/*
 * Initialize rtMinusInf needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetMinusInf(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T minf = 0.0;
  if (bitsPerReal == 32U) {
    minf = rtGetMinusInfF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF00000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    minf = tmpVal.fltVal;
  }

  return minf;
}

/*
 * Initialize rtMinusInfF needed by the generated code.
 * Inf is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetMinusInfF(void)
{
  IEEESingle minfF;
  minfF.wordL.wordLuint = 0xFF800000U;
  return minfF.wordL.wordLreal;
}

/*
 * Initialize rtNaN needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real_T rtGetNaN(void)
{
  size_t bitsPerReal = sizeof(real_T) * (NumBitsPerChar);
  real_T nan = 0.0;
  if (bitsPerReal == 32U) {
    nan = rtGetNaNF();
  } else {
    union {
      LittleEndianIEEEDouble bitVal;
      real_T fltVal;
    } tmpVal;

    tmpVal.bitVal.words.wordH = 0xFFF80000U;
    tmpVal.bitVal.words.wordL = 0x00000000U;
    nan = tmpVal.fltVal;
  }

  return nan;
}

/*
 * Initialize rtNaNF needed by the generated code.
 * NaN is initialized as non-signaling. Assumes IEEE.
 */
static real32_T rtGetNaNF(void)
{
  IEEESingle nanF = { { 0.0F } };

  nanF.wordL.wordLuint = 0xFFC00000U;
  return nanF.wordL.wordLreal;
}

/*
 * Output and update for atomic system:
 *    '<S127>/Two inputs CRL'
 *    '<S154>/Two inputs CRL'
 */
static void TwoinputsCRL(real32_T rtu_Ds, real32_T rtu_Qs, real32_T rtu_sin,
  real32_T rtu_cos, real32_T *rty_Alpha, real32_T *rty_Beta)
{
  /* AlgorithmDescriptorDelegate generated from: '<S132>/a16' */
  arm_inv_park_f32(rtu_Ds, rtu_Qs, rty_Alpha, rty_Beta, rtu_sin, rtu_cos);
}

/*
 * Output and update for action system:
 *    '<S138>/If Action Subsystem'
 *    '<S160>/If Action Subsystem'
 *    '<S168>/If Action Subsystem'
 */
static void IfActionSubsystem(real32_T rtu_In1, real32_T *rty_Out1)
{
  /* Sum: '<S140>/Sum' incorporates:
   *  DataTypeConversion: '<S140>/Convert_back'
   *  DataTypeConversion: '<S140>/Convert_uint16'
   */
  *rty_Out1 = rtu_In1 - (real32_T)(int16_T)(real32_T)floor(rtu_In1);
}

/*
 * Output and update for action system:
 *    '<S138>/If Action Subsystem1'
 *    '<S160>/If Action Subsystem1'
 *    '<S168>/If Action Subsystem1'
 */
static void IfActionSubsystem1(real32_T rtu_In1, real32_T *rty_Out1)
{
  /* Sum: '<S141>/Sum' incorporates:
   *  DataTypeConversion: '<S141>/Convert_back'
   *  DataTypeConversion: '<S141>/Convert_uint16'
   */
  *rty_Out1 = rtu_In1 - (real32_T)(int16_T)rtu_In1;
}

/* Model step function */
void FOC_step(void)
{
  real32_T rtb_Add1;
  real32_T rtb_Add3;
  real32_T rtb_PWM_DutyCycles_idx_1;
  real32_T rtb_add_c_l;
  real32_T rtb_algDD_o1;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_LogicalOperator1;

  /* Outputs for Atomic SubSystem: '<Root>/FOC' */
  /* Outputs for Atomic SubSystem: '<S1>/dq0' */
  /* If: '<S160>/If' incorporates:
   *  Constant: '<S161>/Constant'
   *  Inport: '<Root>/EposReq'
   *  RelationalOperator: '<S161>/Compare'
   */
  if (EposReq < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S160>/If Action Subsystem' incorporates:
     *  ActionPort: '<S162>/Action Port'
     */
    IfActionSubsystem(EposReq, &rtb_Add1);

    /* End of Outputs for SubSystem: '<S160>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S160>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S163>/Action Port'
     */
    IfActionSubsystem1(EposReq, &rtb_Add1);

    /* End of Outputs for SubSystem: '<S160>/If Action Subsystem1' */
  }

  /* End of If: '<S160>/If' */

  /* Gain: '<S155>/indexing' */
  rtb_Add1 *= 800.0F;

  /* Sum: '<S155>/Sum2' incorporates:
   *  DataTypeConversion: '<S155>/Data Type Conversion1'
   *  DataTypeConversion: '<S155>/Get_Integer'
   */
  rtb_add_c_l = rtb_Add1 - (real32_T)(uint16_T)rtb_Add1;

  /* Product: '<S159>/Product1' incorporates:
   *  Constant: '<S155>/offset'
   *  Constant: '<S155>/sine_table_values'
   *  DataTypeConversion: '<S155>/Get_Integer'
   *  Selector: '<S155>/Lookup'
   *  Sum: '<S155>/Sum'
   *  Sum: '<S159>/Sum5'
   */
  rtb_Add3 = (rtConstP.pooled7[(int32_T)((uint16_T)rtb_Add1 + 201U)] -
              rtConstP.pooled7[(int32_T)((uint16_T)rtb_Add1 + 200U)]) *
    rtb_add_c_l;

  /* Outputs for Atomic SubSystem: '<S154>/Two inputs CRL' */
  /* Constant: '<S149>/Constant' incorporates:
   *  Constant: '<S155>/offset'
   *  Constant: '<S155>/sine_table_values'
   *  DataTypeConversion: '<S155>/Get_Integer'
   *  Product: '<S159>/Product'
   *  Selector: '<S155>/Lookup'
   *  Sum: '<S155>/Sum'
   *  Sum: '<S159>/Sum3'
   *  Sum: '<S159>/Sum4'
   *  Sum: '<S159>/Sum5'
   *  Sum: '<S159>/Sum6'
   */
  TwoinputsCRL(0.0F, rtConstB.UnaryMinus, (rtConstP.pooled7[(int32_T)((uint16_T)
    rtb_Add1 + 1U)] - rtConstP.pooled7[(uint16_T)rtb_Add1]) * rtb_add_c_l +
               rtConstP.pooled7[(uint16_T)rtb_Add1], rtb_Add3 +
               rtConstP.pooled7[(int32_T)((uint16_T)rtb_Add1 + 200U)],
               &rtb_add_c_l, &rtb_Add3);

  /* End of Outputs for SubSystem: '<S154>/Two inputs CRL' */

  /* Gain: '<S156>/Ka' */
  IaSim = rtb_add_c_l;

  /* Gain: '<S156>/one_by_two' */
  rtb_add_c_l *= 0.5F;

  /* Gain: '<S156>/sqrt3_by_two' */
  rtb_Add3 *= 0.866025388F;

  /* Gain: '<S156>/Kb' incorporates:
   *  Sum: '<S156>/add_b'
   */
  IbSim = rtb_Add3 - rtb_add_c_l;

  /* Gain: '<S156>/Kc' incorporates:
   *  Sum: '<S156>/add_c'
   */
  IcSim = (0.0F - rtb_add_c_l) - rtb_Add3;

  /* If: '<S168>/If' incorporates:
   *  Constant: '<S169>/Constant'
   *  Inport: '<Root>/EposReq'
   *  RelationalOperator: '<S169>/Compare'
   */
  if (EposReq < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S168>/If Action Subsystem' incorporates:
     *  ActionPort: '<S170>/Action Port'
     */
    IfActionSubsystem(EposReq, &rtb_add_c_l);

    /* End of Outputs for SubSystem: '<S168>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S168>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S171>/Action Port'
     */
    IfActionSubsystem1(EposReq, &rtb_add_c_l);

    /* End of Outputs for SubSystem: '<S168>/If Action Subsystem1' */
  }

  /* End of If: '<S168>/If' */

  /* Gain: '<S152>/indexing' */
  rtb_add_c_l *= 800.0F;

  /* Sum: '<S152>/Sum2' incorporates:
   *  DataTypeConversion: '<S152>/Data Type Conversion1'
   *  DataTypeConversion: '<S152>/Get_Integer'
   */
  rtb_Add1 = rtb_add_c_l - (real32_T)(uint16_T)rtb_add_c_l;

  /* SignalConversion: '<S4>/Signal Conversion' incorporates:
   *  Gain: '<S164>/one_by_3'
   *  Inport: '<Root>/Ia'
   *  Inport: '<Root>/Ib'
   *  Inport: '<Root>/Ic'
   *  Sum: '<S164>/Sum'
   *  Sum: '<S164>/Sum2'
   */
  Ialpha = Ia - ((Ia + Ib) + Ic) * 0.333333343F;

  /* SignalConversion: '<S4>/Signal Conversion1' incorporates:
   *  Gain: '<S164>/one_by_sqrt3_'
   *  Inport: '<Root>/Ib'
   *  Inport: '<Root>/Ic'
   *  Sum: '<S164>/Sum1'
   */
  Ibeta = (Ib - Ic) * 0.577350259F;

  /* Outputs for Atomic SubSystem: '<S151>/Two inputs CRL' */
  /* AlgorithmDescriptorDelegate generated from: '<S165>/a16' incorporates:
   *  Constant: '<S152>/offset'
   *  Constant: '<S152>/sine_table_values'
   *  DataTypeConversion: '<S152>/Get_Integer'
   *  Product: '<S167>/Product'
   *  Product: '<S167>/Product1'
   *  Selector: '<S152>/Lookup'
   *  Sum: '<S152>/Sum'
   *  Sum: '<S167>/Sum3'
   *  Sum: '<S167>/Sum4'
   *  Sum: '<S167>/Sum5'
   *  Sum: '<S167>/Sum6'
   */
  arm_park_f32(Ialpha, Ibeta, &Id, &Iq, (rtConstP.pooled7[(int32_T)((uint16_T)
    rtb_add_c_l + 1U)] - rtConstP.pooled7[(uint16_T)rtb_add_c_l]) * rtb_Add1 +
               rtConstP.pooled7[(uint16_T)rtb_add_c_l], (rtConstP.pooled7
    [(int32_T)((uint16_T)rtb_add_c_l + 201U)] - rtConstP.pooled7[(int32_T)
    ((uint16_T)rtb_add_c_l + 200U)]) * rtb_Add1 + rtConstP.pooled7[(int32_T)
               ((uint16_T)rtb_add_c_l + 200U)]);

  /* End of Outputs for SubSystem: '<S151>/Two inputs CRL' */
  /* End of Outputs for SubSystem: '<S1>/dq0' */

  /* Outputs for Atomic SubSystem: '<S1>/CurCtrl' */
  /* Sum: '<S8>/Sum' incorporates:
   *  Inport: '<Root>/IqReq'
   */
  rtb_algDD_o1 = IqReq - Iq;

  /* Logic: '<S9>/Logical Operator1' incorporates:
   *  Inport: '<Root>/SysEnable'
   *  Inport: '<Root>/mode'
   *  Logic: '<S9>/AND'
   *  Logic: '<S9>/Logical Operator2'
   *  RelationalOperator: '<S126>/FixPt Relational Operator'
   *  UnitDelay: '<S126>/Delay Input1'
   *
   * Block description for '<S126>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtb_LogicalOperator1 = ((mode != rtDW.DelayInput1_DSTATE) || (!SysEnable));

  /* DiscreteIntegrator: '<S109>/Integrator' */
  if (rtb_LogicalOperator1 || (rtDW.Integrator_PrevResetState != 0)) {
    rtDW.Integrator_DSTATE = 0.0F;
  }

  /* Sum: '<S118>/Sum' incorporates:
   *  DiscreteIntegrator: '<S109>/Integrator'
   *  Inport: '<Root>/CurrentKp'
   *  Product: '<S114>/PProd Out'
   */
  rtb_add_c_l = rtb_algDD_o1 * CurrentKp + rtDW.Integrator_DSTATE;

  /* Saturate: '<S116>/Saturation' */
  if (rtb_add_c_l > 0.519615233F) {
    /* SignalConversion: '<S2>/Signal Conversion1' */
    VqReqPU = 0.519615233F;
  } else if (rtb_add_c_l < 0.519615233F) {
    /* SignalConversion: '<S2>/Signal Conversion1' */
    VqReqPU = 0.519615233F;
  } else {
    /* SignalConversion: '<S2>/Signal Conversion1' */
    VqReqPU = rtb_add_c_l;
  }

  /* End of Saturate: '<S116>/Saturation' */

  /* Sum: '<S7>/Sum' incorporates:
   *  Inport: '<Root>/IdReq'
   */
  rtb_Add3 = IdReq - Id;

  /* DiscreteIntegrator: '<S58>/Integrator' */
  if (rtb_LogicalOperator1 || (rtDW.Integrator_PrevResetState_l != 0)) {
    rtDW.Integrator_DSTATE_f = 0.0F;
  }

  /* Sum: '<S67>/Sum' incorporates:
   *  DiscreteIntegrator: '<S58>/Integrator'
   *  Inport: '<Root>/CurrentKp'
   *  Product: '<S63>/PProd Out'
   */
  rtb_Add1 = rtb_Add3 * CurrentKp + rtDW.Integrator_DSTATE_f;

  /* Saturate: '<S65>/Saturation' incorporates:
   *  DeadZone: '<S51>/DeadZone'
   */
  if (rtb_Add1 > 0.519615233F) {
    /* SignalConversion: '<S2>/Signal Conversion' */
    VdReqPU = 0.519615233F;
    rtb_Add1 -= 0.519615233F;
  } else {
    if (rtb_Add1 < -0.519615233F) {
      /* SignalConversion: '<S2>/Signal Conversion' */
      VdReqPU = -0.519615233F;
    } else {
      /* SignalConversion: '<S2>/Signal Conversion' */
      VdReqPU = rtb_Add1;
    }

    if (rtb_Add1 >= -0.519615233F) {
      rtb_Add1 = 0.0F;
    } else {
      rtb_Add1 -= -0.519615233F;
    }
  }

  /* End of Saturate: '<S65>/Saturation' */

  /* Product: '<S55>/IProd Out' incorporates:
   *  Gain: '<S7>/Gain'
   *  Inport: '<Root>/CurrentKi'
   */
  rtb_Add3 *= 0.0001F * CurrentKi;

  /* DeadZone: '<S102>/DeadZone' */
  if (rtb_add_c_l > 0.519615233F) {
    rtb_add_c_l -= 0.519615233F;
  } else if (rtb_add_c_l >= 0.519615233F) {
    rtb_add_c_l = 0.0F;
  } else {
    rtb_add_c_l -= 0.519615233F;
  }

  /* End of DeadZone: '<S102>/DeadZone' */

  /* Product: '<S106>/IProd Out' incorporates:
   *  Gain: '<S8>/Gain'
   *  Inport: '<Root>/CurrentKi'
   */
  rtb_algDD_o1 *= 0.0001F * CurrentKi;

  /* Update for UnitDelay: '<S126>/Delay Input1' incorporates:
   *  Inport: '<Root>/mode'
   *
   * Block description for '<S126>/Delay Input1':
   *
   *  Store in Global RAM
   */
  rtDW.DelayInput1_DSTATE = mode;

  /* Switch: '<S100>/Switch1' incorporates:
   *  Constant: '<S100>/Clamping_zero'
   *  Constant: '<S100>/Constant'
   *  Constant: '<S100>/Constant2'
   *  RelationalOperator: '<S100>/fix for DT propagation issue'
   */
  if (rtb_add_c_l > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S100>/Switch2' incorporates:
   *  Constant: '<S100>/Clamping_zero'
   *  Constant: '<S100>/Constant3'
   *  Constant: '<S100>/Constant4'
   *  RelationalOperator: '<S100>/fix for DT propagation issue1'
   */
  if (rtb_algDD_o1 > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S100>/Switch' incorporates:
   *  Constant: '<S100>/Clamping_zero'
   *  Constant: '<S100>/Constant1'
   *  Logic: '<S100>/AND3'
   *  RelationalOperator: '<S100>/Equal1'
   *  RelationalOperator: '<S100>/Relational Operator'
   *  Switch: '<S100>/Switch1'
   *  Switch: '<S100>/Switch2'
   */
  if ((rtb_add_c_l != 0.0F) && (tmp == tmp_0)) {
    rtb_algDD_o1 = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S109>/Integrator' incorporates:
   *  Switch: '<S100>/Switch'
   */
  rtDW.Integrator_DSTATE += rtb_algDD_o1;
  rtDW.Integrator_PrevResetState = (int8_T)rtb_LogicalOperator1;

  /* Switch: '<S49>/Switch1' incorporates:
   *  Constant: '<S49>/Clamping_zero'
   *  Constant: '<S49>/Constant'
   *  Constant: '<S49>/Constant2'
   *  RelationalOperator: '<S49>/fix for DT propagation issue'
   */
  if (rtb_Add1 > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S49>/Switch2' incorporates:
   *  Constant: '<S49>/Clamping_zero'
   *  Constant: '<S49>/Constant3'
   *  Constant: '<S49>/Constant4'
   *  RelationalOperator: '<S49>/fix for DT propagation issue1'
   */
  if (rtb_Add3 > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S49>/Switch' incorporates:
   *  Constant: '<S49>/Clamping_zero'
   *  Constant: '<S49>/Constant1'
   *  Logic: '<S49>/AND3'
   *  RelationalOperator: '<S49>/Equal1'
   *  RelationalOperator: '<S49>/Relational Operator'
   *  Switch: '<S49>/Switch1'
   *  Switch: '<S49>/Switch2'
   */
  if ((rtb_Add1 != 0.0F) && (tmp == tmp_0)) {
    rtb_Add3 = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S58>/Integrator' incorporates:
   *  DiscreteIntegrator: '<S109>/Integrator'
   *  Switch: '<S49>/Switch'
   */
  rtDW.Integrator_DSTATE_f += rtb_Add3;
  rtDW.Integrator_PrevResetState_l = (int8_T)rtb_LogicalOperator1;

  /* End of Outputs for SubSystem: '<S1>/CurCtrl' */

  /* Outputs for Atomic SubSystem: '<S1>/VoltCtrl' */
  /* Outputs for Enabled SubSystem: '<S129>/Accumulate' incorporates:
   *  EnablePort: '<S135>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S135>/Subsystem' incorporates:
   *  EnablePort: '<S136>/Enable'
   */
  /* Delay: '<S135>/Delay' */
  if (rtDW.Delay_DSTATE) {
    /* SignalConversion generated from: '<S136>/Input' incorporates:
     *  Gain: '<S3>/Gain'
     *  Inport: '<Root>/SpReqPU'
     */
    rtDW.Input = 0.0613083318F * SpReqPU;
  }

  /* End of Delay: '<S135>/Delay' */
  /* End of Outputs for SubSystem: '<S135>/Subsystem' */

  /* Sum: '<S135>/Add' incorporates:
   *  UnitDelay: '<S129>/Unit Delay'
   */
  rtb_Add3 = rtDW.Input + rtDW.Add1;

  /* DataTypeConversion: '<S135>/Data Type Conversion1' incorporates:
   *  DataTypeConversion: '<S135>/Data Type Conversion'
   */
  rtb_add_c_l = (int16_T)(real32_T)floor(rtb_Add3);

  /* Sum: '<S135>/Add1' */
  rtDW.Add1 = rtb_Add3 - rtb_add_c_l;

  /* Update for Delay: '<S135>/Delay' incorporates:
   *  Constant: '<S135>/Constant'
   */
  rtDW.Delay_DSTATE = true;

  /* End of Outputs for SubSystem: '<S129>/Accumulate' */

  /* Gain: '<S129>/scaleOut' */
  EposOpen = rtDW.Add1;

  /* If: '<S138>/If' incorporates:
   *  Constant: '<S139>/Constant'
   *  RelationalOperator: '<S139>/Compare'
   */
  if (EposOpen < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S138>/If Action Subsystem' incorporates:
     *  ActionPort: '<S140>/Action Port'
     */
    IfActionSubsystem(EposOpen, &rtb_add_c_l);

    /* End of Outputs for SubSystem: '<S138>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S138>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S141>/Action Port'
     */
    IfActionSubsystem1(EposOpen, &rtb_add_c_l);

    /* End of Outputs for SubSystem: '<S138>/If Action Subsystem1' */
  }

  /* End of If: '<S138>/If' */

  /* Gain: '<S130>/indexing' */
  rtb_add_c_l *= 800.0F;

  /* Sum: '<S130>/Sum2' incorporates:
   *  DataTypeConversion: '<S130>/Data Type Conversion1'
   *  DataTypeConversion: '<S130>/Get_Integer'
   */
  rtb_Add1 = rtb_add_c_l - (real32_T)(uint16_T)rtb_add_c_l;

  /* Outputs for Atomic SubSystem: '<S127>/Two inputs CRL' */
  /* Constant: '<S3>/Constant' incorporates:
   *  Constant: '<S130>/offset'
   *  Constant: '<S130>/sine_table_values'
   *  Constant: '<S3>/Constant1'
   *  DataTypeConversion: '<S130>/Get_Integer'
   *  Product: '<S137>/Product'
   *  Product: '<S137>/Product1'
   *  Selector: '<S130>/Lookup'
   *  Sum: '<S130>/Sum'
   *  Sum: '<S137>/Sum3'
   *  Sum: '<S137>/Sum4'
   *  Sum: '<S137>/Sum5'
   *  Sum: '<S137>/Sum6'
   */
  TwoinputsCRL(0.0F, -0.1F, (rtConstP.pooled7[(int32_T)((uint16_T)rtb_add_c_l +
    1U)] - rtConstP.pooled7[(uint16_T)rtb_add_c_l]) * rtb_Add1 +
               rtConstP.pooled7[(uint16_T)rtb_add_c_l], (rtConstP.pooled7
    [(int32_T)((uint16_T)rtb_add_c_l + 201U)] - rtConstP.pooled7[(int32_T)
    ((uint16_T)rtb_add_c_l + 200U)]) * rtb_Add1 + rtConstP.pooled7[(int32_T)
               ((uint16_T)rtb_add_c_l + 200U)], &rtb_algDD_o1, &rtb_Add3);

  /* End of Outputs for SubSystem: '<S127>/Two inputs CRL' */

  /* Gain: '<S148>/one_by_two' */
  rtb_add_c_l = 0.5F * rtb_algDD_o1;

  /* Gain: '<S148>/sqrt3_by_two' */
  rtb_Add3 *= 0.866025388F;

  /* Sum: '<S148>/add_b' */
  rtb_Add1 = rtb_Add3 - rtb_add_c_l;

  /* Sum: '<S148>/add_c' */
  rtb_add_c_l = (0.0F - rtb_add_c_l) - rtb_Add3;

  /* MinMax: '<S145>/Max' incorporates:
   *  MinMax: '<S145>/Min'
   */
  rtb_LogicalOperator1 = rtIsNaNF(rtb_Add1);
  if ((rtb_algDD_o1 >= rtb_Add1) || rtb_LogicalOperator1) {
    rtb_Add3 = rtb_algDD_o1;
  } else {
    rtb_Add3 = rtb_Add1;
  }

  /* MinMax: '<S145>/Min' */
  if ((rtb_algDD_o1 <= rtb_Add1) || rtb_LogicalOperator1) {
    rtb_PWM_DutyCycles_idx_1 = rtb_algDD_o1;
  } else {
    rtb_PWM_DutyCycles_idx_1 = rtb_Add1;
  }

  /* MinMax: '<S145>/Max' incorporates:
   *  MinMax: '<S145>/Min'
   */
  rtb_LogicalOperator1 = !rtIsNaNF(rtb_add_c_l);
  if ((!(rtb_Add3 >= rtb_add_c_l)) && rtb_LogicalOperator1) {
    rtb_Add3 = rtb_add_c_l;
  }

  /* MinMax: '<S145>/Min' */
  if ((!(rtb_PWM_DutyCycles_idx_1 <= rtb_add_c_l)) && rtb_LogicalOperator1) {
    rtb_PWM_DutyCycles_idx_1 = rtb_add_c_l;
  }

  /* Gain: '<S145>/one_by_two' incorporates:
   *  MinMax: '<S145>/Max'
   *  MinMax: '<S145>/Min'
   *  Sum: '<S145>/Add'
   */
  rtb_Add3 = (rtb_Add3 + rtb_PWM_DutyCycles_idx_1) * -0.5F;

  /* Switch: '<S128>/Switch' incorporates:
   *  Constant: '<S128>/stop'
   *  Inport: '<Root>/SysEnable'
   */
  if (SysEnable) {
    /* Switch: '<S128>/Switch1' incorporates:
     *  Constant: '<S128>/Constant'
     *  Gain: '<S128>/One_by_Two'
     *  Gain: '<S144>/Gain'
     *  Inport: '<Root>/PhaseOrder'
     *  RelationalOperator: '<S134>/Compare'
     *  Sum: '<S128>/Sum'
     *  Sum: '<S144>/Add1'
     *  Sum: '<S144>/Add2'
     *  Sum: '<S144>/Add3'
     */
    if (!PhaseOrder) {
      rtb_algDD_o1 = (rtb_algDD_o1 + rtb_Add3) * 1.15470052F * 0.5F + 0.5F;
      rtb_PWM_DutyCycles_idx_1 = (rtb_Add3 + rtb_add_c_l) * 1.15470052F * 0.5F +
        0.5F;
      rtb_Add1 = (rtb_Add1 + rtb_Add3) * 1.15470052F * 0.5F + 0.5F;
    } else {
      rtb_algDD_o1 = (rtb_algDD_o1 + rtb_Add3) * 1.15470052F * 0.5F + 0.5F;
      rtb_PWM_DutyCycles_idx_1 = (rtb_Add1 + rtb_Add3) * 1.15470052F * 0.5F +
        0.5F;
      rtb_Add1 = (rtb_Add3 + rtb_add_c_l) * 1.15470052F * 0.5F + 0.5F;
    }

    /* End of Switch: '<S128>/Switch1' */
  } else {
    rtb_algDD_o1 = 0.0F;
    rtb_PWM_DutyCycles_idx_1 = 0.0F;
    rtb_Add1 = 0.0F;
  }

  /* End of Switch: '<S128>/Switch' */

  /* SignalConversion: '<S3>/Signal Conversion' incorporates:
   *  Gain: '<S128>/Gain'
   */
  U = (uint16_T)(8000.0F * rtb_algDD_o1);

  /* SignalConversion: '<S3>/Signal Conversion1' incorporates:
   *  Gain: '<S128>/Gain'
   */
  V = (uint16_T)(8000.0F * rtb_PWM_DutyCycles_idx_1);

  /* SignalConversion: '<S3>/Signal Conversion2' incorporates:
   *  Gain: '<S128>/Gain'
   */
  W = (uint16_T)(8000.0F * rtb_Add1);

  /* End of Outputs for SubSystem: '<S1>/VoltCtrl' */
  /* End of Outputs for SubSystem: '<Root>/FOC' */
}

/* Model initialize function */
void FOC_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* external inputs */
  SpReqPU = 0.1F;
  CurrentKi = 122.394402F;
  CurrentKp = 0.0614F;
  IqReq = 0.1F;
  mode = ((uint8_T)3U);
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
