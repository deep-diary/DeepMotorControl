/*
 * File: FOC.h
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

#ifndef RTW_HEADER_FOC_h_
#define RTW_HEADER_FOC_h_
#ifndef FOC_COMMON_INCLUDES_
#define FOC_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define FOC_M                          (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real32_T Add1;                       /* '<S135>/Add1' */
  real32_T Input;                      /* '<S136>/Input' */
  real32_T Integrator_DSTATE;          /* '<S109>/Integrator' */
  real32_T Integrator_DSTATE_f;        /* '<S58>/Integrator' */
  int8_T Integrator_PrevResetState;    /* '<S109>/Integrator' */
  int8_T Integrator_PrevResetState_l;  /* '<S58>/Integrator' */
  uint8_T DelayInput1_DSTATE;          /* '<S126>/Delay Input1' */
  boolean_T Delay_DSTATE;              /* '<S135>/Delay' */
} DW;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T UnaryMinus;           /* '<S149>/Unary Minus' */
} ConstB;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S130>/sine_table_values'
   *   '<S152>/sine_table_values'
   *   '<S155>/sine_table_values'
   */
  real32_T pooled7[1002];
} ConstP;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;
extern const ConstB rtConstB;          /* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP rtConstP;

/* Model entry point functions */
extern void FOC_initialize(void);
extern void FOC_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Default */
extern real32_T CurrentKi;             /* '<Root>/CurrentKi' */

/* 电流Ki */
extern real32_T CurrentKp;             /* '<Root>/CurrentKp' */

/* 电流Kp */
extern real32_T EposOpen;              /* '<S129>/scaleOut' */

/* 实际电角度 */
extern real32_T EposReq;               /* '<Root>/EposReq' */

/* 实际电角度 */
extern real32_T Ia;                    /* '<Root>/Ia' */

/* a相电流 */
extern real32_T IaSim;                 /* '<S156>/Ka' */

/* a相仿真电流 */
extern real32_T Ialpha;                /* '<S4>/Signal Conversion' */

/* alpha电流 */
extern real32_T Ib;                    /* '<Root>/Ib' */

/* b相电流 */
extern real32_T IbSim;                 /* '<S156>/Kb' */

/* b相仿真电流 */
extern real32_T Ibeta;                 /* '<S4>/Signal Conversion1' */

/* beta电流 */
extern real32_T Ic;                    /* '<Root>/Ic' */

/* c相电流 */
extern real32_T IcSim;                 /* '<S156>/Kc' */

/* c相仿真电流 */
extern real32_T Id;

/* D轴电流 */
extern real32_T IdReq;                 /* '<Root>/IdReq' */

/* D轴电流 */
extern real32_T Iq;

/* Q轴电流 */
extern real32_T IqReq;                 /* '<Root>/IqReq' */

/* Q轴电流 */
extern boolean_T PhaseOrder;           /* '<Root>/PhaseOrder' */

/* 电机相序 */
extern real32_T SpReqPU;               /* '<Root>/SpReqPU' */

/* 斜波处理后的速度 */
extern boolean_T SysEnable;            /* '<Root>/SysEnable' */

/* 使能系统 */
extern uint16_T U;                     /* '<S3>/Signal Conversion' */

/* PWM U */
extern uint16_T V;                     /* '<S3>/Signal Conversion1' */

/* PWM V */
extern real32_T VdReqPU;               /* '<S2>/Signal Conversion' */

/* 斜波处理后的d轴电压 */
extern real32_T VqReqPU;               /* '<S2>/Signal Conversion1' */

/* 斜波处理后的q轴电压 */
extern uint16_T W;                     /* '<S3>/Signal Conversion2' */

/* PWM W */
extern uint8_T mode;                   /* '<Root>/mode' */

/* 工作模式 */

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S14>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S6>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S12>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S13>/Sqrt' : Unused code path elimination
 * Block '<S132>/Data Type Duplicate' : Unused code path elimination
 * Block '<S132>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S129>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S130>/Data Type Duplicate' : Unused code path elimination
 * Block '<S130>/Data Type Propagation' : Unused code path elimination
 * Block '<S140>/Data Type Duplicate' : Unused code path elimination
 * Block '<S141>/Data Type Duplicate' : Unused code path elimination
 * Block '<S131>/Data Type Duplicate' : Unused code path elimination
 * Block '<S131>/Vc' : Unused code path elimination
 * Block '<S148>/Data Type Duplicate' : Unused code path elimination
 * Block '<S149>/Data Type Duplicate' : Unused code path elimination
 * Block '<S156>/Data Type Duplicate' : Unused code path elimination
 * Block '<S157>/Data Type Duplicate' : Unused code path elimination
 * Block '<S157>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S155>/Data Type Duplicate' : Unused code path elimination
 * Block '<S155>/Data Type Propagation' : Unused code path elimination
 * Block '<S162>/Data Type Duplicate' : Unused code path elimination
 * Block '<S163>/Data Type Duplicate' : Unused code path elimination
 * Block '<S164>/Data Type Duplicate' : Unused code path elimination
 * Block '<S165>/Data Type Duplicate' : Unused code path elimination
 * Block '<S165>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S152>/Data Type Duplicate' : Unused code path elimination
 * Block '<S152>/Data Type Propagation' : Unused code path elimination
 * Block '<S170>/Data Type Duplicate' : Unused code path elimination
 * Block '<S171>/Data Type Duplicate' : Unused code path elimination
 * Block '<S129>/scaleIn' : Eliminated nontunable gain of 1
 * Block '<S130>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S138>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S148>/Ka' : Eliminated nontunable gain of 1
 * Block '<S148>/Kb' : Eliminated nontunable gain of 1
 * Block '<S148>/Kc' : Eliminated nontunable gain of 1
 * Block '<S155>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S160>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S164>/Kalphabeta0' : Eliminated nontunable gain of 1
 * Block '<S152>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S168>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S12>/enableInportSatLim' : Unused code path elimination
 * Block '<S12>/enableInportSatMethod' : Unused code path elimination
 * Block '<S6>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S6>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S133>/Offset' : Unused code path elimination
 * Block '<S133>/Unary_Minus' : Unused code path elimination
 * Block '<S158>/Offset' : Unused code path elimination
 * Block '<S158>/Unary_Minus' : Unused code path elimination
 * Block '<S166>/Offset' : Unused code path elimination
 * Block '<S166>/Unary_Minus' : Unused code path elimination
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('ParkTest/FOC')    - opens subsystem ParkTest/FOC
 * hilite_system('ParkTest/FOC/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ParkTest'
 * '<S1>'   : 'ParkTest/FOC'
 * '<S2>'   : 'ParkTest/FOC/CurCtrl'
 * '<S3>'   : 'ParkTest/FOC/VoltCtrl'
 * '<S4>'   : 'ParkTest/FOC/dq0'
 * '<S5>'   : 'ParkTest/FOC/CurCtrl/Current_Controllers'
 * '<S6>'   : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter'
 * '<S7>'   : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id'
 * '<S8>'   : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq'
 * '<S9>'   : 'ParkTest/FOC/CurCtrl/Current_Controllers/Subsystem'
 * '<S10>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D-Q Equivalence'
 * '<S11>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S12>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S13>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/Magnitude_calc'
 * '<S14>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S15>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S16>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S17>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S18>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S19>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S20>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S21>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S22>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S23>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S24>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset'
 * '<S25>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S26>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S27>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S28>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S29>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S30>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S31>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S32>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S33>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S34>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S35>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S36>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S37>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S38>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S39>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S40>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S41>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S42>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S43>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S44>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S45>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S46>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S47>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S48>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S49>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S50>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S51>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S52>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S53>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S54>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S55>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S56>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S57>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S58>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S59>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S60>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S61>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S62>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S63>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S64>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S65>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S66>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S67>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S68>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S69>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S70>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S71>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S72>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S73>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S74>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Id/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S75>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset'
 * '<S76>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S77>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S78>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S79>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S80>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S81>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S82>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S83>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S84>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S85>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S86>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S87>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S88>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S89>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S90>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S91>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S92>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S93>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S94>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S95>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S96>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S97>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S98>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S99>'  : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S100>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S101>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S102>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S103>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S104>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S105>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S106>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S107>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S108>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S109>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S110>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S111>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S112>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S113>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S114>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S115>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S116>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S117>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S118>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S119>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S120>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S121>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S122>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S123>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S124>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S125>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/PI_Controller_Iq/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S126>' : 'ParkTest/FOC/CurCtrl/Current_Controllers/Subsystem/Detect Change1'
 * '<S127>' : 'ParkTest/FOC/VoltCtrl/Inverse Park Transform'
 * '<S128>' : 'ParkTest/FOC/VoltCtrl/Output Scaling'
 * '<S129>' : 'ParkTest/FOC/VoltCtrl/Position Generator'
 * '<S130>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup'
 * '<S131>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator'
 * '<S132>' : 'ParkTest/FOC/VoltCtrl/Inverse Park Transform/Two inputs CRL'
 * '<S133>' : 'ParkTest/FOC/VoltCtrl/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S134>' : 'ParkTest/FOC/VoltCtrl/Output Scaling/Compare To Constant'
 * '<S135>' : 'ParkTest/FOC/VoltCtrl/Position Generator/Accumulate'
 * '<S136>' : 'ParkTest/FOC/VoltCtrl/Position Generator/Accumulate/Subsystem'
 * '<S137>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup/Interpolation'
 * '<S138>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup/WrapUp'
 * '<S139>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S140>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S141>' : 'ParkTest/FOC/VoltCtrl/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S142>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Modulation method'
 * '<S143>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Voltage Input'
 * '<S144>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Modulation method/SVPWM'
 * '<S145>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S146>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Voltage Input/Valphabeta'
 * '<S147>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S148>' : 'ParkTest/FOC/VoltCtrl/Space Vector Generator/Voltage Input/Valphabeta/Inverse Clarke Transform/Two phase input'
 * '<S149>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator'
 * '<S150>' : 'ParkTest/FOC/dq0/Clarke Transform'
 * '<S151>' : 'ParkTest/FOC/dq0/Park Transform'
 * '<S152>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup'
 * '<S153>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Inverse Clarke Transform'
 * '<S154>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Inverse Park Transform'
 * '<S155>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup'
 * '<S156>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Inverse Clarke Transform/Two phase input'
 * '<S157>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Inverse Park Transform/Two inputs CRL'
 * '<S158>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Inverse Park Transform/Two inputs CRL/Switch_Axis'
 * '<S159>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup/Interpolation'
 * '<S160>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup/WrapUp'
 * '<S161>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S162>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S163>' : 'ParkTest/FOC/dq0/3-Phase Sine Voltage Generator/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S164>' : 'ParkTest/FOC/dq0/Clarke Transform/Three phase input'
 * '<S165>' : 'ParkTest/FOC/dq0/Park Transform/Two inputs CRL'
 * '<S166>' : 'ParkTest/FOC/dq0/Park Transform/Two inputs CRL/Switch_Axis'
 * '<S167>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup/Interpolation'
 * '<S168>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup/WrapUp'
 * '<S169>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S170>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S171>' : 'ParkTest/FOC/dq0/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 */
#endif                                 /* RTW_HEADER_FOC_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
