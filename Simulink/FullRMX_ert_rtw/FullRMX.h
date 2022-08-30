/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FullRMX.h
 *
 * Code generated for Simulink model 'FullRMX'.
 *
 * Model version                  : 1.7
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Tue Aug 30 10:57:00 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FullRMX_h_
#define RTW_HEADER_FullRMX_h_
#ifndef FullRMX_COMMON_INCLUDES_
#define FullRMX_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_extmode.h"
#include "sysran_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "MW_AnalogIn.h"
#include "MW_MbedPinInterface.h"
#include "MW_PWM.h"
#include "MW_digitalIO.h"
#endif                                 /* FullRMX_COMMON_INCLUDES_ */

#include "FullRMX_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "MW_target_hardware_resources.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetFinalTime
#define rtmGetFinalTime(rtm)           ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetRTWExtModeInfo
#define rtmGetRTWExtModeInfo(rtm)      ((rtm)->extModeInfo)
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   ((rtm)->Timing.taskTime0)
#endif

#ifndef rtmGetTFinal
#define rtmGetTFinal(rtm)              ((rtm)->Timing.tFinal)
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                (&(rtm)->Timing.taskTime0)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Gain;                         /* '<Root>/Gain' */
  real_T MotorsDutyCycle;              /* '<Root>/Motor DUT%' */
  real_T AnalogInput3;                 /* '<Root>/Analog Input3' */
  real_T AnalogInput2;                 /* '<Root>/Analog Input2' */
  real_T AnalogInput1;                 /* '<Root>/Analog Input1' */
} B_FullRMX_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  mbed_AnalogInput_FullRMX_T obj;      /* '<Root>/Analog Input3' */
  mbed_AnalogInput_FullRMX_T obj_l;    /* '<Root>/Analog Input2' */
  mbed_AnalogInput_FullRMX_T obj_j;    /* '<Root>/Analog Input1' */
  mbed_AnalogInput_FullRMX_T obj_n;    /* '<Root>/Analog Input' */
  mbed_PWMOutput_FullRMX_T obj_d;      /* '<S3>/EN_3' */
  mbed_PWMOutput_FullRMX_T obj_na;     /* '<S2>/EN_1' */
  mbed_PWMOutput_FullRMX_T obj_ny;     /* '<S1>/EN_2' */
  mbed_DigitalWrite_FullRMX_T obj_o;   /* '<S3>/PH_3' */
  mbed_DigitalWrite_FullRMX_T obj_b;   /* '<S3>/NS_3' */
  mbed_DigitalWrite_FullRMX_T obj_e;   /* '<S2>/PH_1' */
  mbed_DigitalWrite_FullRMX_T obj_i;   /* '<S2>/NS_1' */
  mbed_DigitalWrite_FullRMX_T obj_nh;  /* '<S1>/PH_2' */
  mbed_DigitalWrite_FullRMX_T obj_g;   /* '<S1>/NS_2' */
  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Gain_;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Analo;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Ana_f;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Ana_p;   /* synthesized block */

  struct {
    void *AQHandles;
  } TAQSigLogging_InsertedFor_Motor;   /* synthesized block */
} DW_FullRMX_T;

/* Parameters (default storage) */
struct P_FullRMX_T_ {
  real_T AnalogInput_SampleTime;       /* Expression: -1
                                        * Referenced by: '<Root>/Analog Input'
                                        */
  real_T AnalogInput1_SampleTime;      /* Expression: -1
                                        * Referenced by: '<Root>/Analog Input1'
                                        */
  real_T AnalogInput2_SampleTime;      /* Expression: -1
                                        * Referenced by: '<Root>/Analog Input2'
                                        */
  real_T AnalogInput3_SampleTime;      /* Expression: -1
                                        * Referenced by: '<Root>/Analog Input3'
                                        */
  real_T Gain_Gain;                    /* Expression: 4095
                                        * Referenced by: '<Root>/Gain'
                                        */
  real_T MotorDUT_Value;               /* Expression: 100
                                        * Referenced by: '<Root>/Motor DUT%'
                                        */
  real_T MOTOR_DIR_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/MOTOR_DIR'
                                        */
  real_T MOTOR_ON1_Value;              /* Expression: 1
                                        * Referenced by: '<Root>/MOTOR_ON1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_FullRMX_T {
  const char_T *errorStatus;
  RTWExtModeInfo *extModeInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    uint32_T checksums[4];
  } Sizes;

  /*
   * SpecialInfo:
   * The following substructure contains special information
   * related to other components that are dependent on RTW.
   */
  struct {
    const void *mappingInfo;
  } SpecialInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T taskTime0;
    uint32_T clockTick0;
    time_T stepSize0;
    time_T tFinal;
    boolean_T stopRequestedFlag;
  } Timing;
};

/* Block parameters (default storage) */
extern P_FullRMX_T FullRMX_P;

/* Block signals (default storage) */
extern B_FullRMX_T FullRMX_B;

/* Block states (default storage) */
extern DW_FullRMX_T FullRMX_DW;

/* Model entry point functions */
extern void FullRMX_initialize(void);
extern void FullRMX_step(void);
extern void FullRMX_terminate(void);

/* Real-time Model object */
extern RT_MODEL_FullRMX_T *const FullRMX_M;
extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'FullRMX'
 * '<S1>'   : 'FullRMX/Default M1'
 * '<S2>'   : 'FullRMX/Default M4'
 * '<S3>'   : 'FullRMX/Default M5'
 */
#endif                                 /* RTW_HEADER_FullRMX_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
