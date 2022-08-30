/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FullRMX_types.h
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

#ifndef RTW_HEADER_FullRMX_types_h_
#define RTW_HEADER_FullRMX_types_h_
#include "rtwtypes.h"
#include "multiword_types.h"

/* Model Code Variants */

/* Custom Type definition for MATLABSystem: '<S3>/PH_3' */
#include "MW_SVD.h"
#ifndef struct_tag_p5Oj7yF0nd6COoUgI0GAlG
#define struct_tag_p5Oj7yF0nd6COoUgI0GAlG

struct tag_p5Oj7yF0nd6COoUgI0GAlG
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_ANALOGIN_HANDLE;
  real_T SampleTime;
};

#endif                                 /* struct_tag_p5Oj7yF0nd6COoUgI0GAlG */

#ifndef typedef_mbed_AnalogInput_FullRMX_T
#define typedef_mbed_AnalogInput_FullRMX_T

typedef struct tag_p5Oj7yF0nd6COoUgI0GAlG mbed_AnalogInput_FullRMX_T;

#endif                                 /* typedef_mbed_AnalogInput_FullRMX_T */

#ifndef struct_tag_UndvUYqhBVOhRRpUse3fWF
#define struct_tag_UndvUYqhBVOhRRpUse3fWF

struct tag_UndvUYqhBVOhRRpUse3fWF
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_PWM_HANDLE;
};

#endif                                 /* struct_tag_UndvUYqhBVOhRRpUse3fWF */

#ifndef typedef_mbed_PWMOutput_FullRMX_T
#define typedef_mbed_PWMOutput_FullRMX_T

typedef struct tag_UndvUYqhBVOhRRpUse3fWF mbed_PWMOutput_FullRMX_T;

#endif                                 /* typedef_mbed_PWMOutput_FullRMX_T */

#ifndef struct_tag_KxFW01GBdhqk5JOEHU3GlD
#define struct_tag_KxFW01GBdhqk5JOEHU3GlD

struct tag_KxFW01GBdhqk5JOEHU3GlD
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  MW_Handle_Type MW_DIGITALIO_HANDLE;
};

#endif                                 /* struct_tag_KxFW01GBdhqk5JOEHU3GlD */

#ifndef typedef_mbed_DigitalWrite_FullRMX_T
#define typedef_mbed_DigitalWrite_FullRMX_T

typedef struct tag_KxFW01GBdhqk5JOEHU3GlD mbed_DigitalWrite_FullRMX_T;

#endif                                 /* typedef_mbed_DigitalWrite_FullRMX_T */

/* Parameters (default storage) */
typedef struct P_FullRMX_T_ P_FullRMX_T;

/* Forward declaration for rtModel */
typedef struct tag_RTM_FullRMX_T RT_MODEL_FullRMX_T;

#endif                                 /* RTW_HEADER_FullRMX_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
