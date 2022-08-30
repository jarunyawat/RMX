/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: FullRMX.c
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

#include "FullRMX.h"
#include "FullRMX_private.h"

/* Block signals (default storage) */
B_FullRMX_T FullRMX_B;

/* Block states (default storage) */
DW_FullRMX_T FullRMX_DW;

/* Real-time model */
static RT_MODEL_FullRMX_T FullRMX_M_;
RT_MODEL_FullRMX_T *const FullRMX_M = &FullRMX_M_;

/* Model step function */
void FullRMX_step(void)
{
  real_T rtb_AnalogInput_0;

  /* MATLABSystem: '<Root>/Analog Input' */
  if (FullRMX_DW.obj_n.SampleTime != FullRMX_P.AnalogInput_SampleTime) {
    FullRMX_DW.obj_n.SampleTime = FullRMX_P.AnalogInput_SampleTime;
  }

  MW_AnalogIn_Start(FullRMX_DW.obj_n.MW_ANALOGIN_HANDLE);
  MW_AnalogInSingle_ReadResult(FullRMX_DW.obj_n.MW_ANALOGIN_HANDLE,
    &rtb_AnalogInput_0, 7);

  /* Gain: '<Root>/Gain' incorporates:
   *  MATLABSystem: '<Root>/Analog Input'
   */
  FullRMX_B.Gain = FullRMX_P.Gain_Gain * rtb_AnalogInput_0;

  /* MATLABSystem: '<Root>/Analog Input1' */
  if (FullRMX_DW.obj_j.SampleTime != FullRMX_P.AnalogInput1_SampleTime) {
    FullRMX_DW.obj_j.SampleTime = FullRMX_P.AnalogInput1_SampleTime;
  }

  MW_AnalogIn_Start(FullRMX_DW.obj_j.MW_ANALOGIN_HANDLE);

  /* MATLABSystem: '<Root>/Analog Input1' */
  MW_AnalogInSingle_ReadResult(FullRMX_DW.obj_j.MW_ANALOGIN_HANDLE,
    &FullRMX_B.AnalogInput1, 7);

  /* MATLABSystem: '<Root>/Analog Input2' */
  if (FullRMX_DW.obj_l.SampleTime != FullRMX_P.AnalogInput2_SampleTime) {
    FullRMX_DW.obj_l.SampleTime = FullRMX_P.AnalogInput2_SampleTime;
  }

  MW_AnalogIn_Start(FullRMX_DW.obj_l.MW_ANALOGIN_HANDLE);

  /* MATLABSystem: '<Root>/Analog Input2' */
  MW_AnalogInSingle_ReadResult(FullRMX_DW.obj_l.MW_ANALOGIN_HANDLE,
    &FullRMX_B.AnalogInput2, 7);

  /* MATLABSystem: '<Root>/Analog Input3' */
  if (FullRMX_DW.obj.SampleTime != FullRMX_P.AnalogInput3_SampleTime) {
    FullRMX_DW.obj.SampleTime = FullRMX_P.AnalogInput3_SampleTime;
  }

  MW_AnalogIn_Start(FullRMX_DW.obj.MW_ANALOGIN_HANDLE);

  /* MATLABSystem: '<Root>/Analog Input3' */
  MW_AnalogInSingle_ReadResult(FullRMX_DW.obj.MW_ANALOGIN_HANDLE,
    &FullRMX_B.AnalogInput3, 7);

  /* Constant: '<Root>/Motor DUT%' */
  FullRMX_B.MotorsDutyCycle = FullRMX_P.MotorDUT_Value;

  /* MATLABSystem: '<S1>/EN_2' */
  MW_PWM_SetDutyCycle(FullRMX_DW.obj_ny.MW_PWM_HANDLE, FullRMX_B.MotorsDutyCycle);

  /* MATLABSystem: '<S2>/EN_1' */
  MW_PWM_SetDutyCycle(FullRMX_DW.obj_na.MW_PWM_HANDLE, FullRMX_B.MotorsDutyCycle);

  /* MATLABSystem: '<S3>/EN_3' */
  MW_PWM_SetDutyCycle(FullRMX_DW.obj_d.MW_PWM_HANDLE, FullRMX_B.MotorsDutyCycle);

  /* MATLABSystem: '<S1>/PH_2' incorporates:
   *  Constant: '<Root>/MOTOR_DIR'
   */
  MW_digitalIO_write(FullRMX_DW.obj_nh.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_DIR_Value != 0.0);

  /* MATLABSystem: '<S2>/PH_1' incorporates:
   *  Constant: '<Root>/MOTOR_DIR'
   */
  MW_digitalIO_write(FullRMX_DW.obj_e.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_DIR_Value != 0.0);

  /* MATLABSystem: '<S3>/PH_3' incorporates:
   *  Constant: '<Root>/MOTOR_DIR'
   */
  MW_digitalIO_write(FullRMX_DW.obj_o.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_DIR_Value != 0.0);

  /* MATLABSystem: '<S1>/NS_2' incorporates:
   *  Constant: '<Root>/MOTOR_ON1'
   */
  MW_digitalIO_write(FullRMX_DW.obj_g.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_ON1_Value != 0.0);

  /* MATLABSystem: '<S2>/NS_1' incorporates:
   *  Constant: '<Root>/MOTOR_ON1'
   */
  MW_digitalIO_write(FullRMX_DW.obj_i.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_ON1_Value != 0.0);

  /* MATLABSystem: '<S3>/NS_3' incorporates:
   *  Constant: '<Root>/MOTOR_ON1'
   */
  MW_digitalIO_write(FullRMX_DW.obj_b.MW_DIGITALIO_HANDLE,
                     FullRMX_P.MOTOR_ON1_Value != 0.0);

  {                                    /* Sample time: [0.2s, 0.0s] */
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The absolute time is the multiplication of "clockTick0"
   * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
   * overflow during the application lifespan selected.
   */
  FullRMX_M->Timing.taskTime0 =
    ((time_T)(++FullRMX_M->Timing.clockTick0)) * FullRMX_M->Timing.stepSize0;
}

/* Model initialize function */
void FullRMX_initialize(void)
{
  /* Registration code */
  rtmSetTFinal(FullRMX_M, -1);
  FullRMX_M->Timing.stepSize0 = 0.2;

  /* External mode info */
  FullRMX_M->Sizes.checksums[0] = (4006764383U);
  FullRMX_M->Sizes.checksums[1] = (811722171U);
  FullRMX_M->Sizes.checksums[2] = (1578807205U);
  FullRMX_M->Sizes.checksums[3] = (467117198U);

  {
    static const sysRanDType rtAlwaysEnabled = SUBSYS_RAN_BC_ENABLE;
    static RTWExtModeInfo rt_ExtModeInfo;
    static const sysRanDType *systemRan[14];
    FullRMX_M->extModeInfo = (&rt_ExtModeInfo);
    rteiSetSubSystemActiveVectorAddresses(&rt_ExtModeInfo, systemRan);
    systemRan[0] = &rtAlwaysEnabled;
    systemRan[1] = &rtAlwaysEnabled;
    systemRan[2] = &rtAlwaysEnabled;
    systemRan[3] = &rtAlwaysEnabled;
    systemRan[4] = &rtAlwaysEnabled;
    systemRan[5] = &rtAlwaysEnabled;
    systemRan[6] = &rtAlwaysEnabled;
    systemRan[7] = &rtAlwaysEnabled;
    systemRan[8] = &rtAlwaysEnabled;
    systemRan[9] = &rtAlwaysEnabled;
    systemRan[10] = &rtAlwaysEnabled;
    systemRan[11] = &rtAlwaysEnabled;
    systemRan[12] = &rtAlwaysEnabled;
    systemRan[13] = &rtAlwaysEnabled;
    rteiSetModelMappingInfoPtr(FullRMX_M->extModeInfo,
      &FullRMX_M->SpecialInfo.mappingInfo);
    rteiSetChecksumsPtr(FullRMX_M->extModeInfo, FullRMX_M->Sizes.checksums);
    rteiSetTPtr(FullRMX_M->extModeInfo, rtmGetTPtr(FullRMX_M));
  }

  {
    MW_AnalogIn_TriggerSource_Type trigger_val;
    uint32_T pinname;
    mbed_AnalogInput_FullRMX_T *obj;
    mbed_DigitalWrite_FullRMX_T *obj_1;
    mbed_PWMOutput_FullRMX_T *obj_0;

    /* Start for MATLABSystem: '<Root>/Analog Input' */
    FullRMX_DW.obj_n.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_n.isInitialized = 0;
    FullRMX_DW.obj_n.SampleTime = -1.0;
    FullRMX_DW.obj_n.matlabCodegenIsDeleted = false;
    FullRMX_DW.obj_n.SampleTime = FullRMX_P.AnalogInput_SampleTime;
    obj = &FullRMX_DW.obj_n;
    FullRMX_DW.obj_n.isSetupComplete = false;
    FullRMX_DW.obj_n.isInitialized = 1;
    pinname = PB_0;
    obj->MW_ANALOGIN_HANDLE = MW_AnalogInSingle_Open(pinname);
    trigger_val = MW_ANALOGIN_SOFTWARE_TRIGGER;
    MW_AnalogIn_SetTriggerSource(FullRMX_DW.obj_n.MW_ANALOGIN_HANDLE,
      trigger_val, 0U);
    FullRMX_DW.obj_n.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Analog Input1' */
    FullRMX_DW.obj_j.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_j.isInitialized = 0;
    FullRMX_DW.obj_j.SampleTime = -1.0;
    FullRMX_DW.obj_j.matlabCodegenIsDeleted = false;
    FullRMX_DW.obj_j.SampleTime = FullRMX_P.AnalogInput1_SampleTime;
    obj = &FullRMX_DW.obj_j;
    FullRMX_DW.obj_j.isSetupComplete = false;
    FullRMX_DW.obj_j.isInitialized = 1;
    pinname = PB_1;
    obj->MW_ANALOGIN_HANDLE = MW_AnalogInSingle_Open(pinname);
    trigger_val = MW_ANALOGIN_SOFTWARE_TRIGGER;
    MW_AnalogIn_SetTriggerSource(FullRMX_DW.obj_j.MW_ANALOGIN_HANDLE,
      trigger_val, 0U);
    FullRMX_DW.obj_j.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Analog Input2' */
    FullRMX_DW.obj_l.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_l.isInitialized = 0;
    FullRMX_DW.obj_l.SampleTime = -1.0;
    FullRMX_DW.obj_l.matlabCodegenIsDeleted = false;
    FullRMX_DW.obj_l.SampleTime = FullRMX_P.AnalogInput2_SampleTime;
    obj = &FullRMX_DW.obj_l;
    FullRMX_DW.obj_l.isSetupComplete = false;
    FullRMX_DW.obj_l.isInitialized = 1;
    pinname = PC_4;
    obj->MW_ANALOGIN_HANDLE = MW_AnalogInSingle_Open(pinname);
    trigger_val = MW_ANALOGIN_SOFTWARE_TRIGGER;
    MW_AnalogIn_SetTriggerSource(FullRMX_DW.obj_l.MW_ANALOGIN_HANDLE,
      trigger_val, 0U);
    FullRMX_DW.obj_l.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Analog Input3' */
    FullRMX_DW.obj.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj.isInitialized = 0;
    FullRMX_DW.obj.SampleTime = -1.0;
    FullRMX_DW.obj.matlabCodegenIsDeleted = false;
    FullRMX_DW.obj.SampleTime = FullRMX_P.AnalogInput3_SampleTime;
    obj = &FullRMX_DW.obj;
    FullRMX_DW.obj.isSetupComplete = false;
    FullRMX_DW.obj.isInitialized = 1;
    pinname = PA_7;
    obj->MW_ANALOGIN_HANDLE = MW_AnalogInSingle_Open(pinname);
    trigger_val = MW_ANALOGIN_SOFTWARE_TRIGGER;
    MW_AnalogIn_SetTriggerSource(FullRMX_DW.obj.MW_ANALOGIN_HANDLE, trigger_val,
      0U);
    FullRMX_DW.obj.isSetupComplete = true;

    /* Start for MATLABSystem: '<S1>/EN_2' */
    FullRMX_DW.obj_ny.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_ny.isInitialized = 0;
    FullRMX_DW.obj_ny.matlabCodegenIsDeleted = false;
    obj_0 = &FullRMX_DW.obj_ny;
    FullRMX_DW.obj_ny.isSetupComplete = false;
    FullRMX_DW.obj_ny.isInitialized = 1;
    pinname = PA_11;
    obj_0->MW_PWM_HANDLE = MW_PWM_Open(pinname, 300.0, 50.0);
    MW_PWM_Start(FullRMX_DW.obj_ny.MW_PWM_HANDLE);
    FullRMX_DW.obj_ny.isSetupComplete = true;

    /* Start for MATLABSystem: '<S2>/EN_1' */
    FullRMX_DW.obj_na.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_na.isInitialized = 0;
    FullRMX_DW.obj_na.matlabCodegenIsDeleted = false;
    obj_0 = &FullRMX_DW.obj_na;
    FullRMX_DW.obj_na.isSetupComplete = false;
    FullRMX_DW.obj_na.isInitialized = 1;
    pinname = PC_8;
    obj_0->MW_PWM_HANDLE = MW_PWM_Open(pinname, 300.0, 50.0);
    MW_PWM_Start(FullRMX_DW.obj_na.MW_PWM_HANDLE);
    FullRMX_DW.obj_na.isSetupComplete = true;

    /* Start for MATLABSystem: '<S3>/EN_3' */
    FullRMX_DW.obj_d.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_d.isInitialized = 0;
    FullRMX_DW.obj_d.matlabCodegenIsDeleted = false;
    obj_0 = &FullRMX_DW.obj_d;
    FullRMX_DW.obj_d.isSetupComplete = false;
    FullRMX_DW.obj_d.isInitialized = 1;
    pinname = PA_9;
    obj_0->MW_PWM_HANDLE = MW_PWM_Open(pinname, 300.0, 50.0);
    MW_PWM_Start(FullRMX_DW.obj_d.MW_PWM_HANDLE);
    FullRMX_DW.obj_d.isSetupComplete = true;

    /* Start for MATLABSystem: '<S1>/PH_2' */
    FullRMX_DW.obj_nh.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_nh.isInitialized = 0;
    FullRMX_DW.obj_nh.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_nh;
    FullRMX_DW.obj_nh.isSetupComplete = false;
    FullRMX_DW.obj_nh.isInitialized = 1;
    pinname = PC_7;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_nh.isSetupComplete = true;

    /* Start for MATLABSystem: '<S2>/PH_1' */
    FullRMX_DW.obj_e.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_e.isInitialized = 0;
    FullRMX_DW.obj_e.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_e;
    FullRMX_DW.obj_e.isSetupComplete = false;
    FullRMX_DW.obj_e.isInitialized = 1;
    pinname = PC_6;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_e.isSetupComplete = true;

    /* Start for MATLABSystem: '<S3>/PH_3' */
    FullRMX_DW.obj_o.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_o.isInitialized = 0;
    FullRMX_DW.obj_o.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_o;
    FullRMX_DW.obj_o.isSetupComplete = false;
    FullRMX_DW.obj_o.isInitialized = 1;
    pinname = PB_10;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_o.isSetupComplete = true;

    /* Start for MATLABSystem: '<S1>/NS_2' */
    FullRMX_DW.obj_g.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_g.isInitialized = 0;
    FullRMX_DW.obj_g.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_g;
    FullRMX_DW.obj_g.isSetupComplete = false;
    FullRMX_DW.obj_g.isInitialized = 1;
    pinname = PB_2;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_g.isSetupComplete = true;

    /* Start for MATLABSystem: '<S2>/NS_1' */
    FullRMX_DW.obj_i.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_i.isInitialized = 0;
    FullRMX_DW.obj_i.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_i;
    FullRMX_DW.obj_i.isSetupComplete = false;
    FullRMX_DW.obj_i.isInitialized = 1;
    pinname = PA_12;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<S3>/NS_3' */
    FullRMX_DW.obj_b.matlabCodegenIsDeleted = true;
    FullRMX_DW.obj_b.isInitialized = 0;
    FullRMX_DW.obj_b.matlabCodegenIsDeleted = false;
    obj_1 = &FullRMX_DW.obj_b;
    FullRMX_DW.obj_b.isSetupComplete = false;
    FullRMX_DW.obj_b.isInitialized = 1;
    pinname = PB_15;
    obj_1->MW_DIGITALIO_HANDLE = MW_digitalIO_open(pinname, 1);
    FullRMX_DW.obj_b.isSetupComplete = true;
  }
}

/* Model terminate function */
void FullRMX_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Analog Input' */
  if (!FullRMX_DW.obj_n.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_n.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_n.isInitialized == 1) &&
        FullRMX_DW.obj_n.isSetupComplete) {
      MW_AnalogIn_Stop(FullRMX_DW.obj_n.MW_ANALOGIN_HANDLE);
      MW_AnalogIn_Close(FullRMX_DW.obj_n.MW_ANALOGIN_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Analog Input' */
  /* Terminate for MATLABSystem: '<Root>/Analog Input1' */
  if (!FullRMX_DW.obj_j.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_j.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_j.isInitialized == 1) &&
        FullRMX_DW.obj_j.isSetupComplete) {
      MW_AnalogIn_Stop(FullRMX_DW.obj_j.MW_ANALOGIN_HANDLE);
      MW_AnalogIn_Close(FullRMX_DW.obj_j.MW_ANALOGIN_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Analog Input1' */
  /* Terminate for MATLABSystem: '<Root>/Analog Input2' */
  if (!FullRMX_DW.obj_l.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_l.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_l.isInitialized == 1) &&
        FullRMX_DW.obj_l.isSetupComplete) {
      MW_AnalogIn_Stop(FullRMX_DW.obj_l.MW_ANALOGIN_HANDLE);
      MW_AnalogIn_Close(FullRMX_DW.obj_l.MW_ANALOGIN_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Analog Input2' */
  /* Terminate for MATLABSystem: '<Root>/Analog Input3' */
  if (!FullRMX_DW.obj.matlabCodegenIsDeleted) {
    FullRMX_DW.obj.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj.isInitialized == 1) && FullRMX_DW.obj.isSetupComplete) {
      MW_AnalogIn_Stop(FullRMX_DW.obj.MW_ANALOGIN_HANDLE);
      MW_AnalogIn_Close(FullRMX_DW.obj.MW_ANALOGIN_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Analog Input3' */
  /* Terminate for MATLABSystem: '<S1>/EN_2' */
  if (!FullRMX_DW.obj_ny.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_ny.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_ny.isInitialized == 1) &&
        FullRMX_DW.obj_ny.isSetupComplete) {
      MW_PWM_Stop(FullRMX_DW.obj_ny.MW_PWM_HANDLE);
      MW_PWM_Close(FullRMX_DW.obj_ny.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S1>/EN_2' */

  /* Terminate for MATLABSystem: '<S2>/EN_1' */
  if (!FullRMX_DW.obj_na.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_na.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_na.isInitialized == 1) &&
        FullRMX_DW.obj_na.isSetupComplete) {
      MW_PWM_Stop(FullRMX_DW.obj_na.MW_PWM_HANDLE);
      MW_PWM_Close(FullRMX_DW.obj_na.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/EN_1' */

  /* Terminate for MATLABSystem: '<S3>/EN_3' */
  if (!FullRMX_DW.obj_d.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_d.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_d.isInitialized == 1) &&
        FullRMX_DW.obj_d.isSetupComplete) {
      MW_PWM_Stop(FullRMX_DW.obj_d.MW_PWM_HANDLE);
      MW_PWM_Close(FullRMX_DW.obj_d.MW_PWM_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S3>/EN_3' */
  /* Terminate for MATLABSystem: '<S1>/PH_2' */
  if (!FullRMX_DW.obj_nh.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_nh.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_nh.isInitialized == 1) &&
        FullRMX_DW.obj_nh.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_nh.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S1>/PH_2' */

  /* Terminate for MATLABSystem: '<S2>/PH_1' */
  if (!FullRMX_DW.obj_e.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_e.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_e.isInitialized == 1) &&
        FullRMX_DW.obj_e.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_e.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/PH_1' */

  /* Terminate for MATLABSystem: '<S3>/PH_3' */
  if (!FullRMX_DW.obj_o.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_o.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_o.isInitialized == 1) &&
        FullRMX_DW.obj_o.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_o.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S3>/PH_3' */

  /* Terminate for MATLABSystem: '<S1>/NS_2' */
  if (!FullRMX_DW.obj_g.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_g.isInitialized == 1) &&
        FullRMX_DW.obj_g.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_g.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S1>/NS_2' */

  /* Terminate for MATLABSystem: '<S2>/NS_1' */
  if (!FullRMX_DW.obj_i.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_i.isInitialized == 1) &&
        FullRMX_DW.obj_i.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_i.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/NS_1' */

  /* Terminate for MATLABSystem: '<S3>/NS_3' */
  if (!FullRMX_DW.obj_b.matlabCodegenIsDeleted) {
    FullRMX_DW.obj_b.matlabCodegenIsDeleted = true;
    if ((FullRMX_DW.obj_b.isInitialized == 1) &&
        FullRMX_DW.obj_b.isSetupComplete) {
      MW_digitalIO_close(FullRMX_DW.obj_b.MW_DIGITALIO_HANDLE);
    }
  }

  /* End of Terminate for MATLABSystem: '<S3>/NS_3' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
