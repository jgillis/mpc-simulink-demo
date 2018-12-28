#define S_FUNCTION_NAME  LMPC
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include "MPC_step.h"

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    /* Read in CasADi function dimensions */
    int_T n_in  = MPC_step_n_in();
    int_T n_out = MPC_step_n_out();
    int_T sz_arg, sz_res, sz_iw, sz_w;
    MPC_step_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    
    /* Set up simulink input/output ports */
    int_T i;
    if (!ssSetNumInputPorts(S, n_in)) return;
    for (i=0;i<n_in;++i) {
      const int_T* sp = MPC_step_sparsity_in(i);
      /* Dense inputs assumed here */
      ssSetInputPortDirectFeedThrough(S, i, 1);
      ssSetInputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }

    if (!ssSetNumOutputPorts(S, n_out+1)) return;
    for (i=0;i<n_out;++i) {
      const int_T* sp = MPC_step_sparsity_out(i);
      /* Dense outputs assumed here */
      ssSetOutputPortMatrixDimensions(S, i, sp[0], sp[1]);
    }
    /* Status flag */
    ssSetOutputPortMatrixDimensions(S, n_out, 1, 1);
    
    ssSetNumSampleTimes(S, 1);
    
    /* Set up CasADi function work vector sizes */
    ssSetNumRWork(S, sz_w);
    ssSetNumIWork(S, sz_iw);
    ssSetNumPWork(S, sz_arg+sz_res);
    ssSetNumNonsampledZCs(S, 0);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetSimStateCompliance(S, USE_DEFAULT_SIM_STATE);

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR);
}


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    Specifiy that we inherit our sample time from the driving block.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S); 
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    
    /* Read in CasADi function dimensions */
    int_T n_in  = MPC_step_n_in();
    int_T n_out = MPC_step_n_out();
    int_T sz_arg, sz_res, sz_iw, sz_w;
    MPC_step_work(&sz_arg, &sz_res, &sz_iw, &sz_w);
    
    /* Set up CasADi function work vectors */
    void** p = ssGetPWork(S);
    const real_T** arg = (const real_T**) p;
    p += sz_arg;
    real_T** res = (real_T**) p;
    real_T* w = ssGetRWork(S);
    int_T* iw = ssGetIWork(S);
    
    
    /* Point to input and output buffers */
    int_T i;   
    for (i=0; i<n_in;++i) {
      arg[i] = *ssGetInputPortRealSignalPtrs(S,i);
    }
    for (i=0; i<n_out;++i) {
      res[i] = ssGetOutputPortRealSignal(S,i);
    }
    /* Run the CasADi function */
    int ret = MPC_step(arg,res,iw,w,0);
    ssGetOutputPortRealSignal(S, n_out)[0] = ret;
}

static void mdlTerminate(SimStruct *S) {}


#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif