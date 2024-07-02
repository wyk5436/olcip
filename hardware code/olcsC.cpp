#include <Eigen/Dense>
#include <Eigen/MatrixFunctions>
#include "dlqr_custom.h"

extern "C" {
#define S_FUNCTION_NAME olcsC
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <string>

static void init_dmd(Eigen::MatrixXd data, Eigen::MatrixXd& Am, Eigen::MatrixXd& P, Eigen::VectorXd& xkp1) {
    int numCols = data.cols();
    Eigen::MatrixXd X = data.block(0, 0, data.rows(), numCols - 1);
    Eigen::MatrixXd Y = data.block(0, 1, data.rows(), numCols - 1);
    
    for (int k = 0; k < 4; k++) {
        xkp1(k) = data(k,numCols - 1);
    }
    
    Am = Y * (X.transpose()*(X*X.transpose()).inverse());
    P = (X*X.transpose()).inverse();
}

static void online_dmd_update(Eigen::MatrixXd& Am, Eigen::MatrixXd& P, Eigen::VectorXd& xkp1, Eigen::VectorXd x, double uk) {
    double K1 = 0.2065;
    double J = 0.0076;
    double l = 0.337;
    double r = 0.216;
    Eigen::VectorXd B(4);
    B << 0, 0, K1/J, -r*K1/(J*l);
    double dt = 0.01;
    Eigen::VectorXd disc_B = B*dt;
    
    double gamma = 1 / (1 + xkp1.transpose() * P *xkp1);
    Eigen::VectorXd ykp1 = x - disc_B*uk;
    
    Am = Am + gamma*(ykp1 - Am*xkp1)*xkp1.transpose()*P;
    P = P - gamma*P*(xkp1*xkp1.transpose())*P;
    xkp1 = x;
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 0.01);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 4);  // Number of expected parameters

    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; // Parameter mismatch will be reported by Simulink
    }

    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 4);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    
    if (!ssSetNumOutputPorts(S, 2)) return;
    ssSetOutputPortWidth(S, 0, 1);
    ssSetOutputPortWidth(S, 1, 16);
    
    ssSetNumSampleTimes(S, 1);
    
    ssSetNumDWork(S, 8);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);
    
    /* Specifying Dwork vectors*/
    ssSetDWorkDataType(S, 0, SS_DOUBLE);
    ssSetDWorkWidth(S, 0, 4*2000);
    ssSetDWorkName(S, 0, "stateASS");
    ssSetDWorkComplexSignal(S, 0, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 1, SS_DOUBLE);
    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkName(S, 1, "time_calledASS");
    ssSetDWorkComplexSignal(S, 1, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 2, SS_DOUBLE);
    ssSetDWorkWidth(S, 2, 4);
    ssSetDWorkName(S, 2, "control_gainASS");
    ssSetDWorkComplexSignal(S, 2, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 3, SS_DOUBLE);
    ssSetDWorkWidth(S, 3, 1);
    ssSetDWorkName(S, 3, "controlASS");
    ssSetDWorkComplexSignal(S, 3, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 4, SS_DOUBLE);
    ssSetDWorkWidth(S, 4, 4);
    ssSetDWorkName(S, 4, "xkp1ASS");
    ssSetDWorkComplexSignal(S, 4, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 5, SS_DOUBLE);
    ssSetDWorkWidth(S, 5, 16);
    ssSetDWorkName(S, 5, "AmASS");
    ssSetDWorkComplexSignal(S, 5, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 6, SS_DOUBLE);
    ssSetDWorkWidth(S, 6, 16);
    ssSetDWorkName(S, 6, "PASS");
    ssSetDWorkComplexSignal(S, 6, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 7, SS_DOUBLE);
    ssSetDWorkWidth(S, 7, 1);
    ssSetDWorkName(S, 7, "flagASS");
    ssSetDWorkComplexSignal(S, 7, COMPLEX_NO);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

#define MDL_START
static void mdlStart(SimStruct *S) {
    real_T *count = (real_T*) ssGetDWork(S, 1);
    *count = 0;
    
    real_T *u = (real_T*) ssGetDWork(S, 3);
    u[0] = 0.0;
    
    real_T *flag = (real_T*) ssGetDWork(S, 7);
    *flag = 0;
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    double K1 = 0.2065;
    double J = 0.0076;
    double l = 0.337;
    double r = 0.216;
    Eigen::VectorXd B(4);
    B << 0, 0, K1/J, -r*K1/(J*l);

    real_T Q_1 = mxGetPr(ssGetSFcnParam(S, 0))[0];
    real_T Q_2 = mxGetPr(ssGetSFcnParam(S, 1))[0];
    real_T Q_3 = mxGetPr(ssGetSFcnParam(S, 2))[0];
    real_T Q_4 = mxGetPr(ssGetSFcnParam(S, 3))[0];

    Eigen::MatrixXd Q(4,4);
    Q << Q_1, 0, 0, 0,
         0, Q_2, 0, 0,
         0, 0, Q_3, 0,
         0, 0, 0, Q_4;
    Eigen::MatrixXd R(1, 1);
    R << 1;
    double dt = 0.01;
    Eigen::MatrixXd disc_B = dt * B;
    
    real_T        *statePtr   = static_cast<real_T*>(ssGetDWork(S, 0));
    real_T        *countPtr   = (real_T*)ssGetDWork(S, 1);
    real_T        *flagPtr    = (real_T*)ssGetDWork(S, 7);
    InputRealPtrsType xPtrs   = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType tPtrs   = ssGetInputPortRealSignalPtrs(S,1);
    
     
    Eigen::VectorXd vecx(4);
    for (int i = 0; i < 4; ++i) {
        if (xPtrs[i] == nullptr) {
            ssSetErrorStatus(S, "One of the input pointers is null");
            return;
        }
        vecx(i) = static_cast<double>(*xPtrs[i]);
    }
    
    if ((*tPtrs[0] > 2) && (*tPtrs[0] < 3)){ // learning
        *countPtr = *countPtr + 1;
        int i = *countPtr;
        
        if (i == 99){
            *flagPtr = *flagPtr + 1;
        }

        if (i <= 2000){
            int idx = (i-1)*4;
            for (int k = idx; k < idx + 4; k++){
                statePtr[k] = *xPtrs[k - idx];
            }
        } else{
            double recentData[1997];
            for (int k = 3; k < 2000; k++){
                recentData[k-3] = statePtr[k];
            }
            for (int k = 0; k < 1997; k++){
                statePtr[k] = recentData[k];
            }
            for (int k = 1996; k < 2000; k++){
                statePtr[k] = *xPtrs[k - 1996];
            }
        }
    } else if (*flagPtr == 1) {
        *flagPtr = *flagPtr + 1;

        *countPtr = *countPtr + 1;
        int i = *countPtr;
        int idx = (i-1)*4;
        for (int k = idx; k < idx + 4; k++){
            statePtr[k] = *xPtrs[k - idx];
        }
        
        Eigen::VectorXd state_data(idx + 4);
        for (int i = 0; i < idx + 4; i++) {
            state_data(i) = statePtr[i];
        }
        Eigen::Map<Eigen::MatrixXd> mat(state_data.data(), 4, i);
            
        Eigen::MatrixXd Am(4,4);
        Eigen::MatrixXd P(4,4);
        Eigen::VectorXd xkp1(4);
        init_dmd(mat, Am, P, xkp1);
        Eigen::MatrixXd K = dlqr(Am,disc_B,Q,R,0.0000000000001);
        Eigen::MatrixXd u = -K*vecx;

        real_T* KPtr = static_cast<real_T*>(ssGetDWork(S, 2));
        for (int i = 0; i < 4; i++) {
            KPtr[i] = K(i);
        }
        real_T* uPtr = static_cast<real_T*>(ssGetDWork(S, 3));
        uPtr[0] = u(0);
        
        real_T* xkp1Ptr = static_cast<real_T*>(ssGetDWork(S, 4));
        for (int i = 0; i < 4; i++) {
            xkp1Ptr[i] = xkp1(i);
        }
        real_T* AmPtr = static_cast<real_T*>(ssGetDWork(S, 5));
        for (int i = 0; i < 16; i++) {
            AmPtr[i] = Am(i);
        }
        real_T* PPtr = static_cast<real_T*>(ssGetDWork(S, 6));
        for (int i = 0; i < 16; i++) {
            PPtr[i] = P(i);
        }
    } else if (*flagPtr > 1) {
        real_T*    xkp1Ptr   = static_cast<real_T*>(ssGetDWork(S, 4));
        Eigen::Map<Eigen::VectorXd> mappedxkp1(xkp1Ptr, 4, 1);
        
        real_T* PPtr = static_cast<real_T*>(ssGetDWork(S, 6));
        Eigen::Map<Eigen::MatrixXd> mappedP(PPtr, 4 ,4);
        
        real_T* AmPtr = static_cast<real_T*>(ssGetDWork(S, 5));
        Eigen::Map<Eigen::MatrixXd> mappedAm(AmPtr, 4, 4);
        
        real_T* uPtr = static_cast<real_T*>(ssGetDWork(S, 3));
        double uk = *uPtr;
        
        Eigen::MatrixXd Am = mappedAm;
        Eigen::MatrixXd P = mappedP;
        Eigen::VectorXd xkp1 = mappedxkp1;
        Eigen::VectorXd x = mappedxkp1;
        online_dmd_update(Am, P, xkp1, vecx, uk);
        
        Eigen::MatrixXd K = dlqr(Am,disc_B,Q,R,1e-6);
        Eigen::MatrixXd u = -K*vecx;
        
        real_T* KPtr = static_cast<real_T*>(ssGetDWork(S, 2));
        for (int i = 0; i < 4; i++) {
            KPtr[i] = K(i);
        }
        
        uPtr[0] = u(0);
        
        for (int i = 0; i < 4; i++) {
            xkp1Ptr[i] = xkp1(i);
        }
        
        for (int i = 0; i < 16; i++) {
            AmPtr[i] = Am(i);
        }
        
        for (int i = 0; i < 16; i++) {
            PPtr[i] = P(i);
        }
    }
    
    
    real_T* uPtr = static_cast<real_T*>(ssGetDWork(S, 3));
    double u = *uPtr;
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    y[0] = u;
    
    real_T *yy = ssGetOutputPortRealSignal(S, 1);
    real_T* AmPtr = static_cast<real_T*>(ssGetDWork(S, 5));
    for (int i = 0; i < 16; i++) {
        yy[i] = AmPtr[i];
    }
}

static void mdlTerminate(SimStruct *S) {
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"      // Mex glue
#else
#include "cg_sfun.h"       // Code generation glue
#endif
}
