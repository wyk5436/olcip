#include "C:/Users/controls/Documents/MATLAB/Spring24/eigen-3.4.0/Eigen/Dense"
#include "C:/Users/controls/Documents/MATLAB/Spring24/eigen-3.4.0/unsupported/Eigen/MatrixFunctions"
extern "C" {
#define S_FUNCTION_NAME CS.cpp
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"


static void init_dmd(Eigen::MatrixXd data, Eigen::MatrixXd& Am, Eigen::MatrixXd& P, Eigen::VectorXd& xkp1) {
    int numCols = data.cols();
    Eigen::MatrixXd X = data.block(0, 0, data.rows(), numCols - 1);
    Eigen::MatrixXd Y = data.block(0, 1, data.rows(), numCols - 1);
    
    for (int k = 0; k < 4; k++) {
        xkp1(k) = data(k,numCols - 1);
    }
    
    Am = Y * (X*X.transpose()*(X*X.transpose()).inverse());
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

Eigen::MatrixXd dlqr(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol) {
    double diff = 9999;
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd PP;
    while (diff > tol) {
        PP = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        diff = (P - PP).norm();
        P = PP;
    }
    return (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
}

static void mdlInitializeSampleTimes(SimStruct *S) {
    ssSetSampleTime(S, 0, 0.01);
    ssSetOffsetTime(S, 0, 0.0);
}

static void mdlInitializeSizes(SimStruct *S)
{
    ssSetNumSFcnParams(S, 0);  /* Number of expected parameters */
    if (!ssSetNumInputPorts(S, 2)) return;
    ssSetInputPortWidth(S, 0, 4);
    ssSetInputPortWidth(S, 1, 1);
    ssSetInputPortRequiredContiguous(S, 0, 1);
    ssSetInputPortRequiredContiguous(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    
    if (!ssSetNumOutputPorts(S, 1)) return;
    ssSetOutputPortWidth(S, 0, 1);
    
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
    ssSetDWorkName(S, 0, "state");
    ssSetDWorkComplexSignal(S, 0, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 1, SS_UINT32);
    ssSetDWorkWidth(S, 1, 1);
    ssSetDWorkName(S, 1, "time_called");
    ssSetDWorkComplexSignal(S, 1, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 2, SS_DOUBLE);
    ssSetDWorkWidth(S, 2, 4);
    ssSetDWorkName(S, 2, "control_gain");
    ssSetDWorkComplexSignal(S, 2, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 3, SS_DOUBLE);
    ssSetDWorkWidth(S, 3, 1);
    ssSetDWorkName(S, 3, "control");
    ssSetDWorkComplexSignal(S, 3, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 4, SS_DOUBLE);
    ssSetDWorkWidth(S, 4, 4);
    ssSetDWorkName(S, 4, "xkp1");
    ssSetDWorkComplexSignal(S, 4, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 5, SS_DOUBLE);
    ssSetDWorkWidth(S, 5, 16);
    ssSetDWorkName(S, 5, "Am");
    ssSetDWorkComplexSignal(S, 5, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 6, SS_DOUBLE);
    ssSetDWorkWidth(S, 6, 16);
    ssSetDWorkName(S, 6, "P");
    ssSetDWorkComplexSignal(S, 6, COMPLEX_NO);
    
    ssSetDWorkDataType(S, 7, SS_UINT32);
    ssSetDWorkWidth(S, 7, 1);
    ssSetDWorkName(S, 7, "flag");
    ssSetDWorkComplexSignal(S, 7, COMPLEX_NO);

    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    /* specify the sim state compliance to be same as a built-in block */
    ssSetOperatingPointCompliance(S, USE_DEFAULT_OPERATING_POINT);
    ssSetRuntimeThreadSafetyCompliance(S, RUNTIME_THREAD_SAFETY_COMPLIANCE_TRUE);
    ssSetOptions(S, SS_OPTION_EXCEPTION_FREE_CODE);
}

#define MDL_start
static void mdlStart(SimStruct *S) {
    real_T *u = (real_T*) ssGetDWork(S, 3);
    u = 0;
    
    boolean_T *flag = (boolean_T*) ssGetDWork(S, 7);
    flag = 0; 
}


#define MDL_UPDATE
static void mdlUpdate(SimStruct *S, int_T tid)
{
    double K1 = 0.2065;
    double J = 0.0076;
    double l = 0.337;
    double r = 0.216;
    Eigen::VectorXd B(4);
    B << 0, 0, K1/J, -r*K1/(J*l);
    Eigen::MatrixXd Q(4,4);
    Q << 5, 0, 0, 0,
         0, 30, 0, 0,
         0, 0, 0, 0,
         0, 0, 0, 0;
    Eigen::MatrixXd R(1, 1);
    R << 1;
    double dt = 0.01;
    Eigen::MatrixXd disc_B = dt * B;
    
    real_T            *statePtr   = static_cast<real_T*>(ssGetDWork(S, 0));
    InputRealPtrsType xPtrs       = ssGetInputPortRealSignalPtrs(S,0);
    InputRealPtrsType tPtrs       = ssGetInputPortRealSignalPtrs(S,1);
    
    double* flatArray = new double[4];
    for (int i = 0; i < 4; ++i) {
        flatArray[i] = *(xPtrs[i]);
    }
    Eigen::Map<Eigen::VectorXd> mappedvecx(flatArray, 4);
    Eigen::VectorXd vecx = mappedvecx;
    
    if ((*tPtrs[0] > 2) && (*tPtrs[0] < 3)){ // learning
        real_T *countPtr = (real_T*)ssGetDWork(S, 1);
        *countPtr += 1;
        int i = *countPtr;
        
        if (i == 99){
            real_T *flagPtr = (real_T*)ssGetDWork(S, 7);
            *flagPtr += 1;
        }

        if (i <= 2000){
            int idx = (i-1)*4 + 1;
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
    }
    
    real_T* flagPtr = static_cast<real_T*>(ssGetDWork(S, 7));
    if (*flagPtr == 1) {
        *flagPtr = *flagPtr + 1;
        
        real_T* countPtr = static_cast<real_T*>(ssGetDWork(S, 1));
        *countPtr = *countPtr + 1;
        int i = *countPtr;
        int idx = (i-1)*4 + 1;
        for (int k = idx; k < idx + 4; k++){
            statePtr[k] = *xPtrs[k - idx];
        }
        Eigen::VectorXd state_data(idx + 3);
        for (int i = 0; i < idx + 3; i++) {
            state_data(i) = statePtr[i];
        }
        Eigen::Map<Eigen::MatrixXd> mat(state_data.data(), 4, i);
        
        Eigen::MatrixXd Am(4,4);
        Eigen::MatrixXd P(4,4);
        Eigen::VectorXd xkp1(4);
        init_dmd(mat, Am, P, xkp1);
        Eigen::MatrixXd K = dlqr(Am,disc_B,Q,R,0.0001);
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
        Eigen::Map<Eigen::VectorXd> mappedxkp1(xkp1Ptr, 4);
        
        real_T* PPtr = static_cast<real_T*>(ssGetDWork(S, 6));
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> mappedP(PPtr);
        
        real_T* AmPtr = static_cast<real_T*>(ssGetDWork(S, 5));
        Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>> mappedAm(AmPtr);
        
        real_T* uPtr = static_cast<real_T*>(ssGetDWork(S, 3));
        double uk = *uPtr;
        
        
        Eigen::MatrixXd Am = mappedAm;
        Eigen::MatrixXd P = mappedP;
        Eigen::VectorXd xkp1 = mappedxkp1;
        Eigen::VectorXd x = mappedxkp1;
        online_dmd_update(Am, P, xkp1, vecx, uk);
        
        Eigen::MatrixXd K = dlqr(Am,disc_B,Q,R,0.0001);
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
    delete[] flatArray;
}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    real_T* uPtr = static_cast<real_T*>(ssGetDWork(S, 3));
    double u = *uPtr;
    real_T *y = ssGetOutputPortRealSignal(S, 0);
    y[0] = u;
}

static void mdlTerminate(SimStruct *S) {
}

#ifdef MATLAB_MEX_FILE
#include "simulink.c"      // Mex glue
#else
#include "cg_sfun.h"       // Code generation glue
#endif
}