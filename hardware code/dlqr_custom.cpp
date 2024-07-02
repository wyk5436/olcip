#include <Eigen/Dense>

#include "dlqr_custom.h"

#include "mex.h"

Eigen::MatrixXd dlqr(Eigen::MatrixXd A, Eigen::MatrixXd B, Eigen::MatrixXd Q, Eigen::MatrixXd R, double tol) {
    double diff = 9999.0;
    Eigen::MatrixXd P = Q;
    Eigen::MatrixXd PP;
    while (diff > tol) {
        PP = A.transpose() * P * A - A.transpose() * P * B * (R + B.transpose() * P * B).inverse() * B.transpose() * P * A + Q;
        diff = (P - PP).norm();
        P = PP;
    }
    return (R + B.transpose() * P * B).inverse() * B.transpose() * P * A;
}


// void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
//     // Check for proper number of arguments
//     if (nrhs != 5) {
//         mexErrMsgIdAndTxt("MATLAB:dlqr:invalidNumInputs", "Five inputs required.");
//     }
//     if (nlhs > 1) {
//         mexErrMsgIdAndTxt("MATLAB:dlqr:maxlhs", "Too many output arguments.");
//     }

//     // Convert inputs from MATLAB to Eigen
//     Eigen::MatrixXd A = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[0]), mxGetM(prhs[0]), mxGetN(prhs[0]));
//     Eigen::MatrixXd B = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[1]), mxGetM(prhs[1]), mxGetN(prhs[1]));
//     Eigen::MatrixXd Q = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[2]), mxGetM(prhs[2]), mxGetN(prhs[2]));
//     Eigen::MatrixXd R = Eigen::Map<Eigen::MatrixXd>(mxGetPr(prhs[3]), mxGetM(prhs[3]), mxGetN(prhs[3]));
//     double tol = mxGetScalar(prhs[4]);

//     // Call the dlqr function
//     Eigen::MatrixXd K = dlqr(A, B, Q, R, tol);

//     // Convert the result from Eigen to MATLAB
//     plhs[0] = mxCreateDoubleMatrix(K.rows(), K.cols(), mxREAL);
//     Eigen::Map<Eigen::MatrixXd>(mxGetPr(plhs[0]), K.rows(), K.cols()) = K;
// }