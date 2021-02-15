function E = EssentialMatrixFromFundamentalMatrix(F,K)
%% EssentialMatrixFromFundamentalMatrix
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
% Use the camera calibration matrix to esimate the Essential matrix
% Inputs:
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     F - size (3 x 3) fundamental matrix from EstimateFundamentalMatrix
% Outputs:
%     E - size (3 x 3) Essential matrix with singular values (1,1,0)

E = K'*F*K;
[U,d,VT]=svd(E);
d(3,3)=0;
E=U*d*VT;
