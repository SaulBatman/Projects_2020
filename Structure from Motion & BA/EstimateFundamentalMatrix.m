function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

A = zeros(8,9);
for i=1:8
    A(i,:) = [x1(i,1)*x2(i,1) x1(i,1)*x2(i,2) x1(i,1) x1(i,2)*x2(i,1) x1(i,2)*x2(i,2) x1(i,2) x2(i,1) x2(i,2) 1];  
end
[~,~,VT] = svd(A);
V = VT';
x = V(:,8);
F_none = reshape(x,3,3);
[U1,d1,VT1] = svd(F_none);
d1(end,end) = 0;
F = U1*d1*VT1;
