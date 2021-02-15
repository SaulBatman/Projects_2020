function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

%A = zeros(12,3);

%for i=1:6
%   A(2*i-1,:)=[X(i,:)' 0 -x(i,1)*X(i,:)'];
%   A(2*i,:)= [0 X(i,:)' -x(i,2)*X(i,:)'];
%end
%A=zeros(18,12);
zero = zeros(1,4);
X_homo=X;
X_homo(4,:) = 1;
x_homo = x;
x_homo(:,3) = 1;
x_homo=x_homo';
[N,~]=size(x);
A=zeros(2*N,12);
%for i=1:6
%   A(3*i-2,:)=[zero -X_homo(:,i)' x_homo(2,i)*X_homo(:,i)'];
%   A(3*i-1,:)=[X_homo(:,i)' zero  -x_homo(1,i)*X_homo(:,i)'];
%   A(3*i,:)=[-x_homo(2,i)*X_homo(:,i)' x_homo(1,i)*X_homo(:,i)' zero];
%end
for i=1:N
    A(2*i-1,:)=[X_homo(:,i)' zero  -x_homo(1,i).*X_homo(:,i)'];
    A(2*i,:)=[zero X_homo(:,i)' -x_homo(2,i).*X_homo(:,i)'];
end
[~,~,VT] = svd(A);
P = VT(:,12);
P = reshape(P,4,3)';
%{
R = K^(-1)*P(:,1:3);
[U1,d,VT1] = svd(R);
if det(U1*VT1)<0
    R= -(R*R')^(0.5)*R;
    [U1,d,VT1] = svd(R);
    R=U1*VT1;
    t = -K^(-1)*P(:,4)/ d(1,1);
else
    R= (R*R')^(0.5)*R;
    [U1,d,VT1] = svd(R);
    R=U1*VT1;
    t = K^(-1)*P(:,4)/ d(1,1);
end
C = -R'*t;
%}
[K1, R] = rqGivens(P(1:3, 1:3));

%% ensure that the diagonal is positive
if K1(3, 3) < 0
    K1 = -K1;
    R = -R;
end
if K1(2, 2) < 0
    S = [1  0  0 
         0 -1  0
         0  0  1];
    K1 = K1 * S;
    R = S * R;
end
if K1(1, 1) < 0
    S = [-1  0  0 
          0  1  0
          0  0  1];
    K1 = K1 * S;
    R = S * R;
end

%% ensure R determinant == 1
t = linsolve(K, P(:, 4));

if det(R) < 0
    R = -R;
    t = -t;
end

%K = K ./ K(3, 3);
C = -R'*t;
end
