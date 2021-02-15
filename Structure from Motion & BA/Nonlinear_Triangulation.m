function X = Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X)
%% Nonlinear_Triangulation
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
% Refining the poses of the cameras to get a better estimate of the points
% 3D position
% Inputs: 
%     K - size (3 x 3) camera calibration (intrinsics) matrix
%     x
% Outputs: 
%     X - size (N x 3) matrix of refined point 3D locations 
[row, ~] = size(X);
for j=1:row
X(j,:) = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1(j,:), x2(j,:), x3, X(j,:));
end
%X= Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X(2,:));

end

function X = Single_Point_Nonlinear_Triangulation(K, C1, R1, C2, R2, C3, R3, x1, x2, x3, X0)
X0=X0';
focal = K(1,1); px = K(1,3); py = K(2,3); % extract intrinsic parameters
%C1=C1';C2=C2';C3=C3';
J123=zeros(6,24);
for i=1:40
J1=Jacobian_Triangulation(C1,R1,K,X0);
J2=Jacobian_Triangulation(C2,R2,K,X0);
J3=Jacobian_Triangulation(C3,R3,K,X0);
zero27=zeros(2,7);
J123(1:2,:) = [J1(:,1:7) zero27 zero27 J1(:,8:10)];
J123(3:4,:) = [zero27 J2(:,1:7) zero27 J2(:,8:10)];
J123(5:6,:) = [zero27 zero27 J3(:,1:7) J3(:,8:10)];
% image1 reprojection f1
u1=[focal*R1(1,1)+px*R1(3,1) focal*R1(1,2)+px*R1(3,2) focal*R1(1,3)+px*R1(3,3)]*(X0-C1');
v1=[focal*R1(2,1)+px*R1(3,1) focal*R1(2,2)+px*R1(3,2) focal*R1(2,3)+px*R1(3,3)]*(X0-C1');
w1=[R1(3,1) R1(3,2) R1(3,3)]*(X0-C1'); 
f1 = [u1/w1;v1/w1];
% image2 reprojection
u2=[focal*R2(1,1)+px*R2(3,1) focal*R2(1,2)+px*R2(3,2) focal*R2(1,3)+px*R2(3,3)]*(X0-C2')';
v2=[focal*R2(2,1)+px*R2(3,1) focal*R2(2,2)+px*R2(3,2) focal*R2(2,3)+px*R2(3,3)]*(X0-C2')';
w2=[R2(3,1) R2(3,2) R2(3,3)]*(X0-C2')'; 
f2 = [u2/w2;v2/w2];
% image3 reprojection
u3=[focal*R3(1,1)+px*R3(3,1) focal*R3(1,2)+px*R3(3,2) focal*R3(1,3)+px*R3(3,3)]*(X0-C3')';
v3=[focal*R3(2,1)+px*R1(3,1) focal*R3(2,2)+px*R3(3,2) focal*R3(2,3)+px*R3(3,3)]*(X0-C3')';
w3=[R3(3,1) R3(3,2) R3(3,3)]*(X0-C3')'; 
f3 = [u3/w3;v3/w3];

f=[f1;f2;f3];
b = [x1(1);x1(2);x2(1);x2(2);x3(1);x3(2)];

delta_x = pinv((J123'*J123))*J123'*(b-f);
%update f
f = f+J123*delta_x;

%update C1,C2,C3
C1=C1+[delta_x(5);delta_x(6);delta_x(7)];
C2=C2+[delta_x(12);delta_x(13);delta_x(14)];
C3=C3+[delta_x(19);delta_x(20);delta_x(21)];

%update [w x y z] in q1 q2 q3
q1=dcm2quat(R1);q2=dcm2quat(R2);q3=dcm2quat(R3);
q1(1)=q1(1)+delta_x(1);
q1(2)=q1(2)+delta_x(2);
q1(3)=q1(3)+delta_x(3);
q1(4)=q1(4)+delta_x(4);
q2(1)=q2(1)+delta_x(8);
q2(2)=q2(2)+delta_x(9);
q2(3)=q2(3)+delta_x(10);
q2(4)=q2(4)+delta_x(11);
q3(1)=q3(1)+delta_x(15);
q3(2)=q3(2)+delta_x(16);
q3(3)=q3(3)+delta_x(17);
q3(4)=q3(4)+delta_x(18);

R1=quat2dcm(q1);R2=quat2dcm(q2);R3=quat2dcm(q3);

%update X of a single point
X0=X0+delta_x(end-2:end);


end
X=X0;
% form q1 q2 q3 into R1 R2 R3
end

function J = Jacobian_Triangulation(C, R, K, X)
focal = K(1,1); px = K(1,3); py = K(2,3); % extract intrinsic parameters
q = dcm2quat(R);  % form R into quat
% extract pixel coordinate
u=[focal*R(1,1)+px*R(3,1) focal*R(1,2)+px*R(3,2) focal*R(1,3)+px*R(3,3)]*(X-C);
v=[focal*R(2,1)+py*R(3,1) focal*R(2,2)+py*R(3,2) focal*R(2,3)+py*R(3,3)]*(X-C);
w=[R(3,1) R(3,2) R(3,3)]*(X-C); 
% calculate df_dc
du_dc=-[focal*R(1,1)+px*R(3,1) focal*R(1,2)+px*R(3,2) focal*R(1,3)+px*R(3,3)];
dv_dc=-[focal*R(2,1)+py*R(3,1) focal*R(2,2)+py*R(3,2) focal*R(2,3)+py*R(3,3)];
dw_dc=-[R(3,1) R(3,2) R(3,3)];
df_dc=[du_dc/w - u*dw_dc/w^(2); dv_dc/w - v*dw_dc/w^(2)];
% calculate df_dx
du_dx =[focal*R(1,1)+px*R(3,1) focal*R(1,2)+px*R(3,2) focal*R(1,3)+px*R(3,3)];
dv_dx =[focal*R(2,1)+py*R(3,1) focal*R(2,2)+py*R(3,2) focal*R(2,3)+py*R(3,3)];
dw_dx =[R(3,1) R(3,2) R(3,3)];
df_dx=[du_dx/w - u*dw_dx/w^(2); dv_dx/w - v*dw_dx/w^(2)];
% calculate df_dq
du_dR=[focal*(X-C)' 0 0 0 px*(X-C)'];
dv_dR=[0 0 0 focal*(X-C)' px*(X-C)'];
dw_dR=[0 0 0 0 0 0 (X-C)'];
df_dR=[du_dR/w - u*dw_dR/w^(2); dv_dR/w - v*dw_dR/w^(2)];
%q=[qw qx qy qz]
qw=q(1);qx=q(2);qy=q(3);qz=q(4);
dR_dq = [
    [0 -4*qy -4*qz 0]
    [2*qy 2*qx -2*qw -2*qz]
    [2*qz 2*qw -2*qx -2*qy]
    [2*qy 2*qx -2*qw -2*qz]
    [-4*qx 0 -4*qz 0]
    [-2*qw 2*qz 2*qy 2*qx]
    [2*qz -2*qw 2*qx -2*qy]
    [-4*qx -4*qy 0 0]
    [2*qw 2*qz 2*qy 2*qx]
    ];
%df_dq = df_dR*dR_dq;
J = [df_dR*dR_dq df_dc df_dx];
end
