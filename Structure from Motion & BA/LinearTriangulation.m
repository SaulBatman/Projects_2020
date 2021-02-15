function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 3) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 3) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points
P1 = K*R1*[eye(3) -C1];
P2 = K*R2*[eye(3) -C2];
%P1 = K*[R1 C1];
%P2 = K*[R2 C2];
[N,~]=size(x1);
x1_homo = x1;
x2_homo = x2;
x1_homo(:,3) = 1;
x2_homo(:,3) = 1;
for i=1:N
    skew1 = Vec2Skew(x1_homo(i,:));
    skew2 = Vec2Skew(x2_homo(i,:));
    A = [skew1*P1; skew2*P2];
    [~,~,vt] = svd(A);
    X(:,i) = vt(:,end)/vt(end,end);
end
X=X(1:3,:);


