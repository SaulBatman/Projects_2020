function R=q2d(q)
% Template is from UPenn Robotics: Perception Course
% Algorithm implemented by Mingxi Jia
R=[ 1-2*q(3).^2-2*q(4)^2    2*(q(2)*q(3)-q(1)*q(4)) 2*(q(2)*q(4)+q(1)*q(3));
    2*(q(2)*q(3)+q(1)*q(4)) 1-2*q(2)^2-2*q(4)^2     2*(q(3)*q(4)-q(1)*q(2));
    2*(q(2)*q(4)-q(1)*q(3)) 2*(q(3)*q(4)+q(1)*q(2)) 1-2*q(2)^2-2*q(3)^2];
end