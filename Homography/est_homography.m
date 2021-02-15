function [ H ] = est_homography(video_pts, logo_pts)
% est_homography estimates the homography to transform each of the
% video_pts into the logo_pts
% Template is from UPenn
% Algorithms implement by Mingxi Jia
% Inputs:
%     video_pts: a 4x2 matrix of corner points in the video
%     logo_pts: a 4x2 matrix of logo points that correspond to video_pts
% Outputs:
%     H: a 3x3 homography matrix such that logo_pts ~ H*video_pts
% Written for the University of Pennsylvania's Robotics:Perception course

% YOUR CODE HERE
x1=video_pts(1,1);y1=video_pts(1,2);
x1s=logo_pts(1,1);y1s=logo_pts(1,2);

x2=video_pts(2,1);y2=video_pts(2,2);
x2s=logo_pts(2,1);y2s=logo_pts(2,2);

x3=video_pts(3,1);y3=video_pts(3,2);
x3s=logo_pts(3,1);y3s=logo_pts(3,2);

x4=video_pts(4,1);y4=video_pts(4,2);
x4s=logo_pts(4,1);y4s=logo_pts(4,2);

A=[
    [x1,y1,1,0,0,0,-x1*x1s,-y1*x1s,-x1s]
    [0,0,0,x1,y1,1,-x1*y1s,-y1*y1s,-y1s]
    [x2,y2,1,0,0,0,-x2*x2s,-y2*x2s,-x2s]
    [0,0,0,x2,y2,1,-x2*y2s,-y2*y2s,-y2s]
    [x3,y3,1,0,0,0,-x3*x3s,-y3*x3s,-x3s]
    [0,0,0,x3,y3,1,-x3*y3s,-y3*y3s,-y3s]
    [x4,y4,1,0,0,0,-x4*x4s,-y4*x4s,-x4s]
    [0,0,0,x4,y4,1,-x4*y4s,-y4*y4s,-y4s]
    ];

[U,S,V] = svd(A);
H=(reshape(V(:,9),3,3))';

end

