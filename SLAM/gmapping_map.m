a = [0 1 0.9 0
    0 1 1 1
    0 0 0 1
    0 1 1 1
    0 0 1 1];
b = position_G1;
c = position_G2;
b(end+1,end+1) = 0;
c(end+1,end+1) = 0;
figure(1)
colormap([1 1 1;0 0 0]),pcolor(b)
figure(2)
colormap([1 1 1;0 0 0]),pcolor(c)
axis image ij off