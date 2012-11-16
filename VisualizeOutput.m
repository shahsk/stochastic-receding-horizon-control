clear all
clc

load('Output')
dx = 0.05;
dy = 0.05;
dtheta = pi/20;

i0 = -1:dx:1;
i1 = -1:dy:1;
i2 = -pi:dtheta:pi;
prob = reshape(Output,41,41,41);
[p,q,r]=meshgrid(i0,i1,i2);

h = surf(p(:,:,21),q(:,:,21),prob(:,:,21));
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Function g(q)')
title('Plot of g(q) for angle theta = pi/2')