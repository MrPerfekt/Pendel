echo off;
hold on;
xlim([0,3]);
ylim([0,3]);
zlim([0,3]);

P_L=[0,0,0];
phi_L=0*pi/180;
theta_L=90*pi/180;
d_L=[sin(theta_L)*cos(phi_L),sin(theta_L)*sin(phi_L),cos(theta_L)];
d_L=d_L/norm(d_L);


P_M=[3,0,0];
r_M=sqrt(2);
phi_M=160*pi/180;
theta_M=90*pi/180;
n_M=[sin(theta_M)*cos(phi_M),sin(theta_M)*sin(phi_M),cos(theta_M)];
%n_M=n_M/norm(n_M);


%n_M*X == n_M*P_mirror
%n_M*X == n_M*(P_M+r_M*n_M)
%n_M*(P_L+lambda*d_L) == n_M*(P_M+r_M*n_M)
%n_M*(lambda*d_L) == n_M*(P_M+r_M*n_M-P_L)
lambda = dot(n_M,(P_M+r_M*n_M-P_L))/dot(n_M,d_L);


spiegelPunkt=P_M+r_M*n_M;
spiegelHitPunkt=P_L+lambda*d_L;
cosAlpa=dot(n_M,-d_L);
d_S=n_M*cosAlpa*2+d_L;
d_S=d_S/norm(d_S);

disp(d_L);
disp(n_M);
disp(lambda);
disp(acos(cosAlpa)*180/pi);

dots=[P_L;P_M];
plot3(dots(:,1),dots(:,2),'*');
lines=[P_L;spiegelHitPunkt];
plot3(lines(:,1), lines(:,2), lines(:,3),'-b');
lines=[P_M;spiegelPunkt];
plot3(lines(:,1), lines(:,2), lines(:,3),'-r');
lines=[spiegelHitPunkt;spiegelPunkt];
plot3(lines(:,1), lines(:,2), lines(:,3),'-y');
lines=[spiegelHitPunkt;+spiegelHitPunkt+d_S];
plot3(lines(:,1), lines(:,2), lines(:,3),'-g');
lines=[spiegelHitPunkt;spiegelHitPunkt+n_M];
plot3(lines(:,1), lines(:,2), lines(:,3),'-m');


%point = P_M+r_M*n_M;
%normal = n_M;
%%# a plane is a*x+b*y+c*z+d=0
%%# [a,b,c] is the normal. Thus, we have to calculate
%%# d and we're set
%d = -point*normal'; %'# dot product for less typing
%%# create x,y
%[xx,yy]=ndgrid(1:10,1:10);
%%# calculate corresponding z
%z = (-normal(1)*xx - normal(2)*yy - d)/normal(3);
%%# plot the surface
%surf(xx,yy,z)

hold off;
