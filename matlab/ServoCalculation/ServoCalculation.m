echo on;

P_L=[0,0,0];
phi_L=0;
theta_L=0;
d_L=1*[sin(theta_L)*cos(phi_L),sin(theta_L)*sin(phi_L),cos(theta_L)];


P_M=[15,1,0];
r_M=sqrt(2);
phi_M=135*pi/180;
theta_M=45*pi/180;
n_M=[sin(theta_M)*cos(phi_M),sin(theta_M)*sin(phi_M),cos(theta_M)];

pts=[P_L;P_M];
plot3(pts(:,1), pts(:,2), pts(:,3));

%n_M*X == n_M*P_mirror
%n_M*X == n_M*(P_M+r_M*n_M)
%n_M*(P_L-lambda*d_L) == n_M*(P_M+r_M*n_M)
%n_M*(lambda*d_L) == n_M*(P_M+r_M*n_M-P_L)
lambda = dot(n_M,(P_M+r_M*n_M-P_L))/dot(n_M,d_L);


point = P_M+r_M*n_M;
normal = n_M;
%# a plane is a*x+b*y+c*z+d=0
%# [a,b,c] is the normal. Thus, we have to calculate
%# d and we're set
d = -point*normal'; %'# dot product for less typing
%# create x,y
[xx,yy]=ndgrid(1:10,1:10);
%# calculate corresponding z
z = (-normal(1)*xx - normal(2)*yy - d)/normal(3);
%# plot the surface
surf(xx,yy,z)


disp(n_M);
disp(lambda);