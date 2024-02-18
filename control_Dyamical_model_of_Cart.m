clc
close all

m=40;  I = 5; 
dx_left = 0.1; dy_left = 0.02; dz_left = 0; 
d_left = [-dx_left;dy_left;dz_left]; 
G_T_com_left = [1 0 -d_left(2); 0 1 -d_left(1); 0 0 1];
G_com_left = transpose(G_T_com_left); 

dx_right = dx_left;  dy_right = -dy_left;  dz_right = dz_left;
d_right = [dx_right;-dy_right; dz_right];
G_T_com_right = [1 0 d_right(2); 0 1 -d_right(1); 0 0 1];
G_com_right = transpose(G_T_com_right);

sigma = 10*[1 0 0;
         0 2 0;
         0 0 2];
Theta = 40*pi/180;
tspan = 0:0.1:10 ;
y0 = [0 0 Theta 0 0 0 0 ];

[t,y] = ode45(@(t,y) odefcn1(t,y,G_com_left,G_com_right, m,I, sigma), tspan, y0);

lambda = zeros(1,length(t));
for i = 1:length(tspan)
    tmp = odefcn1(t,y(i,:),G_com_left,G_com_right, m,I,sigma);
    lambda(i) = tmp(7);
end

fprintf('xPosition of cart after %d seconds is %d \n', tspan(end), y(end,1))
fprintf('yPosition of cart after %d seconds is %d \n', tspan(end), y(end,2))

figure(1)
subplot(2, 3, 1) ,plot(t, y(:, 1)); title('xPosition in meters')
subplot(2, 3, 2) ,plot(t, y(:, 2)); title('yPosition in m')
subplot(2, 3, 3) ,plot(t, (y(:, 3)*180/pi)); title('theta in Degrees')
subplot(2, 3, 4) ,plot(t, y(:, 4)); title('xVelocity in m/s')
subplot(2, 3, 5) ,plot(t, y(:, 5)); title('yVelocity in m/s')
subplot(2, 3, 6) ,plot(t, (y(:, 6)*57.2958)); title('angular Velocity in deg/s')

xmin=-1; xmax=16; ymin=xmin; ymax=xmax;
xx=y(:,1);  yy=y(:,2);
len = length(xx);
for j = 1:len
    jj = j+1;
    if (xx(j) == xx(end))
        xx(j+1) = xx(j);
        yy(j+1) = yy(j);
    end  
    counter = j;
    path(xx(j),yy(j),xx(jj), yy(jj),xmin, xmax, ymin,ymax,j, len)
end
hold off
pause (3)
xx=y(:,1);  yy=y(:,2); theta = y(:,3);
for i = 1:length(yy)
    boxCart(xx(i), yy(i), theta(i),xmin, xmax, ymin,ymax)
    hold off 
end 


figure(4)
plot(t, lambda)
title('Reaction force in Y-direction(Constraint Force)')
xlabel('Time'); ylabel('Force in N')



function dydt = odefcn1(t, y, G_com_left,G_com_right,m,I,sigma)

[F_right, F_left ] = compute_control(y,t);

theta = y(3);

c = cos(theta);   s = sin(theta);
R_I =[c, -s, 0;
      s, c, 0;
      0, 0, 1];

G_I_left  = R_I * G_com_left;
F_com_I_left = (G_I_left * transpose(R_I) * F_left);

G_I_right = R_I * G_com_right; 
F_com_R_right = (G_I_right * transpose(R_I) * F_right);

F_com_I = zeros(4,1);
F_com_I(1:3,1) = F_com_I_left + F_com_R_right; 


LAMBDA = [-s c 0];
M = zeros(4,4);
M(1:3,1:3) = [m 0 0; 0 m 0; 0 0 I];
M(4,1:3) = LAMBDA ;
M(1:3,4) = transpose(LAMBDA);

N = zeros(4,1);
N(1:3,1) =  sigma * [y(4); y(5); y(6)];
RR = M\ (F_com_I-N);

  dydt = zeros(7,1);
  dydt(1:3) = y(4:6);
  dydt(4:7) = RR(1:4);
end

function [F1,F2] = compute_control(y,t)

yd = sin(t)

Fx_left= 25; 
Fy_left = 0; 
tau_left = 0;
F2 = [Fx_left; Fy_left; tau_left];

Fx_right = Fx_left; 
Fy_right = Fy_left; 
tau_right = tau_left;
F1 = [Fx_right; Fy_right; tau_right];

end

function boxCart(x, y, theta,xmin, xmax, ymin,ymax)
    s = 0.5;
    x_box_vertices = [0.5*s, -0.5*s, -0.5*s,  0.5*s, 0.5*s];
    y_box_vertices = [0.5*s,  0.5*s, -0.5*s, -0.5*s, 0.5*s];
    q=[x_box_vertices;y_box_vertices];
    rotation=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    box_vertices_rotated=rotation*q;
    x_actual = box_vertices_rotated(1,:) + [x x x x x];
    y_actual = box_vertices_rotated(2,:) + [y y y y y];
    figure(3)
    patch(x_actual,y_actual,'yellow');
    axis([ xmin xmax ymin ymax])
    axis square
    title('2D Animation of Cart')
    xlabel('X-axis (meters)'); ylabel('Y-axis (meters)') 
    pause(0.1)
   
end 

function path(x,y,x_next, y_next, xmin, xmax, ymin,ymax, j, len)
    figure(2)
    hold on 
    axis([xmin xmax ymin ymax])
    axis square
    if j == 1
        plot(x, y, 'or', "DisplayName", "Start")
    end 
    if j == len
        plot(x, y, 'pb', "DisplayName", "End") 
    end 
    plot([x x_next], [y y_next],'k')
    title('2D Animation Trajectory plot of Cart')
    xlabel('X-axis (meters)'); ylabel('Y-axis (meters)') 
    pause(0.1)

    %legend({'Start','End'},'Location','northwest','NumColumns',2)
    axis equal
end 


