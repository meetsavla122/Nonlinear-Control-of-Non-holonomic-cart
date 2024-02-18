clc
clear all
close all

constraints_active = false;
ca = false;
control_active = true; 
if constraints_active == false 
    n=3;
else 
    n=4;
end 
m=50;  I = 10; 
vectorQ=[10,2,3,4,5,6]; vectorR=[1,2,3];
Q = diag(vectorQ); R=diag(vectorR);
dx_left = 0.1; dy_left = 0.0; dz_left = 0; 
d_left = [-dx_left;dy_left;dz_left]; 
G_T_com_left = [1 0 -d_left(2); 0 1 -d_left(1); 0 0 1];
G_com_left = transpose(G_T_com_left); 

dx_right = dx_left;  dy_right = -dy_left;  dz_right = dz_left;
d_right = [dx_right;-dy_right; dz_right];
G_T_com_right = [1 0 d_right(2); 0 1 -d_right(1); 0 0 1];
G_com_right = transpose(G_T_com_right);

Fx_left= 25; Fy_left = 0; Fz_left = 0;  tau_left = 0;
F_left = [Fx_left; Fy_left; tau_left];

Fx_right = Fx_left; Fy_right = Fy_left; Fz_right = Fz_left; tau_right = tau_left;
F_right = [Fx_right; Fy_right; tau_right];


    sigma = 10*[1 0 0;
         0 2 0;
         0 0 2];
    Theta = 0*pi/180;
    dt = 0.1; tspan = 0:dt:5;
    len_tspan = length(tspan);
    
    y0=zeros(len_tspan,3);
    y = zeros(len_tspan,3);
    y0(1,:) = [0 0 Theta];
    
    [desired] = desired_trajectory(tspan,control_active);
    for e=1:len_tspan
        desired_ode = [desired(1,e); y0(e,2);desired(3,e)];
        [error] = error_calculation(desired_ode,[y0(e,1);y0(e,2);y0(e,3)]);
        [ynew] = rungeKutta4(tspan(e),y0(e), @(t,x)dynamics(t, y,m,I, sigma,Q,R, false,tspan,error,dt,n,G_com_left,G_com_right,F_left,F_right,control_active),dt);
        y(e,:) = [ynew(1) ynew(2) ynew(3)];
        y0(e+1,:) = [y(e,1) y(e,2) y(e,3)] ;
        counter=e;
    end


    % lambda = zeros(1,length(t));
    % for i = 1:length(tspan)
    %     tmp = odefcn1(t,y(i,:),G_com_left,G_com_right, m,I,F_left,F_right,sigma);
    %     lambda(i) = tmp(7);
    % end
    
    fprintf('xPosition of cart after %d seconds is %d \n', tspan(end), y(end,1))
    fprintf('yPosition of cart after %d seconds is %d \n', tspan(end), y(end,2))
    
    figure(1)
    subplot(2, 3, 1) ,plot(tspan, y(:,1)); title('xPosition in meters')
    subplot(2, 3, 2) ,plot(tspan, y(:,2)); title('yPosition in m')
    subplot(2, 3, 3) ,plot(tspan, (y(:,3)*180/pi)); title('theta in Degrees')
    % subplot(2, 3, 4) ,plot(tspan, y(:,4)); title('xVelocity in m/s')
    % subplot(2, 3, 5) ,plot(tspan, y(:,5)); title('yVelocity in m/s')
    % subplot(2, 3, 6) ,plot(tspan, (y(:,6)*57.2958)); title('angular Velocity in deg/s')
    
    xmin=-2; xmax=16; ymin=xmin; ymax=xmax;
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
        axis([xmin xmax ymin ymax])
        if j == len 
            plot(desired(1,:), desired(2,:))
        end 
    end
    hold off
    pause (3)
    xx=y(:,1);  yy=y(:,2); theta = y(:,3);
    for i = 1:length(yy)
        boxCart(xx(i), yy(i), theta(i),xmin, xmax, ymin,ymax)
    end 
    
    
    % figure(4)
    % plot(t, lambda)
    % title('Reaction force in Y-direction(Constraint Force)')
    % xlabel('Time'); ylabel('Force in N')

