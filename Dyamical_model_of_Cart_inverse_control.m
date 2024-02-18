clc
clear all
close all

ca = false;
constraints_active = ca;
control_active = true; 

Theta = 0*pi/180;

if constraints_active == false
    n=3;
    initial_parameters = [0 0 Theta];
else 
    n=4;
    initial_parameters = [0 0 Theta 0]
end 

m=50;  I = 20; 

vectorQ=[1,20,3,4,5,6];  vectorR=[1,2,3];
Q = diag(vectorQ); R=diag(vectorR);
dx_left = 0.1; dy_left = 0.05; dz_left = 0; 
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



if control_active==true
    sigma = 10*[10 0 0;
         0 20 0;
         0 0 20];
   
    dt = 0.1; tspan = 0:dt:15;
    len_tspan = length(tspan);
    
    y0=zeros(len_tspan,n);
    y = zeros(len_tspan,n);
    y0(1,:) = initial_parameters;
    
    % [desired] = desired_trajectory(tspan,control_active,y0(e,1),y0(e,2));
    % plot(desired(:,1), desired(:,2) )
    
    x_desired = zeros(length(tspan),1); y_desired = zeros(length(tspan),1); 
theta_desired = zeros(length(tspan),1); 

[desired] = desired_trajectory(tspan,control_active,y0(1),y0(2),points(e));

    for e=1:length(tspan)
        
        %desired_ode = [desired(1); y0(2);desired(3)];
        %[error] = error_calculation(desired_ode,[y0(e,1);y0(e,2);y0(e,3)]);
        e_ref = desired(:,1);  y_ref = desired(:,2);
        [error, error_arg]=find_distance(y0(e,1), y0(e,2), e_ref ,  )
        error_vec(:,e) = error;
        [ynew] = rungeKutta4(tspan(e),y0(e), @(t,x)dynamics(t, y,m,I, sigma,Q,R,ca,error_arg tspan,error,dt,n,G_com_left,G_com_right,F_left,F_right,control_active),dt);
        y(e,:) = ynew;
        if constraints_active == true
            y0(e+1,:) = [y(e,1) y(e,2) y(e,3) y(e,4)] ;
        else 
            y0(e+1,:) = [y(e,1) y(e,2) y(e,3)] ;
        end 
        counter=e
    end


    % lambda = zeros(1,length(t));
    % for i = 1:length(tspan)
    %     tmp = odefcn1(t,y(i,:),G_com_left,G_com_right, m,I,F_left,F_right,sigma);
    %     lambda(i) = tmp(7);
    % end
    
    fprintf('xPosition of cart after %d seconds is %d \n', tspan(end), y(end,1))
    fprintf('yPosition of cart after %d seconds is %d \n', tspan(end), y(end,2))
    
    figure(2)
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
    end
    hold off
    pause (3)
    xx=y(:,1);  yy=y(:,2); theta = y(:,3);
    for i = 1:length(yy)
        boxCart(xx(i), yy(i), theta(i),xmin, xmax, ymin,ymax)
    end 
    
    
    % figure
    % plot(t, lambda)
    % title('Reaction force in Y-direction(Constraint Force)')
    % xlabel('Time'); ylabel('Force in N')


else 
    sigma = 10*[1 0 0;
         0 2 0;
         0 0 2];
tspan = 0:0.1:10 ;
y0 = [0 0 Theta 0 0 0 0 ];

[t,y] = ode45(@(t,y) odefcn1(t,y,G_com_left,G_com_right, m,I,F_left,F_right, sigma), tspan, y0);

lambda = zeros(1,length(t));
for i = 1:length(tspan)
    tmp = odefcn1(t,y(i,:),G_com_left,G_com_right, m,I,F_left,F_right,sigma);
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


end 

