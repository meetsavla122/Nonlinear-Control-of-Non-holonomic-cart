function [desired] = desired_trajectory(tspan,control_active)


    points = linspace(0,50,200);
        % if control_active == false 
        %     return
       
        x_desired = 0:0.1:5;
        y_desired = zeros(1,length(x_desired)); %12*sin(x_desired(i)*0.3-0.1*tspan(i))
theta_desired= zeros(1,length(x_desired)) + atan2(y_desired,x_desired); %-(Theta) ;  +(30*pi/180);

desired = [x_desired; y_desired; theta_desired];

% figure
% plot(x_desired, y_desired)
% figure(2)
% plot(theta_desired*57)
end 