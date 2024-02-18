function [desired] = desired_trajectory(tspan,control_active)


    points = linspace(0,50,200);
        % if control_active == false 
        %     return
       
        x_desired = 0;
        y_desired = 5 ; %12*sin(x_desired(i)*0.3-0.1*tspan(i))
theta_desired= 0; %-(Theta) ;  +(30*pi/180);

desired = [x_desired, y_desired, theta_desired];

% figure
% plot(x_desired, y_desired)
% figure(2)
% plot(theta_desired*57)
end 