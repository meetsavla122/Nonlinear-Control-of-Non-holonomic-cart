function [ynew] = rungeKutta4(t, y0,dynamics, dt)
    f1 = dynamics(t,y0); 
    f2 = dynamics(t+(dt/2),y0+(dt*f1/2)); 
    f3 = dynamics(t+(dt/2),y0+(dt*f2/2));  
    f4 = dynamics(t+dt,    y0+(dt*f3)); 

    ynew = y0 + (dt/6)*(f1 + (2*f2) + (2*f3) + f4);
end 
