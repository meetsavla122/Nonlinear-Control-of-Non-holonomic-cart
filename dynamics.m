function [dydt,F_com_I] = dynamics(t, y,m,I, sigma,Q,R, constraints,tspan,error,dt,n,G_com_left,G_com_right,F_left,F_right,control_active )
theta = y(3);

% 1) evaluate goal trajectory y_g(t) and obtain goal position and velocity
% 2) evaluate error eq (7)
% 3) compute feedback (fb) forces K*error (K is 3x6 matrix obtained from LQR)
% 4) compute feed forward (ff) part eq (10)
% 5) project combined forces (ff and db) to inidividual grasp points (e.g.
% pseudoinverse of grasp matrix)
% 6) apply to system 



c = cos(theta);   s = sin(theta);
R_I =[c, -s, 0;
      s, c, 0;
      0, 0, 1];

LAMBDA = [-s c 0];

if constraints == true
    M= zeros(n,n);
    M(1:3,1:3) = [m 0 0; 0 m 0; 0 0 I];
    M(4,1:3) = LAMBDA ;
    M(1:3,4) = transpose(LAMBDA)
else
    M(1:3,1:3) = [m 0 0; 0 m 0; 0 0 I];
end

M_controllability = M(1:3,1:3); 
[rankss, A, B] = Controllability_check(M_controllability);

[K] = lqr(A,B,Q,R);

N = zeros(n,1);
N(1:3,1) =  sigma * [y(4); y(5); y(6)];

N_error_dynamics = N(1:3,1);


if control_active == true

    F_left = M_controllability*(error./(2*dt)) + N_error_dynamics - (K*[error; error./dt]);
    
    G_I_left  = R_I * G_com_left;
    F_com_I_left = (G_I_left * transpose(R_I) * F_left);
 
    % G_I_right = R_I * G_com_right; 
    % F_com_R_right = (G_I_right * transpose(R_I) * F_right);
    % 
    F_com_I = zeros(n,1);
    F_com_I(1:3,1) = F_com_I_left %+ F_com_R_right; 
else 
    G_I_left  = R_I * G_com_left;
    F_com_I_left = (G_I_left * transpose(R_I) * F_left);

    G_I_right = R_I * G_com_right; 
    F_com_R_right = (G_I_right * transpose(R_I) * F_right);

    F_com_I = zeros(n,1);
    F_com_I(1:3,1) = F_com_I_left + F_com_R_right ;
    
end 

RR = M\ (F_com_I-N);

if constraints == true
    dydt = RR(1:4);
else
      % dydt = zeros(n,1);
      % dydt(1:3) = y(4:6);
      % dydt(4:6) = RR(1:3);
    
      dydt = zeros(1,n);
      dydt = RR(1:3);
end 

end