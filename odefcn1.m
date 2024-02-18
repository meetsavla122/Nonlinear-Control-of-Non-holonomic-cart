function dydt = odefcn1(t, y, G_com_left,G_com_right,m,I,F_left, F_right, sigma)
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