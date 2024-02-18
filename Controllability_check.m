function [rankss, A, B] = Controllability_check(M)
    A = [zeros(3), eye(3); zeros(3), zeros(3)];
    B = [zeros(3);1\M];
    
    C=(ctrb(A,B));
    rankss=rank(C);
end