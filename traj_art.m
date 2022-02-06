function [q1_d,q2_d,qp1_d,qp2_d,qpp1_d,qpp2_d]=traj_art(q1_t0,q2_t0,q1_tf,q2_tf,t0,tf,t)
%Vitesses initiales et finales
qp1_t0 = 0; qp1_tf = 0;
qp2_t0 = 0; qp2_tf = 0;

Vecteur_init_fin_q1 = [q1_t0;qp1_t0;q1_tf;qp1_tf];
Vecteur_init_fin_q2 = [q2_t0;qp2_t0;q2_tf;qp2_tf];

M = [ 1 t0 t0^2 t0^3;...
    0 1 2*t0 3*t0^2;...
    1 tf tf^2 tf^3;...
    0 1 2*tf 3*tf^2];

Coeff_direct_q1 = M\Vecteur_init_fin_q1;
Coeff_direct_q2 = M\Vecteur_init_fin_q2;

q1_d = Coeff_direct_q1(1) + Coeff_direct_q1(2)*t + Coeff_direct_q1(3)*t.^2 + Coeff_direct_q1(4)*t.^3; 
q2_d = Coeff_direct_q2(1) + Coeff_direct_q2(2)*t + Coeff_direct_q2(3)*t.^2 + Coeff_direct_q2(4)*t.^3; 

q1_d=q1_d*pi/180;
q2_d=q2_d*pi/180;

qp1_d = Coeff_direct_q1(2) + 2*Coeff_direct_q1(3)*t + 3*Coeff_direct_q1(4)*t.^2;
qp2_d = Coeff_direct_q2(2) + 2*Coeff_direct_q2(3)*t + 3*Coeff_direct_q2(4)*t.^2;

qpp1_d = 2*Coeff_direct_q1(3) + 6*Coeff_direct_q1(4)*t;
qpp2_d = 2*Coeff_direct_q2(3) + 6*Coeff_direct_q2(4)*t; 

end