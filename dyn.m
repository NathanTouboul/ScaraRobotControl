function M = dyn(t2)
% Renvoie la matrice d'inertie du modèle dynamique direct
% à techniquement mettre suivant les rotations des moteurs selon l'énoncé
global Jm1 Jm2 L1 L2 n1 n2 m1 m2

M = zeros(2); 
M(1,1) = Jm1/n1+(5/12)*m1*L1^2+m2*(L1^2+(5/12)*L2^2+L1*L2*cos(t2));
M(1,2) = m2*(L1^2+(5/12)*L2^2+L1*L2*cos(t2));
M(2,1) = m2*(L1^2+(5/12)*L2^2+L1*L2*cos(t2));
M(2,2) = Jm2/n2 + m2*(L1^2+(5/12)*L2^2+L1*L2*cos(t2));

end

