function H = dyn2(t1,t2,tp1,tp2)

% Renvoie la matrice H du modèle dynamique direct : coriolis, centrifuges et
% gravités
global L1 L2 m1 m2

H = [ -m2*L1*L2*tp2*sin(t2)*(tp1+tp2)
      -(1/2)*m2*L1*L2*sin(t2)*(tp2^2-tp1^2)]; 
end

