function H = dyn_mc_2(t1,t2,tp1,tp2)

% Renvoie la matrice H du modèle dynamique direct : coriolis, centrifuges et
% gravités
global L1 L2 m1 m2 mc

H = [ -L1*L2*tp2*sin(t2)*(m2+2*mc)*(tp1+tp2)
      -L1*L2*sin(t2)*(0.5*m2+mc)*(tp2^2-tp1^2)]; 
end
