%==============================================================
% Barbier Claire - Touboul Nathan
% Projet Robotique : Etude et commande d'un robot Scara 2 axes
%==============================================================

close all
clear all

global L1 L2 q1min q1max q2min q2max mc

L1=0.5;
L2=0.4;
t=0:pi/180:2*pi;   %vecteur temps
nt=length(t);
mc = 0; 
S=1e2*diag([1,1])
P=1e2*diag([1,1])
qs_15=timeseries(0,0)
qps_15=timeseries(0,0)
Kp_15=110*diag([1,1])
Kd_15=2*diag([1,1])
q1min=0*pi/180;    %butée articulaire
q1max=270*pi/180;
q2min=-90*pi/180;
q2max=90*pi/180;

q11=linspace(q1min,q1max,nt/4);   %q1 et q2 varient de 0 à 2pi
q12=linspace(q1max,q1max,nt/4);
q13=linspace(q1max,q1min,nt/4);
q14=linspace(q1min,q1min,nt/4);
q15=linspace(q1min,q1max,nt/4);
q21=linspace(q2min,q2min,nt/4);
q22=linspace(q2min,q2max,nt/4);
q23=linspace(q2max,q2max,nt/4);
q24=linspace(q2max,q2min,nt/4);
q25=linspace(0,0,nt/4);

q1=[q11 q12 q13 q14 q15];
q2=[q21 q22 q23 q24 q25];

% X=visualisation(q1,q2,t);
% 
% X_worksp=X(3,:);
% Y_worksp=X(4,:);
% save XY_worskpace X_worksp Y_worksp 

%% Question 3 Modèle inverse
x=0;
y=0.8;
cost2=(x^2+y^2-L1^2-L2^2)/(2*L1*L2);
sint2=sqrt(1-(cost2)^2);
teta2=atan2(sint2,cost2);
teta1=atan2(y,x)-atan2(L2*sint2,L1+L2*cost2);

%% Question 4 trajectoire cercle de rayon 0.7
phi=linspace(45*pi/180,315*pi/180,nt/6);
x=0.7*cos(phi);
y=0.7*sin(phi);
[teta1,teta2]=mgi(x,y);
% figure
% hold on
% plot(phi,teta1)
% plot(phi,teta2)
% legend('q1','q2'); xlabel('Temps (s)'); ylabel('Angles (rad)')
%X=visualisation(teta1,teta2,t);

%% Question 5 trajectoire linéaire
t1=linspace(0,1,60);
x1=0.7*t1-0.6;
y1=0.6*t1;
[teta1,teta2]=mgi(x1,y1);
% figure
% hold on
% plot(t1,teta1)
% plot(t1,teta2)

%X=visualisation(teta1,teta2,t);

%% Question 6
J=[-L1*sin(teta1)-L2*cos(teta1+teta2) -L2*sin(teta1+teta2);...
    L1*cos(teta1)+L2*cos(teta1+teta2) L2*cos(teta1+teta2)];

%% Question 8
q1min2=-175*pi/180;    %butée articulaire
q1max2=175*pi/180;
q2min2=-175*pi/180;
q2max2=175*pi/180;

%Nouvel espace de travail
q11=linspace(q1min2,q1max2,nt/4);   
q12=linspace(q1max2,q1max2,nt/4);
q13=linspace(q1max2,q1min2,nt/4);
q14=linspace(q1min2,q1min2,nt/4);
q15=linspace(q1min2,q1max2,nt/4);
q21=linspace(q2min2,q2min2,nt/4);
q22=linspace(q2min2,q2max2,nt/4);
q23=linspace(q2max2,q2max2,nt/4);
q24=linspace(q2max2,q2min2,nt/4);
q25=linspace(0,0,nt/4);

q1=[q11 q12 q13 q14 q15];
q2=[q21 q22 q23 q24 q25];

%X=visualisation(q1,q2,t);

%Coordonnées opérationnelles des points A,B,C et D
[Xa,Ya]=mgd(1*pi/180,-1*pi/180);  
[Xb,Yb]=mgd(85*pi/180,-175*pi/180);
[Xc,Yc]=mgd(175*pi/180,-175*pi/180);
[Xd,Yd]=mgd(89*pi/180,-1*pi/180);

%Nouvelles coordonnées des points B' et C'
[teta1b,teta2b]=mgi(0.15,Ya);
[teta1c,teta2c]=mgi(Xd,0.15);
rad2deg([teta1b,teta2b]);
rad2deg([teta1c,teta2c]);

%Tracé de la trajectoire
t2=linspace(0,1,100);
xab=(0.15-Xa)*t2+Xa;
yab=Ya*ones(1,length(t2));
[q1ab,q2ab]=mgi(xab,yab);
xbc=(Xd-0.15)*t2+0.15;
ybc=(0.15-Ya)*t2+Ya;
[q1bc,q2bc]=mgi(xbc,ybc);
xcd=Xd*ones(1,length(t2));
ycd=(Yd-0.15)*t2+0.15;
[q1cd,q2cd]=mgi(xcd,ycd);

q1=[q1ab q1bc q1cd];
q2=[q2ab q2bc q2cd];

%X=visualisation(q1,q2,t);

%Vitesses articulaires sur la trajectoire
for i=1:length(q1ab)
    tetapab(:,i)=Jacobinv(q1ab(i),q2ab(i))*[1;0];
    tetapbc(:,i)=Jacobinv(q1bc(i),q2bc(i))*[1/sqrt(2);1/sqrt(2)];
    tetapcd(:,i)=Jacobinv(q1cd(i),q2cd(i))*[0;1];
end
figure
hold on
Z1=[tetapab(1,:) tetapbc(1,:) tetapcd(1,:)];
Z2=[tetapab(2,:) tetapbc(2,:) tetapcd(2,:)];
plot(1:size(Z1,2),Z1,'LineWidth',2)
plot(1:size(Z2,2),Z2,'LineWidth',2)
legend('theta point 1','theta point 2')
xlabel('Temps')
ylabel('Vitesses articulaires')

%% Question 9 : Trajectoire articulaire
% Interpolation cubique
% Timing trajectoires
t0 = 0; t1 = 1; t2 = 1.5; 
pas = 0.01;
% Vecteurs temps
t_trajet=t0:pas:t1;   
t_pause=t0:pas:t2-t1;
t_complet=t0:pas:(t1+t2)+2*pas;

%[xa,ya]=mgd(q1_init*pi/180,q2_init*pi/180)
%[xb,yb]=mgd(q1_fin*pi/180,q2_fin*pi/180)

% Positions initiale et finale
q1_t0 = 100; q1_tf = -20; 
q2_t0 = 50; q2_tf = 60; 

%Trajectoire aller
[q1_da,q2_da,qp1_da,qp2_da,qpp1_da,qpp2_da]=traj_art(q1_t0,q2_t0,q1_tf,q2_tf,t0,t1,t_trajet);

%attente pendant t2-t1
[q1_dp,q2_dp,qp1_dp,qp2_dp,qpp1_dp,qpp2_dp]=traj_art(q1_tf,q2_tf,q1_tf,q2_tf,t0,t2-t1,t_pause);

% Trajectoire inverse
[q1_dr,q2_dr,qp1_dr,qp2_dr,qpp1_dr,qpp2_dr]=traj_art(q1_tf,q2_tf,q1_t0,q2_t0,t0,t1,t_trajet);

q1_d=[q1_da q1_dp q1_dr];
q2_d=[q2_da q2_dp q2_dr];
%visualisation(q1_d,q2_d,t_complet);

qp1_d=[qp1_da qp1_dp qp1_dr];
qp2_d=[qp2_da qp2_dp qp2_dr];

qpp1_d=[qpp1_da qpp1_dp qpp1_dr];
qpp2_d=[qpp2_da qpp2_dp qpp2_dr];

% figure
% plot(t_complet,q1_d,t_complet,q2_d)   %vérification position continue
% grid on
% xlabel('Temps (s)'); ylabel('Position (rad)'); legend('q1','q2')
% figure
% plot(t_complet,qp1_d,t_complet,qp2_d)   %vitesses continues
% grid on
% xlabel('Temps (s)'); ylabel('Vitesse (deg/s)'); legend('v1','v2')
% figure
% plot(t_complet,qpp1_d,t_complet,qpp2_d)  %accélérations
% grid on
% xlabel('Temps (s)'); ylabel('Accélération (deg²/s)'); legend('a1','a2')
% 

%% Modèle dynamique

global Jm1 Jm2 n1 n2 m1 m2 L1 L2 
L1 = 0.5; L2 = 0.4; % problème sur les L - réécriture - surement à cause des codes externes
Jm1 = 3200*1e-3*1e-4; % en kgm2 
Jm2 = Jm1; 
n1 = 100; n2 = n1; 
mr = 1; %kg % masse réducteur 
m1 = 5 +mr ; m2 = 4+mr;  % en kg 
T1 = 2 ; % en Nm
T2 = 3 ; % en Nm
Tau = [T1;T2]; 

%% Commande - Question 12 - Correcteur proportionnel dérivé 
% 
% Trajectoire souhaitée 
t1=linspace(0,1,60);
x1=0.7*t1-0.6;
y1=0.6*t1;
%Réglages simulation
fin = 1;
dt=1/59;
%Calcul des coordonnées articulaires
[teta1,teta2]=mgi(x1,y1);
plot(t1,teta1,t1,teta2)
grid on
for i=1:length(teta1)
    tetap(:,i)=Jacobinv(teta1(i),teta2(i))*[0.7;0.6];
end
A=[teta1;teta2];
qs=timeseries(A',t1);
qps=timeseries(tetap',t1); 
Kp = 1e3*diag([1 1]);
Kd = 20*diag([1 1]); 

%% Question 14 Commande linéarisante

%Trajectoire et vitesses sont les mêmes que ci dessus seuls les gains
%changent
S=1e2*diag([1 1]);
P=1e2*diag([1 1]);

X=visualisation(out.commandelin(:,2)',out.commandelin(:,3)',t1);

%% Question 15 trajectoire en cercle

%Trajectoire et vitesse souhaitées
centre=[0.7, 0];
rayon=0.05;
z=linspace(0,2*pi,60);
x_cercle= centre(1,1)+rayon*cos(z);
y_cercle= centre(1,2)+rayon*sin(z);
xp_cercle= -rayon*sin(z);
yp_cercle= rayon*cos(z);
%Régalges simulation
fin_15 = 2*pi;
dt_15=2*pi/59;
%Calcul dans l'espace des coordonnées articulaires
[teta1_15,teta2_15]=mgi(x_cercle,y_cercle);
for i=1:length(teta1)
    tetap_15(:,i)=Jacobinv(teta1_15(i),teta2_15(i))*[xp_cercle(i);yp_cercle(i)];
end
A=[teta1_15;teta2_15];
%Paramètres simulation
qs_15=timeseries(A',z);
qps_15=timeseries(tetap_15',z);
Kp_15 = 110*diag([1 1]);
Kd_15 = 2*diag([1 1]);


%% Question 15 b - Trace des resultats

% Tracé de la trajectoire de sortie de commande
X_15=visualisation(commande_cercle(:,2)',commande_cercle(:,3)',z);



%% Question 16 ajout de la masse ajout�e

mc = 5; %en kg %Param�tre � changer pour faire varier la masse � transporter
% Trajectoire souhaitée 
t1=linspace(0,1,60);
x1=0.7*t1-0.6;
y1=0.6*t1;
%Réglages simulation
fin = 1;
dt=1/59;
%Calcul des coordonnées articulaires
[teta1,teta2]=mgi(x1,y1);
for i=1:length(teta1)
    tetap(:,i)=Jacobinv(teta1(i),teta2(i))*[0.7;0.6];
end
A=[teta1;teta2];
qs=timeseries(A',t1);
qps=timeseries(tetap',t1); 
Kp = 1e3*diag([1 1]);
Kd = 20*diag([1 1]); 

%% Question 16b Visualisation des r�sultats
X_16=visualisation(out.commandePD_mc(:,1)',out.commandePD_mc(:,2)',t1);
X_16=visualisation(out.commandelin_mc(:,1)',out.commandelin_mc(:,2)',t1);

%% Question 17
plot(t1,out.perturb_PD(:,2)',t1,out.perturb_PD(:,3)') %trajectoire rectiligne avec perturbation
plot(t1,out.commandePD(:,2),t1,out.commandePD(:,3)) %trajectoire rectiligne sans perturbation (r�sultat qu12)
grid on
hold on
figure
hold on
grid on
plot(t1,out.perturb_lin(:,2)',t1,out.perturb_lin(:,3)') %trajectoire rectiligne avec perturbation
plot(t1,out.commandelin(:,2)',t1,out.commandelin(:,3)') %trajectoire rectiligne sans perturbation (r�sultat qu14)


