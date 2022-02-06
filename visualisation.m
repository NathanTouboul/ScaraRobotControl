%===========================================================
% Minh Tu Pham 20/08/2018
% visualisation dynamique plan robot SCARA 2ddl
%===========================================================

function X=visualisation(q1,q2,t);

global L1 L2

% q1: trajectoire de la variable articulaire bras 1
% q2: trajectoire de la variable articulaire bras 2
% t: temps des trajectoires q1 et q2 (meme dimension)

q = [q1 ; q2] ;
c1=cos(q(1,:));
s1=sin(q(1,:));
c12=cos(q(1,:)+q(2,:));
s12=sin(q(1,:)+q(2,:));
X1(1,:)=L1*c1;
X1(2,:)=L1*s1;
X2(1,:)=L1*c1+L2*c12;
X2(2,:)=L1*s1+L2*s12;

figure;

for i=1:length(q),
    
    if i>1
    delete(z);
    end
    
    plot(0,0,'ok','linewidth',6);
    axis equal
    grid on
    hold on;
    plot([0 X1(1,i)],[0 X1(2,i)],'k','linewidth',5);
    plot(X1(1,i),X1(2,i),'ok','linewidth',2);
    plot([X1(1,i) X2(1,i)],[X1(2,i) X2(2,i)],'k','linewidth',5);
    z=findobj('Color','k');
    
    if i==1
    plot(X2(1,i),X2(2,i),'b','linewidth',0.5);
    else
    plot([X2(1,i-1) X2(1,i)],[X2(2,i-1) X2(2,i)],'b');
    title('Espace de travail')
    xlabel('X')
    ylabel('Y')
    end
    
    pause(0.0001);
    axis([-(L1+L2) L1+L2 -(L1+L2) L1+L2]);
  
end

X=[X1;X2];