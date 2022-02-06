function [teta1,teta2]=mgi(x,y)
global L1 L2
cost2=(x.^2+y.^2-L1^2-L2^2)/(2*L1*L2);
sint2=sqrt(1-(cost2).^2);
teta2=atan2(sint2,cost2);
teta1=(atan2(y,x)-atan2(L2*sint2,L1+L2*cost2));
for i=1:length(teta1)
    if (teta1(i)<0)
        teta1(i)=teta1(i)+2*pi;
    end
end

end