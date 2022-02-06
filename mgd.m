function [x,y]=mgd(t1,t2)
global L1 L2
x= L2*cos(t1+t2)+L1*cos(t1);
y= L2*sin(t1+t2)+L1*sin(t1);
end