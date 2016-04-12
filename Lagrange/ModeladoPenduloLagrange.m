clc
clear all
close all

dt=0.01;
t=0:dt:20;

m=1;
b=0.5;
setpoint=(pi)*ones(1,length(t));
L=1;
g=9.810;
Tao=1/dt;

kp=50;
kd=10;
ki=10;

T(1)=0;
Tp(1)=0;
E(1)=0;
dE(1)=0;
iE(1)=0;

for n=1:length(t)-1
    
%   Tpp(n)=(-b*Tp(n)+Tao-(m*g*L*sin(T(n))))/(m*(L.^2)); %Para la dirac con el rosamiento del aire
   Tpp(n)=(kp*E(n)+kd*dE(n)+ki*iE(n)-(-b*Tp(n))-(m*g*L*sin(T(n))))/(m*(L.^2));
   Tp(n+1)=sum(Tpp.*dt);
   T(n+1)=sum(Tp.*dt);
%   Tao=0;  %Para la dirac

   %Control%
   E(n+1)=setpoint(n+1)-T(n+1);
   dE(n+1)=(E(n+1)-E(n))/dt;
   iE(n+1)=sum(E.*dt);
   
end

hold on
grid on
plot(t,T)
plot(t,setpoint,'r')
axis([0 20 -5 5])
hold off

figure
hold on
grid on
for n=1:length(t)
    
    cla(1)
    plot([0 L*sin(T(n))],[0 -L*cos(T(n))])
    plot(L*sin(T(n)),-L*cos(T(n)),'o')
    axis([-1 1 -1 1])
    pause(dt)  
    
end
hold off