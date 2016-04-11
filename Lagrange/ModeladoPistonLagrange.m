clc 
clear all
close all


t0=0;
tf=20;
dt=0.01;

t=t0:dt:tf;

m1=1;
m2=1;
g=9.81;
l=1;
Tao=[1/dt;0];
Xp(:,1)=[0;0];
X(:,1)=[0;0];
b=0.5;

setpoint=[(pi/4)*ones(1,length(t));(1)*ones(1,length(t))];
E(:,1)=[0;0];
dE(:,1)=[0;0];
iE(:,1)=[0;0];

kp=500;
kd=80;
ki=0;

for n=1:length(t)-1
    
    M=[m1*(l^2)+m2*((l+X(2,n)).^2), 0
        0, m2];
    C=[m2*(l+X(2,n))*Xp(2,n), m2*Xp(1,n)*(l+X(2,n))
        -m2*Xp(1,n)*(l+X(2,n)), 0];
    
    G=[m1*g*l*cos(X(1,n))+m2*g*(l+X(2,n))*cos(X(1,n))
        m2*g*sin(X(1,n))];
    
    Xpp(:,n)=(kp*E(:,n)+kd*dE(:,n)+ki*iE(:,n))-((inv(M))*(-b*Xp(:,n)-C*Xp(:,n)-G));
    Xp(:,n+1)=[sum(Xpp(1,:))*dt;sum(Xpp(2,:))*dt];
    X(:,n+1)=[sum(Xp(1,:))*dt;sum(Xp(2,:))*dt];
    Tao(:,n+1)=[0;0];
    
    %Control%
    E(:,n+1)=[setpoint(1,n)-X(1,n+1);setpoint(2,n)-X(2,n+1)];
    dE(:,n+1)=[(E(1,n+1)-E(1,n))/dt;(E(2,n+1)-E(2,n))/dt];
    iE(:,n+1)=[sum(E(1,n))*dt;sum(E(2,n))*dt];
    
end

hold on
grid on
plot(t,X(1,:))
plot(t,X(2,:),'r')
plot(t,setpoint,'g')
hold off


figure,
hold on
grid on
for n=1:length(t)
    
    cla
    x0=0;
    y0=0;
    x1=l*cos(X(1,n));
    y1=l*sin(X(1,n));
    x2=(l+X(2,n))*cos(X(1,n));
    y2=(l+X(2,n))*sin(X(1,n));
    plot([x0,x1,x2],[y0,y1,y2]);
    plot([x0,x1,x2],[y0,y1,y2],'o');
    axis([-3 3 -3 3])
    pause(dt)
    
end
hold off

