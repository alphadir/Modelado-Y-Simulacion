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
Tao=[0;0];
Xp(:,1)=[0;0];
X(:,1)=[0;0];
b=0.5;

setpoint=[(1)*ones(1,length(t));(pi/2)*ones(1,length(t))];
E(:,1)=[0;0];
dE(:,1)=[0;0];
iE(:,1)=[0;0];

kp=500;
kd=20;
ki=0;

for n=1:length(t)-1
    
    M=[m1+m2, -m2*l.*sin(X(2,n))
        -m2.*(l.*sin(X(2,n))), m2*(l^2)];
    C=[0, -m2.*Xp(2,n).*cos(X(2,n))
        0, 0];
    
    G=[0
        m2*g*l.*cos(X(2,n))];
    
    Xpp(:,n)=(kp*E(:,n)+kd*dE(:,n)+ki*iE(:,n))-((inv(M))*(-b*Xp(:,n)-C*Xp(:,n)-G));
%     Xpp(:,n)=(inv(M))*(Tao-b*Xp(:,n)-C*Xp(:,n)-G);
    Xp(:,n+1)=[sum(Xpp(1,:))*dt;sum(Xpp(2,:))*dt];
    X(:,n+1)=[sum(Xp(1,:))*dt;sum(Xp(2,:))*dt];
%     Tao(:,n+1)=[0;0];
    
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
    x1=X(1,n);
    y1=0;
    x2=X(1,n)+(l*cos(X(2,n)));
    y2=l*sin(X(1,n));
    plot([x0,x1],[y0,y1]);
    plot([x0,x1],[y0,y1],'o');
    plot([x1,x2],[y1,y2]);
    plot([x1,x2],[y1,y2],'o');    
    axis([-3 3 -3 3])
    pause(dt)
    
end
hold off
