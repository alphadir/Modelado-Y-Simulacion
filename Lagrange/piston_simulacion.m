clc 
clear all
close all


t0=0;
tf=30;
dt=0.01;

t=t0:dt:tf;

m1=1;
m2=1;
g=9.81;
l=1;
Tao=[0;0];
Xp(:,1)=[0;0];
X(:,1)=[0;0];
b=0;

for i=1:length(t)-1
    M=[m1*(l.^2)+m2*((l+X(2,i)).^2) 0;0 m2];
    C=[m2*(l+X(2,i))*Xp(2,i) m2*Xp(1,i)*(l+X(2,i));-m2*Xp(1,i)*(l+X(2,i)) 0];
    G=[m1*g*l*cos(X(1,i))+m2*g*(l+X(2,i))*cos(X(1,i));m2*g*sin(X(1,i))];
    
    Xpp(:,i)=(inv(M))*(Tao-b*Xp(:,i)-C*Xp(:,i)-G);
    Xp(:,i+1)=[sum(Xpp(1,:))*dt;sum(Xpp(2,:))*dt];
    X(:,i+1)=[sum(Xp(1,:))*dt;sum(Xp(2,:))*dt];
end

hold on
plot(t,X(1,:))
plot(t,X(2,:),'r')
hold off

%Animación

figure,
hold on
for i=1:length(t)
    
    cla
    x0=0;
    y0=0;
    x1=l*cos(X(1,i));
    y1=l*sin(X(1,i));
    x2=(l+X(2,i))*cos(X(1,i));
    y2=(l+X(2,i))*sin(X(1,i));
    plot([x0,x1,x2],[y0,y1,y2]);
    plot([x0,x1,x2],[y0,y1,y2],'o');
    axis([-3 3 -3 3])
    pause(dt)
end
hold off

