clc
clear all
close all

m1=1;
m2=1;
k1=1;
k2=1;
k3=1;
b=0;
u=1;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0];
X(:,1)=[0;0];

U=[u;0];
M=[m1 0
    0 m2];
B=[b -b
    -b b];
K=[k1+k2 -k2
    -k2 k2+k3];

for i=1:length(t)-1

    Xpp(:,i)=(inv(M))*(U-(B*Xp(:,i))-(K*X(:,i)));
    Xp(:,i+1)=[sum(Xpp(1,:)*dt);sum(Xpp(2,:)*dt)];
    X(:,i+1)=[sum(Xp(1,:)*dt);sum(Xp(2,:)*dt)];
    
end

figure
hold on
grid on
plot(t,X(1,:))
plot(t,X(2,:),'k')
hold off

figure
hold on
for i=1:length(t)
    cla(1)
    plot([-100 100],[0 0],'k')
    plot([0 0],[-100 100],'k')
    area([X(1,i) X(1,i)+1],[1 1])
    area([X(2,i)+3 X(2,i)+4],[1 1])
    axis([-10 10 -10 10])
    pause(dt)
end
    
