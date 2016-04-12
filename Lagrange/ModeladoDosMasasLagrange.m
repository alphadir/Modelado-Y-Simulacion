clc
clear all
close all

g=9.810;
m1=1;
m2=1;
f1=1;
f2=5;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0];
X(:,1)=[0;0];

FA=[f1;f2];
M=[m1+m2 0
    0 m2];
C=[0 0
    0 0];
G=[0;m2*g];

for i=1:length(t)-1

    Xpp(:,i)=(inv(M))*(FA-(C*Xp(:,i))-G);
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
    area([X(1,i) X(1,i)+1],[X(2,i)+2 X(2,i)+2],X(2,i))
    axis([-10 10 -10 10])
    pause(dt)
end