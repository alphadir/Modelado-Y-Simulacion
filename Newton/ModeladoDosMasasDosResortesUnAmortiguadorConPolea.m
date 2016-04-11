clc
clear all
close all

m1=1;
m2=1;
k1=1;
k2=0;
b1=0;
fa=1;
g=9.81;
W=m2*g;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0];
X(:,1)=[0;0];

FA=[fa;-W];
M=[m1 0
    0 m2];
B=[0 0
    0 b1];
K=[k1 -k1
    -k1 k1+k2];

for i=1:length(t)-1

    Xpp(:,i)=(inv(M))*(FA-(B*Xp(:,i))-(K*X(:,i)));
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
    area([-X(1,i) -X(1,i)+1],[1 1])
    area([4 5],[X(2,i)-4 X(2,i)-4],X(2,i)-2)
    axis([-10 10 -10 10])
    pause(dt)
end