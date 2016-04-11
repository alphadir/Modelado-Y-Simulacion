clc
clear all
close all

m1=1;
m2=1;
m3=1;
k1=1;
k2=1;
k3=1;
b1=1;
b2=1;
b3=1;
%fa=1;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0;0];
X(:,1)=[0;0;0];

%FA=[0,0,fa];
%%FA=zeros(3,length(t));
%%FA(3,1)=1/dt;
M=[m1 0 0
    0 m2 0
    0 0 m3];
B=[b1+b2 -b2 0
    -b2 b2+b3 -b3
    0 -b3 b3];
K=[k1+k2 -k2 0
    -k2 k2+k3 -k3
    0 -k3 k3];

for i=1:length(t)-1
    if i==1
        FA=[1/dt;0;0];
    else
        FA=[0;0;0];
    end
        Xpp(:,i)=(inv(M))*(FA-(B*Xp(:,i))-(K*X(:,i)));
        Xp(:,i+1)=[sum(Xpp(1,:)*dt);sum(Xpp(2,:)*dt);sum(Xpp(3,:)*dt)];
        X(:,i+1)=[sum(Xp(1,:)*dt);sum(Xp(2,:)*dt);sum(Xp(3,:)*dt)];
    
    
end

figure
hold on
grid on
plot(t,X(1,:))
plot(t,X(2,:),'k')
plot(t,X(3,:),'r')
hold off

figure
grid on
hold on
for i=1:length(t)
    cla(1)
    plot([-100 100],[0 0],'k')
    plot([0 0],[-100 100],'k')
    area([X(1,i) X(1,i)+1],[1 1])
    area([X(2,i)+2 X(2,i)+3],[1 1])
    area([X(3,i)+4 X(3,i)+5],[1 1])
    axis([-10 10 -10 10])
    pause(dt)
end