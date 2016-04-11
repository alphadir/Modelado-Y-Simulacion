clc
clear all
close all

m1=1;
m2=1;
k1=1;
k2=1;
b1=1;
b2=3;
b3=3;
fa=1;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0];
X(:,1)=[0;0];

FA=[0;fa];
M=[m1 0
    -m2 m2];
B=[b1+b2 -b3
    0 b3];
K=[k1 0
    -k2 k2];

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

%figure
% hold on
% for i=1:length(t)
%     cla(1)
%     plot([-100 100],[0 0],'k')
%     plot([0 0],[-100 100],'k')
%     area([X(1,i) X(1,i)+1],[1 1])
%     area([X(2,i)+3 X(2,i)+4],[1 1])
%     axis([-10 10 -10 10])
%     pause(dt)
% end