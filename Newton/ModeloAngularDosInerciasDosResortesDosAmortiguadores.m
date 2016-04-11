clc
clear all
close all

j1=1;
j2=1;
k1=1;
k2=1;
b1=1;
b2=1;
fa=1;

t0=0;
dt=0.01;
tf=100;

t=t0:dt:tf;

Xp(:,1)=[0;0];
X(:,1)=[0;0];

FA=[fa;0];
J=[j1 0
    0 j2];
B=[b1 0
    0 b2];
K=[k2 -k2
    -k2 k1+k2];

for i=1:length(t)-1

    Xpp(:,i)=(inv(J))*(FA-(B*Xp(:,i))-(K*X(:,i)));
    Xp(:,i+1)=[sum(Xpp(1,:)*dt);sum(Xpp(2,:)*dt)];
    X(:,i+1)=[sum(Xp(1,:)*dt);sum(Xp(2,:)*dt)];
    
end

figure
hold on
grid on
plot(t,X(1,:),'r--')
plot(t,X(2,:),'k*')
hold off

% figure
% hold on
% for i=1:length(t)
%     cla(1)
%     plot([-100 100],[0 0],'k')
%     plot([0 0],[-100 100],'k')
%     area([-X(1,i) -X(1,i)+1],[1 1])
%     area([4 5],[X(2,i)-4 X(2,i)-4],X(2,i)-2)
%     axis([-10 10 -10 10])
%     pause(dt)
% end