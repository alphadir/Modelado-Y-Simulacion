clc
clear all
close all

t0=0;
tf=10;
dt=0.01;

t=t0:dt:tf;
x(1)=0;
xp(1)=0;

for l=1:length(t)-1
    %F=1*ones(1,length(t));
    %%F=zeros(1,length(t));
    %%F(1)=1/dt;
    F=t;
    b=0;
    k=0;
    m=5;
    xpp(l)=(F(l)-b*xp(l)-k*x(l))/m;
    xp(l+1)=sum(xpp*dt);
    x(l+1)=sum(xp*dt);
end

%plot(t,x)

figure
hold on
for l=1:length(t)
    cla(1)
    
    area([x(l) x(l)+1],[1 1],-1)
    plot([-100 100],[0 0],'k')
    plot([0 0],[-100 100],'k')
    axis([-10 10 -10 10])
    title(num2str(t(l)))
    pause(dt)
end
hold off

figure
hold on
plot(t,x)
plot(t,xp,'r')
plot(t(1:length(t)-1),xpp,'g*')
plot(t,F,'k')
hold off