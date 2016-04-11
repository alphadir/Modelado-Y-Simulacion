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
    F=1;
    b=0;
    k=1;
    m=1;
    xpp(l)=(F-b*xp(l)-k*x(l))/m;
    xp(l+1)=sum(xpp*dt);
    x(l+1)=sum(xp*dt);
end

%plot(t,x)

figure
hold on
for l=1:length(t)
    cla(1)
    
    area([-1 1],[-x(l)-1 -x(l)-1],-x(l))
    plot([-100 100],[0 0],'k')
    plot([0 0],[-100 100],'k')
    axis([-3 3 -3 3])
    
    pause(dt)
end
hold off