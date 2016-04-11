clc
clear all
close all

t=0:0.01:10;

%u=sin(t);

hold on
grid on
for b=5:-0.02:-1
    %cla(1)
    m=1;    
    k=1;

    sys=tf([1],[m b k]);

    %y=impulse(sys,t);
    y=step(sys,t);
    %y=lsim(sys,u,t);
    
    pzmap(sys)
    %plot(t,y)
    axis([-5 5 -5 5])
    pause(0.05)
end
hold off