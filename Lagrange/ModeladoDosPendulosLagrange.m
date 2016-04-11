clc 
clear all
close all


t0=0;
tf=20;
dt=0.01;

t=t0:dt:tf;

m1=1;
m2=1;
g=9.81;
l1=1;
l2=1;
Tao(:,1)=[0;1/dt];
Tp(:,1)=[0;0];
T(:,1)=[0;0];
b=0.5;

setpoint=[(pi/2)*ones(1,length(t));(pi/4)*ones(1,length(t))];
E(:,1)=[0;0];
dE(:,1)=[0;0];
iE(:,1)=[0;0];

kp=50;
kd=20;
ki=0;

for n=1:length(t)-1
    
    M=[m1*(l1.^2)+m2*(l2.^2)+m2*(l2.^2)+2*m2*l1*l2*cos(T(2,n))              m2*(l2.^2)+m2*l1*l2*cos(T(2,n));
        m2*l2.^2+m2*l1*l2*cos(T(2,n))                                       m2*l2.^2];
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  
    C=[-2*m2*l1*l2*Tp(2,n)*sin(T(2,n))                                      -m2*l1*l2*Tp(2,n)*sin(T(2,n));
        -m2*l1*l2*Tp(2,n)*sin(T(2,n))+m2*l1*l2*(Tp(2,n)+Tp(1,n))*sin(T(2,n)) 0];
    
    G=[m1*g*l1*sin(T(1,n))+m2*g*l1*sin(T(1,n))+m2*g*l2*sin(T(1,n)+T(2,n)); 
                                                                            m2*g*l2*sin(T(1,n)+T(2,n))];
    
%    Tpp(:,n)=inv(M)*(Tao(:,n)-b*Tp(:,n)-C*Tp(:,n)-G); %Para la dirac con el rosamiento del aire
    Tpp(:,n)=(kp*E(:,n)+kd*dE(:,n)+ki*iE(:,n))-((inv(M))*(-b*Tp(:,n)-C*Tp(:,n)-G));
    Tp(:,n+1)=[sum(Tpp(1,:))*dt;sum(Tpp(2,:))*dt];
    T(:,n+1)=[sum(Tp(1,:))*dt;sum(Tp(2,:))*dt];
%    Tao(:,n+1)=[0;0];   %Para la dirac
    
    %Control%
    E(:,n+1)=[setpoint(1,n)-T(1,n+1);setpoint(2,n)-T(2,n+1)];
    dE(:,n+1)=[(E(1,n+1)-E(1,n))/dt;(E(2,n+1)-E(2,n))/dt];
    iE(:,n+1)=[sum(E(1,n))*dt;sum(E(2,n))*dt];
        
end

hold on
grid on
plot(t,T(1,:))
plot(t,T(2,:),'r')
plot(t,setpoint,'g')
hold off


figure,
hold on
grid on
for n=1:length(t)
    
    cla
    plot([0 l1*sin(T(1,n))],[0 -l1*cos(T(1,n))]);
    plot([l1*sin(T(1,n))],[-l1*cos(T(1,n))],'o')
    plot([l1*sin(T(1,n)) l2*sin(T(2,n)+T(1,n))+l1*sin(T(1,n))],[-l1*cos(T(1,n)) -l2*cos(T(2,n)+T(1,n))-l1*cos(T(1,n))]);
    plot([l1*sin(T(1,n))+l2*sin(T(2,n)+T(1,n))],[-l1*cos(T(1,n))-l2*cos(T(2,n)+T(1,n))],'o')
    axis([-3 3 -3 3])
    pause(dt)
    
end
hold off