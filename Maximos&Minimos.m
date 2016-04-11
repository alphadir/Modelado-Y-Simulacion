clc
clear all
close all

x=-10:0.01:10;
% for l=1:length(x)
%     if x(l)>=0
%         fx(l)=(x(l).^4)+(4.*x(l).^3)-(2.*x(l).^2)-(12.*x(l));
%     else 
%         fx(l)=0;
%     end
% end
fx=(x.^4)+(4.*x.^3)-(2.*x.^2)-(12.*x);
dfx=Derivada(fx,0.01);
ddfx=Derivada(dfx,0.01);

hold on
grid on
plot(x,dfx,'r')
plot(x,ddfx, 'g')
plot([-100 100],[0 0], 'k')
plot([0 0],[-100 100], 'k')
plot(x,fx)
axis([-15 15 -30 30])
hold off