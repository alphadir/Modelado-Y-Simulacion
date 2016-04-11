function dxt=Derivada(xt,dt) 
%Esta es mi función derivada.

for i=2:length(xt) 
    dxt(i-1)=(xt(i)-xt(i-1))./dt; 
end 

dxt(length(xt))=dxt(length(xt)-1);