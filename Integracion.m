function intxt=Integracion(xt,dt)

for i=1:length(xt)
    intxt(i)=sum(xt(1:i).*dt);
end