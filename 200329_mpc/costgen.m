function [H,h,const]=costgen(P,S,Q,R,dim,x0)

Qbar=kron(eye(dim.N),Q); 

H=S'*Qbar*S+kron(eye(dim.N),R);   
h=S'*Qbar*P*x0;
const=x0'*P'*Qbar*P*x0;

end