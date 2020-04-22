function [H,h]=costgen(P,S,Q,R,dim)

Qbar = kron(eye(dim.N),Q); 

H    = S'*Qbar*S+kron(eye(dim.N),R);   
h    = S'*Qbar*P;
end