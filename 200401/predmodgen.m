function [P,S,W]=predmodgen(LTI,dim)

%Prediction matrices generation
%This function computes the prediction matrices to be used in the
%optimization problem

%Prediction matrix from initial state
P=zeros(dim.ny*(dim.N),dim.nx);
for k=0:dim.N-1
    P(k*dim.ny+1:(k+1)*dim.ny,:)=LTI.C*LTI.A^k;
end

%Prediction matrix from input
S=zeros(dim.ny*(dim.N),dim.nu*(dim.N));
for k=1:dim.N-1
    for i=0:k-1
        S(k*dim.ny+1:(k+1)*dim.ny,i*dim.nu+1:(i+1)*dim.nu)=LTI.C*LTI.A^(k-1-i)*LTI.B;
    end
end
