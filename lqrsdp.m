function [K] = lqrsdp(alpha1,alpha2,alpha3,state_size)
%assign A,B1,B,Q,R
gamma_s=linspace(0.8,0.05,state_size/2);
gamma_v=linspace(1,0.1,state_size/2);
gamma_u=1;

A=zeros(state_size,state_size);
A(1,2)=-1;
A(2,1)=alpha1;
A(2,2)=-alpha2;
for i=1:(state_size/2-1)
    A(2*i+1,2*i)=1;
    A(2*i+1,2*i+2)=-1;
    A(2*i+2,2*i)=alpha3;
    A(2*i+2,2*i+1)=alpha1;
    A(2*i+2,2*i+2)=-alpha2;
end
Q=zeros(state_size);
for i=1:state_size/2
    Q(2*i-1,2*i-1) = gamma_s(i);
    Q(2*i,2*i) = gamma_v(i);
end
B1 = zeros(state_size,state_size/2);
B1(1,1)=1;
B1(2,1)=alpha3;
% for i=1:state_size/2
%     B1(2*i,i) = 1;
% end

B = zeros(state_size,1);
B(2)=1;

R = gamma_u*eye(1,1);

epsilon   = 1e-5;

%% Call Yalmip

n = size(A,1);  % number of states
m = size(B,2);  % number of inputs


% variables
X = sdpvar(n);
Z = sdpvar(m,n);
Y = sdpvar(m);

% constraints
Constraints = [A*X - B*Z + (A*X - B*Z)' + B1*B1' <=0,
               X - epsilon*eye(n)>=0,
               [Y Z;Z' X] >=0];
           
obj   = trace(Q*X) + trace(R*Y);
%opts  = sdpsettings('solver','sedumi');   % call SDP solver: sedumi
opts  = sdpsettings('solver','mosek');     % call SDP solver: mosek, this is often better

sol     = optimize(Constraints,obj,opts);

Xd = value(X);
Zd = value(Z);

K = Zd*Xd^(-1);

end
