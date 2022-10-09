function [u] = CBF_Control_tth(X,u_ref,alpha1,alpha2,alpha3,s_star,v_star,tilde_vh)

A=[];
b=[];
state_size=size(X);
f=zeros(state_size(1),state_size(1));
%assign fx
f(1,2)=-1;
f(2,1)=alpha1;
f(2,2)=-alpha2;
for i=1:(state_size(1)/2-1)
    f(2*i+1,2*i)=1;
    f(2*i+1,2*i+2)=-1;
    f(2*i+2,2*i)=alpha3;
    f(2*i+2,2*i+1)=alpha1;
    f(2*i+2,2*i+2)=-alpha2;
end
%assign gx
g=zeros(1,state_size(1));
g(2)=1;

relax_coef=1;

for index=1:state_size(1)/2
    delta_b=zeros(1,state_size(1));
    tau=0.4;
    if index==1
        delta_b(1)=1;
        delta_b(2)=-tau;
    else
        delta_b(1)=-1*relax_coef;
        delta_b(2)=tau*relax_coef;
        delta_b(index*2-1)=1;
        delta_b(index*2)=-tau;
    end

    eta=X(index*2-1)+s_star-tau*(X(index*2)+v_star);
    if index==1
        A=zeros(1,state_size(1)/2);
        A(1)=-delta_b(2);
    else
        eta=eta-relax_coef*(X(1)+s_star-tau*(X(2)+v_star));
        temp=zeros(1,state_size(1)/2);
        temp(1)=-delta_b(2);
        temp(index)=-1;
        A=[A;temp];
    end
    K=5;
    b=[b;delta_b*f*X+K*eta];
end


H=eye(state_size(1)/2);
H(1,1)=1;
f_ = zeros(state_size(1)/2,1);
f_(1)=-2*u_ref;

options =  optimset('Display','off');
[u_slacks, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
u=u_slacks(1);

end





