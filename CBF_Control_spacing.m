function [u] = CBF_Control_spacing(X,u_ref,alpha1,alpha2,alpha3,s_star,v_star,tilde_vh)%,v_star,tilde_vh
%spacing
A=[];
b=[];
state_size=size(X);
f=zeros(state_size(1),state_size(1));
tau=4;
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

for index=1:state_size(1)/2
    s_safe=3;%tau*(X(index*2)-X(index*2-2));
    delta_b=zeros(1,state_size(1));
    delta_b(index*2-1)=1;
%     delta_b(index*2)=-tau;
%     delta_b(index*2-2)=tau;
    eta=[X(index*2-1)+s_star-s_safe];
    
    while delta_b(2)==0
        delta_b=delta_b*f;
        eta=[eta,delta_b*X];
    end
    if index==1
        A=zeros(1,state_size(1)/2);
        A(1)=-delta_b(2);
    else
        temp=zeros(1,state_size(1)/2);
        temp(1)=-delta_b(2);
        temp(index)=-1;
        A=[A;temp];
    end
    
    
    %poles=[1000,5,4.9,4.8];
    %K=fliplr(poly(poles));
    if index==1
        K=[1.5,5];
    elseif index==2
        K=[10,8.5];
    elseif index==3
        K=[5,3,1.5];%ones(1,length(eta)+1);
    end
    b=[b;delta_b*f*X+K*eta'];%(1:length(K)-1)
end

%for disturbance from head vehicle
% if nargin==8
%     tau=3.5;
%     h=X(1)+s_star-tau*X(2)+tau*tilde_vh;%-tilde_vh
%     A=[A;tau];%tau;
%     coef=12;
%     b=[b;-X(2)-tau*(alpha1*X(1)-alpha2*X(2))+coef*h];
% end

H=eye(state_size(1)/2);
H(1,1)=1;
f_ = zeros(state_size(1)/2,1);
f_(1)=-2*u_ref;
options =  optimset('Display','off');

[u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
u=u(1);

end





