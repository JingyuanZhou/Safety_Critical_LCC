classdef disturbance_observer
properties
    A
    B
    L
    C
    H
    q
    x
    d
    alpha
    alpha1
    alpha2
    alpha3
end

methods
    function item=disturbance_observer(state_size,alpha1,alpha2,alpha3)
        item.alpha1=alpha1;
        item.alpha2=alpha2;
        item.alpha3=alpha3;
        %assign A
        f=zeros(state_size,state_size);
        f(1,2)=-1;
        f(2,1)=alpha1;
        f(2,2)=-alpha2;
        for i=1:(state_size/2-1)
            f(2*i+1,2*i)=1;
            f(2*i+1,2*i+2)=-1;
            f(2*i+2,2*i)=alpha3;
            f(2*i+2,2*i+1)=alpha1;
            f(2*i+2,2*i+2)=-alpha2;
        end
        item.A=f;
        %assign B
        g=zeros(1,state_size);
        g(2)=1;
        item.B=g;
        %assign H
        h=zeros(1,state_size);
        h(1)=1;
        h(2)=alpha3;
        item.H=h;

        %assign C
        C1=zeros(1,state_size);
        C2=zeros(1,state_size);
        C3=zeros(1,state_size);
        C1(1)=1;
        C2(2)=1;
        C3(state_size)=1;
        item.C=[C1',C2',C3'];
        %item.C(state_size)=1;

        %assign x
        %item.x=zeros(state_size,1);
        item.x=[-15;15;-10;10;-5;5];%[1;1;1;1;1;1];
        
        %assign q
        item.q=[2,1,0.5];

        %assign L
        L=place(item.A',item.C,[-1.5,-2.5,-3.5,-4.5,-5.5,-6.5]);
        item.L=L';

        %assign d,alpha
        item.d=0;
        item.alpha=0;
    end

    function obs=predict(obs,y,u,step,update_d)
        x_dot=obs.A*obs.x+obs.B'*u+obs.L*(y-obs.C'*obs.x)+obs.H'*obs.d;
        obs.x=obs.x+x_dot*step;

        if update_d==1
            alpha_dot=-obs.q*obs.C'*(obs.A*obs.x+obs.B'*u+obs.H'*obs.d);
            obs.alpha=obs.alpha+step*alpha_dot;
            obs.d=obs.alpha+obs.q*y;
        end
    end

    function [obs,u]=DCBF(obs,X,u_ref,s_star,v_star)
        a=[];
        b=[];
        state_size=size(X);
        f=obs.A;
        g2=obs.H;
        disturbance=obs.d;
        %disturbance observer parameters
        omega=7;
        v=1;
        beta=10;
        alpha=15;
    
        for index=1:2%state_size(1)/2
            s_safe=3;%tau*(X(index*2)-X(index*2-2));
            delta_b=zeros(1,state_size(1));
            delta_b(index*2-1)=1;
            eta=[X(index*2-1)+s_star-s_safe];
            
            while delta_b(2)==0
                delta_b=delta_b*f;
                eta=[eta,delta_b*X];
            end
            a=[a;-delta_b(2)];
            
            if index==1
                K=[1.5,5];
            elseif index==2
                K=[10,8.5];
            elseif index==3
                K=[5,3,1.5];%ones(1,length(eta)+1);
            end
            
            other_part=omega^2/(2*v*beta)+beta*norm(delta_b*g2')^2/(4*alpha-2*K(end)-2*v);
            b=[b;delta_b*f*X+K*eta'+delta_b*g2'*obs.d-other_part];%(1:length(K)-1)
        end
        h=1;
        f_ = -2*u_ref;
        options =  optimset('Display','off');
        
        [u, ~, exitflag, ~] = quadprog(h, f_, a, b, [], [], [], [], [], options);
        if exitflag==-2
            u=b(1)/a(1);
            %[u, ~, exitflag, ~] = quadprog(H, f_, A(length(A)), b(length(b)), [], [], [], [], [], options);
        end
    end
end

end