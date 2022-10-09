classdef observer_CBF_Control
properties
    %SO/DO
    A
    B
    L
    C
    H
    q
    z
    %estimated state/disturbance
    x
    d
    %OVM
    alpha1
    alpha2
    alpha3
    %SOB-CBF
    N
    theta_head
    theta_following_1
    theta_following_2
    error_ls
    error_temp
    theta_ls
end

methods
    function item=observer_CBF_Control(state_size,alpha1,alpha2,alpha3)
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
        L=place(item.A',item.C,[-1.5,-2.5,-3.5,-4.5,-5.5,-6.5,-7,5,-8.5,-9.5,-10.5]);
        item.L=L';

        %assign d,alpha
        item.d=0;
        item.z=0;

        %assign N
        item.N=5;

        %assign theta
        item.theta_head=zeros(state_size,item.N);
        item.theta_following_1=zeros(state_size,item.N);
        item.theta_following_2=zeros(state_size,item.N);
        
        item.error_ls=[];
        item.error_temp=0;

        item.theta_ls=[];
    end

    function obs=predict(obs,y,u,step,update_d)
        x_dot=obs.A*obs.x+obs.B'*u+obs.L*(y-obs.C'*obs.x)+obs.H'*obs.d;
        obs.x=obs.x+x_dot*step;

        if update_d==1
            z_dot=-obs.q*obs.C'*(obs.A*obs.x+obs.B'*u+obs.H'*obs.d);
            obs.z=obs.z+step*z_dot;
            obs.d=obs.z+obs.q*y;
        end
    end

    function p = phi(obs,k,t)
        omega=0.05;
        if k==0
            p=1;
        elseif mod(k,2)==1
            p=cos(k*omega*t);
        elseif mod(k,2)==0
            p=sin(k*omega*t);
        end
    end

    function [obs,u]=CBF_Control(obs,X,u_ref,t,s_star,v_star)
        a=[];
        b=[];
        state_size=size(X);
        f=obs.A;
        g2=obs.H;
        %disturbance observer parameters
        omega=7;
        v=1;
        beta=10;
        alpha=15;
    
        %state observer parameters
        theta_upper=0.3;
        lamb=1.2;
        D=27;
        E=0.1;
        
        phi_ls=zeros(1,obs.N);
        for i=1:obs.N
            phi_ls(i)=obs.phi(i,t);
        end

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

                epsilon=1;
                new_f=0;
                %update theta and assign gain
                if index==2
                    for i=1:obs.N
                        new_f=new_f+obs.theta_following_1(:,i)*obs.phi(i,t);
                    end
                elseif index==3
                    for i=1:obs.N
                        new_f=new_f+obs.theta_following_2(:,i)*obs.phi(i,t);
                    end
                end
            end
            K=5;
            eta=X(index*2-1)+s_star-tau*(X(index*2)+v_star);
            if index==1
                a=zeros(1,state_size(1)/2);
                a(1)=-delta_b(2);
                b=[delta_b*f*X+K*eta];
            else
                eta=eta-relax_coef*(X(1)+s_star-tau*(X(2)+v_star));
                temp=zeros(1,state_size(1)/2);
                temp(1)=-delta_b(2);
                temp(index)=-1;
                a=[a;temp];
                delta_hx=delta_b';
                delta_ht=delta_b*f*X;
                if index==2
                    %update theta
                    dot_theta=-theta_upper^2/(2*epsilon)*delta_hx*phi_ls-K*obs.theta_following_1;
                    obs.theta_following_1=obs.theta_following_1+dot_theta*0.01;
                elseif index==3
                    dot_theta=-theta_upper^2/(2*epsilon)*delta_hx*phi_ls-K*obs.theta_following_2;
                    obs.theta_following_2=obs.theta_following_2+dot_theta*0.01;
                end
                other_part=-K*obs.N*epsilon+delta_ht-E*norm(delta_hx);
                b=[b;(delta_b*f+new_f')*X+K*eta+other_part];
            end
        end
        
        H_=eye(state_size(1)/2);
        H_(1,1)=1;
        f_ = zeros(state_size(1)/2,1);
        f_(1)=-2*u_ref;
        
        options =  optimset('Display','off');
        [u_slacks, ~, exitflag, ~] = quadprog(H_, f_, a, b, [], [], [], [], [], options);
    end

    function [obs,u]=observer_CBF(obs,X,u_ref,t,s_star)
        a=[];
        b=[];
        state_size=size(X);
        f=obs.A;
        g2=obs.H;
        %disturbance observer parameters
        omega=7;
        v=1;
        beta=10;
        alpha=15;
    
        %state observer parameters
        theta_upper=0.3;
        lamb=1.2;
        D=27;
        E=0.1;
        
        phi_ls=zeros(1,obs.N);
        for i=1:obs.N
            phi_ls(i)=obs.phi(i,t);
        end

        for index=2:state_size(1)/2
            f=obs.A;
            g=obs.B;
            delta_b=zeros(1,state_size(1));
            delta_b(index*2-1)=1;
            epsilon=1;

            new_f=0;
            %update theta and assign gain
            if index==2
                K=[5,2];
                for i=1:obs.N
                    new_f=new_f+obs.theta_following_1(:,i)*obs.phi(i,t);
                end
            elseif index==3
                K=[5,2,0.2];%[5,1,0.2];
                for i=1:obs.N
                    new_f=new_f+obs.theta_following_2(:,i)*obs.phi(i,t);
                end
            end
            
            obs.error_ls=[obs.error_ls,norm(obs.error_temp-new_f)];
        
            s_safe=0.5;
            eta=[X(index*2-1)+s_star-s_safe-D*exp(-lamb*t)];
            i=1;
            while delta_b(2)==0
                delta_b=delta_b*f+K(i)*delta_b;
                eta=[eta,(delta_b)*X];%(delta_b+new_f')
                i=i+1;
            end
            eta(end)=eta(end)-epsilon;
            delta_hx=delta_b';
            delta_ht=delta_b*f*X;
            if index==2
                %update theta
                dot_theta=-theta_upper^2/(2*epsilon)*delta_hx*phi_ls-K(i)*obs.theta_following_1;
                obs.theta_following_1=obs.theta_following_1+dot_theta*0.01;
            elseif index==3
                dot_theta=-theta_upper^2/(2*epsilon)*delta_hx*phi_ls-K(i)*obs.theta_following_2;
                obs.theta_following_2=obs.theta_following_2+dot_theta*0.01;
            end

            other_part=-K(i)*obs.N*epsilon+delta_ht-E*norm(delta_hx);
            a=[a;-delta_b(2)];
            b=[b;(delta_b*f+new_f')*X+K(i)*eta(end)+other_part];%K*eta'
        end

        s_safe=3;
        delta_b=zeros(1,state_size(1));
        delta_b(1)=1;
        eta=[X(1)+s_star-s_safe];
        
        while delta_b(2)==0
            delta_b=delta_b*f;
            eta=[eta,delta_b*X];
        end
        a=[a;-delta_b(2)];
        K=[5,1.5];

        other_part=omega^2/(2*v*beta)+beta*norm(delta_b*g2')^2/(4*alpha-2*K(end)-2*v);
        b=[b;delta_b*f*X+K*eta'+delta_b*g2'*obs.d-other_part];%(1:length(K)-1)

        h=1;
        f_ = -2*u_ref;
        options =  optimset('Display','off');
        
        [u, ~, exitflag, ~] = quadprog(h, f_, a, b, [], [], [], [], [], options);
        if exitflag==-2
            u=b(end)/a(end);
            %[u, ~, exitflag, ~] = quadprog(H, f_, A(length(A)), b(length(b)), [], [], [], [], [], options);
        end
    end
end

end