classdef state_observer
    properties
        A
        B
        L
        C
        x
        N
        theta_head
        theta_following_1
        theta_following_2
        error_ls
        error_temp
        theta_ls
        alpha1
        alpha2
        alpha3
    end
    methods
        function item=Observer(state_size,alpha1,alpha2,alpha3)
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
            
            %assign L
            L=place(item.A',item.C,[-1.5,-2.5,-3.5,-4.5,-5.5,-6.5]);
            item.L=L';
            e=real(eig(item.A-item.L*item.C'));

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

        function obs=predict(obs,y,u,step)
            x_dot=obs.A*obs.x+obs.B'*u+obs.L*(y-obs.C'*obs.x);
            obs.error_temp=obs.L*(y-obs.C'*obs.x);
            obs.x=obs.x+x_dot*step;
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

        function [u,obs] = CBF_Control(obs,X,u_ref,t,v_star,s_star,tilde_vh,ah)
            a=[];
            b=[];
            state_size=size(X);
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
            
            %for disturbance from head vehicle
            if nargin==8
                tau=3.5;
                miu=6;%6
                delta_hx=zeros(state_size(1),1);
                delta_hx(1,1)=1;
                delta_hx(2,1)=-tau;
                delta_ht=X(2)-tilde_vh+lamb*D*exp(-lamb*t)+tau*ah;
                
                dot_theta=-theta_upper^2/(2*epsilon)*delta_hx*phi_ls-miu*obs.theta_head;
                obs.theta_head=obs.theta_head+dot_theta*0.01;

                h=X(1)+s_star-tau*X(2)+tau*tilde_vh-D*exp(-lamb*t)-epsilon;
                a=[a;2*tau];

                new_f=0;
                for i=1:obs.N
                    new_f=new_f+obs.theta_head(:,i)*obs.phi(i,t);
                end
                obs.error_ls=[obs.error_ls,norm(obs.error_temp-new_f)];
                new_f=new_f+obs.A*X;
                Lfh=new_f'*delta_hx;
                
                other_part=-miu*obs.N*epsilon+delta_ht-E*norm(delta_hx);
                b=[b;Lfh+miu*h+other_part];
            end
            
            H=1;
            f_ = -2*u_ref;
            options =  optimset('Display','off');
            
            [u, ~, exitflag, ~] = quadprog(H, f_, a, b, [], [], [], [], [], options);
            if exitflag==-2
                u=b(length(b))/a(length(a));
            end
            
        end
    end
end
