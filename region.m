function [lcc,lcc_cbf]=region(dis_t,dis_a,id,K)
% -------------------------------------------------------------------------
%   Parameter setup
%--------------------------------------------------------------------------
% mode of the LCC system
FD_bool = 0;       % 0. CF-LCC; 1. FD-LCC

m       = 0;       % number of preceding vehicles
n       = 4;      % number of following vehicles
PerturbedID = 0;   % perturbation on vehicle
                   % 0. Head vehicle
                   % 1 - m. Preceding vehicles
                   % m+2 - n+m+1. Following vehicles
PerturbedType = 2; % perturbation type
                   % 1:Sine-wave Perturbation;  2: Braking
                   
% ------------------------------------------
% Parameters in the car-following model
% ------------------------------------------
alpha = 0.6; % Driver Model: OVM
beta  = 0.9;
s_st  = 5;
s_go  = 35;

% Traffic equilibrium
v_star   = 20;   % Equilibrium velocity
acel_max = 7;
dcel_max = -7;
v_max    = 40;
s_star   = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; % Equilibrium spacing

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;

% Simulation length
TotalTime = 15;
Tstep     = 0.01;
NumStep   = TotalTime/Tstep;

lcc=1;
lcc_cbf=1;
% ------------------------------------------------------------------------
% Experiment starts here
% ------------------------------------------------------------------------
% Mix or not
%   mix  = 0: All HDVs 
%   mix  = 1: there exists one CAV -- Free-driving LCC
for mix = 0:1 
    switch mix  
        case 1
            ActuationTime = 0; % LCC controller work or not: 0 - Controller Work; Large: won't work
        case 0
            ActuationTime = 0; %999999
    end
    
    % -----------------------------------------------
    % Define state variables
    % -----------------------------------------------
    % Initial State for each vehicle
    S     = zeros(NumStep,m+n+2,3);
    dev_s = 0;
    dev_v = 0;
    co_v  = 1.0;
    v_ini = co_v*v_star; %Initial velocity
    % from - dev to dev
    S(1,:,1) = linspace(0,-(m+n+1)*s_star,m+n+2)' + (rand(m+n+2,1)*2*dev_s - dev_s);
    % The vehicles are uniformly distributed on the straight road with a random deviation
    S(1,:,2) = v_ini * ones(m+n+2,1) + (rand(m+n+2,1)*2*dev_v-dev_v);
    
    % meaning of parameters
    % 1:        head vehicle
    % 2-(m+1):  Preceding vehicles
    % m+2:      CAV
    % (m+3)-(m+n+2): Following vehicles
    
    ID = zeros(1,m+n+2);
    if mix
        ID(m+2) = 1;
    end
    
    X = zeros(2*(m+n+1),NumStep);
    u = zeros(NumStep,1);               % 0. HDV  1. CAV
    V_diff = zeros(NumStep,m+n+1);      % Velocity Difference
    D_diff = zeros(NumStep,m+n+1);      % Following Distance
   
    % ---------------------------------------------------------
    % Simulation starts here
    % ---------------------------------------------------------
    for k = 1:NumStep - 1
        % Update acceleration
        V_diff(k,:) = S(k,1:(end-1),2) - S(k,2:end,2); 
        D_diff(k,:) = S(k,1:(end-1),1) - S(k,2:end,1);
        cal_D = D_diff(k,:); % For the boundary of Optimal Veloicity Calculation
        for i = 1:m+n+1
            if cal_D(i) > s_go
                cal_D(i) = s_go;
            elseif cal_D(i) < s_st
                cal_D(i) = s_st;
            end
        end
        
        % nonlinear OVM Model
        % V_d = v_max/2*(1-cos(pi*(h-h_st)/(h_go-h_st)));
        % a2 = alpha*(V_d-v2)+beta*(v1-v2);
        acel = alpha*(v_max/2*(1-cos(pi*(cal_D-s_st)/(s_go-s_st))) - S(k,2:end,2))+beta*V_diff(k,:);
        acel(acel>acel_max) = acel_max;
        acel(acel<dcel_max) = dcel_max;
        
        S(k,2:end,3) = acel;
        S(k,1,3)     = 0; % the preceding vehicle
        
        % Perturbation type
        switch PerturbedType
            case 1     % sine wave
                P_A = 0.2;
                P_T = 15;
                if k*Tstep>20 && k*Tstep<20+P_T
                    S(k,PerturbedID+1,3) = P_A*cos(2*pi/P_T*(k*Tstep-20));
                end
            case 2     % braking
                if id==1
                    if (k*Tstep>0)&&(k*Tstep<=0+dis_t)
                        S(k,PerturbedID+1,3) = -dis_a;
                    end
                    if (k*Tstep>dis_t)&&(k*Tstep<=2*dis_t)
                        S(k,PerturbedID+1,3) = dis_a;
                    end
                else
                    if (k*Tstep>0)&&(k*Tstep<=0+dis_t)
                        S(k,PerturbedID+id,3) = dis_a;
                    end
                end
        end
        
        X(1:2:end,k) = reshape(D_diff(k,:),m+n+1,1) - s_star;
        X(2:2:end,k) = reshape(S(k,2:end,2),m+n+1,1) - v_star;
        if k > ActuationTime/Tstep
            u(k) = K*X(:,k);
            if mix
                u(k)=CBF_Control(X(:,k),u(k),alpha1,alpha2,alpha3,s_star,v_star,S(k,1,2)-v_star);
            end
            if u(k) > acel_max
                u(k) = acel_max;
            elseif u(k) < dcel_max
                u(k) = dcel_max;
            end
            S(k,m+2,3) = u(k);
        end
        
        S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
        S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        
        for temp=1:m+n+1
            if S(k+1,temp+1,1)>S(k+1,temp,1)
                if mix
                    lcc_cbf=0;
                else
                    lcc=0;
                end
            end
        end
        if lcc_cbf==0 & lcc==0
            return;
        end
    end
end

end