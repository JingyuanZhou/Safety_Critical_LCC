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
TotalTime = 50;
Tstep     = 0.01;
NumStep   = TotalTime/Tstep;

% type
if_obs=false;
addpath /Users/zhoujingyuan/mosek/9.3/toolbox/r2015aom
% ------------------------------------------------------------------------
% Some output information
% ------------------------------------------------------------------------
fprintf('============================================================\n')
fprintf('    Demo: Car-Following Leading Cruise Control \n')
fprintf('          By Jiawei Wang, Yang Zheng \n')
fprintf('============================================================\n')
fprintf('Number of HDV vehicles behind: %d\n',n)
fprintf('Perturbation vehicle Id      : %d\n',PerturbedID)
fprintf('---------------------------\n')
fprintf('HDV car-following model: optimal velocity model (OVM) \n')
fprintf('Parameter setup in HDV car-following model: \n')
fprintf('    alpha  beta  s_st  s_go  v_max \n    %4.2f  %4.2f  %4.2f  %4.2f  %4.2f\n',alpha,beta,s_st,s_go,v_max)
fprintf('Coefficients in linearized HDV car-following model: \n')
fprintf('    alpha1  alpha2  alpha3 \n    %4.2f    %4.2f    %4.2f \n',alpha1,alpha2,alpha3)
fprintf('---------------------------\n')
fprintf('Simulation length (time step): %d  (%4.2f)\n',TotalTime,Tstep)  % this can be improved
fprintf('-----------------------------------------------------------\n')
fprintf('   Simulation beigns ...')

% ------------------------------------------------------------------------
% Experiment starts here
% ------------------------------------------------------------------------
% Mix or not
%   mix  = 0: All HDVs 
%   mix  = 1: there exists one CAV -- Free-driving LCC
%s_th_head_relax=[];
%lipu=[];
tic
for mix = 0:1 
    switch mix  
        case 1
            ActuationTime = 0; % LCC controller work or not: 0 - Controller Work; Large: won't work
        case 0
            ActuationTime = 0; %999999
    end

    state_size=(n+1)*2;
    C1=zeros(1,state_size);
    C2=zeros(1,state_size);
    C3=zeros(1,state_size);
    C1(1)=1;
    C2(2)=1;
    C3(state_size)=1;
    C=[C1',C2',C3'];
    
    if if_obs
        obs=observer_CBF_Control(state_size,alpha1,alpha2,alpha3);
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
    % LCC controller: the following choice is used in our paper
    % ---------------------------------------------------------
    K = zeros(1,2*(n+1));
    if FD_bool
        K(1:6) = [0,-0.5,-0.2,0.05,-0.1,0.05];
    else
        K(1:6) = [0.2,-0.5,-0.2,0.05,-0.1,0.05];
        K = -lqrsdp(alpha1,alpha2,alpha3,state_size);
    end
    
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
%                 if (k*Tstep>15)&&(k*Tstep<=17)
%                     S(k,PerturbedID+1,3) = -7;
%                 end
%                 if (k*Tstep>17)&&(k*Tstep<=17.5)
%                     S(k,PerturbedID+1,3) = 0;
%                 end
%                 if (k*Tstep>17.5)&&(k*Tstep<=19.5)
%                     S(k,PerturbedID+1,3) = 7;
%                 end

                if (k*Tstep>15)&&(k*Tstep<=17)
                    S(k,PerturbedID+6,3) = 6;
                end
        end
        
        X(1:2:end,k) = reshape(D_diff(k,:),m+n+1,1) - s_star;
        X(2:2:end,k) = reshape(S(k,2:end,2),m+n+1,1) - v_star;
        y=C'*X(:,k);
        if k > ActuationTime/Tstep
            if if_obs
                obs=obs.predict(y,u(k),Tstep,1);
                u(k) = K*obs.x;
            else
                u(k) = K*X(:,k);
            end

            if mix
                if if_obs
                    [obs,u(k)] = obs.CBF_Control(obs.x,u(k),k*Tstep,s_star,v_star);
                    %u(k) = CBF_Control(X(:,k),u(k),alpha1,alpha2,alpha3,s_star);
                else
                    u(k) = CBF_Control_tth(X(:,k),u(k),alpha1,alpha2,alpha3,s_star,v_star,S(k,1,2)-v_star);
                end
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
    end
    
    
   %% Data Recording
    if mix
        S_LCC = S;
    else
        S_HDV = S;
    end
end
tsim = toc;
%save s_th_head_relax

fprintf('  ends at %6.4f seconds \n', tsim);
fprintf('   Computing average abosulte velocity error ... \n');
% -------------------------------------------------------------------------
% Calculate Average Abosulte Velocity Error
% -------------------------------------------------------------------------
VelocityDeviation_HDV = 0;
VelocityDeviation_LCC = 0;

for i=20/Tstep:40/Tstep
    VelocityDeviation_HDV = VelocityDeviation_HDV + sum(abs(S_HDV(i,2:end,2)-v_star));
    VelocityDeviation_LCC = VelocityDeviation_LCC + sum(abs(S_LCC(i,2:end,2)-v_star));
end
VelocityDeviation_HDV = VelocityDeviation_HDV*Tstep/20/11;
VelocityDeviation_LCC = VelocityDeviation_LCC*Tstep/20/11;
fprintf(' Looking ahead only (all HDVs)  |   LCC       |  Improvement rate \n')
fprintf(' %.2f                           |  %.2f       |  %6.4f \n',VelocityDeviation_HDV,VelocityDeviation_LCC,(VelocityDeviation_HDV-VelocityDeviation_LCC)/VelocityDeviation_HDV)

fprintf('   Computing total fuel consumption ... \n');
% -------------------------------------------------------------------------
% Calculate Fuel Consumption
% -------------------------------------------------------------------------
FuelConsumption_HDV = 0;
FuelConsumption_LCC = 0;

%List_HDV=[];
List_LCC=[];
List_LCC_CBF=[];
for i=10/Tstep:50/Tstep
    R_HDV  = 0.333 + 0.00108*S_HDV(i,2:end,2).^2 + 1.2*S_HDV(i,2:end,3);
    F_HDV  = 0.444 + 0.09*R_HDV.*S_HDV(i,2:end,2) + 0.054 * max(0,S_HDV(i,2:end,3)).^2.*S_HDV(i,2:end,2);
    F_HDV(R_HDV <= 0) = 0.444;
    List_LCC=[List_LCC,sum(F_HDV)*Tstep];
    FuelConsumption_HDV = FuelConsumption_HDV + sum(F_HDV)*Tstep;

    R_LCC  = 0.333 + 0.00108*S_LCC(i,2:end,2).^2 + 1.2*S_LCC(i,2:end,3);
    F_LCC  = 0.444 + 0.09*R_LCC.*S_LCC(i,2:end,2) + 0.054 * max(0,S_LCC(i,2:end,3)).^2.*S_LCC(i,2:end,2);
    F_LCC(R_LCC <= 0) = 0.444;
    List_LCC_CBF=[List_LCC_CBF,sum(F_LCC)*Tstep];
    FuelConsumption_LCC = FuelConsumption_LCC + sum(F_LCC)*Tstep;
end
fprintf(' Looking ahead only (all HDVs)  |   LCC       |  Improvement rate \n')
fprintf(' %.2f                         |  %.2f     |  %6.4f \n',FuelConsumption_HDV,FuelConsumption_LCC,(FuelConsumption_HDV-FuelConsumption_LCC)/FuelConsumption_HDV)

% ------------------------------------------------------------------------
%  Plot Video 
% ------------------------------------------------------------------------
fprintf('-----------------------------------------------------------\n')
fprintf('    Now record a video for demonstration, please wait ... \n')

videoOutput = 0;  % whether save into a video file
vehicleSize = 12; % MarkerSize
FSize = 16;
VehicleColor = [93,40,132]/255;
if FD_bool
    videoFile = ['Video\FDLCC_Comparison_BrakeID_',num2str(PerturbedID),'.mp4'];
else
    videoFile = ['Video\CFLCC_Comparison_BrakeID_',num2str(PerturbedID),'.mp4'];
end

figure(1);
set(0,'defaultfigurecolor','w');
Position1 = [0.1,0.6,0.8,0.3];
Position2 = [0.1,0.15,0.8,0.3];

ax1 = subplot('Position',Position1);
plot(linspace(0,2000,2000),zeros(2000,1),'--','Linewidth',0.5,'Color','k');
hold on;

for id = 2:n+2
    line1(id) = plot(linspace(S_HDV(1,id-1,1),S_HDV(1,id,1),10),-5*ones(10,1),'Linewidth',1,'Color',VehicleColor);
    hold on;
end
for id = 1:n+2    
    position1(id) = plot(S_HDV(1,id,1),-5,'o');
    position1(id).MarkerSize = vehicleSize;
    if id == 2
        position1(id).MarkerFaceColor = [0,176,240]/255;
        position1(id).MarkerEdgeColor = 'none';
    else
        position1(id).MarkerFaceColor = [0.7,0.7,0.7];
        position1(id).MarkerEdgeColor = 'none';
    end
    hold on;
end

original_x1 = S_HDV(1,2,1)+20;
pstart1 = plot(original_x1,0,'o','MarkerSize',vehicleSize/2,...
    'MarkerFaceColor','k','MarkerEdgeColor','none');

axis([original_x1-250,original_x1,-15,15]);
set(gcf,'Position',[150,100,600,450]);
% axis off;

text1 = title('Time = 0 s','Interpreter','latex','Fontsize',FSize);
%text1.HorizontalAlignment = 'center';
set(gca,'TickLabelInterpreter','latex');
set(gca,'YTick',-6:6:6);

yl = ylabel('Velocity Perturbation ($\mathrm{m/s}$)','Interpreter','Latex','Color','k','FontSize',FSize);
yl.Position = [-262,-10,-1];
yl.Position(1) = yl.Position(1) + 15*15;

ax2 = subplot('Position',Position2);
plot(linspace(0,2000,2000),zeros(2000,1),'--','Linewidth',0.5,'Color','k');
hold on;

for id = 2:n+2
    line2(id) = plot(linspace(S_LCC(1,id-1,1),S_LCC(1,id,1),10),5*ones(10,1),'Linewidth',1,'Color',VehicleColor);
    hold on;
end

for id = 1:n+2
    position2(id) = plot(S_LCC(1,id,1),5,'o');
    position2(id).MarkerSize = vehicleSize;
    if id == 2
        position2(id).MarkerFaceColor = [0,176,240]/255;
        position2(id).MarkerEdgeColor = 'none';
    else
        position2(id).MarkerFaceColor = [0.7,0.7,0.7];
        position2(id).MarkerEdgeColor = 'none';
    end
    hold on;
end


original_x2 = S_LCC(1,2,1)+20;
pstart2     = plot(original_x2,0,'o','MarkerSize',vehicleSize/2,...
    'MarkerFaceColor','k','MarkerEdgeColor','none');

axis([original_x2-250,original_x2,-15,15]);
set(gcf,'Position',[150,100,700,330]);
% axis off;
set(gca,'TickLabelInterpreter','latex');
set(gca,'YTick',-6:6:6);

xlabel('Position ($\mathrm{m}$)','Interpreter','Latex','Color','k','FontSize',FSize);

original_x1 = original_x1 + 15*15;
original_x2 = original_x2 + 15*15;

if FD_bool
    line1(2).Visible = 'off';
    line2(2).Visible = 'off';
    pstart1.Visible = 'off';
    pstart2.Visible = 'off';
end

dt = 0.02;

if videoOutput
    myVideo = VideoWriter(videoFile,'MPEG-4');
    myVideo.FrameRate = 1/dt;
    open(myVideo);
end

for i=15/Tstep:dt/Tstep:50/Tstep  
    ax1;
    for id = 2:n+2
        line1(id).XData = linspace(S_HDV(i,id-1,1),S_HDV(i,id,1),10);
        line1(id).YData = linspace(S_HDV(i,id-1,2)-15,S_HDV(i,id,2)-15,10);
    end
    
    for id = 1:n+2
        position1(id).XData = S_HDV(i,id,1);
        position1(id).YData = S_HDV(i,id,2)-15;
    end
    original_x1 = S_HDV(i,1,1);%original_x1 + 15*dt;
    yl.Position(1) = yl.Position(1) + 15*dt;
    pstart1.XData = original_x1;
    
    ax1.XLim = [original_x1-250,original_x1];
    
    %     text1.Position(1) = 0.5*(2*original_x1-250);
    text1.String = ['Time = ',num2str(i*Tstep,'%4.1f'),' s'];
    
    ax2;
    for id = 2:n+2
        line2(id).XData = linspace(S_LCC(i,id-1,1),S_LCC(i,id,1),10);
        line2(id).YData = linspace(S_LCC(i,id-1,2)-15,S_LCC(i,id,2)-15,10);
    end
    
    for id = 1:n+2
        position2(id).XData = S_LCC(i,id,1);
        position2(id).YData = S_LCC(i,id,2)-15;
    end
    %original_x2 = original_x2 + 15*dt;
    original_x2 = S_HDV(i,1,1);%S_LCC(i,2,1)+20;
    pstart2.XData = original_x2;
    
    ax2.XLim = [original_x2-250,original_x2];
    
    frame = getframe(gcf);
    if videoOutput
        writeVideo(myVideo,frame);
    end
    drawnow;
    pause(0.01);  
end

if videoOutput
    close(myVideo);
end

