region_img_lcc=zeros(71,71);
region_img_lcc_cbf=zeros(71,71);

id=1;
alpha = 0.6; % Driver Model: OVM
beta  = 0.9;
s_st  = 5;
s_go  = 35;
v_star   = 20;   % Equilibrium velocity
v_max    = 40;
s_star   = acos(1-v_star/v_max*2)/pi*(s_go-s_st)+s_st; % Equilibrium spacing

% linearized model
alpha1 = alpha*v_max/2*pi/(s_go-s_st)*sin(pi*(s_star-s_st)/(s_go-s_st));
alpha2 = alpha+beta;
alpha3 = beta;
state_size=10;
addpath /Users/zhoujingyuan/mosek/9.3/toolbox/r2015aom
K = -lqrsdp(alpha1,alpha2,alpha3,state_size);

count=0;
for t=0:0.1:7
    count=count+1
    for a=0:0.1:7
        [region_img_lcc(round(t*10)+1,round(a*10)+1),region_img_lcc_cbf(round(t*10)+1,round(a*10)+1)]=region(t,a,id,K);
        if region_img_lcc(round(t*10)+1,round(a*10)+1)==0 & region_img_lcc_cbf(round(t*10)+1,round(a*10)+1)==0
            break;
        end
    end
end

figure(1)
subplot(1,2,1)
imshow(region_img_lcc,[]);
ylabel('duration/s')
xlabel('accel/(m/s^2)')
axis on
yticks([0 10 20 30 40 50 60 70])
yticklabels({'0s','1s','2s','3s','4s','5s','6s','7s'})
xticks([0 10 20 30 40 50 60 70])
xticklabels({'0m/s^2','1m/s^2','2m/s^2','3m/s^2','4m/s^2','5m/s^2','6m/s^2','7m/s^2'})

subplot(1,2,2)
imshow(region_img_lcc_cbf,[]);
ylabel('duration/s')
xlabel('accel/(m/s^2)')
axis on
yticks([0 10 20 30 40 50 60 70])
yticklabels({'0s','1s','2s','3s','4s','5s','6s','7s'})
xticks([0 10 20 30 40 50 60 70])
xticklabels({'0m/s^2','1m/s^2','2m/s^2','3m/s^2','4m/s^2','5m/s^2','6m/s^2','7m/s^2'})

temp=~region_img_lcc+~region_img_lcc_cbf;
figure(2)
imshow(temp,[]);
ylabel('duration/s')
xlabel('accel/(m/s^2)')
axis on
yticks([0 10 20 30 40 50 60 70])
yticklabels({'0s','1s','2s','3s','4s','5s','6s','7s'})
xticks([0 10 20 30 40 50 60 70])
xticklabels({'0m/s^2','1m/s^2','2m/s^2','3m/s^2','4m/s^2','5m/s^2','6m/s^2','7m/s^2'})

if id==1
    save(['head_vehicle_',num2str(id-1),'_region.mat'],'region_img_lcc','region_img_lcc_cbf')
else
    save(['following_vehicle_',num2str(id-2),'_region.mat'],'region_img_lcc','region_img_lcc_cbf')
end

% y1=[];
% y2=[];
% for t=0:0.2:7
%     for a=0.2:0.2:7
%         if region_img_lcc(round(t*10)+1,round(a*10)+1)==0  & region_img_lcc(round(t*10)+1,round((a-0.2)*10)+1)==1
%             y1=[y1,a];
%         end
%         if region_img_lcc_cbf(round(t*10)+1,round(a*10)+1)==0  & region_img_lcc_cbf(round(t*10)+1,round((a-0.2)*10)+1)==1
%             y2=[y2,a];
%         end
%     end
% end
% 
% figure(3)
% plot(0:0.2:7,y1)
% hold on;
% plot(0:0.2:7,y2)
% xlim([0 7]);
% ylim([0 7]);


