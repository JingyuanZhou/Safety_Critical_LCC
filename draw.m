link="/Users/zhoujingyuan/Desktop/journal paper/img/";

t=0:0.01:49.99;
set(groot,'defaultAxesFontName','Times')
set(groot,'defaultAxesFontSize',22)
fts_xy=30;

% figure(1)
% load('h_ls.mat')
% figure(1)
% l=100*40;
% plot(t(1:l),h_cbf(1:l),'LineWidth',2)
% hold on
% plot(t(1:l),h_obs(1:l),'LineWidth',2)
% yline(0,'--','LineWidth',1.5);
% xlabel('t/s')
% ylabel('h(x)/m')
% legend('estimated states with CBF','observer-based CBF')
% legend('boxoff')
% f = gcf;
%exportgraphics(f,link+'s_following.pdf')
%exportgraphics(f,link+'s_following.pdf')

%space
figure(2)
plot(t(1:l),S_HDV(1:l,1,1),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,2,1),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,3,1),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,4,1),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','northwest')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Position/(m)')
xlim([1 l/100])
ylim([S_HDV(1,4,1) S_HDV(l,1,1)])
%title('estimated states with CBF')
f = gcf;
%exportgraphics(f,link+'est_position_head.pdf')
%exportgraphics(f,link+'est_position_following.pdf')

figure(3)
plot(t(1:l),S_LCC(1:l,1,1),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,2,1),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,3,1),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,4,1),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','northwest')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Position/(m)')
xlim([1 l/100])
ylim([S_LCC(1,4,1) S_LCC(l,1,1)])
%title('observer-based CBF')
f = gcf;
%exportgraphics(f,link+'obs_position_head.pdf')
%exportgraphics(f,link+'obs_position_following.pdf')

%velocity
figure(4)
plot(t(1:l),S_HDV(1:l,1,2),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,2,2),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,3,2),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,4,2),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','southeast')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Velocity/(m/s)')
xlim([1 l/100])
%title('estimated states with CBF')
f = gcf;
%exportgraphics(f,link+'est_velocity_head.pdf')
%exportgraphics(f,link+'est_velocity_following.pdf')

figure(5)
plot(t(1:l),S_LCC(1:l,1,2),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,2,2),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,3,2),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,4,2),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','southeast')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Velocity/(m/s)')
xlim([1 l/100])
%title('observer-based CBF')
f = gcf;
%exportgraphics(f,link+'obs_velocity_head.pdf')
%exportgraphics(f,link+'obs_velocity_following.pdf')

%acc
figure(6)
plot(t(1:l),S_HDV(1:l,1,3),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,2,3),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,3,3),'LineWidth',2)
hold on;
plot(t(1:l),S_HDV(1:l,4,3),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','southeast')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Acceleration/(m/s^2)')
xlim([1 l/100])
%title('estimated states with CBF')
f = gcf;
%exportgraphics(f,link+'est_acc_head.pdf')
%exportgraphics(f,link+'est_acc_following.pdf')

figure(7)
plot(t(1:l),S_LCC(1:l,1,3),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,2,3),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,3,3),'LineWidth',2)
hold on;
plot(t(1:l),S_LCC(1:l,4,3),'LineWidth',2)

legend('Head HDV','Leading CAV','Following HDV 1','Following HDV 2','Location','southeast')
legend('boxoff')
xlabel('Time/(s)')
ylabel('Acceleration/(m/s^2)')
xlim([1 l/100])
%title('observer-based CBF')
f = gcf;
%exportgraphics(f,link+'obs_acc_head.pdf')
%exportgraphics(f,link+'obs_acc_following.pdf')



