t=0:0.01:49.99;
set(groot,'defaultAxesFontName','Times')
set(groot,'defaultAxesFontSize',22)
fts_xy=30;
link="/Users/zhoujingyuan/Desktop/journal paper/";

figure(1)
load('s_nominal_relax.mat')
load('s_spacing_relax.mat')
load('s_th_relax.mat')
load('s_th2_relax.mat')
load('s_tc_relax.mat')

l=100*40;
plot(t(1:l),s_nominal_relax(1:l),'LineWidth',2)
hold on
plot(t(1:l),s_spacing_relax(1:l),'LineWidth',2)
plot(t(1:l),s_th_relax(1:l),'LineWidth',2)
plot(t(1:l),s_th2_relax(1:l),'LineWidth',2)
plot(t(1:l),s_tc_relax(1:l),'LineWidth',2)

yline(0,'--','LineWidth',1.5);
xlabel('t/s')
ylabel('spacing/m')
legend('nominal','CSP','TTH','FBTTH','TTC',Location='best')
legend('boxoff')
f = gcf;
exportgraphics(f,link+'bc_following.pdf')

figure(2)
load('s_nominal_relax.mat')
load('s_spacing_head_relax.mat')
load('s_th_head_relax.mat')
load('s_th2_head_relax.mat')
load('s_tc_head_relax.mat')

l=100*40;
plot(t(1:l),s_nominal_relax(1:l),'LineWidth',2)
hold on
plot(t(1:l),s_spacing_head_relax(1:l),'LineWidth',2)
%plot(t(1:l),s_th_head_relax(1:l),'LineWidth',2)
plot(t(1:l),lipu(1:l),'LineWidth',2)
plot(t(1:l),s_th2_head_relax(1:l),'LineWidth',2)
plot(t(1:l),s_tc_head_relax(1:l),'LineWidth',2)

yline(0,'--','LineWidth',1.5);
xlabel('t/s')
ylabel('spacing/m')
legend('nominal','CSP','TTH','FBTTH','TTC',Location='best')
legend('boxoff')
f = gcf;
exportgraphics(f,link+'bc_head.pdf')