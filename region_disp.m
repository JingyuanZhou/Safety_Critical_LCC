link="/Users/zhoujingyuan/Desktop/journal paper/img_region/";
set(groot,'defaultAxesFontName','Times')
set(groot,'defaultAxesFontSize',24)
fts_xy=30;

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

y1=[];
y2=[];
for t=0:0.1:7
    flag1=1;
    flag2=1;
    for a=0.1:0.1:7
        if region_img_lcc(round(t*10)+1,round(a*10)+1)==0  & region_img_lcc(round(t*10)+1,round((a-0.1)*10)+1)==1
            y1=[y1,a];
            flag1=0;
        end
        if region_img_lcc_cbf(round(t*10)+1,round(a*10)+1)==0  & region_img_lcc_cbf(round(t*10)+1,round((a-0.1)*10)+1)==1
            y2=[y2,a];
            flag2=0;
        end
    end
    if flag1
        y1=[y1,7];
    end
    if flag2
        y2=[y2,7];
    end
end

t=0:0.1:7;
figure(3)
area(t(y2>=0),y2(y2>=0),'FaceColor',[0.2 0.6 0.5],'EdgeColor',[0.2 0.6 0.5])
hold on;
area(t(y1>=0),y1(y1>=0),'FaceColor',[0.2 0.5 0.6],'EdgeColor',[0.2 0.5 0.6])
legend('CBF-QP based controller','Nominal controller')
xlim([0 7]);
ylim([0 7]);
yticks([0 1 2 3 4 5 6 7])
yticklabels({'0','1','2','3','4','5','6','7'})
xticks([0 1 2 3 4 5 6 7])
xticklabels({'0','-1','-2','-3','-4','-5','-6','-7'})
%xticklabels({'0','1','2','3','4','5','6','7'})
ylabel('Duration/(s)','FontSize',fts_xy)
xlabel('Acceleration/(m/s^2)','FontSize',fts_xy)
f = gcf;
exportgraphics(f,link+'head_vehicle_region.pdf')




