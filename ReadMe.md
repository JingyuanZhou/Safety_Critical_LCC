### ITSC 2022 paper: Safety Critical Control of Mixed-autonomy Traffic via a Single Autonomous Vehicle

1. car_following_lcc.m: 主程序，leading cruise control的仿真

2. CBF_Control_tth.m: 使用time-to-headway作为CBF candidate的CBF-QP控制器

3. CBF_Control_spacing.m：使用spacing作为CBF candidate的CBF-QP控制器

4. CBF_Control_ttc.m：使用time-to-collision作为CBF candidate的CBF-QP控制器

5. CBF_Control_fbtth.m：使用forced-based-time-to-headway作为CBF candidate的CBF-QP控制器

6. observer_CBF_Control.m：observer和基于observer的CBF-QP控制器

7. lqrsdp.m：使用LQR优化名义控制器的增益

8. draw_hx.m：画出使用不同CBF-candidate的CBF-QP控制器的h-t图

9. region.m，region_draw.m，region_disp.m：region_draw.m计算safety region并保存至.mat文件，region_disp.m画出safety region图。

10. state and disturbance observer文件夹：LCC中state observer和disturbance observer的设计

    
