time1=out.result.time;
index=out.result.data(:,1);
toc=out.result.data(:,2);
vref=out.result.data(:,4);



time=out.out.time;
x=out.out.signals.values(:,1);
y=out.out.signals.values(:,2);
yaw=out.out.signals.values(:,3);
Vx=out.out.signals.values(:,4);
Vy=out.out.signals.values(:,5);
Avy=out.out.signals.values(:,6);

Beta=out.out.signals.values(:,14);
Ax=out.out.signals.values(:,7);
Ay=out.out.signals.values(:,45);






figure(2)
plot(x,y,'b')
title("轨迹")

figure(3)
plot(time, yaw,'r')
title("偏航角")

figure(4)
plot(time, Vx,'r')
hold on;
plot(time1, vref*3.6,'b')
title("速度")
ylim([90 120]);


figure(5)
plot(time, Beta,'r')
title("质心侧偏角")


time = downsample(time,100, 0);
Ax = downsample(Ax,100, 0);
Ay = downsample(Ay,100, 0);
Avy= downsample(Avy,100, 0);

figure(6)
plot(time, Avy,'r')
title("横摆角速度")

figure(7)
plot(time, Ax,'r')
title("纵向加速度")
ylim([-0.5 0.5])

figure(8)
plot(time, Ay,'r')
title("横向加速度")



