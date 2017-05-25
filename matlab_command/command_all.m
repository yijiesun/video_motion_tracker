function command_all(dir)
%clc;
%clear;
close all;
%dir='e:\test\new\1949';

global gyro video gyroNum videoNum videodata minD;
gname=sprintf('%s%s%s',dir,'\','gyro.txt');
vname=sprintf('%s%s%s',dir,'\','video_sift.txt');
gyrodata = load(gname);
videodata = load(vname);
gyroNum=size(gyrodata,1);
videoNum=size(videodata,1);

gyro=zeros(gyroNum,2);
video=zeros(videoNum,2);
gyro(:,1)=gyrodata(:,1);
gyro(:,2)=gyrodata(:,2);

for i=1:gyroNum
gyro(i,1)=(gyro(i,1)-gyrodata(1,1))/1000000;
end

%%%%%%%%%%%%main loop%%%%%%%%%%%%
find_fps();
%%%%%%%%%%%%main loop%%%%%%%%%%%%

%claculator curve stuff use best parameter as above
video=zeros(videoNum,2);
for i=1:videoNum
video(i,1)=videodata(i,1)/minD(1);%minD(1,1)save best fps
video(i,2)=videodata(i,2)*140;
end

gyroadd=zeros(gyroNum,2);
for i=1:gyroNum
gyroadd(i,1)=gyro(i,1)+minD(2);
end

index=0;
interpoint=[];
for i=1:gyroNum
if(gyroadd(i,1)>video(1,1)&&gyroadd(i,1)<video(videoNum,1))
    index=index+1;
    interpoint(index,1)=gyroadd(i,1);
    interpoint(index,2)=gyro(i,2);
end
end
videointer=interp1(video(:,1),video(:,2),interpoint(:,1),'spline');  

figure(1);
plot(video(:,1),video(:,2),'Color',[1 0 0])
hold on
plot(gyro(:,1),gyro(:,2),'Color',[0 1 0])
title(['对齐前前的video(红)与gyro(绿)轨迹,fps:',num2str(minD(1))]);
saveas(1,[dir,'\修正前的轨迹fps',num2str(minD(1)),'.fig']);
%close(1);
 
figure(2);
plot(interpoint(:,1),videointer,'Color',[1 0 0])
hold on
plot(interpoint(:,1),interpoint(:,2),'Color',[0 1 0])
title(['对齐后的轨迹：GYRO的平移量:',num2str(minD(2)),'相关系数：',num2str(1-minD(3))]);
saveas(2,[dir,'\对齐后的轨迹GYRO的平移量',num2str(minD(2)),'.fig']);
%close(2);
end

