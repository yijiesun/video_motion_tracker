function command(dir)
fid = fopen([dir,'\command.tmp']);
tline=fgetl(fid);
index=findstr(tline,' ');
ghead=str2num(tline(1:index(1)));
gend=str2num(tline(index(1)+1:index(2)));
frame=str2num(tline(index(2)+1:index(3)));

gname=sprintf('%s%s%s',dir,'\','gyro.txt');
vname=sprintf('%s%s%s',dir,'\','video_sift.txt');
fps=(frame-1)/((gend-ghead)/1000000);
gyrodata = load(gname);
gyro(:,1)=gyrodata(:,1);
gyro(:,2)=gyrodata(:,2);
gyroNum=size(gyro,1);
for i=1:gyroNum
gyro(i,1)=(gyro(i,1)-ghead)/1000000;
gyro(i,2)=gyro(i,2)/5.2-10;
end

videodata = load(vname);
video(:,1)=videodata(:,1);
video(:,2)=videodata(:,2);
for i=1:size(video,1)
video(i,1)=video(i,1);
video(i,1)=video(i,1)/fps;
video(i,2)=video(i,2)*14;
end

tmp=0;
for offset=0:0.001:0.2
tmp=tmp+1;

for i=1:gyroNum
gyroadd(i,1)=gyro(i,1)+offset;
end

videointer=interp1(video(:,1),video(:,2),gyroadd(:,1),'spline');  

result=0;

for i=1:gyroNum
result=result+(gyro(i,2)-videointer(i))^2;
end
res(tmp,1)=offset;
res(tmp,2)=sqrt(result/gyroNum);

end

%minR=0;
%for p=3:180
%if(res(p+1,2)>=res(p,2))
%minR=res(p,1);
%break;
%end
%end

minX=res(1,2);
for p=2:180
if(minX>=res(p,2))
minX=res(p,2);
end
end

minR=0;
for p=1:180
if(res(p,2)==minX)
minR=res(p,1);
break;
end
end

for i=1:gyroNum
gyroadd(i,1)=gyro(i,1)+minR;
end

videointer=interp1(video(:,1),video(:,2),gyroadd(:,1),'spline');  
videoorig=interp1(video(:,1),video(:,2),gyro(:,1),'spline');  

figure(1);
plot(gyro(:,1),videoorig,'Color',[1 0 0])
hold on
plot(gyro(:,1),gyro(:,2),'Color',[0 1 0])
title(['未经修正前的video(红)与gyro(绿)轨迹,fps:',num2str(fps)]);
saveas(1,[dir,'\修正前的轨迹fps',num2str(fps),'.fig']);
%close(1);
 
figure(2);
plot(gyroadd(:,1),videointer,'Color',[1 0 0])
hold on
plot(gyroadd(:,1),gyro(:,2),'Color',[0 1 0])
title(['修正后的轨迹：欧式距离最小时，GYRO的平移量:',num2str(minR)]);
saveas(2,[dir,'\修正后的轨迹GYRO的平移量',num2str(minR),'.fig']);
%close(2);

figure(3);
plot(res(:,1),res(:,2))
title('GYRO平移量与两条曲线欧式距离的分布');
saveas(3,[dir,'\GYRO平移量与两条曲线欧式距离的分布.fig']);
%close(3);
end
