function find_offset()

global gyro video minX gyroNum videoNum interpoint;
tmp=0;
res=zeros(200,2);
for offset=-1:0.01:1%second loop
tmp=tmp+1;

gyroadd=zeros(gyroNum);
for i=1:gyroNum
gyroadd(i)=gyro(i,1)+offset;%将gyro向右平移
end

%防止定义域外插值
index=0;
interpoint=[];
for i=1:gyroNum
if(gyroadd(i)>video(1,1)&&gyroadd(i)<video(videoNum,1))
    index=index+1;
    interpoint(index,1)=gyroadd(i);
    interpoint(index,2)=gyro(i,2);
end
end
videointer=interp1(video(:,1),video(:,2),interpoint(:,1),'spline'); %interpolation 

%result=0;
%interpoint_size=size(interpoint,1);
%for i=1:interpoint_size%claculator euclidean distance of two curves
%result=result+abs(interpoint(i,2)-videointer(i));
%end
%res(tmp,1)=offset;
%res(tmp,2)=result/interpoint_size;

%求相关系数，或者用上面注释的求欧式距离，都可以计算曲线之间的相似度
a=zeros(2,2);
a=corrcoef(interpoint(:,2),videointer);
res(tmp,1)=offset;
res(tmp,2)=1-a(1,2);

end%second loop

minX(1)=res(1,1);%offset
minX(2)=res(1,2);%Euclidean distance
for p=2:200%find smallest Euclidean distance of two curves
if(abs(minX(2))>=abs(res(p,2)))
    minX(1)=res(p,1);
    minX(2)=res(p,2);
end
end

end%function