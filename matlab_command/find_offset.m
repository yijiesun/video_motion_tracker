function find_offset()

global gyro video minX gyroNum videoNum interpoint;
tmp=0;
res=zeros(200,2);
for offset=-1:0.01:1
tmp=tmp+1;

gyroadd=zeros(gyroNum);
for i=1:gyroNum
gyroadd(i)=gyro(i)+offset;
end

index=0;

for i=1:gyroNum
if(gyroadd(i)>video(1,1)&&gyroadd(i)<video(videoNum,1))
    index=index+1;
    interpoint(index,1)=gyroadd(i);
    interpoint(index,2)=gyro(i,2);
end
end
videointer=interp1(video(:,1),video(:,2),interpoint(:,1),'spline'); %interpolation 

result=0;

interpoint_size=size(interpoint,1);
for i=1:interpoint_size%claculator euclidean distance of two curves
result=result+(interpoint(i,2)-videointer(i))^2;
end
res(tmp,1)=offset;
res(tmp,2)=sqrt(result/interpoint_size);

end%second loop


minX(1)=res(1,1);%offset
minX(2)=res(1,2);%Euclidean distance
for p=2:200%find smallest Euclidean distance of two curves
if(abs(minX(2))>=abs(res(p,2)))
    minX(1)=res(p,1);
    minX(2)=res(p,2);
end

end%function
end