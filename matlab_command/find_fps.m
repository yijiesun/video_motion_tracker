function find_fps()
global video minX videoNum videodata minD;
hwait=waitbar(0,'请等待>>>>>>>>');
fps=30;
dyn=0;
fps_arry=zeros(200,3);
for fps_dyn=fps-1:0.01:fps+1
dyn=dyn+1;
   str=['正在运行中',num2str(dyn),'/400'];
   waitbar(dyn/400,hwait,str); 

for i=1:videoNum
%video(i,1)=(videodata(i,1)+0.1-1)/minD(1);%if use 10 piece SIFT
video(i,1)=videodata(i,1)/fps_dyn;
video(i,2)=videodata(i,2)*140;
end

%%%%%%%%%%%%second loop%%%%%%%%%%%%
find_offset();
%%%%%%%%%%%%second loop%%%%%%%%%%%%

%fps_arry:fps value,offset
fps_arry(dyn,1)=fps_dyn;%fps
fps_arry(dyn,2)=minX(1);%offset
fps_arry(dyn,3)=minX(2);%distance
end%main loop

%find smallest offset with deferent fps
minD(1)=fps_arry(1,1);%fps
minD(2)=fps_arry(1,2);%offset
minD(3)=fps_arry(1,3);%distance
for p=2:200
    str=['正在运行中',num2str(p+200),'/400'];
    waitbar((p+200)/400,hwait,str);
if(abs(minD(3))>=abs(fps_arry(p,3)))
minD(1)=fps_arry(p,1);
minD(2)=fps_arry(p,2);
minD(3)=fps_arry(p,3);
end
end
close(hwait);
end