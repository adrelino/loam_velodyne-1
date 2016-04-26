function r = loam()
velo_1 = csvread('/home/sebastian/Git/rosbuild_ws/loam/loam_velodyne/input_130.csv.csv');
velo_2 = csvread('/home/sebastian/Git/rosbuild_ws/loam/loam_velodyne/input_131.csv.csv');
T = getGT();
T_t = getVeloToCam()
diff = getDiff(T{2},T{3})
velo_1 = transformPC(velo_1',T_t);
velo_1 = velo_1(1:3,:)';
%pcshow(v1(1:3,:)','r')
%hold on;
%ptCloudOut = pctransform(pointCloud(velo_2),affine3d([diff(1:3,1:3) zeros(3,1); [0 0 0 1]]'));
v2 = transformPC(velo_2',T_t);
%velo_2 = transformPC(v2(1:3,:),diff);
velo_2 = v2(1:3,:);
velo_2 = velo_2(1:3,:)';
%pcshow(back(1:3,:)','b')




%%
%s=computeSmoothness(velo_1)
%%

theta = [0;0;0;0;0;0];
lambda = 0.001;
point_k1 = velo_2;
for j = 1:10
    
    R_temp = getR(theta(4),theta(5),theta(6));
    t_temp = [theta(1);theta(2);theta(3)];
    for i = 1:length(velo_2)
        plane_temp(i,:) = R_temp*velo_2(i,:)' +t_temp;
    end
    
    
    
    [normal_k,point_k,point_k1,point_temp] = computePoints(velo_1,plane_temp,velo_2,100);
    length(normal_k)
    
    for i = 1:length(normal_k)
        J(i,:) = getPlaneJacobi([0 0 0 0 0 0],normal_k{i}',point_k{i}',point_k1{i}');
        d(i,1) = getPlaneDistance2(normal_k{i},point_k{i}',point_k1{i}');
        w(i) = norm(point_k1{i});
    end
    %W=diag(w/sum(w));
    W=diag(ones(length(normal_k),1));
    [sum(d) theta']
    cov(d)
    up = pinv(J'*J+lambda*diag(diag(J'*W*J))) * J' * d;
    
    theta = theta - up;
    
end

clf;
pcshow(velo_1,'r');
hold on;
pcshow(plane_temp,'b')
for i=1:length(normal_k)
    quiver3(point_k{i}(1),point_k{i}(2),point_k{i}(3),normal_k{i}(1),normal_k{i}(2),normal_k{i}(3));
end

R_temp = getR(theta(4),theta(5),theta(6));
t_temp = [theta(1);theta(2);theta(3)];
-pinv(R_temp)*t_temp

%
%         error = velo_2(i,:) - candidates(1,:);
%         angle(i) = atan2(norm(cross(x,error)), dot(x,error));
%         error_n(i) = norm(error);
%error_n

end

function [normal_k,point_k,point_k1,point_temp] = computePoints(velo_1,velo_2,fortemp,n_max)

n = length(velo_2);
cubes = 1;
i = 1;
maxdir = [0 0 0];
j=1;
for k=1:n
    if cubes < 9 && i<n && j<n_max
        candidates = getNearestPoints(velo_1,velo_2(i,:),15);
        if ~isempty(candidates)
            [C,m] = computeCovariance(candidates);
            [V,D] = eig(C);
            if D(3,3) > 3*D(1,1) && D(2,2) > 3*D(1,1) && abs(D(3,3) - D(2,2)) < 0.3
               % [v,idx] = max(abs(V(1:3,1)));
               % if maxdir(idx) < (n_max/3)-1
                    normal_k{j} = V(1:3,1);
                    point_k{j} = m;
                    point_k1{j} = velo_2(i,:);
                    point_temp{j} = fortemp(i,:);
                    j = j + 1;
                %    maxdir(idx) = maxdir(idx) + 1;
                %    maxdir
               % end
            end
        end
    else
        break
    end
   % if j>(n_max/8 * cubes)
   %     i = (n/8) * cubes;
    %    cubes = cubes + 1;
    %end
    %i = i + randi(1000);
    i = i + 100;
    n - i; 
end
end

function s = computeSmoothness(points)
j=1
for i = 6 : 1000 %length(points) - 5
    
    temp = sum([points(i-5:i-1,:);points(i+1:i+6,:); - 10 * points(i,:)]);
    
    s(j,1) = norm(temp)^2;
    j = j +1
end
end

function [C,m] = computeCovariance(points)
m = mean(points);
m_points = bsxfun(@minus,points,m);

C = zeros(3,3);
for i=1:length(points)
    C(1,1) = C(1,1) + m_points(i,1) * m_points(i,1);
    C(1,2) = C(1,2) + m_points(i,1) * m_points(i,2);
    C(1,3) = C(1,3) + m_points(i,1) * m_points(i,3);
    C(2,1) = C(1,2);
    C(2,2) = C(2,2) + m_points(i,2) * m_points(i,2);
    C(2,3) = C(2,3) + m_points(i,2) * m_points(i,3);
    C(3,1) = C(1,3);
    C(3,2) = C(2,3);
    C(3,3) = C(3,3)+ m_points(i,3) * m_points(i,3);
end
C = C./length(points);

end

function r = getNearestPoints(velo,query,n)
MdlKDT = KDTreeSearcher(velo);
r = 1.05; % Search radius
IdxKDT = rangesearch(MdlKDT,query,r);
if length(IdxKDT{1})>=n
    r = velo(IdxKDT{1}(1:n),:);
else
    r = [];
end
end

function T = getGT()
gt=readGT('/home/sebastian/Dropbox/KITTI/poses/00.txt');
for i=1:100
    T{i} = [reshape(gt(i,:),[],3)'; 0 0 0 1];
end
end

function T = getDiff(T_1,T_2)
T = pinv(T_1) * T_2
end

function r = transformPC(pc,T)
r = zeros(4,length(pc));
for i=1:length(pc)
    r(:,i) = T*[pc(:,i);1];
end
end 

function T = getVeloToCam()
R_vel = [7.967514e-03, -9.999679e-01, -8.462264e-04; -2.771053e-03, 8.241710e-04, -9.999958e-01; 9.999644e-01, 7.969825e-03, -2.764397e-03];
t_vel = [-1.377769e-02; -5.542117e-02; -2.918589e-01];
T = [R_vel t_vel;0 0 0 1];
end