%*****************TASK 1****************
disp('TASK 1');
%2D VERTICES
vert2D_1 = [1374 1020; 1344 1137; 1373 1612; 2238 1005; 2313 1115; 2279 1589];
vert2D_2 = [1659 932; 1403 987; 1419 1407; 2205 1151; 1925 1232; 1913 1734; 2176 1618];
vert2D_3 = [1887 857; 1585 854; 1938 1149; 1539 1147; 1548 1657; 1920 1649];
vert2D_4 = [2317 976; 2076 908; 2287 1394; 1710 1198; 1460 1096; 1477 1567; 1714 1687];
vert2D_5 = [2297 1126; 2243 1005; 2260 1606; 1306 1118; 1357 997; 1341 1600];
vert2D_6 = [1746 1178; 2054 1101; 2044 1574;1760 1671; 1320 936; 1587 882; 1350 1345];
vert2D_7 = [1599 1185; 1984 1188; 1970 1679; 1614 1680; 1649 909; 1940 906];
vert2D_8 = [1460 1141; 1703 1236; 1700 1729; 1471 1609; 2052 968; 2297 1027; 2268 1447];

all2D = {vert2D_1, vert2D_2, vert2D_3, vert2D_4, vert2D_5, vert2D_6, vert2D_7, vert2D_8};

%3D VERTICES
vert3D = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0; 0.165 0.063 0];

vert3D_1 = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0];
vert3D_2 = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0; 0.165 0.063 0];
vert3D_3 = [0 0.063 0.093; 0 0 0.093; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0; 0.165 0.063 0];
vert3D_4 = [0 0.063 0.093; 0 0 0.093; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0; 0.165 0.063 0];
vert3D_5 = [0 0.063 0.093; 0 0 0.093; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0.063 0];
vert3D_6 = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0.063 0];
vert3D_7 = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093];
vert3D_8 = [0 0.063 0.093; 0 0 0.093; 0 0 0; 0 0.063 0; 0.165 0.063 0.093; 0.165 0 0.093; 0.165 0 0];

all3D = {vert3D_1, vert3D_2, vert3D_3, vert3D_4, vert3D_5, vert3D_6, vert3D_7, vert3D_8};

triangles3D = [1 5 4; 4 5 8; 5 7 8; 5 6 7; 1 6 5; 1 2 6; 1 3 2; 1 4 3; 4 8 7; 4 7 3; 6 3 7; 6 2 3];

%INTRINSIC CAMERA
Fx = 2960.37845;
Fy = Fx;
Cx = 1841.68855;
Cy = 1235.23369;
A = [ Fx 0 Cx; 0 Fy Cy; 0 0 1];

focalLength = [ Fx; Fy ];
principalPoint = [ Cx; Cy ];
imageSize = [ 2456; 3680 ];
intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

colors = {'k', 'g', 'b', 'y', 'm', 'c', 'w', 'r'};

figure(100)
plot3(vert3D(:,1),vert3D(:,2),vert3D(:,3), 'Marker','.', 'MarkerSize', 10, 'Color', 'r', 'LineStyle', 'none');
grid on
hold on
plot3(0,0,0,'g*');
hold on
axis equal

allSIFTfeatures3D = [];
allSIFTdescriptor = [];
for l = 1:8
    %ROTATION AND TRANSLATION FOR EACH IMAGE
    [R1c, T1c] = estimateWorldCameraPose(all2D{l}, all3D{l}, intrinsics, 'MaxReprojectionError', 3);

    %%INVERSE
    [R1, T1] = cameraPoseToExtrinsics(R1c, T1c);
    R1 = R1';
    
    %2D = A*[R1, T1]*3D     %VERIFY IF POINT FROM 3D TO 2D IS CORRECT
    
    plotCamera('Orientation', R1c, 'Location', T1c, 'Size', 0.03);
   	hold on

    %SIFT FEATURES
    imgSet=imageSet('images/init_texture/');
    I = read(imgSet,l);
    Itr = single(rgb2gray(I));
    [f, d] = vl_sift(Itr);
    siftsize = size(f);
    
    good_sift3D = [];
    good_sift2D = [];
    for j = 1:siftsize(2)
        %GET 3D POINT
        Rinv = inv(R1);
        tmp2 = inv(A)*[f(1:2,j);1];
        tmp = [tmp2 - T1'];
        M = inv(R1)*tmp;

        for i = 1:12
            vert0 = vert3D(triangles3D(i,1),:);
            vert1 = vert3D(triangles3D(i,2),:);
            vert2 = vert3D(triangles3D(i,3),:);
            [slope, bias] = giveLine(T1c', M);
            [flag, u, v, t] = rayTriangleIntersection(T1c', slope, vert0', vert1', vert2');
            
            if flag == 1
                n = surfaceNorm(vert0, vert1, vert2);
                if dot(slope, n)>0
                    intersection = T1c' + slope*t;
                    good_sift3D = [good_sift3D; intersection'];
                    good_sift2D = [good_sift2D; f(1:2,j)'];
                    
                    allSIFTfeatures3D = [allSIFTfeatures3D; intersection'];
                    allSIFTdescriptor = [allSIFTdescriptor, d(:,j)];
                end                
            end
        end
    end
    
    size3d = size(good_sift3D);
    for k = 1:size3d(1) 
        plot3(good_sift3D(k,1),good_sift3D(k,2),good_sift3D(k,3),'Color', colors{l}, 'Marker','.', 'MarkerSize', 4);
        hold on
    end
    
end

%***************TASK 2*************
disp('TASK 2');
detection_R = {};
detection_T = {};
all_bestin_2D = {};
all_bestin_3D = {};
sift_2D = {};
all_matches = {};
imgSet = imageSet('images/detection/');
for i = 1:imgSet.Count
    I = read(imgSet,i);
    Itr = single(rgb2gray(I));
    
    [feat_temp, d_temp] = vl_sift(Itr);
    [matches, scores] = vl_ubcmatch(allSIFTdescriptor, d_temp) ;
    
    [best_model, best_inl2D, best_inl3D] = ransac(1000, 400, feat_temp, allSIFTfeatures3D, matches, intrinsics, A);
    
    detection_R{i} = best_model(:,1:3);
    detection_T{i} = best_model(:,4);
    all_bestin_2D{i} = best_inl2D;
    all_bestin_3D{i} = best_inl3D;
    sift_2D{i} = feat_temp;
    all_matches{i} = matches;
end

%****************TASK 3*************%
disp('TASK 3');
for i = 1:imgSet.Count  
    I = read(imgSet,i);
    size_best = size(all_bestin_3D{i});
   
    [R_opt, T_opt] = Levenberg(all_bestin_3D{i}, all_bestin_2D{i}, detection_R{i}, detection_T{i}, A, 100);
    %[R_opt, T_opt] = IRLS(sift_2D{i}, allSIFTfeatures3D, all_matches{i}, detection_R{i}, detection_T{i}, A);     
    
    marks = [];
    marks_opt = [];
    for index = 1:8
        point_old = A*[detection_R{i}, detection_T{i}]*([vert3D(index,:)';1]);
        point_old = point_old/point_old(3);
        
        point_opt = A*[R_opt, T_opt]*([vert3D(index,:)';1]);
        point_opt = point_opt/point_opt(3);

        marks = [marks; point_old(1:2)'];
        marks_opt = [marks_opt; point_opt(1:2)'];
    end
    
    figure(i)
    hold on
    imshow(I)
    hold on
    plot(marks(:,1), marks(:,2), 'r*')
    hold on
    plot(marks_opt(:,1), marks_opt(:,2), 'b*')
    pause(2)
end

%***************TASK 4********************
if false
best_R = {};
best_T = {};
imgSet = imageSet('images/tracking/');

for i = 1:imgSet.Count
    I = read(imgSet,i);
    Itr = single(rgb2gray(I));
    
    [feat_temp, d_temp] = vl_sift(Itr);
    [matches, scores] = vl_ubcmatch(allSIFTdescriptor, d_temp) ;
    
    if i == 1
        [best_model, best_inl2D, best_inl3D] = ransac(1000, 400, feat_temp, allSIFTfeatures3D, matches, intrinsics, A);
        R_opt = best_model(:,1:3);
        T_opt = best_model(:,4);
    end

    [R_opt, T_opt] = IRLS(feat_temp, allSIFTfeatures3D, matches, R_opt, T_opt, A);
         
    marks = [];
    for index = 1:8
        point_old = A*[R_opt, T_opt]*([vert3D(index,:)';1]);
        point_old = point_old/point_old(3);
        
        marks = [marks; point_old(1:2)'];
    end
    
    figure(i)
    hold on
    imshow(I)
    hold on
    plot(marks(:,1), marks(:,2), 'b*')
    pause(2)
end
end


function [final_weights] = calc_w(resid, c)
    resid_size = size(resid);
    resid_x = resid(1:2:resid_size);
    resid_y = resid(2:2:resid_size);

    sigma_x = 1.48257968 * mad(resid_x);
    sigma_y = 1.48257968 * mad(resid_y);

    %Compute weights
    weights = [];
    for j = 1:resid_size
        if mod(j,2) == 0
            val = resid(j)/sigma_y;
        else
            val = resid(j)/sigma_x;
        end

        if abs(val) < c
            w = (1 - ((val^2)/(c^2)) )^2;
        else
            w = 0;
        end
        weights = [weights; w];
    end
    
    for j = 1:2:resid_size 
        if weights(j) == 0 || weights(j+1) == 0
            weights(j) = 0;
            weights(j+1) = 0;
        end
    end
    
    final_weights = diag(weights);       
end

function [energy_rob] = calc_ro(e,c)
    if e<=c
        d=(e/c)^2;
        s1=(1-d)^3;
        s2=(1-s1);
        d2=(c^2 / 6);
        energy_rob = d2 * s2;
    else
        energy_rob = (c^2 / 6);
    end
end

function [Jac] = jacobian(point3D, R, Rexp, T, A)
    Jac = [];
    sizePoints = size(point3D);
    
    for j = 1:sizePoints(1)
        current3Dpoint = point3D(j,:);
        proj2D = A*[R, T]*[current3Dpoint,1]';

        U = proj2D(1);
        V = proj2D(2);
        W = proj2D(3);
        der1 = [1/W  0  -U/(W^2); 0  1/W  -V/(W^2)];

        der2a = derivRodr(Rexp, R, 1)*current3Dpoint';
        der2b = derivRodr(Rexp, R, 2)*current3Dpoint';
        der2c = derivRodr(Rexp, R, 3)*current3Dpoint';

        Jtmp = der1*A*[der2a der2b der2c eye(3)];
        Jac = [Jac; Jtmp];
    end
end

function [err] = energy(point2D, point3D, R, T, A)
    sizeP = size(point2D);

    err = [];
    for i = 1:sizeP(1)
        proj2D = A*[R, T]*[point3D(i,:),1]';
        proj2D = proj2D/proj2D(3);

        err = [err; ( (point2D(i,1) - proj2D(1))^2 + (point2D(i,2) - proj2D(2))^2)];
    end
end

function [err] = residual(point2D, point3D, R, T, A)
    sizeP = size(point2D);

    err = [];
    for i = 1:sizeP(1)
        proj2D = A*[R, T]*[point3D(i,:),1]';
        proj2D = proj2D/proj2D(3);
        
        err = [err; ( proj2D(1) - point2D(i,1) ); ( proj2D(2) - point2D(i,2) )];
    end
end

function [val] = derivRodr(v, R, i)
    basis = [0;0;0];
    basis(i) = basis(i) + 1;
    
    skew = getSkewMatrix(v);
    
    tmp = cross(v, ((eye(3)-R)*basis));
    val = (v(i) * skew + getSkewMatrix(tmp))*R;
    val = val / (norm(v)^2);
end

function [rodr] = getSkewMatrix(v)
    rodr = [0, -v(3), v(2); v(3), 0, -v(1); -v(2) v(1) 0];
end

function [Rnew, Tnew] = IRLS(point2D, point3D, matches, R, T, A)
    c = 4.685;
    threshold = 0.001;
    u = threshold + 1;
    lambda = 0.001;
    
    point3D = point3D(matches(1,:),:);
    point2D = point2D(1:2,matches(2,:))';

    Rnew = R;
    Tnew = T;
    Rexp = rotationMatrixToVector(R);
    Pexp = [Rexp, Tnew'];
    for i = 1:10
        if u <= threshold
            break
        end
        
        resid = residual(point2D, point3D, Rnew, Tnew, A);
        final_weights = calc_w(resid, c);
        
        e = transpose(resid)*resid;
        e_rob = calc_ro(e,c);
        
        Jac = jacobian(point3D, Rnew, Pexp(:, 1:3), Tnew, A);
        delta = ( (Jac'*final_weights*Jac + lambda*eye(6) ) \ (Jac'*final_weights*resid))';
        
        P_temp = Pexp - delta;
        new_en = energy(point2D, point3D, rotationVectorToMatrix(P_temp(1:3)), P_temp(4:6)', A);

        if abs(sum(new_en)) > abs(sum(e_rob))
            lambda = 10 * lambda;
        else
            lambda = lambda / 10;
            Pexp = P_temp;
            Rnew = rotationVectorToMatrix(Pexp(1:3));
            Tnew = Pexp(4:6)';
        end           
        u = norm(delta);
    end
end

function [Rnew, Tnew, delta] = Levenberg(sift3D, sift2D, R, T, A, iters)
    lambda = 0.001;
    threshold = 0.001;
    u = threshold + 1;
    
    match3D = sift3D;
    match2D = sift2D;

    Rexp = rotationMatrixToVector(R);
    res = [Rexp, T'];
    Rnew = R;
    Tnew = T;
    for i = 1:iters
        if u <= threshold
            break
        end
        
        Jac = jacobian(match3D, Rnew, res(1:3), Tnew, A);
        e = energy(match2D, match3D, Rnew, Tnew, A);
        resid = residual(match2D, match3D, Rnew, Tnew, A);
        
        delta = ( (Jac'*Jac + lambda*eye(6) ) \ (Jac'*resid))';
        P_temp = res - delta;
       
        e_new = energy(match2D, match3D, rotationVectorToMatrix(P_temp(1:3)), P_temp(4:6)', A);
        
        if abs(sum(e_new)) > abs(sum(e))
            lambda = 10 * lambda;
        else
            lambda = lambda / 10;
            res = P_temp;
            Rnew = rotationVectorToMatrix(res(1:3));
            Tnew = res(4:6)';
        end
        
        u = norm(delta);
    end
end

function [best_model, best_inl2D, best_inl3D] = ransac(iter, th, sift2D, sift3D, matches, intrinsics, A)
    best_model = [];
    best_inl2D = [];
    best_inl3D = [];
    best_inls = 0;
    matchSize = size(matches);
    
    for r = 1:iter
        rand_indices = randi([1 matchSize(2)],1,4);
        randMatches = matches(:,rand_indices);
        
        chosen3D = sift3D(randMatches(1,:),:);
        chosen2D = sift2D(1:2,randMatches(2,:))';
        
        try 
           [Rc, Tc] = estimateWorldCameraPose(chosen2D, chosen3D, intrinsics, 'MaxReprojectionError', 1e302);
        catch
           continue  %IN CASE I COMPUTE WITH ALL COMPLETELY FAR OUTLIERS
        end
        
        [R, T] = cameraPoseToExtrinsics(Rc, Tc);
        R = R';
        
        %REPROJECTION ERROR
        points_inside = 0;

        currentInl2D = [];
        currentInl3D = [];
        for b = 1:matchSize(2)
            point2D = A*[R, T']*[sift3D(matches(1,b), :)';1];
            point2D = point2D/point2D(3);
            
            realPoint2D = sift2D(1:2,matches(2,b));
            error = ( (point2D(1) - realPoint2D(1))^2 + (point2D(2) - realPoint2D(2))^2 );
            
            if error < th
                points_inside = points_inside + 1;
                
                currentInl2D = [currentInl2D; sift2D(1:2,matches(2,b))'];
                currentInl3D = [currentInl3D; sift3D(matches(1,b), :)];
            end        
        end  
        
        if points_inside > best_inls
            best_model = [R, T'];
            best_inl2D = currentInl2D;
            best_inl3D = currentInl3D;
            best_inls = points_inside;
        end
    end
end 

function [flag, u, v, t] = rayTriangleIntersection(o, d, p0, p1, p2)
    epsilon = 0.00001;

    e1 = p1-p0;
    e2 = p2-p0;
    q  = cross(d,e2);
    a  = dot(e1,q); % determinant of the matrix M

    if (a>-epsilon && a<epsilon) 
        % the vector is parallel to the plane (the intersection is at infinity)
        [flag, u, v, t] = deal(0,0,0,0);
        return;
    end
    
    f = 1/a;
    s = o-p0;
    u = f*dot(s,q);
    
    if (u<0.0)
        % the intersection is outside of the triangle
        [flag, u, v, t] = deal(0,0,0,0);
        return;          
    end
    
    r = cross(s,e1);
    v = f*dot(d,r);
    
    if (v<0.0 || u+v>1.0)
        % the intersection is outside of the triangle
        [flag, u, v, t] = deal(0,0,0,0);
        return;
    end
    
    t = f*dot(e2,r); % verified! 
    flag = 1;
    return;
end

function n = surfaceNorm(x1, x2, x3)
    V = x3 - x1;
    U = x2 - x1;
    
    Nx = U(2)*V(3) - U(3)*V(2);
    Ny = U(3)*V(1) - U(1)*V(3);
    Nz = U(1)*V(2) - U(2)*V(1);
    
    n = [Nx; Ny; Nz];
end

function [s, b] = giveLine(P1, P2)
    s = P1 - P2;
    b = P2;
end