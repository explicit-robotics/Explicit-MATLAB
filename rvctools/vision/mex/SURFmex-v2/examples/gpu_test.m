%SURFmex example function
%   Petter Strandmark 2008
%   petter.strandmark@gmail,com  
clc
close all

%% Create images
I = rgb2gray(imread('peppers.png'));
I2 = double(I) + 15*randn(size(I));
I2 = max(0,I2);
I2 = min(255,I2);
I2 = uint8(I2);

%% Calculate interest points

clear options
options.verbose = 1;

for iter = 1:50
    tic
    [points descr] = surfpoints_gpu(I);
    fprintf('Time taken to calculate %d points: %.5f seconds\n',size(points,2),toc);
    tic
    [tmp1 tmp2] = surfpoints(I);
    fprintf('Time taken to calculate %d points in OpenCV: %.5f seconds\n',size(tmp1,2),toc);
    tic
    [points2 descr2] = surfpoints_gpu(I2);
    fprintf('Time taken to calculate %d points: %.5f seconds\n',size(points2,2),toc);
    tic
    [tmp1 tmp2] = surfpoints(I2);
    fprintf('Time taken to calculate %d points in OpenCV: %.5f seconds\n',size(tmp1,2),toc);


    thresh = 0.7;
    tic
    matches = surfmatch(descr,descr2,thresh);
    fprintf('Time taken to match points: %.3f seconds\n',toc);

end
%% Show
figure(1);
imshow(I);
hold on
figure(2)
imshow(I2);
hold on

figure(1);
plot(points(1,:), points(2,:), 'b+');
for i = 1:size(matches,2)
    %Distance between correspondences
    d = sum( (points(:,matches(1,i)) - points2(:,matches(2,i))).^2 );
    if d < 1
        %Inlier
        plot(points(1,matches(1,i)), points(2,matches(1,i)), 'g+');
    else
        %Outlier
        plot(points(1,matches(1,i)), points(2,matches(1,i)), 'r+');
    end
end

figure(2);
plot(points(1,:), points(2,:), 'b+');
for i = 1:size(matches,2)
    %Distance between correspondences
    d = sum( (points(:,matches(1,i)) - points2(:,matches(2,i))).^2 );
    if d < 1
        %Inlier
        plot(points2(1,matches(2,i)), points2(2,matches(2,i)), 'g+');
    else
        %Outlier
        plot(points2(1,matches(2,i)), points2(2,matches(2,i)), 'r+');
    end
end

