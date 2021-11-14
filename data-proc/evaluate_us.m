%% evaluate US images
clc; clear; close all

% read data
date = '11-12-2021';
id1_imgs = dir(['../data/clarius_data/',date,'/ID1/*.JPEG']); 
id2_imgs = dir(['../data/clarius_data/',date,'/ID2/*.JPEG']); 
id3_imgs = dir(['../data/clarius_data/',date,'/ID3/*.JPEG']); 
id4_imgs = dir(['../data/clarius_data/',date,'/ID4/*.JPEG']);  

nfiles = length(id1_imgs); 
bscan = cell(4, nfiles);
for ii=1:nfiles
   curr_file = id1_imgs(ii).name;
   curr_img = imread(['../data/clarius_data/',date,'/ID1/',curr_file]);
   bscan{1,ii} = rgb2gray(curr_img);
end
for ii=1:nfiles
   curr_file = id2_imgs(ii).name;
   curr_img = imread(['../data/clarius_data/',date,'/ID2/',curr_file]);
   bscan{2,ii} = rgb2gray(curr_img);
end
for ii=1:nfiles
   curr_file = id3_imgs(ii).name;
   curr_img = imread(['../data/clarius_data/',date,'/ID3/',curr_file]);
   bscan{3,ii} = rgb2gray(curr_img);
end
for ii=1:nfiles
   curr_file = id4_imgs(ii).name;
   curr_img = imread(['../data/clarius_data/',date,'/ID4/',curr_file]);
   bscan{4,ii} = rgb2gray(curr_img);
end

% remove annotations
for id = 1:4
    for smpl = 1:nfiles
        curr_img = bscan{id,smpl};
        curr_img(260:285,240:265) = 0;  % remove clarius watermark
        curr_img = curr_img(240+1:end-140,70+1:end-70); % crop
        bscan{id,smpl} = curr_img;
    end
end

imagesc(curr_img); colormap gray
clear curr_file

%% find max intensity ROI
figure
roi = cell(4,5); bg = cell(4,5);
hori_neighbor = 20; vert_neighbor = 100;
for i = 1:size(bscan,1)*size(bscan,2)
    % binarize & limit FOV
    bscan_bw = imbinarize(bscan{i}, 0.8);
    bscan_bw(1:60,:) = 0; bscan_bw(:,[1:200,end-200:end]) = 0;
    % horizontal bounding
    hori_sum = sum(bscan_bw');
    [~,r] = sort(hori_sum,'descend'); 
    r(hori_sum(r)<0.5*max(hori_sum) | abs(r - r(1))>hori_neighbor) = [];
    % vertical bounding
    vert_sum = sum(bscan_bw(r,:));
    [~,c] = sort(vert_sum,'descend'); 
    c(vert_sum(c)<0.5*max(vert_sum) | abs(c - c(1))>vert_neighbor) = [];
    % store roi, bg & visualize
    imagesc(bscan{i}); colormap gray
    roi_x=min(c); roi_y=min(r); roi_w=max(c)-min(c); roi_h=max(r)-min(r);
    roi{i} = bscan{i}(roi_y:roi_y+roi_h, roi_x:roi_x+roi_w);
    bg{i} = bscan{i}(roi_y+2*roi_h:roi_y+4*roi_h, roi_x:roi_x+roi_w);
    rectangle('Position',[roi_x,roi_y,roi_w,roi_h],'EdgeColor','r','LineWidth',1)
    rectangle('Position',[roi_x,roi_y+2*roi_h,roi_w,roi_h],'EdgeColor','y','LineWidth',1)
    pause(0.15)
end

%% calculate CNR
% get ROI
CNR = cell(4,5);
for i = 1:size(bscan,1)*size(bscan,2)
    CNR{i} = abs(mean(roi{i},'all')-mean(bg{i},'all'))/ ...
        sqrt(var(double(roi{i}),1,'all')+var(double(bg{i}),1,'all'));
end

%% clutter based ROI
bw = imbinarize(bscan{10}, 0.9);
bw(1:60,:) = 0; % remove top reflection
[row,col] = find(bw == 1);
% figure
% imagesc(bscan{1})
% hold on
% plot(col, row, '.r')
% axis ij
rng(1); k = 4;
Label = kmeans([row, col],k,'MaxIter',800,'Start','uniform');
figure
imagesc(bscan{10}); colormap gray
hold on
for lb = 1:k
    plot(col(Label==lb),row(Label==lb),'.'); axis ij
end
legend()
