% ============================================ 
% file name: validate_densepose.m
% description: validate densepose
% author: Xihan Ma
% ============================================ 
%% prepare data
clc; clear; close all

% read raw data (xlsread only works on win)
[~, mk_data_raw] = xlsread('../data/marker_pos_log.csv');
[~, dp_data_raw] = xlsread('../data/dp_target_log.csv');

% extract number from str
mk_data.pix = zeros(size(mk_data_raw,1),2,4);
mk_data.xyz = zeros(size(mk_data_raw,1),3,4);
dp_data.pix = zeros(size(dp_data_raw,1),2,8);
dp_data.xyz = zeros(size(dp_data_raw,1),3,8);

% marker
for row = 1:size(mk_data_raw,1)
    for col = 1:size(mk_data_raw,2)/2
        data_arr = textscan(mk_data_raw{row,col}, '(%f, %f)');
        mk_data.pix(row,:,col) = [data_arr{1:2}];
    end
end
for row = 1:size(mk_data_raw,1)
    for col = size(mk_data_raw,2)/2+1:size(mk_data_raw,2)
        data_arr = textscan(mk_data_raw{row,col}, '(%f, %f, %f)');
        mk_data.xyz(row,:,col-size(mk_data_raw,2)/2) = [data_arr{1:3}];
    end
end
% densepose targets
for row = 1:size(dp_data_raw,1)
    for col = 1:size(dp_data_raw,2)/2
        data_arr = textscan(dp_data_raw{row,col}, '(%f, %f)');
        dp_data.pix(row,:,col) = [data_arr{1:2}];
    end
end
for row = 1:size(dp_data_raw,1)
    for col = size(dp_data_raw,2)/2+1:size(dp_data_raw,2)
        data_arr = textscan(dp_data_raw{row,col}, '(%f, %f, %f)');
        dp_data.xyz(row,:,col-size(dp_data_raw,2)/2) = [data_arr{1:3}];
    end
end
clear row col data_arr

%% calculate densepose error
%TODO: remove bad data
mk_pix_avg = round(mean(mk_data.pix(10:end,:,:),1)); mk_pix_avg = reshape(mk_pix_avg,2,4)';
mk_xyz_avg = mean(mk_data.xyz(10:end,:,:),1); mk_xyz_avg = reshape(mk_xyz_avg,3,4)';
nFrms = 50; frmOffset = 40;

reg1_pix_err = zeros(nFrms,1);
reg2_pix_err = zeros(nFrms,1);
reg3_pix_err = zeros(nFrms,1);
reg4_pix_err = zeros(nFrms,1);

reg1_xyz_err = zeros(nFrms,1);
reg2_xyz_err = zeros(nFrms,1);
reg3_xyz_err = zeros(nFrms,1);
reg4_xyz_err = zeros(nFrms,1);

for i = frmOffset:frmOffset+nFrms-1
    % pix err
    reg1_pix_err(i-frmOffset+1) = norm(dp_data.pix(i,:,4) - mk_pix_avg(1,:));
    reg2_pix_err(i-frmOffset+1) = norm(dp_data.pix(i,:,2) - mk_pix_avg(2,:));
    reg3_pix_err(i-frmOffset+1) = norm(dp_data.pix(i,:,3) - mk_pix_avg(3,:));
    reg4_pix_err(i-frmOffset+1) = norm(dp_data.pix(i,:,1) - mk_pix_avg(4,:));
    % xyz err (not taking z into consideration)
    reg1_xyz_err(i-frmOffset+1) = norm(dp_data.xyz(i,1:2,4) - mk_xyz_avg(1,1:2));
    reg2_xyz_err(i-frmOffset+1) = norm(dp_data.xyz(i,1:2,2) - mk_xyz_avg(2,1:2));
    reg3_xyz_err(i-frmOffset+1) = norm(dp_data.xyz(i,1:2,3) - mk_xyz_avg(3,1:2));
    reg4_xyz_err(i-frmOffset+1) = norm(dp_data.xyz(i,1:2,1) - mk_xyz_avg(4,1:2));
end

%% visualize densepose err
figure('Position',[1920/3,1080/3,600,450])        % x, y, w, h
tiledlayout(1,2)
ax1 = nexttile;
boxchart([reg1_pix_err,reg2_pix_err,reg3_pix_err,reg4_pix_err],'BoxFaceColor','#77AC30');
xlabel(ax1,'target ID')
ylim([0, 30]); 
ylabel(ax1,'error in image [pix]')
ax1.YGrid = 'on';
box on
ax2 = nexttile;
boxchart([reg1_xyz_err,reg2_xyz_err,reg3_xyz_err,reg4_xyz_err].*100)
xlabel(ax2,'target ID')
ylim([0, 5]);
ylabel(ax2,'error in xy [cm]')
ax2.YGrid = 'on';
box on

%%
% rgb = imread('../data/mk_frame.png');
rgb = imread('../data/color_frame.png');
figure('Position',[1920/3,1080/3,640,640])
imagesc(rgb); axis on
hold on
scatter(mk_pix_avg(1,1),mk_pix_avg(1,2),50,[0 0.4470 0.7410],'d','filled')
scatter(mk_pix_avg(2,1),mk_pix_avg(2,2),50,[0.8500 0.3250 0.0980],'d','filled')
scatter(mk_pix_avg(3,1),mk_pix_avg(3,2),50,[0.9290 0.6940 0.1250],'d','filled')
scatter(mk_pix_avg(4,1),mk_pix_avg(4,2),50,[0.4940 0.1840 0.5560],'d','filled')

scatter(dp_data.pix(:,1,4),dp_data.pix(:,2,4),60,[0.2 0.6470 0.5410],'.','LineWidth',.5)
scatter(dp_data.pix(:,1,2),dp_data.pix(:,2,2),60,[0.8500 0.5250 0.1880],'.','LineWidth',.5)
scatter(dp_data.pix(:,1,3),dp_data.pix(:,2,3),60,[0.7290 0.8940 0.2150],'.','LineWidth',.5)
scatter(dp_data.pix(:,1,1),dp_data.pix(:,2,1),60,[0.4940 0.3840 0.5560],'.','LineWidth',.5)

legend('target1 ref','target2 ref','target3 ref','target4 ref','target1','target2','target3','target4')
