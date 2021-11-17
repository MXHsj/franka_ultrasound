% ============================================ 
% file name: validate_surface_normal
% description: validate patch-based surface normal calculation
% author: Xihan Ma
% ============================================ 
%% read data
clc; clear; close all;

dir1 = '../data/surface_normal/11-15-2021-reg1-surf_norm.csv';
dir2 = '../data/surface_normal/11-10-2021-reg2-surf_norm.csv';
dir3 = '../data/surface_normal/11-15-2021-reg7-surf_norm.csv';
dir4 = '../data/surface_normal/11-15-2021-reg8-surf_norm.csv';
dir5 = '../data/surface_normal/11-10-2021-reg3-surf_norm.csv';
dir6 = '../data/surface_normal/11-15-2021-reg4-surf_norm.csv';
dir7 = '../data/surface_normal/11-15-2021-reg9-surf_norm.csv';
dir8 = '../data/surface_normal/11-15-2021-reg10-surf_norm.csv';
dir9 = '../data/surface_normal/11-10-2021-reg5-surf_norm.csv';
dir10 = '../data/surface_normal/11-10-2021-reg6-surf_norm.csv';
dir11 = '../data/surface_normal/11-10-2021-reg11-surf_norm.csv';
dir12 = '../data/surface_normal/11-10-2021-reg12-surf_norm.csv';


data1_raw = csvread(dir1); data1 = removeBadData(data1_raw);
data2_raw = csvread(dir2); data2 = removeBadData(data2_raw);
data3_raw = csvread(dir3); data3 = removeBadData(data3_raw);
data4_raw = csvread(dir4); data4 = removeBadData(data4_raw);
data5_raw = csvread(dir5); data5 = removeBadData(data5_raw);
data6_raw = csvread(dir6); data6 = removeBadData(data6_raw);
data7_raw = csvread(dir7); data7 = removeBadData(data7_raw);
data8_raw = csvread(dir8); data8 = removeBadData(data8_raw);
data9_raw = csvread(dir9); data9 = removeBadData(data9_raw);
data10_raw = csvread(dir10); data10 = removeBadData(data10_raw);
data11_raw = csvread(dir11); data11 = removeBadData(data11_raw);
data12_raw = csvread(dir12); data12 = removeBadData(data12_raw);

nKept = 100;
data1 = data1(1:nKept*3,:); 
data2 = data2(1:nKept*3,:);
data3 = data3(1:nKept*3,:);
data4 = data4(1:nKept*3,:);
data5 = data5(1:nKept*3,:);
data6 = data6(1:nKept*3,:);
data7 = data7(1:nKept*3,:);
data8 = data8(1:nKept*3,:);
data9 = data9(1:nKept*3,:);
data10 = data10(1:nKept*3,:);
data11 = data11(1:nKept*3,:);
data12 = data12(1:nKept*3,:);

% calculate errors
% rotation
est_vec1 = [data1(1:3:end-2,10),data1(2:3:end-1,10),data1(3:3:end,10)];
gt_vec1 = [(data1(1:3:end-2,11)),data1(2:3:end-1,11),data1(3:3:end,11)];
rot_err1 = angleBetweenVectors(est_vec1,gt_vec1);

est_vec2 = [data2(1:3:end-2,10),data2(2:3:end-1,10),data2(3:3:end,10)];
gt_vec2 = [data2(1:3:end-2,11),data2(2:3:end-1,11),data2(3:3:end,11)];
rot_err2 = angleBetweenVectors(est_vec2,gt_vec2);

est_vec3 = [data3(1:3:end-2,10),data3(2:3:end-1,10),data3(3:3:end,10)];
gt_vec3 = [data3(1:3:end-2,11),data3(2:3:end-1,11),data3(3:3:end,11)];
rot_err3 = angleBetweenVectors(est_vec3,gt_vec3);

est_vec4 = [data4(1:3:end-2,10),data4(2:3:end-1,10),data4(3:3:end,10)];
gt_vec4 = [data4(1:3:end-2,11),data4(2:3:end-1,11),data4(3:3:end,11)];
rot_err4 = angleBetweenVectors(est_vec4,gt_vec4);

est_vec5 = [data5(1:3:end-2,10),data5(2:3:end-1,10),data5(3:3:end,10)];
gt_vec5 = [data5(1:3:end-2,11),data5(2:3:end-1,11),data5(3:3:end,11)];
rot_err5 = angleBetweenVectors(est_vec5,gt_vec5);

est_vec6 = [data6(1:3:end-2,10),data6(2:3:end-1,10),data6(3:3:end,10)];
gt_vec6 = [data6(1:3:end-2,11),data6(2:3:end-1,11),data6(3:3:end,11)];
rot_err6 = angleBetweenVectors(est_vec6,gt_vec6);

est_vec7 = [data7(1:3:end-2,10),data7(2:3:end-1,10),data7(3:3:end,10)];
gt_vec7 = [data7(1:3:end-2,11),data7(2:3:end-1,11),data7(3:3:end,11)];
rot_err7 = angleBetweenVectors(est_vec7,gt_vec7);

est_vec8 = [data8(1:3:end-2,10),data8(2:3:end-1,10),data8(3:3:end,10)];
gt_vec8 = [data8(1:3:end-2,11),data8(2:3:end-1,11),data8(3:3:end,11)];
rot_err8 = angleBetweenVectors(est_vec8,gt_vec8);

est_vec9 = [data9(1:3:end-2,10),data9(2:3:end-1,10),data9(3:3:end,10)];
gt_vec9 = [data9(1:3:end-2,11),data9(2:3:end-1,11),data9(3:3:end,11)];
rot_err9 = angleBetweenVectors(est_vec9,gt_vec9);

est_vec10 = [data10(1:3:end-2,10),data10(2:3:end-1,10),data10(3:3:end,10)];
gt_vec10 = [data10(1:3:end-2,11),data10(2:3:end-1,11),data10(3:3:end,11)];
rot_err10 = angleBetweenVectors(est_vec10,gt_vec10);

est_vec11 = [data11(1:3:end-2,10),data11(2:3:end-1,10),data11(3:3:end,10)];
gt_vec11 = [data11(1:3:end-2,11),data11(2:3:end-1,11),data11(3:3:end,11)];
rot_err11 = angleBetweenVectors(est_vec11,gt_vec11);

est_vec12 = [data12(1:3:end-2,10),data12(2:3:end-1,10),data12(3:3:end,10)];
gt_vec12 = [data12(1:3:end-2,11),data12(2:3:end-1,11),data12(3:3:end,11)];
rot_err12 = angleBetweenVectors(est_vec12,gt_vec12);

rot_errs = [rot_err1,rot_err2,rot_err3,rot_err4,rot_err5,rot_err6, ...
            rot_err7,rot_err8,rot_err9,rot_err10,rot_err11,rot_err12];
        
% translation
trans_err1 = sqrt((data1(1:3:end-2,12)-data1(1:3:end-2,1)).^2+ ...
                  (data1(2:3:end-1,12)-data1(2:3:end-1,1)).^2+ ...
                  (data1(3:3:end,12)-data1(3:3:end,1)).^2).*1000;

trans_err2 = sqrt((data2(1:3:end-2,12)-data2(1:3:end-2,1)).^2+ ...
                  (data2(2:3:end-1,12)-data2(2:3:end-1,1)).^2+ ...
                  (data2(3:3:end,12)-data2(3:3:end,1)).^2).*1000;
              
trans_err3 = sqrt((data3(1:3:end-2,12)-data3(1:3:end-2,1)).^2+ ...
                  (data3(2:3:end-1,12)-data3(2:3:end-1,1)).^2+ ...
                  (data3(3:3:end,12)-data3(3:3:end,1)).^2).*1000;
              
trans_err4 = sqrt((data4(1:3:end-2,12)-data4(1:3:end-2,1)).^2+ ...
                  (data4(2:3:end-1,12)-data4(2:3:end-1,1)).^2+ ...
                  (data4(3:3:end,12)-data4(3:3:end,1)).^2).*1000;
              
trans_err5 = sqrt((data5(1:3:end-2,12)-data5(1:3:end-2,1)).^2+ ...
                  (data5(2:3:end-1,12)-data5(2:3:end-1,1)).^2+ ...
                  (data5(3:3:end,12)-data5(3:3:end,1)).^2).*1000;
              
trans_err6 = sqrt((data6(1:3:end-2,12)-data6(1:3:end-2,1)).^2+ ...
                  (data6(2:3:end-1,12)-data6(2:3:end-1,1)).^2+ ...
                  (data6(3:3:end,12)-data6(3:3:end,1)).^2).*1000;
              
trans_err7 = sqrt((data7(1:3:end-2,12)-data7(1:3:end-2,1)).^2+ ...
                  (data7(2:3:end-1,12)-data7(2:3:end-1,1)).^2+ ...
                  (data7(3:3:end,12)-data7(3:3:end,1)).^2).*1000;
              
trans_err8 = sqrt((data8(1:3:end-2,12)-data8(1:3:end-2,1)).^2+ ...
                  (data8(2:3:end-1,12)-data8(2:3:end-1,1)).^2+ ...
                  (data8(3:3:end,12)-data8(3:3:end,1)).^2).*1000;
              
trans_err9 = sqrt((data9(1:3:end-2,12)-data9(1:3:end-2,1)).^2+ ...
                  (data9(2:3:end-1,12)-data9(2:3:end-1,1)).^2+ ...
                  (data9(3:3:end,12)-data9(3:3:end,1)).^2).*1000;
              
trans_err10 = sqrt((data10(1:3:end-2,12)-data10(1:3:end-2,1)).^2+ ...
                  (data10(2:3:end-1,12)-data10(2:3:end-1,1)).^2+ ...
                  (data10(3:3:end,12)-data10(3:3:end,1)).^2).*1000;
              
trans_err11 = sqrt((data11(1:3:end-2,12)-data11(1:3:end-2,1)).^2+ ...
                  (data11(2:3:end-1,12)-data11(2:3:end-1,1)).^2+ ...
                  (data11(3:3:end,12)-data11(3:3:end,1)).^2).*1000;              

              
trans_err12 = sqrt((data12(1:3:end-2,12)-data12(1:3:end-2,1)).^2+ ...
                  (data12(2:3:end-1,12)-data12(2:3:end-1,1)).^2+ ...
                  (data12(3:3:end,12)-data12(3:3:end,1)).^2).*1000; 
              
trans_errs = [trans_err1,trans_err2,trans_err3,trans_err4, ...
              trans_err5,trans_err6,trans_err7,trans_err8, ...
              trans_err9,trans_err10,trans_err11,trans_err12];  

trans_x_err = [data1(1:3:end-2,12)-data1(1:3:end-2,1), ...
               data2(1:3:end-2,12)-data2(1:3:end-2,1), ...
               data3(1:3:end-2,12)-data3(1:3:end-2,1), ...
               data4(1:3:end-2,12)-data4(1:3:end-2,1), ...
               data5(1:3:end-2,12)-data5(1:3:end-2,1), ...
               data6(1:3:end-2,12)-data6(1:3:end-2,1), ...
               data7(1:3:end-2,12)-data7(1:3:end-2,1), ...
               data8(1:3:end-2,12)-data8(1:3:end-2,1), ...
               data9(1:3:end-2,12)-data9(1:3:end-2,1), ...
               data10(1:3:end-2,12)-data10(1:3:end-2,1), ...
               data11(1:3:end-2,12)-data11(1:3:end-2,1), ...
               data12(1:3:end-2,12)-data12(1:3:end-2,1)].*1000;

trans_y_err = [data1(2:3:end-1,12)-data1(2:3:end-1,1), ...
               data2(2:3:end-1,12)-data2(2:3:end-1,1), ...
               data3(2:3:end-1,12)-data3(2:3:end-1,1), ...
               data4(2:3:end-1,12)-data4(2:3:end-1,1), ...
               data5(2:3:end-1,12)-data5(2:3:end-1,1), ...
               data6(2:3:end-1,12)-data6(2:3:end-1,1), ...
               data7(2:3:end-1,12)-data7(2:3:end-1,1), ...
               data8(2:3:end-1,12)-data8(2:3:end-1,1), ...
               data9(2:3:end-1,12)-data9(2:3:end-1,1), ...
               data10(2:3:end-1,12)-data10(2:3:end-1,1), ...
               data11(2:3:end-1,12)-data11(2:3:end-1,1), ...
               data12(2:3:end-1,12)-data12(2:3:end-1,1)].*1000;

trans_z_err = [data1(3:3:end,12)-data1(3:3:end,1), ...
               data2(3:3:end,12)-data2(3:3:end,1), ...
               data3(3:3:end,12)-data3(3:3:end,1), ...
               data4(3:3:end,12)-data4(3:3:end,1), ...
               data5(3:3:end,12)-data5(3:3:end,1), ...
               data6(3:3:end,12)-data6(3:3:end,1), ...
               data7(3:3:end,12)-data7(3:3:end,1), ...
               data8(3:3:end,12)-data8(3:3:end,1), ...
               data9(3:3:end,12)-data9(3:3:end,1), ...
               data10(3:3:end,12)-data10(3:3:end,1), ...
               data11(3:3:end,12)-data11(3:3:end,1), ...
               data12(3:3:end,12)-data12(3:3:end,1)].*1000;

%% 3D bar chart
figure
% translation
subplot(2,1,1)
trans_err_map = fliplr(reshape(mean(trans_errs),4,3)');
% trans_x_err_map = reshape(mean(trans_x_err),4,3)';
% trans_y_err_map = reshape(mean(trans_y_err),4,3)';
% trans_z_err_map = reshape(mean(trans_z_err),4,3)';
bar3(trans_err_map, 0.4);
hold on
line([1,1],[1,1],[trans_err_map(1,1),trans_err_map(1,1)+std(trans_errs(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[trans_err_map(1,2),trans_err_map(1,2)+std(trans_errs(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[trans_err_map(1,3),trans_err_map(1,3)+std(trans_errs(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[trans_err_map(1,4),trans_err_map(1,4)+std(trans_errs(:,4))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[trans_err_map(2,1),trans_err_map(2,1)+std(trans_errs(:,5))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[trans_err_map(2,2),trans_err_map(2,2)+std(trans_errs(:,6))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[trans_err_map(2,3),trans_err_map(2,3)+std(trans_errs(:,7))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[trans_err_map(2,4),trans_err_map(2,4)+std(trans_errs(:,8))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[trans_err_map(3,1),trans_err_map(3,1)+std(trans_errs(:,9))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[trans_err_map(3,2),trans_err_map(3,2)+std(trans_errs(:,10))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[trans_err_map(3,3),trans_err_map(3,3)+std(trans_errs(:,11))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[trans_err_map(3,4),trans_err_map(3,4)+std(trans_errs(:,12))],'color','r','LineWidth',1.5)
xlabel('HORIZ')
xticklabels({'4','3','2','1'})
ylabel('VERT')
zlabel('translation error [mm]')
view([37.5,45])

% rotation
subplot(2,1,2)
rot_err_map = fliplr(reshape(mean(rot_errs),4,3)');
bar3(rot_err_map, 0.4);
hold on
line([1,1],[1,1],[rot_err_map(1,1),rot_err_map(1,1)+std(rot_errs(:,1))],'color','r','LineWidth',1.5)
line([2,2],[1,1],[rot_err_map(1,2),rot_err_map(1,2)+std(rot_errs(:,2))],'color','r','LineWidth',1.5)
line([3,3],[1,1],[rot_err_map(1,3),rot_err_map(1,3)+std(rot_errs(:,3))],'color','r','LineWidth',1.5)
line([4,4],[1,1],[rot_err_map(1,4),rot_err_map(1,4)+std(rot_errs(:,4))],'color','r','LineWidth',1.5)

line([1,1],[2,2],[rot_err_map(2,1),rot_err_map(2,1)+std(rot_errs(:,5))],'color','r','LineWidth',1.5)
line([2,2],[2,2],[rot_err_map(2,2),rot_err_map(2,2)+std(rot_errs(:,6))],'color','r','LineWidth',1.5)
line([3,3],[2,2],[rot_err_map(2,3),rot_err_map(2,3)+std(rot_errs(:,7))],'color','r','LineWidth',1.5)
line([4,4],[2,2],[rot_err_map(2,4),rot_err_map(2,4)+std(rot_errs(:,8))],'color','r','LineWidth',1.5)

line([1,1],[3,3],[rot_err_map(3,1),rot_err_map(3,1)+std(rot_errs(:,9))],'color','r','LineWidth',1.5)
line([2,2],[3,3],[rot_err_map(3,2),rot_err_map(3,2)+std(rot_errs(:,10))],'color','r','LineWidth',1.5)
line([3,3],[3,3],[rot_err_map(3,3),rot_err_map(3,3)+std(rot_errs(:,11))],'color','r','LineWidth',1.5)
line([4,4],[3,3],[rot_err_map(3,4),rot_err_map(3,4)+std(rot_errs(:,12))],'color','r','LineWidth',1.5)
xlabel('HORIZ')
xticklabels({'4','3','2','1'})
ylabel('VERT')
zlabel('rotation error [rad]')
view([37.5,45])
set(gcf, 'Position',  [500, 200, 350, 550])

%% 2D subplots translation + rotation
figure('Position',[1920/3,1080/3,350*2,400])
tlo = tiledlayout(3,4);

for i = 1:12
ax = nexttile;
yyaxis left
boxchart([rot_errs(:,i),NaN(nKept,1)],'JitterOutliers','on','MarkerStyle','.')
if i == 5
    ylabel('[rad]');
end
ylim([0,0.55]);
yyaxis right
boxchart([NaN(nKept,1),trans_errs(:,i)],'JitterOutliers','on','MarkerStyle','.')
if i == 8
    ylabel('[mm]');
end
ylim([0, 10]);
set(gca,'xtick',[]); ax.YGrid = 'on'; 
end
