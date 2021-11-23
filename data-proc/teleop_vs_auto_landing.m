%% compare robot motion during teleop landing & autonomous landing
clc; clear; close all

% prepare data
id = 3;
ref_data_dir = ['../data/robot/11-22-2021-robot_state_groundtruth', num2str(id), '.csv'];
tele_data_dir = ['../data/robot/11-22-2021-robot_motion_manual', num2str(id), '.csv'];
auto_data_dir = ['../data/robot/11-22-2021-robot_motion_auto', num2str(id), '.csv'];

ref_data_raw = csvread(ref_data_dir);
tele_data_raw = csvread(tele_data_dir);
auto_data_raw = csvread(auto_data_dir);

ref_pose_flat = ref_data_raw(:,1:16);
tele_pose_flat = tele_data_raw(:,1:16);
auto_pose_flat = auto_data_raw(:,1:16);

ref_pose = reshape(ref_pose_flat,4,4)';
ref_pose_xyz = ref_pose(1:3,4).*1000;
ref_pose_rpy = rotm2eul(ref_pose(1:3,1:3));
ref_pose_rpy(ref_pose_rpy(:,3)<0,3) = ref_pose_rpy(ref_pose_rpy(:,3)<0,3) + 2*pi;

tele_pose_rpy = -1.*ones(length(tele_pose_flat),3);
tele_pose_xyz = -1.*ones(length(tele_pose_flat),3);
for i = 1:length(tele_pose_flat)
    tele_pose = reshape(tele_pose_flat(i,:),4,4)';
    tele_pose_rpy(i,:) = rotm2eul(tele_pose(1:3,1:3));
    tele_pose_xyz(i,:) = tele_pose(1:3,4).*1000;
end
tele_pose_rpy(tele_pose_rpy(:,3)<0,3) = tele_pose_rpy(tele_pose_rpy(:,3)<0,3) + 2*pi;

auto_pose_rpy = -1.*ones(length(tele_pose_flat),3);
auto_pose_xyz = -1.*ones(length(tele_pose_flat),3);
for i = 1:length(auto_pose_flat)
    auto_pose = reshape(auto_pose_flat(i,:),4,4)';
    auto_pose_rpy(i,:) = rotm2eul(auto_pose(1:3,1:3));
    auto_pose_xyz(i,:) = auto_pose(1:3,4).*1000;
end
auto_pose_rpy(auto_pose_rpy(:,3)<0,3) = auto_pose_rpy(auto_pose_rpy(:,3)<0,3) + 2*pi;

%% calculate normalized error
tele_err_xyz = ones(3,length(tele_pose_flat)).*ref_pose_xyz - tele_pose_xyz';
tele_err_rpy = ones(3,length(tele_pose_flat)).*ref_pose_rpy' - tele_pose_rpy';
tele_contact_timing = find(tele_data_raw(:,end)==1, 1);

auto_err_xyz = ones(3,length(auto_pose_flat)).*ref_pose_xyz - auto_pose_xyz';
auto_err_rpy = ones(3,length(auto_pose_flat)).*ref_pose_rpy' - auto_pose_rpy';
auto_contact_timing = find(auto_data_raw(:,end)==1, 1);

%% plot data
figure('Position',[1920/3,1080/3,800,400])
tlo = tiledlayout(1,2); title(tlo, 'manual-landing'); xlabel(tlo, 'time [sec]');
nexttile;
plot(1:length(tele_pose_flat),tele_err_xyz,'LineWidth',1)
legend('x','y','z','Location','southeast')
xl = xline(tele_contact_timing,'--','contact mode on'); xl.LabelVerticalAlignment = 'middle'; xl.LabelHorizontalAlignment = 'center'; hasbehavior(xl,'legend',false)
xlim([0, length(tele_pose_flat)])
ylabel('translation error [mm]')
nexttile;
plot(1:length(tele_pose_flat),rad2deg(tele_err_rpy),'LineWidth',1)
legend('row','pitch','yaw','Location','southeast')
xl = xline(tele_contact_timing,'--','contact mode on'); xl.LabelVerticalAlignment = 'middle'; xl.LabelHorizontalAlignment = 'center'; hasbehavior(xl,'legend',false)
xlim([0 length(tele_pose_flat)])
ylabel('rotation error [deg]')

figure('Position',[1920/3,1080/3,800,400])
tlo = tiledlayout(1,2); title(tlo,'auto-landing'); xlabel(tlo, 'time [sec]');
nexttile;
plot(1:length(auto_pose_flat),auto_err_xyz,'LineWidth',1)
legend('x','y','z','Location','southeast')
xl = xline(auto_contact_timing,'--','contact mode on'); xl.LabelVerticalAlignment = 'middle'; xl.LabelHorizontalAlignment = 'center'; hasbehavior(xl,'legend',false)
xlim([0, length(auto_pose_flat)])
ylabel('translation error [mm]')
nexttile;
plot(1:length(auto_pose_flat),rad2deg(auto_err_rpy),'LineWidth',1)
legend('row','pitch','yaw','Location','southeast')
xl = xline(auto_contact_timing,'--','contact mode on'); xl.LabelVerticalAlignment = 'middle'; xl.LabelHorizontalAlignment = 'center'; hasbehavior(xl,'legend',false)
xlim([0, length(auto_pose_flat)])
ylabel('rotation error [deg]')
