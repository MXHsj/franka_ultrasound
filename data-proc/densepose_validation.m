%% validate densepose
clc; clear; close all

fid = fopen('../data/marker_pos_log_mod.csv','w');
s = fileread('../data/marker_pos_log.csv');
s = strrep(s,'"','');
fprintf(fid,s);
fclose(fid);

% mk_pos = csvread('../data/marker_pos_log.csv');
% dp_tar = csvread('../data/dp_target_log.csv');


