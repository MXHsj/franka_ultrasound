%% solving for hand-eye transformation
% author: Jakub

clc; clear; close all

matr_2 = eye(4);
matr_2(3,4) = -242.30 - 295;
m2 = matr_2;

matr_3 = makehgtform('xrotate',pi/8);
m3 = m2*matr_3;

matr_4 = makehgtform('translate',[0,0,220.57]);
m4 = m3*matr_4;

matr_5 = makehgtform('translate',[-17.5,0,11]);
m5 = m4*matr_5;