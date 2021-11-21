%% visualize densepose
clc; clear; close all

RGB = imread('../data/densepose/color_frame.png');
IUV = imread('../data/densepose/IUV.png');

% crop_v = 40; 
% RGB = RGB(crop_v:end-crop_v, crop_v:end-crop_v,:);
% IUV = IUV(crop_v:end-crop_v, crop_v:end-crop_v,:);

target_u = [60, 100, 60, 100];
target_v = [155, 155, 105, 105];

ind = 4;
[u2x,u2y] = find(IUV(:,:,2)==target_u(ind) & IUV(:,:,3)==1);
[v2x,v2y] = find(IUV(:,:,1)==target_v(ind) & IUV(:,:,3)==1);

[x_intersects, x_r, x_c] = intersect(u2x,v2x);
[y_intersects, y_r, y_c] = intersect(u2y,v2y);

i_mask = IUV(:,:,3)==1;
u_mask = IUV(:,:,2)==target_u(ind)&IUV(:,:,3)==1;
v_mask = IUV(:,:,1)==target_v(ind)&IUV(:,:,3)==1;
% u_mask(u_mask == 0) = nan;
% v_mask(v_mask == 0) = nan;

i_mask3C = cat(3, i_mask, i_mask, i_mask);
u_mask3C = cat(3, u_mask, u_mask, u_mask);
v_mask3C = cat(3, v_mask, v_mask, v_mask);
RGB_m = RGB;
RGB_m(u_mask3C) = 0;
RGB_m(v_mask3C) = 0;

% imagesc(imsubtract(RGB,uint8(50*i_mask3C)))
imshow(RGB)
hold on
% imagesc(RGB_m,'AlphaData',.5);
% imagesc(i_mask)
% imagesc(u_mask,'AlphaData',.1)
% imagesc(v_mask,'AlphaData',.1)
% plot(x_intersects,x_r,'.b')
% plot(mean(x_intersects),mean(y_intersects),'.b','LineWidth',1);
axis tight off
% I = IUV(:,:,3);
contour(IUV(:,:,2),10)  % U
contour(IUV(:,:,1),10)  % V
