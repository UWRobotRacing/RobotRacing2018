clc

shadowname = 'shadowcrop.png';
shadow = imread(shadowname);
% subplot(1,1,1), imagesc(shadow), title('Shadow'), grid on;

% Try using Lab colour space for shadows
shadow_Lab = rgb2lab(shadow);
% subplot(2,2,1), imagesc(shadow_Lab);
% subplot(2,2,2), imagesc(shadow_Lab(:,:,1));
% subplot(2,2,3), imagesc(shadow_Lab(:,:,2));
% subplot(2,2,4), imagesc(shadow_Lab(:,:,3));

% Get the mean of L, a, b planes
mean_l = mean(mean(shadow_Lab(:,:,1)));
mean_a = mean(mean(shadow_Lab(:,:,2)));
mean_b = mean(mean(shadow_Lab(:,:,3)));

% Extract shadows using best method based on mean values
stdev_l = std(shadow_Lab(:,:,1));
shadow_threshold = mean_l - (stdev_l / 1);                      % config value

if mean_a + mean_b > 256
    disp("Threshold L value");
    shadow_pixels = shadow_Lab(:,:,1) <= shadow_threshold;
else
    disp("Threshold L and b value");
    shadow_pixels = shadow_Lab(:,:,1) <= shadow_threshold & shadow_Lab(:,:,3) <= shadow_threshold;
end

% Morphological operations to clean up misclassified pixels
shadow_pixels_morph = bwmorph(shadow_pixels, 'clean');
shadow_pixels_morph = bwmorph(shadow_pixels_morph, 'close');
shadow_pixels_morph = bwareaopen(shadow_pixels_morph, 30);      % config value

subplot(2,2,1), imagesc(shadow), title('Shadow'), grid on;
subplot(2,2,2), imagesc(shadow_pixels), title('Shadow Pixels'), grid on;
subplot(2,2,3), imagesc(shadow_pixels_morph), title('Shadow Morphed'), grid on;

% Shadow regions
connected_shadow_regions = bwconncomp(shadow_pixels_morph);

subplot(2,2,4), imshow(label2rgb(labelmatrix(connected_shadow_regions), @jet, [.5 .5 .5])), title('Shadow Regions')
hold on

% Loop over every shadow region
for i = 1:connected_shadow_regions.NumObjects
    mask = false(connected_shadow_regions.ImageSize);
    mask(connected_shadow_regions.PixelIdxList{i}) = true;
    expanded_mask = imdilate(mask, true(6));
    
    [B,L] = bwboundaries(expanded_mask, 'noholes');
    boundary = B{1};
    plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 0.5)
    
    % use expanded mask to process
end



















