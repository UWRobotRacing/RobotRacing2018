%% Load original images
im1name = 'whitecurve.png';
im2name = 'whitelane.png';

image1 = imread(im1name);
image2 = imread(im2name);

figure,
subplot(2,2,1), imagesc(image1), title('White Curve'), grid on;
subplot(2,2,2), imagesc(image2), title('White Lane'), grid on;

%% Convert from RGB to HSV
% image1_HSV = rgb2hsv(image1);
% image2_HSV = rgb2hsv(image2);
image1_HSL = rgb2hsl2(image1);
image2_HSL = rgb2hsl2(image2);

% subplot(2,2,1), imagesc(image1), title('White Curve'), grid on;
% subplot(2,2,2), imagesc(image1_HSL), title('White Curve HSL');
% subplot(2,2,3), imagesc(image2), title('White Lane'), grid on;
% subplot(2,2,4), imagesc(image2_HSL), title('White Lane HSL');
%
% subplot(2,2,1),imagesc(image2),title('Original Image')
% subplot(2,2,2),imagesc(image2_HSL(:,:,1)),title('Hue'),colorbar
% subplot(2,2,3),imagesc(image2_HSL(:,:,2)),title('Saturation'),colorbar
% subplot(2,2,4),imagesc(image2_HSL(:,:,3)),title('Lightness'),colorbar
% 
% subplot(2,2,1), imagesc(image1_HSV), title('White Curve HSV');
% subplot(2,2,2), imagesc(image2_HSV), title('White Lane HSV');
% subplot(2,2,3), imagesc(image1_HSL), title('White Curve HSL');
% subplot(2,2,4), imagesc(image2_HSL), title('White Lane HSL');

%% Extract yellow and white
image1_white = image1_HSL(:, :, 3) > 0.8;
image2_white = image2_HSL(:, :, 3) > 0.8;

image1_yellow = (image1_HSL(:,:,1) > 40 & image1_HSL(:,:,1) < 80) & image1_HSL(:, :, 3) > 0.25 & image1_HSL(:, :, 2) > 0.25;
image2_yellow = (image2_HSL(:,:,1) > 40 & image2_HSL(:,:,1) < 80) & image2_HSL(:, :, 3) > 0.25 & image2_HSL(:, :, 2) > 0.25;

subplot(2,2,1), imagesc(image1_white), title('White Curve White');
subplot(2,2,2), imagesc(image2_white), title('White Lane White');
subplot(2,2,3), imagesc(image1_yellow), title('White Curve Yellow');
subplot(2,2,4), imagesc(image2_yellow), title('White Lane Yellow');

%% Combine
image1_extract = double(image1 & (image1_white | image1_yellow));
image2_extract = double(image2 & (image2_white | image2_yellow));

subplot(2,2,1), imagesc(image1), title('White Curve');
subplot(2,2,2), imagesc(image2), title('White Lane');
subplot(2,2,3), imagesc(image1_extract), title('White Curve Extract');
subplot(2,2,4), imagesc(image2_extract), title('White Lane Extract');

%% Gaussian blur
image1_gauss = imgaussfilt(image1_extract, 1.2);
image2_gauss = imgaussfilt(image2_extract, 1.2);

subplot(2,2,1), imagesc(image1_extract), title('White Curve Extract');
subplot(2,2,2), imagesc(image2_extract), title('White Lane Extract');
subplot(2,2,3), imagesc(image1_gauss), title('White Curve Gauss');
subplot(2,2,4), imagesc(image2_gauss), title('White Lane Gauss');

%% Canny edge detection
image1_edge = edge(rgb2gray(image1_gauss), 'canny');
image2_edge = edge(rgb2gray(image2_gauss), 'canny');

subplot(2,2,1), imagesc(image1_gauss), title('White Curve Gauss');
subplot(2,2,2), imagesc(image2_gauss), title('White Lane Gauss');
subplot(2,2,3), imagesc(image1_edge), title('White Curve Canny');
subplot(2,2,4), imagesc(image2_edge), title('White Lane Canny');

%% Region of interest
handle = imshow(image1_edge);
roi = impoly(gca, [198 176;334 176;511 287.5;45 287.5]); % depends on image size
mask = createMask(roi, handle);

image1_roi = mask & image1_edge;
image2_roi = mask & image2_edge;

subplot(2,2,1), imagesc(image1_edge), title('White Curve Canny');
subplot(2,2,2), imagesc(image2_edge), title('White Lane Canny');
subplot(2,2,3), imagesc(image1_roi), title('White Curve Region');
subplot(2,2,4), imagesc(image2_roi), title('White Lane Region');

%% Hough transform
fillgap = 200;
minlength = 40;

% White curve
[H, theta, rho] = hough(image1_roi);
peaks = houghpeaks(H, 5, 'threshold', ceil(0.1*max(H(:))));
lines1 = houghlines(image1_roi, theta, rho, peaks, 'Fillgap', fillgap, 'MinLength', minlength);

subplot(2,1,1), imshow(image1_roi), title('White Curve'), hold on
plothoughlines(lines1);

% White lane
[H, theta, rho] = hough(image2_roi);
peaks = houghpeaks(H, 5, 'threshold', ceil(0.1*max(H(:))));
lines2 = houghlines(image2_roi, theta, rho, peaks, 'Fillgap', fillgap, 'MinLength', minlength);

subplot(2,1,2), imshow(image2_roi), title('White Lane'), hold on
plothoughlines(lines2);

%% Separate left and right lanes
clf('reset');

subplot(2,1,1), imshow(image1_roi), title('White Curve'), hold on
lines1 = plotleftrightlines(lines1);

subplot(2,1,2), imshow(image2_roi), title('White Lane'), hold on
lines2 = plotleftrightlines(lines2);

















