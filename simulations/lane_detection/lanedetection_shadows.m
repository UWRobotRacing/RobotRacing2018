%{
Proof-of-concept for doing lane detection with shadow removal
%}

%% Load original images
imname = 'shadowcrop1.png';
image = imread(imname);

figure,
subplot(1,1,1), imagesc(image), title('Shadow'), grid on;

%% Remove shadows
no_shadows = remove_shadows(image);

subplot(1,2,1), imagesc(image), title('Original'), grid on;
subplot(1,2,2), imagesc(no_shadows), title('Shadows Removed');

%% Convert from RGB to HSL
image_HSL = rgb2hsl(no_shadows);

subplot(1,2,1), imagesc(no_shadows), title('RGB');
subplot(1,2,2), imagesc(image_HSL(:,:,3)), title('HSL');

%TODO: everything below this
%% Extract yellow and white
image_white = image_HSL(:,:,3) > 0.8;
image_yellow = (image_HSL(:,:,1) > 40 & image_HSL(:,:,1) < 80) & image_HSL(:, :, 3) > 0.25 & image_HSL(:, :, 2) > 0.25;

subplot(1,2,1), imagesc(image_white), title('White');
subplot(1,2,2), imagesc(image_yellow), title('Yellow');

%% Combine
image_extract = double(image & (image_white | image_yellow));

subplot(1,2,1), imagesc(image), title('White Curve');
subplot(1,2,2), imagesc(image_extract), title('White Curve Extract');

%% Gaussian blur
image_gauss = imgaussfilt(image_extract, 1.2);

subplot(1,2,1), imagesc(image_extract), title('White Curve Extract');
subplot(1,2,2), imagesc(image_gauss), title('White Curve Gauss');

%% Canny edge detection
image_edge = edge(rgb2gray(image_gauss), 'canny');

subplot(1,2,1), imagesc(image_gauss), title('White Curve Gauss');
subplot(1,2,2), imagesc(image_edge), title('White Curve Canny');

%% Region of interest
handle = imshow(image_edge);
roi = impoly(gca, [198 176;334 176;511 287.5;45 287.5]); % depends on image size
mask = createMask(roi, handle);

image_roi = mask & image_edge;

subplot(1,2,1), imagesc(image_edge), title('White Curve Canny');
subplot(1,2,2), imagesc(image_roi), title('White Curve Region');

%% Hough transform
fillgap = 200;
minlength = 40;

% White curve
[H, theta, rho] = hough(image_roi);
peaks = houghpeaks(H, 5, 'threshold', ceil(0.1*max(H(:))));
lines = houghlines(image_roi, theta, rho, peaks, 'Fillgap', fillgap, 'MinLength', minlength);

subplot(2,1,1), imshow(image_roi), title('White Curve'), hold on
plothoughlines(lines);

%% Separate left and right lanes
clf('reset');

subplot(2,1,1), imshow(image_roi), title('White Curve'), hold on
lines = plotleftrightlines(lines);

















