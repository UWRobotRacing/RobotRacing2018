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

%% Perspective transform
% corners = [150,150; 1,size(image,1); 275,150; size(image,2),size(image,1) ];
% 
% target_corners = corners;
% target_corners([1 3],1) = [-150; 390];
% 
% ptransform = fitgeotrans(corners,target_corners,'Projective');
% image_perspective = imwarp(no_shadows,ptransform);
% 
% subplot(1,2,1), imagesc(no_shadows), title('Original');
% subplot(1,2,2), imagesc(image_perspective), title('Perspective Transform');

%% Convert from RGB to HSV
image_HSV = rgb2hsv(no_shadows);

subplot(1,2,1), imagesc(no_shadows), title('RGB');
subplot(1,2,2), imagesc(image_HSV(:,:,3)), title('Value');

%% Extract yellow and white
image_white = image_HSV(:,:,2) < 0.25 & image_HSV(:,:,3) > 0.85;
image_yellow = image_HSV(:,:,1) < 0.2 & image_HSV(:,:,2) > 0.45 & image_HSV(:,:,3) > 0.85;

subplot(2,2,1), imagesc(image_HSV(:,:,1)), title('Hue');
subplot(2,2,2), imagesc(image_HSV(:,:,2)), title('Saturation');
subplot(2,2,3), imagesc(image_white), title('White');
subplot(2,2,4), imagesc(image_yellow), title('Yellow');

%% Combine
image_extract = double(image & (image_white | image_yellow));

subplot(1,2,1), imagesc(image), title('Original');
subplot(1,2,2), imagesc(image_extract), title('White and Yellow');

%% Gaussian blur
image_gauss = imgaussfilt(image_extract, 1.2);

subplot(1,2,1), imagesc(image_extract), title('White and Yellow');
subplot(1,2,2), imagesc(image_gauss), title('Gaussian Blurred');

%% Canny edge detection
image_edge = edge(rgb2gray(image_gauss), 'canny');

subplot(1,2,1), imagesc(image_gauss), title('Gaussian Blurred');
subplot(1,2,2), imagesc(image_edge), title('Canny Edge');

%% Region of interest
% TODO: perform this earlier in the pipeline to save computation time
handle = imshow(image_edge);
roi = impoly(gca, [250 150;300 150;400 287.5;250 287.5]); % depends on image size
mask = createMask(roi, handle);

image_roi = mask & image_edge;

subplot(1,2,1), imagesc(image_edge), title('Canny Edge');
subplot(1,2,2), imagesc(image_roi), title('ROI: One White Line');

% TODO: Everything below this line
%% Perspective transform
corners = [150,150; 1,size(image,1); 275,150; size(image,2),size(image,1) ];

target_corners = corners;
target_corners([1 3],1) = [-150; 390];

ptransform = fitgeotrans(corners,target_corners,'Projective');
image_perspective = imwarp(image_roi,ptransform);

subplot(1,2,1), imagesc(image_roi), title('ROI: One White Line');
subplot(1,2,2), imagesc(image_perspective), title('Perspective Transform');

%% Fit curve to points

















