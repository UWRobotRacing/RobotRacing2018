% Try using Lab colour space to detect and remove shadows
function [o_image] = remove_shadows(i_image)
    % Configurable constants
    THRESHOLD_TOLERANCE = 0.8;
    MIN_SHADOW_AREA = 30;
    MASK_DILATION = 6;
    EXPANDED_SHADOW_MASK_DILATION = 10;
    EXPANDED_EDGE_MASK_DILATION = 8;
    MED_FILTER_KERNEL_SIZE = 15;

    % Get the mean of L, a, b planes
    shadow_Lab = rgb2lab(i_image);
    mean_l = mean(mean(shadow_Lab(:,:,1)));
    mean_a = mean(mean(shadow_Lab(:,:,2)));
    mean_b = mean(mean(shadow_Lab(:,:,3)));

    % Extract shadows using best method based on mean values
    stdev_l = std(shadow_Lab(:,:,1));
    shadow_threshold = mean_l - (stdev_l / THRESHOLD_TOLERANCE);

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
    shadow_pixels_morph = bwareaopen(shadow_pixels_morph, MIN_SHADOW_AREA);
    shadow_pixels_morph = imdilate(shadow_pixels_morph, true(MASK_DILATION));

    subplot(3,2,1), imagesc(shadow), title('Shadow'), grid on;
    subplot(3,2,2), imagesc(shadow_pixels), title('Shadow Pixels'), grid on;
    subplot(3,2,3), imagesc(shadow_pixels_morph), title('Shadow Morphed'), grid on;

    % Shadow regions
    connected_shadow_regions = bwconncomp(shadow_pixels_morph);

    subplot(3,2,4), imshow(label2rgb(labelmatrix(connected_shadow_regions), @jet, [.5 .5 .5])), title('Shadow Regions')
    hold on

    corrected_image = shadow;
    % Loop over every shadow region
    for i = 1:connected_shadow_regions.NumObjects
        mask = false(connected_shadow_regions.ImageSize);
        mask(connected_shadow_regions.PixelIdxList{i}) = true;
        expanded_mask = imdilate(mask, true(EXPANDED_SHADOW_MASK_DILATION));

        [B,~] = bwboundaries(expanded_mask, 'noholes');
        boundary = B{1};
        plot(boundary(:,2), boundary(:,1), 'w', 'LineWidth', 0.5);

        % Get RGB averages in masks
        outer_r = 0;
        outer_g = 0;
        outer_b = 0;
        outer_count = 0;

        inner_r = 0;
        inner_g = 0;
        inner_b = 0;
        inner_count = 0;

        img_r = corrected_image(:,:,1);
        img_g = corrected_image(:,:,2);
        img_b = corrected_image(:,:,3);

        % Get RGB values for ratios between expanded and inner masks
        for x = 1:size(expanded_mask, 1)            % super inefficient
            for y = 1:size(expanded_mask, 2)
                if expanded_mask(x,y) == 1
                    r = double(img_r(x,y));
                    g = double(img_g(x,y));
                    b = double(img_b(x,y));

                    if mask(x,y) == 1
                        % Inner mask
                        inner_r = inner_r + r;
                        inner_g = inner_g + g;
                        inner_b = inner_b + b;

                        inner_count = inner_count + 1;
                    else
                        % Outer mask
                        outer_r = outer_r + r;
                        outer_g = outer_g + g;
                        outer_b = outer_b + b;

                        outer_count = outer_count + 1;
                    end
                end
            end
        end

        r_ratio = (outer_r / outer_count) / (inner_r / inner_count);
        g_ratio = (outer_g / outer_count) / (inner_g / inner_count);
        b_ratio = (outer_b / outer_count) / (inner_b / inner_count);

        if isnan(r_ratio)
            r_ratio = 1;
        end
        if isnan(g_ratio)
            g_ratio = 1;
        end
        if isnan(b_ratio)
            b_ratio = 1;
        end

        % Multiply the mask area by the ratio to remove shadow
        for x = 1:size(expanded_mask, 1)            % super inefficient
            for y = 1:size(expanded_mask, 2)
                if mask(x,y) == 1
                    r = double(img_r(x,y));
                    g = double(img_g(x,y));
                    b = double(img_b(x,y));

                    r = r * r_ratio;
                    g = g * g_ratio;
                    b = b * b_ratio;

                    if r > 255
                       r = 255; 
                    end
                    if g > 255
                       g = 255; 
                    end
                    if b > 255
                       b = 255; 
                    end

                    img_r(x,y) = r;
                    img_g(x,y) = g;
                    img_b(x,y) = b;
                end
            end
        end

        corrected_image(:,:,1) = img_r;
        corrected_image(:,:,2) = img_g;
        corrected_image(:,:,3) = img_b;
    end

    subplot(3,2,5), imagesc(corrected_image), title('Removed Shadows'), grid on;

    % THIS IS THE SLOWEST STEP
    % Fix over-illuminated edges with a median filter on shadow region edges
    median_filtered = rgb2hsv(corrected_image);
    h = floor(MED_FILTER_KERNEL_SIZE / 2);

    for i = 1:connected_shadow_regions.NumObjects
        mask = false(connected_shadow_regions.ImageSize);
        mask(connected_shadow_regions.PixelIdxList{i}) = true;
        mask = edge(mask, 'canny');
        mask = imdilate(mask, true(EXPANDED_EDGE_MASK_DILATION));

        for x = (1+h):(size(mask, 1)-h)
            for y = (1+h):(size(mask, 2)-h)
                if mask(x,y) == 1
                    kernel = medfilt2(median_filtered(x-h:x+h,y-h:y+h,3), [MED_FILTER_KERNEL_SIZE MED_FILTER_KERNEL_SIZE]); 
                    median_filtered(x, y, 3) = kernel(h+1,h+1);
                end
            end
        end

    end

    median_filtered = hsv2rgb(median_filtered);
    subplot(3,2,6), imagesc(median_filtered), title('Fix Over-illumination'), grid on;

    o_image = median_filtered;
end