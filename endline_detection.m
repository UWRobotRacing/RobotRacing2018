currstate = linestate.START;
v = VideoReader('new_video.mp4');

while (hasFrame(v))
    %imname = 'mag2.jpg';
    %image1 = imread(imname);
    image1 = readFrame(v);
    subplot(3,3,1), imshow(image1);
    
    image1_HSV = rgb2hsv(image1);
    magenta = (image1_HSV(:,:,1) > 19/24 & image1_HSV(:,:,1) < 23/24) & image1_HSV(:,:,2) > 1/4 & image1_HSV(:,:,3) > 2/4;

    n = 5;
    filt1 = ones(n,n)/n^2;
    filtsize = 5; 
    sigma = 3;
    filt_gauss = fspecial('gaussian', filtsize, sigma);
    image1_gauss = imfilter(magenta, filt1);

    mag = imfill(image1_gauss, 'holes');
    subplot(3,3,2), imshow(mag);

    labeledImage = logical(mag); 
    coloredLabels = label2rgb (labeledImage, 'hsv', 'k', 'shuffle'); % pseudo random color labels
    subplot(3,3,3), imshow(coloredLabels);

    blobMeasurements = regionprops(labeledImage, 'Area', 'Centroid');
    numberOfBlobs = size(blobMeasurements, 1);
    for k = 1 : numberOfBlobs
        blobArea = blobMeasurements(k).Area;                % Get area.
        blobCentroid = blobMeasurements(k).Centroid;		% Get centroid one at a time
        %fprintf(1,'#%2d  %11.1f %8.1f\n', k, blobArea, blobCentroid); 
    end

    allBlobAreas = [blobMeasurements.Area];
    keeperIndexes = find(allBlobAreas > 1500); 
    binimg = ismember(labeledImage, keeperIndexes);
    numofBlobs = numel(keeperIndexes);
    
    if (numofBlobs > 0)
        if (currstate == linestate.START)
            currstate = linestate.STARTL;
        elseif (currstate == linestate.MIDDLE)
            currstate = linestate.ENDL;
        end
    else
        if (currstate == linestate.STARTL)
            currstate = linestate.MIDDLE;
        elseif (currstate == linestate.ENDL)
            currstate = linestate.FINISH;
        end
    end  
    display(currstate);
    
    subplot(3, 3, 4), imshow(binimg);
    
    pause(0.1);
end