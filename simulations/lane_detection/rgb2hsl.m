%function hsl=rgb2hsl(rgb)
function [h, s, l] = rgb2hsl(varargin)
[r, g, b, isColorMap, isEmptyInput, isThreeChannel] = parseInputs(varargin{:});

%Converts Red-Green-Blue Color value to Hue-Saturation-Luminance Color value
%
%Usage
%       HSL = rgb2hsl(RGB)
%
%   converts RGB, a M X 3 color matrix with values between 0 and 1
%   into HSL, a M X 3 color matrix with values between 0 and 1
%

for i=1:size(rgb,1)
    mx = max(rgb(i,:));%max of the 3 colors
    mn = min(rgb(i,:));%min of the 3 colors
    imx = find(rgb(i,:) == mx);%which color has the max
    hsl(i,3) = (mx+mn)/2;%luminance is half of max value + min value
    if(mx-mn) == 0 %if all three colors have same value, 
        hsl(i,2) = 0;%then s=0 and 
        hsl(i,1) = 0;%h is undefined but for practical reasons 0
        return;
    end
    if hsl(i,3) < 0.5
        hsl(i,2) = (mx-mn)/(mx+mn);
    else
        hsl(i,2) = (mx-mn)/(2-(mx+mn));
    end
    switch(imx(1))%if two colors have same value and be the maximum, use the first color
    case 1 %Red is the max color
        hsl(i,1) = ((rgb(i,2)-rgb(i,3))/(mx-mn))/6;
    case 2 %Green is the max color
        hsl(i,1) = (2+(rgb(i,3)-rgb(i,1))/(mx-mn))/6;
    case 3 %Blue is the max color
        hsl(i,1) = (4+(rgb(i,1)-rgb(i,2))/(mx-mn))/6;
    end
    if hsl(i,1) < 0 %if hue is negative, add 1 to get it within 0 and 1
        hsl(i,1) = hsl(i,1)+1;
    end
end

hsl = round(hsl*100000)/100000; %Sometimes the result is 1+eps instead of 1 or 0-eps instead of 0 ... so to get rid of this I am rounding to 5 decimal places)

if(~isEmptyInput)

    if(isThreeChannel)
        imageIn(:,:,1) = r;
        imageIn(:,:,2) = g;
        imageIn(:,:,3) = b;
    elseif(isColorMap)
        imageIn = reshape(varargin{1},size(varargin{1},1),1,size(varargin{1},2));
    else
        imageIn = r;        
    end

    h = images.internal.rgb2hsvmex(imageIn);

    if(nargout == 3)
        s = h(:,:,2);
        l = h(:,:,3);
        h = h(:,:,1);        
    elseif(isColorMap)
        h = reshape(h,size(h,1), size(h,3));
    end
    
else
    if(isThreeChannel)
        h = r;
        s = g;
        l = b;
    else
        h = r;
    end
end

function [r, g, b, isColorMap, isEmptyInput, isThreeChannel] = parseInputs(varargin)

isColorMap = 0;
isEmptyInput = 0;
isThreeChannel = 0;

if (nargin == 1)
    r = varargin{1};
    g = [];
    b = [];
    if (ndims(r)==3)
        if isempty(r)
            isEmptyInput = 1;
            return
        end
        if(size(r,3) ~= 3)
            error(message('MATLAB:rgb2hsv:invalidInputSizeRGB'));
        end

        validateattributes(r, {'uint8', 'uint16', 'double', 'single'}, {'real'}, mfilename, 'RGB', 1);

    elseif ismatrix(r) %M x 3 matrix for M colors.
        
        isColorMap = 1;
        if(size(r,2) ~=3)
            error(message('MATLAB:rgb2hsv:invalidSizeForColormap'));
        end

        validateattributes(r, {'double'}, {'real','nonempty','nonsparse'}, mfilename, 'M');

        if((any(r(:) < 0) || any(r(:) > 1)))
            error(message('MATLAB:rgb2hsv:badMapValues'));
        end
        
    else
        error(message('MATLAB:rgb2hsv:invalidInputSize'));
    end

elseif (nargin == 3)
    isThreeChannel = 1;
    r = varargin{1};
    g = varargin{2};
    b = varargin{3};

    if isempty(r) || isempty(g) || isempty(b)
        isEmptyInput = 1;
        return
    end  
    
    validateattributes(r, {'uint8', 'uint16', 'double', 'single'}, {'real', '2d'}, mfilename, 'R', 1);
    validateattributes(g, {'uint8', 'uint16', 'double', 'single'}, {'real', '2d'}, mfilename, 'G', 2);
    validateattributes(b, {'uint8', 'uint16', 'double', 'single'}, {'real', '2d'}, mfilename, 'B', 3);

    if ~isa(r, class(g)) || ~isa(g, class(b)) || ~isa(r, class(b)) % mixed type inputs.
        r = im2double(r);
        g = im2double(g);
        b = im2double(b);
    end
    
    if ~isequal(size(r),size(g),size(b))
        error(message('MATLAB:rgb2hsv:InputSizeMismatch'));
    end
    
else
    error(message('MATLAB:rgb2hsv:WrongInputNum'));
end