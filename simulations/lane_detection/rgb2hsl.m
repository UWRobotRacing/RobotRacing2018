function [hslimage] = rgb2hsl2(rgbimage)
    numRows = size(rgbimage, 1);
    numCols = size(rgbimage, 2);
    hslimage = zeros(numRows, numCols, 3);
    
    for i = 1:numRows
        for j = 1:numCols
            R = double(rgbimage(i, j, 1)) / 255;
            G = double(rgbimage(i, j, 2)) / 255;
            B = double(rgbimage(i, j, 3)) / 255;
            
            maxRGB = max([R G B]);
            minRGB = min([R G B]);
            
            delta = maxRGB - minRGB;
            
            % L calculation
            L = (maxRGB + minRGB) / 2;
            
            % S calculation
            S = 0;
            
            if delta ~= 0
                S = delta / (1 - abs((2*L)-1));
            end
            
            % H calculation
            H = 0;
            
            if delta ~= 0
                if maxRGB == R
                    H = mod((G-B)/delta, 6);
                elseif maxRGB == G
                    H = (B-R)/delta + 2;
                elseif maxRGB == B
                    H = (R-G)/delta + 4;
                end
                
                H = H * 60;
            end
            
            %L = uint8(L*100);
            %S = uint8(S*100);
            %H = uint8(H);
            
            hslimage(i, j, :) = [H S L];
        end
    end
end