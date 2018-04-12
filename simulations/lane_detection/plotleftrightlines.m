function lines = plotleftrightlines(lines)
    for i = 1:length(lines)
        xy = [lines(i).point1; lines(i).point2];
        deltay = xy(2, 2) - xy(1, 2);
        deltax = xy(2, 1) - xy(1, 1);

        if (deltax ~= 0)
            slope = deltay / deltax;
            color = 'green';

            if slope > 0.5
                lines(i).side = 'right';
                color = 'red';
            elseif slope < -0.5
                lines(i).side = 'left';
                color = 'blue';
            end

            plot(xy(:,1),xy(:,2),'LineWidth',2,'Color',color);
        end
    end
end