function path(x,y,x_next, y_next, xmin, xmax, ymin,ymax, j, len)
    figure(3)
    hold on 
    axis([xmin xmax ymin ymax])
    axis square
    if j == 1
        plot(x, y, 'or', "DisplayName", "Start")
    end 
    if j == len
        plot(x, y, 'pb', "DisplayName", "End") 
    end 
    plot([x x_next], [y y_next],'k')
    title('2D Animation Trajectory plot of Cart')
    xlabel('X-axis (meters)'); ylabel('Y-axis (meters)') 
    pause(0.1)
    axis([xmin xmax ymin ymax])
    %legend({'Start','End'},'Location','northwest','NumColumns',2)
    axis equal
end 