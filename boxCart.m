function boxCart(x, y, theta,xmin, xmax, ymin,ymax)
    hold off 
    s = 0.5;
    x_box_vertices = [0.5*s, -0.5*s, -0.5*s,  0.5*s, 0.5*s];
    y_box_vertices = [0.5*s,  0.5*s, -0.5*s, -0.5*s, 0.5*s];
    q=[x_box_vertices;y_box_vertices];
    rotation=[cos(theta) -sin(theta);sin(theta) cos(theta)];
    box_vertices_rotated=rotation*q;
    x_actual = box_vertices_rotated(1,:) + [x x x x x];
    y_actual = box_vertices_rotated(2,:) + [y y y y y];
    figure(4)
    patch(x_actual,y_actual,'yellow');
    axis([ xmin xmax ymin ymax])
    axis square
    title('2D Animation of Cart')
    xlabel('X-axis (meters)'); ylabel('Y-axis (meters)') 
    pause(0.15)
    
end 