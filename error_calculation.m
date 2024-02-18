function [error] = error_calculation(desired,actual)
    error = desired - actual;
end 


% function [error, error_arg]=find_distance(x, y, x_ref, y_ref)
% [error, error_arg] = min(sqrt(sum([x-x_ref;y-y_ref].^2)));