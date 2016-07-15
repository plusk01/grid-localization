function xt = sub2state( i, j, k, depth )
%STATE2SUB Takes MATLAB matrix subscripts and returns the pose

    if (nargin < 4)
        depth = 4;
    end
    
    depth2deg = @(index)(index-1)*(360/depth);
    deg2depth = @(deg)(deg/(360/depth))+1;

    xt = [j-1   i-1   depth2deg(k)];
end
