function xt = ind2state( siz, index )
%IND2STATE Takes a MATLAB matrix index (one number) and returns the pose
    
    [i, j, k] = ind2sub(siz, index);
    
    if (length(siz) == 3)
        depth = siz(3);
    else
        depth = 4;
    end
    
    xt = sub2state(i, j, k, depth);

end