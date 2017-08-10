function flag = triangle_intersection(P1, P2)
% triangle_test : returns true if the triangles overlap and false otherwise

%%% All of your code should be between the two lines of stars.
% *******************************************************************

flag = false;

%
flag1 = getCross(P1(1,1),P1(1,2),P1(2,1),P1(2,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2));
flag2 = getCross(P1(1,1),P1(1,2),P1(2,1),P1(2,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag3 = getCross(P1(1,1),P1(1,2),P1(2,1),P1(2,2),P2(3,1),P2(3,2),P2(1,1),P2(1,2));

flag4 = getCross(P1(2,1),P1(2,2),P1(3,1),P1(3,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2));
flag5 = getCross(P1(2,1),P1(2,2),P1(3,1),P1(3,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag6 = getCross(P1(2,1),P1(2,2),P1(3,1),P1(3,2),P2(3,1),P2(3,2),P2(1,1),P2(1,2));

%
flag7 = getCross(P1(3,1),P1(3,2),P1(1,1),P1(1,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2));
flag8 = getCross(P1(3,1),P1(3,2),P1(1,1),P1(1,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag9 = getCross(P1(3,1),P1(3,2),P1(1,1),P1(1,2),P2(3,1),P2(3,2),P2(1,1),P2(1,2));

flag = flag1 || flag2 || flag3 || flag4 || flag5 || flag6 || flag7 || flag8 || flag9;

if (flag == true)
	return;
end

flag10 = PointInTriangle(P1(1,1),P1(1,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag11 = PointInTriangle(P1(2,1),P1(2,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag12 = PointInTriangle(P1(3,1),P1(3,2),P2(1,1),P2(1,2),P2(2,1),P2(2,2),P2(3,1),P2(3,2));
flag13 = PointInTriangle(P2(1,1),P2(1,2),P1(1,1),P1(1,2),P1(2,1),P1(2,2),P1(3,1),P1(3,2));
flag14 = PointInTriangle(P2(2,1),P2(2,2),P1(1,1),P1(1,2),P1(2,1),P1(2,2),P1(3,1),P1(3,2));
flag15 = PointInTriangle(P2(3,1),P2(3,2),P1(1,1),P1(1,2),P1(2,1),P1(2,2),P1(3,1),P1(3,2));

flag = flag10 || flag11 || flag12 || flag13 || flag14 || flag15;

% *******************************************************************
end

function cross = getCross(x1,y1,x2,y2,x3,y3,x4,y4)
	cross = false;
	
    denom  = (y4-y3) * (x2-x1) - (x4-x3) * (y2-y1);
    a = (x4-x3) * (y1-y3) - (y4-y3) * (x1-x3);
    b = (x2-x1) * (y1-y3) - (y2-y1) * (x1-x3);

    % Are the line coincident? 
    if abs(a) < eps && abs(b) < eps && abs(denom) < eps
        cross = true;
    else
        % Are the line parallel 
        if abs(denom) < eps
            cross = false;
        else
            % Is the intersection along the the segments 
            mua = a / denom;
            mub = b / denom;
            if mua < 0 || mua > 1 || mub < 0 || mub > 1
                cross = false;
            else
                cross = true;
            end
        end
    end 	
end

function ret_val = ComputePoints(x1,y1,x2,y2,x3,y3)
	ret_val = (x1 - x3) * (y2 - y3) - (x2 - x3) * (y1 - y3);
end

function flag = PointInTriangle(ptx,pty,v1x,v1y,v2x,v2y,v3x,v3y)
	flag = false;
    b1 = false;
	b2 = false; 
	b3 = false;

	if (ComputePoints(ptx,pty,v1x,v1y,v2x,v2y) < 0)
		b1 = true;
	end
	if (ComputePoints(ptx,pty,v2x,v2y,v3x,v3y) < 0)
		b2 = true;
	end
	if (ComputePoints(ptx,pty,v3x,v3y,v1x,v1y) < 0)
		b3 = true;
	end	

	if ((b1 == b2) && (b2 == b3))
		flag = true;
	end
end