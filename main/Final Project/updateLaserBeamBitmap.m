
% function assumes that a laser scanner was already used to 'scan' the
% environment. The scanner has returned an angle vector and a range vector.
% The scanner's pose in world coordinates is given by Tl. The scanner shot rays from
% -angleSpan/2 to +angleSpan/2 with step angleStep.
% Now, the robot's perception of the enviroment is the bitmap (boolean occupancy grid with obstacles) with origin at (0,0) and uper NE corner (Xmax, Ymax),
% This function uses the laser scanner output to populate the obstacle pixels in the bitmap 
% and the free space (free pixels) in the bitmap.
function p = updateLaserBeamBitmap(angle, range, Tl, R, C, Xmax, Ymax)
global bitmap; % this is the robot's perceived environment, not the actual environment

%transform laser origin to world frame
P1 = Tl*[0 0 1]';
x1=P1(1);     y1=P1(2);

if (isinf(range)) % handle Inf return values
    range = Xmax^2+Ymax^2;  % assign arbitrary huge value
end

%first produce target point for laser in scanner frame
Xl = range * cos(angle);
Yl = range * sin(angle);

%Transform target point in world frame
P2 = Tl*[Xl Yl 1]';
x2=P2(1); y2=P2(2);

%clip laser beam to boundary polygon so that 'infinite' (rangeMax) range
% extends up to the boundary
dy = y2-y1; dx = x2-x1;
% ATTENTION: if dy or dx is close to 0 but negative, make it almost zero
% positive
if (abs(y2-y1)) < 1E-3
    dy = 1E-6;
end
if (abs(x2-x1)) < 1E-3
    dx = 1E-6;
end
edge = clipLine([x1,y1,dx,dy],[0 Xmax 0 Ymax]);
%laser origin is always inside map
%decide if clipping is necessary
l1 = sqrt( (x1-edge(3))^2 + (y1 - edge(4))^2);
if range >= l1
    x2 = edge(3); y2 = edge(4);
end

% map the laser and obstacle world points to integer coordninates
[ I1, J1 ] = XYtoIJ(x1, y1, Xmax, Ymax, R, C); % laser source -> pixel
[ I2, J2 ] = XYtoIJ(x2, y2, Xmax, Ymax, R, C); % detected obstacle pixel

%update detected obstacle pixel
bitmap(I2, J2) = 1;
% use bresenham to find all pixels that are between laser and obstacle
l=bresenhamFast(I1,J1,I2,J2);
%[l1 l2]=size(l);
for k=1:length(l)-1 %skip the target pixel
    bitmap(l(k,1),l(k,2)) = 0; % free pixels
end

p = length(l) + 1;  % number of updated pixels

end

