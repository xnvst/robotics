% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

% occupancy value of unexplored pixels
unknown = mode(reshape(map, size(map,1)*size(map,2), 1));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
% myResolution = param.resol;
% % the origin of the map in pixels
% myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%M =                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
%P = repmat(myPose(:,1), [1, M]);

% for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
% 
%     % 1) Propagate the particles 
%
%       
%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
%
%     %   2-2) For each particle, calculate the correlation scores of the particles
%
%     %   2-3) Update the particle weights         
%  
%     %   2-4) Choose the best particle to update the pose
%     
%     % 3) Resample if the effective number of particles is smaller than a threshold
% 
%     % 4) Visualize the pose on the map as needed
%    
% 
% end

resol = param.resol; % map resolution
origin = param.origin; % origin in pixel

sig = [0.08, 0, 0; 0, 0.08, 0; 0, 0, 0.08]; % noise for particle movement

myPose(:,1) = param.init_pose; % init position

M = 200; % number of particles

P = repmat(myPose(:,1), [1, M]);

thr = ceil(3.5/5*size(scanAngles,1)); % set the score threshold as 70%

for j = 2:N
    maxscore = 0;
    while maxscore < thr
        Q=P+(randn(size(P,2),3)*sig)'; % particles movement
        score = zeros(size(Q,2), 1); % scores
        for k = 1:size(Q,2) % calculate score for each particle
            occ_x = ceil( (ranges(:,j) .* cos(scanAngles+Q(3,k)) + Q(1,k) )  * resol + origin(1) );
            occ_y = ceil( (-ranges(:,j) .* sin(scanAngles+Q(3,k)) + Q(2,k) ) * resol + origin(2) );
            ids = occ_x > 0 & occ_x <= size(map,2) & occ_y > 0 & occ_y <= size(map,1);
            score(k) = size( map( map (sub2ind(size(map), occ_y(ids), occ_x(ids)) ) > unknown ), 1);
        end
        [maxscore, index] = max(score); % select particle with maximum score
    end
    myPose(:,j) = Q(:,index); % set pose(j) as the optimal particle

    Q = Q(:,score >= thr); % select particles with high score
    P = repmat(Q, 1, ceil(M/size(Q,2)) ); % regenerate particles
end

end

