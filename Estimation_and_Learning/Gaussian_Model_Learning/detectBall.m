% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
% mu = 
% sig = 
% thre = 

load('mu.mat');
load('sig.mat');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
Id=double(I); % array in size of (row, col, 3)

row=size(Id,1); 
col=size(Id,2);

% x_i - mu
for i=1:3
	Id(:,:,i) = Id(:,:,i) - mu(i);
end

% reshape the image to a matrix in size of (row*col, 3)
Id=reshape(Id,row*col,3);
 
% calc possibility using gaussian distribution
% be careful of using * and .* in matrix multiply
Id = exp(-0.5* sum(Id*inv(sig).*Id, 2)) ./ (2*pi)^1.5 ./ det(sig)^0.5;

% reshape back, now each pixels is with the value of the possibility
Id=reshape(Id,row,col);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%

% segI = 
% loc = 
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

% set threshold
thr=8e-06;

% binary image about if each pixel 'is ball'
Id=Id>thr;

% find the biggest ball area
segI = false(size(Id));

CC = bwconncomp(Id);
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
segI(CC.PixelIdxList{idx}) = true;
%figure, imshow(segI); hold on;

S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
%plot(loc(1), loc(2),'r+');

end
