dim_size = size(Samples)
N = dim_size(1)

mu = mean(Samples); % mu

sig=zeros(3,3); % sigma
for i=1:N
	data=double(Samples(i,:));
	sig=sig+(data-mu)'*(data-mu)/N;
end

% save the coefficients to files
save('mu.mat', 'mu');
save('sig.mat', 'sig');