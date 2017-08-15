imagepath = './train';
Samples = [];
for k=1:19
    I = imread(sprintf('%s/%03d.png',imagepath,k)); % read files into RGB
    R = I(:,:,1);
    G = I(:,:,2);
    B = I(:,:,3);
    
    % Collect samples 
    disp('');
    disp('INTRUCTION: Click along the boundary of the ball. Double-click when you get back to the initial point.')
    disp('INTRUCTION: You can maximize the window size of the figure for precise clicks.')
    figure(1), mask = roipoly(I); 
    figure(2), imshow(mask); title('Mask');

    sample_ind = find(mask > 0); % select marked pixels
    
    R = R(sample_ind);
    G = G(sample_ind);
    B = B(sample_ind);
    
    Samples = [Samples; [R G B]]; % insert selected pixels into samples
    
    disp('INTRUCTION: Press any key to continue. (Ctrl+c to exit)')
    pause
end

save('Samples.mat', 'Samples'); % save the samples to file

figure, 
scatter3(Samples(:,1),Samples(:,2),Samples(:,3),'.');
title('Pixel Color Distribubtion');
xlabel('Red');
ylabel('Green');
zlabel('Blue');