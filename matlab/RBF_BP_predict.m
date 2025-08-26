% Test the combined network with new input data
% Load the trained network from a file

load('C:\Users\admin\Desktop\Labview2018\RBF_BP_train.mat', 'combined_net');

test_input = [0.5 0.5 1; 0.2 0.8 2]; % Arbitrary number of test input samples
predicted_output = combined_net(test_input);

disp("Test Input:");
disp(test_input);
disp("Predicted Output:");
disp(predicted_output);



