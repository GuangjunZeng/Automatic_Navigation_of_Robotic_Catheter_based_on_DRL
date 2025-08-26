load('trained_net_2inputs_2outputs.mat', 'net');

matrix = [
    129, -402, 380;
    754.275, 636.359, 629.983
];

predicted_input = matrix;

predicted_output = net(predicted_input);