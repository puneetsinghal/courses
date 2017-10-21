function [input_od] = relu_backward(output, input, layer)
% RELU backward

% Args:
% output: a cell array contains output data and shape information
% input: a cell array contains input data and shape information
% layer: one cnn layer, defined in testLeNet.m

% Returns:
% input_od: gradients w.r.t input data

input_od = (input.data > 0) .*output.diff;
end
