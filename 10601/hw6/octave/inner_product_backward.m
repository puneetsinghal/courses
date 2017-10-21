function [param_grad, input_od] = inner_product_backward(output, input, layer, param)
% Fully connected layer backward

% Args:
% output: a cell array contains output data and shape information
% input: a cell array contains input data and shape information
% layer: one cnn layer, defined in testLeNet.m
% param: parameters, a cell array

% Returns:
% para_grad: a cell array stores gradients of parameters
% input_od: gradients w.r.t input data

param_grad.b = sum(output.diff, 2)';
param_grad.w = input.data*output.diff';
input_od = param.w*output.diff;

end
