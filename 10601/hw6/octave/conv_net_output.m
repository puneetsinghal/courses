% ########################################################################
% #######     DO NOT MODIFY, READ ONLY IF YOU ARE INTERESTED       #######
% ########################################################################

function [cp, param_grad, output] = conv_net_output(params, layers, data, labels)
% Args:
% params: a cell array that stores hyper parameters
% layers: a cell array that defines LeNet
% data: input data with shape (784, batch size)
% labels: label with shape (batch size)

% Defines the CNN. It takes the configuration of the network structure (defined in layers), 
% the parameters of each layer (param), the input data (data) and label (labels) and
% does feed forward and backward propagation, returns the cost (cp) and
% gradient w.r.t all the parameters (param_grad).

% ########################## IMPORTANT ###################################
% WHEN THIS FUNCTION IS CALLED WITH ONLY ONE OUTPUT ARGUMENT, 
% IT PERFORMS ONLY FORWARD PROPAGATION
% WHEN IT IS CALLED WITH TWO OR MORE OUTPUT ARGUMENTS, 
% IT PERFORMS BOTH FORWARD AND BACKWARD PROPAGATION


l = length(layers);
batch_size = layers{1}.batch_size;
assert(strcmp(layers{1}.type, 'DATA') == 1, 'first layer must be data layer');

% prepare the output of data layer
output{1}.data = data;
output{1}.height = layers{1}.height;
output{1}.width = layers{1}.width;
output{1}.channel = layers{1}.channel;
output{1}.batch_size = layers{1}.batch_size;
output{1}.diff = 0;

% forward
for i = 2:l-1
    switch layers{i}.type
        case 'CONV'
            % forward of conv layer
            output{i} = conv_layer_forward(output{i-1}, layers{i}, params{i-1});
        case 'POOLING'
            % forward of pooling layer
            output{i} = pooling_layer_forward(output{i-1}, layers{i});
        case 'IP'
            % forward of inner product layer
            output{i} = inner_product_forward(output{i-1}, layers{i}, params{i-1});
        case 'RELU'
            % forward of relu layer
            output{i} = relu_forward(output{i-1}, layers{i});
        case 'ELU'
            % forward of elu layer
            output{i} = elu_forward(output{i-1}, layers{i});
    end
end
i = l;
assert(strcmp(layers{i}.type, 'LOSS') == 1, 'last layer must be loss layer');

% forward and backward of loss layer
wb = [params{i-1}.w(:); params{i-1}.b(:)];
[cost, grad, input_od, percent] = mlrloss(wb, output{i-1}.data, labels, layers{i}.num, 0, 1);
if nargout >= 2
    param_grad{i-1}.w = reshape(grad(1:length(params{i-1}.w(:))), size(params{i-1}.w));
    param_grad{i-1}.b = reshape(grad(end - length(params{i-1}.b(:)) + 1 : end), size(params{i-1}.b));
    param_grad{i-1}.w = param_grad{i-1}.w / batch_size;
    param_grad{i-1}.b = param_grad{i-1}.b /batch_size;
end

cp.cost = cost/batch_size;
cp.percent = percent;

% backward propagation
if nargout >= 2
    for i = l-1:-1:2
        switch layers{i}.type
            case 'CONV'
                % backward of conv layer
                output{i}.diff = input_od; % set the gradient of output{i}
                [param_grad{i-1}, input_od] = conv_layer_backward(output{i}, output{i-1}, layers{i}, params{i-1});
            case 'POOLING'
                % backward of pooling layer
                output{i}.diff = input_od;
                [input_od] = pooling_layer_backward(output{i}, output{i-1}, layers{i});
                % no parameter in pooling layer, set to empty
                param_grad{i-1}.w = [];
                param_grad{i-1}.b = [];
            case 'IP'
                % backward of inner product layer
                output{i}.diff = input_od;
                [param_grad{i-1}, input_od] = inner_product_backward(output{i}, output{i-1}, layers{i}, params{i-1});
            case 'RELU'
                % backward of relu layer
                output{i}.diff = input_od;
                [input_od] = relu_backward(output{i}, output{i-1}, layers{i});
                % empty parameter
                param_grad{i-1}.w = [];
                param_grad{i-1}.b = [];
            case 'ELU'
                % backward of elu layer
                output{i}.diff = input_od;
                [input_od] = elu_backward(output{i}, output{i-1}, layers{i});
                % empty parameter
                param_grad{i-1}.w = [];
                param_grad{i-1}.b = [];
        end
        % normalize by batch size
        param_grad{i-1}.w = param_grad{i-1}.w / batch_size;
        param_grad{i-1}.b = param_grad{i-1}.b / batch_size;
    end
end

end
