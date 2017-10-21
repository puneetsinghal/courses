function [input_od] = pooling_layer_backward(output, input, layer)
% Pooling backward

% Args:
% output: a cell array contains output data and shape information
% input: a cell array contains input data and shape information
% layer: one cnn layer, defined in testLeNet.m

% Returns:
% input_od: gradients w.r.t input data

    h_in = input.height;
    w_in = input.width;
    c = input.channel;
    batch_size = input.batch_size;
    k = layer.k;

    input_od = zeros(size(input.data));
    h_out = output.height;
    w_out = output.width;
    % Max pooling
    input_n.height = h_in;
    input_n.width = w_in;
    input_n.channel = c;
    for n = 1:batch_size
        temp_data_diff = reshape(output.diff(:, n), [h_out*w_out, c]);
        temp_data_diff = reshape(temp_data_diff', [1, c, h_out, w_out]);
        temp_data_diff = repmat(temp_data_diff, [k*k, 1, 1, 1]);
        temp_data = reshape(output.data(:, n), [h_out*w_out, c]);
        temp_data = reshape(temp_data', [1, c, h_out, w_out]);
        temp_data = repmat(temp_data, [k*k, 1, 1, 1]);
        input_n.data = input.data(:, n);
        col = im2col_conv(input_n, layer, h_out, w_out);
        col = reshape(col, [k*k, c, h_out, w_out]);
        % col_diff = temp_data_diff.*(temp_data == col);
        col_diff = temp_data_diff.*(abs(temp_data - col) < eps);
        im = col2im_conv(col_diff(:), input, layer, h_out, w_out);
        input_od(:, n) = im(:);
    end

end
