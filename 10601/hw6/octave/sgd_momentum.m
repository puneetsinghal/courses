function [params, param_winc] = sgd_momentum(w_rate, b_rate, mu, weight_decay, params, param_winc, param_grad)
% update the parameter with sgd with momentum

%% function input
% w_rate (scalar): learning rate for w at current step
% b_rate (scalar): learning rate for b at current step
% mu (scalar): momentum
% weight_decay (scalar): weigth decay of w
% params (cell array): original weight parameters
% param_winc (cell array): buffer to store history gradient accumulation
% param_grad (cell array): gradient of parameter

%% function output
% params (cell array): updated parameters
% param_winc (cell array): updated buffer

    for i =1:length(params)
        param_winc{i}.w = mu*param_winc{i}.w + w_rate*(param_grad{i}.w + weight_decay*params{i}.w);
        params{i}.w = params{i}.w  - param_winc{i}.w;
        param_winc{i}.b = mu*param_winc{i}.b + b_rate*param_grad{i}.b;
        params{i}.b = params{i}.b  - param_winc{i}.b;
    end
end
