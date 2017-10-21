format long

image = [0 0 4 0;...
    0 2 5 2;...
    0 0 1 0;...
    0 0 2 0];
k1 = [1 1; 2 0];
k2 = [2 1; 0 2];


w = [1, 0.5, 1, 2,   -1,   1,  1, -1;...
0.5,   1, 2, 1, -0.5, 0.5, -1,  1;...
0.25,  -1, 1, 1,  0.5,  -1, -1, -1];

b= [0; 0; 0];

afterPooling = [8, 14, 7, 9, 14, 14, 11, 12]';

ip_output = w*afterPooling + b;

relu_output = max(ip_output,0);

softmax_output = exp(relu_output)/sum(exp(relu_output));

%%
-log(softmax_output);

-1/softmax_output(2)*(...
    (sum(exp(relu_output))*exp(relu_output(2))*0-exp(relu_output(2))*exp(relu_output(1)))/sum(exp(relu_output))^2*1*afterPooling(1) + ...
    (sum(exp(relu_output))*exp(relu_output(2))*1-exp(relu_output(2))*exp(relu_output(2)))/sum(exp(relu_output))^2*1*0 + ...
    (sum(exp(relu_output))*exp(relu_output(2))*0-exp(relu_output(2))*exp(relu_output(3)))/sum(exp(relu_output))^2*0*0)


-1/softmax_output(2)*(...
    (sum(exp(relu_output))*exp(relu_output(2))*0-exp(relu_output(2))*exp(relu_output(1)))/sum(exp(relu_output))^2*1*...
        (w(1,1)*image(1,2) + w(1,2)*image(1,3) + w(1,3)*image(2,2) + w(1,4)*image(2,3)) + ...
    (sum(exp(relu_output))*exp(relu_output(2))*1-exp(relu_output(2))*exp(relu_output(2)))/sum(exp(relu_output))^2*1*...
        (w(2,1)*image(1,2) + w(2,2)*image(1,3) + w(2,3)*image(2,2) + w(2,4)*image(2,3)) + ...
    (sum(exp(relu_output))*exp(relu_output(2))*0-exp(relu_output(2))*exp(relu_output(3)))/sum(exp(relu_output))^2*0*...
        (w(3,1)*image(1,2) + w(3,2)*image(1,3) + w(3,3)*image(2,2) + w(3,4)*image(2,3)))
    
    
    
    
    






