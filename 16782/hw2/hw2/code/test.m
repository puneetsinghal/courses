

armstart = load('startConfiguration.txt');
armgoal = load('goalConfigurations.txt');

time = zeros(20,4);
numSamples = zeros(20,4);
planLength = zeros(20,4);

for i=1:20
    disp([i,j]);
    for j = 3:3
        [planLength(i,j), time(i,j), numSamples(i,j)] = ...
            runtest('map2.txt', armstart(i,:), armgoal(i,:), j-1);
    end
end