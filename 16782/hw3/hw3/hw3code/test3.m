blocksV = [0 1 3 5 6 7 8 9];
trianglesV = [2 4];
TableIndex = 10;
onV_start = [0 1; 2 0; 5 3; 6 5; 4 6; 8 7; 9 8; 1 10; 3 10; 7 10];
clearV_start = [2 4 9];
onV_goal = [6 9; 1 6; 0 1; 4 0];
% onV_goal = [6 8; 1 6; 0 1; 4 0];
clearV_goal = [4];
moveActionIndex = 0;
moveToTableActionIndex = 1;
tic
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);
toc