blocksV = [0 1 3];
trianglesV = [2];
TableIndex = 4;
onV_start = [0 1; 2 3; 1 4; 3 4];
clearV_start = [0 2];
onV_goal = [2 1];
clearV_goal = [2];
moveActionIndex = 0;
moveToTableActionIndex = 1;
tic
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);
toc