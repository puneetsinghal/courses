blocksV = [0 1 3 4 5 6 7 8];
trianglesV = [2];
TableIndex = 9;
onV_start = [2 0;0 1;1 3;3 4;4 5;5 6;6 7;7 8; 8 9];
clearV_start = [2];
onV_goal = [2 8];
clearV_goal = [2];
moveActionIndex = 0;
moveToTableActionIndex = 1;
runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);
% 
% pause();
% 
% blocksV = [0 1 3 4 5 6 7 8 9];
% trianglesV = [2];
% TableIndex = 10;
% onV_start = [2 0;0 1;1 3;3 4;4 5;5 6;6 7;7 8; 8 9; 9 10];
% clearV_start = [2];
% onV_goal = [2 9];
% clearV_goal = [2];
% moveActionIndex = 0;
% moveToTableActionIndex = 1;
% runtest(blocksV, trianglesV, TableIndex, onV_start, clearV_start, onV_goal, clearV_goal, moveActionIndex, moveToTableActionIndex);