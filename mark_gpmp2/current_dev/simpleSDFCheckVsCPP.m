%% For in the C++ test script
map = zeros(5,5,5);
map(1,1,1) = 1;
cell_size = 1;

field = signedDistanceField3D(map, cell_size);

%% To plot the output of a c++ matrix