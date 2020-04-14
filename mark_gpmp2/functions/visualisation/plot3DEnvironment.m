function plot3DEnvironment(dataset, X, Y, Z)
%PLOT3DENVIRONMENT Summary of this function goes here
%   Detailed explanation goes here


    isovalue=0.999;
    
    if isstruct(dataset)
        data = dataset.map;
    else
        data = dataset;
    end
    
    % Get or calculate the mesh grid
    if  nargin <2       
        % If the data comes from our simulation
        if isstruct(dataset)
            [X, Y, Z] = getEnvironmentMesh(dataset);
        end

        % If the input is a test matrix
        if isfloat(dataset)
            grid_X = 1 : size(data, 1);
            grid_Y = 1 : size(data, 2);
            grid_Z = 1 : size(data, 3);
            [X, Y, Z] = meshgrid(grid_X,grid_Y,grid_Z);
        end
            
    end
    
    % Plot
    surf1=isosurface(X, Y, Z, data, isovalue);
    p1 = patch(surf1);
%         isonormals(X, Y, Z, data, p1);
%         set(p1,'FaceColor','red','EdgeColor','none','FaceAlpha',0.1); % set the color, mesh and transparency level of the surface
%         title('Environment');
%     xlabel('x');
%     ylabel('y');
%     zlabel('z');
    
end

