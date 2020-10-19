
predicted_coord = [1,30,70];
sdf_size = [10,12,13];



rows_range  = (obj.map_size(1) + 1) - predicted_coord(1):predicted_coord(1) + sdf_size(1) - 1;
cols_range  = predicted_coord(2):predicted_coord(2) + sdf_size(2) - 1;
z_range     = predicted_coord(3):predicted_coord(3) + sdf_size(3) - 1;

flip_rows_range = obj.map_size(1) + 1 - rows_range;
                
                
                
                valid_row_mask  = flip_rows_range    >= 1 & flip_rows_range   <= obj.map_size(1);
valid_col_mask  = cols_range    >= 1 & cols_range   <= obj.map_size(2);
valid_z_mask    = z_range       >= 1 & z_range      <= obj.map_size(3);

flip_rows_range  = flip_rows_range(valid_row_mask);
cols_range  = cols_range(valid_col_mask);
z_range     = z_range(valid_z_mask);

new_sdf_rows_range  = 1:sdf_size(1);
new_sdf_rows_range  = new_sdf_rows_range(valid_row_mask);
new_sdf_cols_range  = 1:sdf_size(2);
new_sdf_cols_range  = new_sdf_cols_range(valid_col_mask);
new_sdf_z_range     = 1:sdf_size(3);
new_sdf_z_range     = new_sdf_z_range(valid_z_mask);


                
                arr1 = 1:10;
mask1 = logical([0,1,1,1,1,1,0,0,1,1]);

arr1 = arr1(mask1);