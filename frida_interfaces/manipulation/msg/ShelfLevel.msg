# One detected shelf board, in base_link (z up). Heights are the board's TOP surface.
float32 height      # base_link Z of the surface an object rests on
float32 thickness   # board thickness (default when the underside was not seen)
float32 front_x     # nearest x of the surface seen
float32 back_x      # farthest x of the surface seen
float32 y_min
float32 y_max
float32 clearance   # free height above the surface up to the next board (0 = unknown)
int32 num_points    # surface points that voted for this level
