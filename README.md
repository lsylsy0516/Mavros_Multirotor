# Mavros_Multirotor
vins-fusion 本身的是FLU系 即 前左上
根据我搜索的结果，vision_to_pose应该传入ENU（东北天）坐标系或者body系12，而local_position/pose是ENU坐标系34。mavros会自动将输入的坐标值转换到NED坐标系中，再通过mavlink发送给飞控

vision所发布的就是local所对应的
mavros/vision_pose/pose == mavros/local_position/pose 
当我/mavros/local_position/pose 发布为ENU时 
QGC上看的local_position_end 为正确的，yeah！
