<launch>
    <!-- 启动 realsense2_camera 节点，但将输出定向到日志文件 -->
    <include file="$(find realsense2_camera)/launch/vins_rs_camera.launch" />

    <!-- 启动 vins_node 节点，并将其输出显示在终端上 -->
    <node name="vins" pkg="vins" type="vins_node" args="/home/cadc-pc/Desktop/CADC-2023/cadc_ws/src/vins-fusion-gpu/config/realsense_d435i/realsense_stereo_imu_config.yaml" output="screen" />
</launch>