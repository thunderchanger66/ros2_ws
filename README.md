# 注意事项
start_detect集成了人脸检测和结果展示  

# 安装依赖
python inquirements: dlib scikit-image opencv-python numpy pandas  

# 编译
colcon build  
source ./install/setup.bash  

# 启动
ros2 launch start_detect run_all.launch.py
