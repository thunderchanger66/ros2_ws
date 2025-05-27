# 注意事项
show_py为主要功能包，test.py为可以使用的代码  
ai_msgs为适配RDK发送的自定义消息  
show_image为前期c++测试功能包  
需要先放入照片至show_py/Resources/faceS/<your_name>/，运行getFaceDB.py存入数据  

# 安装依赖
python inquirements: dlib scikit-image opencv-python numpy pandas  

# 编译
colcon build  
source ./install/setup.bash  

# 启动
ros2 run show_py test  
