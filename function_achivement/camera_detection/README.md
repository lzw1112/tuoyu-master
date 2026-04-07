1 打开相机节点，读取摄像头信息
在工作空间内 source devel/setup.bash
打开相机节点 rosrun camera_implementation camera_node
显示图像 rosrun image_view image_view image/:=camera/image

2 边缘检测
打开节点 rosrun camera_implementation canny_node
显示图像 rosrun image_view image_view image/:=camera/canny_image

3 霍夫变换检测直线
打开节点 rosrun camera_implementation hough_node
显示图像 rosrun image_view image_view image/:=camera/hough_image

4 行人&车辆检测
打开节点 python3 detect_node.py
显示图像 rosrun image_view image_view image/:=camera/detect_image
