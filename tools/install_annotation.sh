cd $pwd ..

sudo cp src/rviz_cloud_annotation/lib/librviz_cloud_annotation_com.so  /usr/lib/librviz_cloud_annotation_com.so

sudo cp src/rviz_cloud_annotation/lib/librviz_cloud_annotation_plugin.so  /usr/lib/librviz_cloud_annotation_plugin.so

catkin_make

source devel/setup.bash
