地図を作るためのpackage

How to do 
$ rosrun hsr_hector make_map.sh
・#地図を作るためにHSR移動中 
・↓ある程度できたら 
$ rosrun map_server map_saver

そしたらmap.yamlとmap.pgmができてるはず。
/usr/ros_ws/hsr_ws/src/erasers_navigation/navigation/navigation_start/maps/{new_directory}
/usr/ros_ws/hsr_ws/src/erasers_navigation/navigation/navigation_start/prohibition_maps/{new_directory}
新しいディレクトリを作成して作成したファイルを上記の２箇所へコピーする。

navigation_start/launchのstatic_map_file ,prohibition_map_fileパラメータを新しいディレクトリの名前にする

<arg name="static_map_file"  default="$(find navigation_start)/maps/maps/{new_directory}/map.yaml"/>
<arg name="prohibition_map_file"  default="$(find navigation_start)/maps/prohibition_maps/{new_directory}/map.yaml"/>

RVIZを立ち上げて反映されているか確認する。
roscd navigation_start/rviz
rviz -d config.rviz














そしてHSRの内部の/etc/opt/tmc/robot/conf.d/{new_directory}に置く 
/etc/opt/tmc/robot/docker.hsrb.user
のMAP_PATHに{new_directory}までのパスを書く。 
$ sudo systemctl restart docker.hsrb.robot
「HSR スタート」と言ったら緊急停止を押して、解除... 
多分地図が変わってる。

依存関係
hector_mapping
(tmcのパッケージの依存になってるhazu)
