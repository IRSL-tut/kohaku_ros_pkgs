# kohaku_ros_pkgs #

KohakuシリーズのロボットをROSで制御するためのパッケージ群です。

### 依存ライブラリ ###

* libhr4c_comm: hr4cプロトコル実装ライブラリ。バイナリファイルの形式で提供されます。

### 依存パッケージ ###

* realsense_gazebo_plugin

gazeboにてRealsense D435のカメラのエミュレーションを行うために以下のパッケージを
catkin_ws/srcフォルダにgit cloneし、ビルドする必要があります。
https://github.com/pal-robotics/realsense_gazebo_plugin

### ビルド ###

catkinワークスペースで以下を入力します。

```bash
catkin build
source devel/setup.bash
```
### 実機とコントローラ単独での立ち上げ(Kohaku DualArm)

本プログラムを作動するコンピュータとロボット制御ボードがネットワーク接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch kohaku_ros_driver kohaku_dualarm_control.launch
```

### 実機とコントローラ+MoveItでの立ち上げ(Kohaku DualArm)

本プログラムを作動するコンピュータとロボット制御ボードがネットワーク接続された状態で、
任意のディレクトリ内で以下を入力します。

```bash
roslaunch kohaku_launch kohaku_dualarm_bringup.launch
```

### シミュレータ+MoveItでの立ち上げ(Kohaku DualArm)

任意のディレクトリ内で以下を入力します。

```bash
roslaunch kohaku_launch kohaku_dualarm_bringup.launch sim:=true
```

以上。
