# 概要

Kohakuシリーズのロボット実機のROSインタフェースを提供するROSパッケージです。

# Nodes

## kohaku_ros_driver_node

KohakuModel4実機と通信を行い、センサ値をトピックの形で出力するとともに各種機能をサービスで提供するノード。

### Subscribed topics

なし

### Published topics

- joint_states (sensor_msgs/JointState)

 関節角度、トルク、関節速度を配信するトピック

- joint_currents (std_msgs/Float64MultiArray)

 関節の電流値を配信するトピック

### Services

- servo_all_on(std_srvs/Empty)

 全軸のサーボをONにする

- servo_all_off(std_srvs/Empty)

 全軸のサーボをOFFにする

- go_to_home_position(std_srvs/Empty)

 ホーム位置に移動する

- go_to_rest_position(std_srvs/Empty)

 アームレスト上の待機位置に移動する

- alarm_reset(kohaku_ros_driver/SetJointNo)

 関節を指定してアラームをリセットする

- servo_on(kohaku_ros_driver/SetJointNo)

 関節を指定してサーボをONにする

- servo_off(kohaku_ros_driver/SetJointNo)

 関節を指定してサーボをOFFにする

- set_control_mode(kohaku_ros_driver/SetInt8Array)

 各関節の制御モードを設定する。1: 位置制御、2: 速度制御、3: 電流制御、4: トルク制御。

- get_control_mode(kohaku_ros_driver/GetInt8Array)

 各関節の制御モードを取得する

- set_joint_trajectory(kohaku_ros_driver/SetJointTrajectory)

 各関節に対して目標値と遷移時間を指定して補間動作を実行する

- enable_zerog_mode(std_srvs/SetBool)

 ZeroGモードを有効／無効にする

- grasp(std_srvs/Empty)

 接触があるまでハンドを握り込む動作を実行する

### Parameters

- ip_addr 接続しようとする腕を制御するコントローラのIPアドレス。デフォルト値は '192.168.1.238'
- sampling_rate センサー値を出力する周期。デフォルトは100Hz
