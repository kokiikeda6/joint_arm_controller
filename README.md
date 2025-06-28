# joint_arm_controller
piper2台でリーダフォロワを行う

#### 使用方法
1. piper(フォロワ)とpiper(リーダ)にそれぞれcan to usbを取り付け，PCと接続(フォロワを接続してからリーダを接続)
2. portの登録
```
cd piper_ros
sudo ethtool -i can0 | grep bus #フォロワのbus確認
bash can_activate.sh can0 1000000 "3-1:1.0" # 確認したbus入力
sudo ethtool -i can1 | grep bus #リーダのbus確認
bash can_activate.sh can1 1000000 "3-3:1.0" # 確認したbus入力

```
3. piperを2台ともスレーブモードにする
```
cd piper_sdk_demo/V2
python3 piper_slave_config.py　# デフォルトではcan0が指定されている
```
4. node起動
```
# ターミナル1
ros2 launch piper start_single_piper.launch.py can_port:=can1 auto_enable:=false gripper_exist:=true gripper_val_mutiple:=2

# ターミナル2
ros2 run joint_arm_controller joint_publisher 

```

5. 角度指定
```
#joint1 -150.0 ~ 150.0
ros2 param set /piper_arm_node joint1 60.0
```