# تشغيل روبوت Teknofest

## ⚠️ مهم جداً: نظف النظام أولاً

قبل البدء، نظف جميع العمليات القديمة:
```bash
killall -9 gz gzclient rviz2 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
ros2 daemon stop
sleep 2
ros2 daemon start
```

## خطوات التشغيل

افتح **4 نوافذ Terminal** منفصلة:

### Terminal 1 - المحاكاة
```bash
cd $HOME/tekno_robot_simulation
source install/setup.bash
ros2 launch articubot_one launch_sim.launch.py \
  world:=./src/articubot_one/worlds/teknofest_map.world \
  gui:=false
```
**انتظر 5-10 ثواني حتى يبدأ Gazebo**

### Terminal 2 - واجهة Gazebo (اختياري)
```bash
gzclient
```

### Terminal 3 - SLAM
```bash
cd $HOME/tekno_robot_simulation
source install/setup.bash
ros2 launch articubot_one online_async_launch.py use_sim_time:=true
```

### Terminal 4 - RViz
```bash
cd $HOME/tekno_robot_simulation
source install/setup.bash
ros2 run rviz2 rviz2
```

## فحص النظام

### تحقق من Topics النشطة:
```bash
ros2 topic list
```

يجب أن ترى:
- `/scan` - LiDAR
- `/odom` - Odometry
- `/tf` و `/tf_static` - Transforms
- `/robot_description`
- `/joint_states`
- `/camera/image_raw`

### تحقق من Controllers:
```bash
ros2 control list_controllers
```

### إذا لم تعمل Controllers:
هذه مشكلة معروفة في gz_ros2_control. الحل:
1. البيانات الأساسية (scan, camera) تعمل عبر ros_gz_bridge
2. Odometry يأتي من Gazebo مباشرة
3. لا حاجة لـ lidar_py_pkg

## إيقاف النظام بشكل صحيح

في كل Terminal اضغط `Ctrl+C`

إذا لم تتوقف العمليات:
```bash
pkill -9 -f "gz sim"
pkill -9 -f ros_gz
pkill -9 -f rviz2
```

## حفظ الخريطة

بعد استكشاف البيئة:
```bash
cd $HOME/tekno_robot_simulation
ros2 run nav2_map_server map_saver_cli -f my_map
```

## ملاحظات مهمة

1. **لا تستخدم** السكريبتات المعقدة - استخدم الأوامر الأصلية فقط
2. **افتح terminals منفصلة** لكل أمر
3. **انتظر** بين كل خطوة حتى يستقر النظام
4. **RViz قد يتجمد** - هذه مشكلة معروفة، أعد تشغيله
