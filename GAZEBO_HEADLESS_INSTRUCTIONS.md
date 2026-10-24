# تعليمات التشغيل - Gazebo Headless Mode مع RViz

## المشكلة
Gazebo يفشل بسبب مشكلة GLX في الرسوميات:
```
X Error of failed request: GLXBadFBConfig
```

## الحل
تشغيل Gazebo بدون واجهة رسومية (headless mode) واستخدام RViz للمراقبة.

---

## طريقة التشغيل

### Terminal 1: تشغيل Gazebo والروبوت (بدون واجهة)
```bash
cd $HOME/tekno_robot_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch articubot_one launch_sim.launch.py
```

**ملاحظة:** لن ترى نافذة Gazebo، لكن المحاكاة تعمل في الخلفية مع جميع الحساسات (الليدار والكاميرا).

---

### Terminal 2: تشغيل RViz للمراقبة
```bash
cd $HOME/tekno_robot_simulation
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch articubot_one rviz_view.launch.py
```

---

## إعدادات RViz الموصى بها

بعد فتح RViz، أضف العناصر التالية:

1. **RobotModel** - لعرض نموذج الروبوت
2. **LaserScan**
   - Topic: `/scan`
3. **Camera**
   - Topic: `/camera/image_raw`
4. **TF** - لعرض إطارات الإحداثيات
5. **Odometry**
   - Topic: `/odom`

تأكد من:
- تعيين Fixed Frame إلى `odom` أو `base_link`
- تفعيل `use_sim_time` في Global Options

---

## للتحكم بالروبوت

### باستخدام لوحة التحكم (Joystick):
الإعدادات موجودة بالفعل في launch_sim.launch.py

### باستخدام لوحة المفاتيح (Terminal 3):
```bash
source /opt/ros/jazzy/setup.bash
source $HOME/tekno_robot_simulation/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/cmd_vel_joy
```

---

## التحقق من المواضيع (Topics)

```bash
# عرض جميع المواضيع
ros2 topic list

# التحقق من بيانات الليدار
ros2 topic echo /scan

# التحقق من بيانات الكاميرا
ros2 topic hz /camera/image_raw

# التحقق من موضع الروبوت
ros2 topic echo /odom
```

---

## إيقاف المحاكاة

في Terminal 1 (Gazebo):
```bash
Ctrl+C
```

في Terminal 2 (RViz):
```bash
Ctrl+C
```

---

## ملاحظات مهمة

1. **جميع الحساسات تعمل**: الليدار والكاميرا وجميع العقد تعمل حتى بدون واجهة Gazebo
2. **الأداء أفضل**: بدون واجهة رسومية، المحاكاة أسرع وتستهلك موارد أقل
3. **RViz للمراقبة فقط**: RViz لا يحاكي الفيزياء، فقط يعرض البيانات من Gazebo
4. **متغيرات البيئة**: تم إضافة متغيرات لفرض الرسم البرمجي وتجنب أخطاء GPU

---

## استكشاف الأخطاء

### إذا استمرت مشكلة GLX:
جرّب إضافة هذه المتغيرات قبل التشغيل:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export LIBGL_ALWAYS_INDIRECT=1
ros2 launch articubot_one launch_sim.launch.py
```

### إذا كانت الصورة من الكاميرا بطيئة:
يمكنك تعطيل image_bridge مؤقتاً في launch_sim.launch.py

---

## التعديلات المُنفذة

1. ✅ تحديث `launch_sim.launch.py` لإضافة متغيرات بيئة إضافية
2. ✅ إزالة `--headless-rendering` من معاملات Gazebo (يسبب مشاكل)
3. ✅ إنشاء `rviz_view.launch.py` لتشغيل RViz بشكل منفصل
4. ✅ جميع الحساسات والعقد تعمل (الليدار، الكاميرا، المحركات)
