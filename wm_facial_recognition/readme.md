**Script pour la demo du 1^^er^^ novembre 2016.**
Ce script sert à faire dire à sara le nombre de persones qui ce trouvent devant elle.

**Comment utiliser le script**
Pour utiliser ce script :

1. Rouler le launcher de la vision 1
```shell
roslaunch openni2_launch openni2.launch 
```
2. Rouler le launcher du script 2
```shell
roslaunch wm_facial_recognition facerecognition.launch
```
3. Ordonner a sara de parler 3
```python
#import the necessary libraries
from std_msgs.msg import Bool

#register the publisher on the std_msgs.msg.Bool topic "facial_recognition/start"
self.pub_start = rospy.Publisher('facial_recognition/start', std_msgs.msg.Bool,  queue_size=1)

#sent True on the topic facial_recognition/start
self.pub_start.publish(True)
```

4. Reccuperer le signal de fin de job 4
```python
#import the necessary libraries
from std_msgs.msg import Bool

#register the subscriber on the std_msgs.msg.Bool topic "facial_recognition/stop"
peopleSub = rospy.Subscriber("facial_recognition/stop", std_msgs.msg.Bool, callbackPeople)

#read True in the topic facial_recognition/stop in result
result = False
def callbackPeoble(data):
  result = data
```
