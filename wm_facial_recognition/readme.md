**Script pour la demo du 1er novembre 2016.**
Ce script sert à faire dire à sara le nombre de persones qui ce trouvent devant elle.

**Comment utiliser le script**
Pour utiliser ce script :

* Rouler le launcher de la vision
```shell
roslaunch openni2_launch openni2.launch 
```
* Rouler le launcher du script
```shell
roslaunch wm_facial_recognition facerecognition.launch
```
* Ordonner a sara de parler
```python
#import the necessary libraries
from std_msgs.msg import Bool

#register the publisher on the std_msgs.msg.Bool topic "facial_recognition/start"
self.pub_start = rospy.Publisher('facial_recognition/start', std_msgs.msg.Bool,  queue_size=1)

#sent True on the topic facial_recognition/start
self.pub_start.publish(True)
```

* Reccuperer le signal de fin de job
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
