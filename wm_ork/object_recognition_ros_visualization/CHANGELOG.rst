^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package object_recognition_ros_visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.7 (2015-06-25)
------------------
* Merge pull request `#4 <https://github.com/wg-perception/object_recognition_ros_visualization/issues/4>`_ from v4hn/fix-empty-hull-segfault
  fix segfault when table with empty hull is received
* fix segfault when table with empty hull is received
* update conf file for docs
* Contributors: Vincent Rabaud, v4hn

0.3.6 (2014-12-21)
------------------
* Merge pull request `#3 <https://github.com/wg-perception/object_recognition_ros_visualization/issues/3>`_ from v4hn/colorful-tables
  Add color property to table display
* Add color property to table display
  Now tables can have different colors.
  This is helpful if you get tables from different sources...
* Merge pull request `#2 <https://github.com/wg-perception/object_recognition_ros_visualization/issues/2>`_ from v4hn/moc-boost-1.57
  Fix build with qt4's moc & boost 1.57
* Fix build with qt4's moc & boost 1.57
  This is a common workaround to make sure moc doesn't see
  preprocessorvariables it doesn't like in boost...
* Contributors: Michael Görner, Vincent Rabaud

0.3.5 (2014-09-20)
------------------
* remove useless dependency
* Contributors: Vincent Rabaud

0.3.4 (2014-09-20)
------------------
* clean up dependencies
* Contributors: Vincent Rabaud

0.3.3 (2014-09-18)
------------------
* add documentation
* clean CMake and package.xml
* remove the ecto cells for the object_recognition_msgs
  they are build in object_recognition_ros
* initial commit
* Contributors: Ha Dang, Vincent Rabaud
