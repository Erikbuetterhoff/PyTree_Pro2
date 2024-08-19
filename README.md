# Erstellen eines Behavior Trees für die Drohne im Projekt "5URVIVE"

Dieses Repo enthält einen Behavior Tree erstellt mit pytrees und pytree_ros, welcher autonome Verhalten einer Drohne ermöglichen soll.

### Haupt launch-file 
```
ros2 launch pytree_full full_tree.launch.py 
```
Um das Verhalten des Trees überprüfen zu können, können die im "5URVIV" Projekt aufgezeichneten Rosbags genutzt werden.
# Install py-trees-tree-viewer on humble
```
sudo apt-get install libqt5webengine5 libqt5webenginewidgets5 python3-pyqt5.qtwebengine
```
```
pip install pyqtwebengine
```
```
git clone https://github.com/splintered-reality/py_trees_js.git
git clone https://github.com/splintered-reality/py_trees_ros_viewer.git
```
```
colcon build --symlink-install
```
Now it should be working
```
py-trees-tree-viewer
```
# Installing psdk wrapper
```
sudo apt install ros-humble-psdk-wrapper ros-humble-psdk-interfaces
```
