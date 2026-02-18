echo $ROS_DISTRO
ls
cd src
ls
ros2 pkg create --build-type ament_cmake --license Apache-2.0 motorctl_cpp
ls
chmod -R 777 motorctl_cpp/
colcon build --packages-select motorctl_cpp
colcon clean
rosclean
cd ..
ls
colcon build --packages-select motorctl_cpp
cd src/
ls
chmod -R 777 log/
chmod -R 777 install/
chmod -R 777 build/
cd ..
colcon build
rossetup
ros2 run motorctl_cpp slave
chmod -R 777 .
pip3 install roboticstoolbox-python
pip3 install roboticstoolbox-python --break-system-packages
python3 - << 'EOF'
import roboticstoolbox as rtb
print(rtb.version)
robot = rtb.models.Panda()
print(robot.n)
EOF

python3 - << 'EOF'
import roboticstoolbox as rtb
print(rtb.version)
EOF

python3 - << 'EOF'
import roboticstoolbox as rtb
EOF

which python3
which pip3
pip3 -v
pip3 -V
python3 -V
which pip3
which python3
pip3 show roboticstoolbox-python
python3 -m pip show roboticstoolbox-python
pip3 show roboticstoolbox-python
python3 -m pip install -U pip
python3 -m pip install -U pip --break-system packages
python3 -m pip install --no-cache-dir roboticstoolbox-python spatialmath-python
python3 -m pip install --no-cache-dir --break-system-packages     roboticstoolbox-python spatialmath-python
python3 - << 'EOF'
import roboticstoolbox as rtb
EOF

python3 -m pip install --no-cache-dir --break-system-packages   --ignore-installed filelock   roboticstoolbox-python spatialmath-python
python3 - << 'EOF'
import roboticstoolbox as rtb
print(rtb.__version__)
robot = rtb.models.Panda()
print(robot.n)
EOF

python3 - << 'EOF'
import roboticstoolbox as rtb
robot = rtb.models.Panda()
print(robot.n)
EOF

python3 -m pip uninstall -y   roboticstoolbox-python spatialmath-python spatialgeometry swift-sim rtb-data pgraph-python
python3 -m pip uninstall -y   roboticstoolbox-python spatialmath-python spatialgeometry swift-sim rtb-data pgraph-python --break-system-packages
python3 -m pip install --no-cache-dir --break-system-packages   --ignore-installed filelock   roboticstoolbox-python spatialmath-python
python3 - << 'EOF'
import roboticstoolbox as rtb
robot = rtb.models.Panda()
print(robot.n)
EOF

[200~python3 - << 'EOF'
from mpl_toolkits.mplot3d import Axes3D
print("Axes3D OK")

python3 - << 'EOF'
from mpl_toolkits.mplot3d import Axes3D
print("Axes3D OK")
EOF

sudo apt update
python3 -m pip uninstall -y matplotlib
python3 -m pip uninstall -y matplotlib --break-system-packages
sudo apt install -y python3-matplotlib
python3 - << 'EOF'
from mpl_toolkits.mplot3d import Axes3D
print("Axes3D OK")
EOF

python3 - << 'EOF'
import roboticstoolbox as rtb
robot = rtb.models.Panda()
print(robot.n)
EOF

python3 -m pip install --no-cache-dir --break-system-packages   --force-reinstall numpy==1.26.4
python3 - << 'EOF'
import numpy as np
print("numpy version:", np.version)
EOF

python3 - << 'EOF'
import numpy as np
print("numpy version:", np.__version__)
EOF

python3 - << 'EOF'
import roboticstoolbox as rtb
robot = rtb.models.Panda()
print(robot.n)
EOF

python3 - << 'EOF'
from mpl_toolkits.mplot3d import Axes3D
print("Axes3D OK")
EOF

python3 - << 'EOF' import roboticstoolbox as rtb robot = rtb.models.Panda() robot.plot(robot.qz, backend="swift") EOF
python3 -m pip show swift-sim | sed -n '1,5p'
test.py
python3 test.py 
python3 -m pip show websocket
python3 -m pip show websockets
python3 -m pip install --break-system-packages --no-cache-dir   --force-reinstall "websockets==11.0.3"
python3 test.py 
ss -ltnp | grep python || true
apt install -y iproute2
ss -ltnp | grep python || true
python3 test.py 
python3 test.py
exit
ls
python3 test.py
exit
python3 test.py
exit
ls
exit
ss -ltnp | grep python || true
apt update
apt install -y iproute2
ss -ltnp | grep python || true
exit
ss -ltnp | grep python || true
apt install -y iproute2
apt update
apt install -y iproute2
ss -ltnp | grep python || true
ls
exit
chmod -R 777 .
exit
rossetup
ros2 run motorctl_cpp slave
rossetup
ros2 run motorctl_cpp slave
colcon build
rossetup
ros2 run motorctl_cpp slave
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ros2 run motorctl_cpp slave
exit
rossetup
ros2 run motorctl_cpp master 
colcon build
rosseup
rossetup
ros2 run motorctl_cpp master 
ldconfig -p | grep soem
ldconfig -p
colcon build
rossetup
ros2 run motorctl_cpp master 
exit
colcon build
rossetup
ros2 run motorctl_cpp slave
colcon build
ros2 run motorctl_cpp slave
exit
chmod -R 777 .
exit
ls
rosclean
colcon build
exit
rosclean
colcon build
exit
