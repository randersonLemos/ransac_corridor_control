import os
from rospkg import RosPack

print os.path.expandvars('$ROS_WORKSPACE') + '/devel/include'


rospack = RosPack()

for p in rospack.list():
    if os.path.exists(rospack.get_path(p) + '/include'):
        print p
        print rospack.get_path(p) + '/include'
