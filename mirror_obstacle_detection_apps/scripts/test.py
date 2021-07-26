#!/usr/bin/env python
# license removed for brevity

import rospy

rospy.init_node('test')
def publisher():

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException: pass