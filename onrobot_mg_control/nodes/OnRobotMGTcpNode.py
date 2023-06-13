#!/usr/bin/env python3

import rospy
import onrobot_mg_control.baseOnRobotMG
import onrobot_mg_modbus_tcp.comModbusTcp
from onrobot_mg_control.msg import OnRobotMGInput
from onrobot_mg_control.msg import OnRobotMGOutput


class OnRobotMGTcp:
    """ OnRobotMGTcp connects to the gripper with Modbus/TCP.

        Attributes:
            gripper (onrobot_mg_control.baseOnRobotMG.onrobotbaseMG):
                instance of onrobotbaseMG used for the connection establishment
            pub (rospy.Publisher): the publisher for OnRobotMGInput

            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self):
        # Gripper is a MG gripper with a Modbus/TCP connection
        self.gripper = \
            onrobot_mg_control.baseOnRobotMG.onrobotbaseMG()
        self.gripper.client = \
            onrobot_mg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address received as an argument
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotMGInput'
        self.pub = rospy.Publisher(
            'OnRobotMGInput', OnRobotMGInput, queue_size=1)

        # The Gripper command is received from the topic 'OnRobotMGOutput'
        rospy.Subscriber('OnRobotMGOutput',
                         OnRobotMGOutput,
                         self.gripper.refreshCommand)

        self.mainLoop()

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """

        prev_msg = []
        while not rospy.is_shutdown():
            # Get and publish the Gripper status
            status = self.gripper.getStatus()
            self.pub.publish(status)

            rospy.sleep(0.05)
            # Send the most recent command
            if not prev_msg == self.gripper.message:  # find new message
                rospy.loginfo(rospy.get_name()+f": Sending message. {self.gripper.message}")
                self.gripper.sendCommand()
                prev_msg = self.gripper.message
            rospy.sleep(0.05)


if __name__ == '__main__':
    try:
        ip = rospy.get_param('/onrobot/ip', '192.168.1.1')
        port = rospy.get_param('/onrobot/port', '502')
        changer_addr = rospy.get_param('/onrobot/changer_addr', '65')
        dummy = rospy.get_param('/onrobot/dummy', False)
        rospy.init_node(
            'OnRobotMGTcpNode', anonymous=True, log_level=rospy.DEBUG)
        OnRobotMGTcp()
    except rospy.ROSInterruptException:
        pass
