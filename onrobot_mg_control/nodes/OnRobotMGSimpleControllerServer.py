#!/usr/bin/env python3

import rospy
from onrobot_mg_control.msg import OnRobotMGOutput
from onrobot_mg_control.srv import SetCommand, SetCommandResponse


class OnRobotMGNode:
    """ OnRobotMGNode handles setting commands.

        Attributes:
            pub (rospy.Publisher): the publisher for OnRobotMGOutput
            command (OnRobotMGOutput): command to be sent
            set_command_srv (rospy.Service): set_command service instance

            handleSettingCommand:
                Handles sending commands via socket connection.
            genCommand:
                Updates the command according to the input character.
    """

    def __init__(self):
        self.pub = rospy.Publisher(
            'OnRobotMGOutput', OnRobotMGOutput, queue_size=1)
        self.command = OnRobotMGOutput()
        self.set_command_srv = rospy.Service(
            "/onrobot_mg/set_command",
            SetCommand,
            self.handleSettingCommand)

    def handleSettingCommand(self, req):
        """ Handles sending commands via socket connection. """

        rospy.loginfo(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        rospy.sleep(1)
        return SetCommandResponse(
            success=None,  # TODO: implement
            message=None)  # TODO: implement

    def genCommand(self, char, command):
        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotMGOutput): command to be sent

            Returns:
                command: command message with parameters set
        """

        if char == 'g':
            command.rCMD = 1
            command.rMGS = 75
        if char == 'gw':
            command.rCMD = 1
            command.rMGS = 25
        if char == 'gm':
            command.rCMD = 1
            command.rMGS = 50
        if char == 'gs':
            command.rCMD = 1
            command.rMGS = 100
        if char == 'r':
            command.rCMD = 0
            command.rMGS = 0
        
        # If the command entered is a int, assign this value to r
        try:
            if int(char) == 0:
                command.rCMD = 0
                command.rMGS = 0
            else:
                command.rCMD = 1
                command.rMGS = min(100, int(char))
        except ValueError:
            pass
        
        return command


if __name__ == '__main__':
    rospy.init_node(
        'OnRobotMGSimpleControllerServer',
        anonymous=True,
        log_level=rospy.DEBUG)
    node = OnRobotMGNode()
    rospy.spin()
