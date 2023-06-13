#!/usr/bin/env python3

import rospy
from onrobot_mg_control.msg import OnRobotMGInput


class onrobotbaseMG:
    """ onrobotbaseMG sends commands and receives the status of MG gripper.

        Attributes:
            message (list[int]): message including commands to be sent

            verifyCommand:
                Verifies that the value of each variable satisfy its limits.
            refreshCommand:
                Updates the command sent during the next sendCommand() call.
    """

    def __init__(self):
        # Initiating output message as an empty list
        self.message = []

        # Note: after the instantiation,
        # a ".client" member must be added to the object

    def verifyCommand(self, command):
        """ Verifies that the value of each variable satisfy its limits.

            Args:
                command (OnRobotMGOutput): command message to be verified

            Returns:
                command (OnRobotMGOutput): verified command message
        """

        # Verifying that each variable is in its correct range
        command.rMGS = max(0, command.rMGS)
        command.rMGS = min(100, command.rMGS)

        # Verifying that the selected mode number is available
        if command.rCMD not in [0, 1, 2]:
            rospy.signal_shutdown(
                rospy.get_name() +
                ": Select the command number from" +
                "0 (release), 1 (grip), or 2 (smart grip).")

        # Returning the modified command
        return command

    def refreshCommand(self, command):
        """ Updates the command sent during the next sendCommand() call.

            Args:
                command (OnRobotMGOutput): command to be refreshed
        """

        # Limiting the value of each variable
        command = self.verifyCommand(command)

        # Initiating command as an empty list
        self.message = []

        # Building the command with each output variable
        self.message.append(command.rCMD)
        self.message.append(command.rMGS)

    def sendCommand(self):
        """ Sends the command to the Gripper. """

        self.client.sendCommand(self.message)

    def getStatus(self):
        """ Requests the gripper status and return OnRobotMGInput message.

            Returns:
                message (list[int]): message including commands to be sent
        """

        # Acquiring status from the Gripper
        status = self.client.getStatus()

        # Messaging to output
        message = OnRobotMGInput()

        # Assignning the values to their respective variables
        message.gGDT = status[0]
        message.gWDT = status[1]
        message.gWDR = status[6]
        message.gSGF = status[5]
        message.gMGS = status[7]

        return message
