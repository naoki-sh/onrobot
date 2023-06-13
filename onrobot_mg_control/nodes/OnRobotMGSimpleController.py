#!/usr/bin/env python3

import rospy
from onrobot_mg_control.msg import OnRobotMGOutput


def genCommand(char, command):
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


def askForCommand(command):
    """ Asks the user for a command to send to the gripper.

        Args:
            command (OnRobotMGOutput): command to be sent

        Returns:
            input(strAskForCommand) (str): input command strings
    """

    currentCommand = 'Simple OnRobot MG Controller\n-----\nCurrent command:'
    currentCommand += ' rCMD = ' + str(command.rCMD)
    currentCommand += ', rMGS = ' + str(command.rMGS)
    
    rospy.loginfo(currentCommand)

    strAskForCommand = '-----\nAvailable commands\n\n'
    strAskForCommand += 'g: Grip (Magnet strength: 75%)\n'
    strAskForCommand += 'r: Release (Magnet strength: 0%)\n'
    strAskForCommand += 'gw: Grip with weak force (Magnet strength: 25%)\n'
    strAskForCommand += 'gm: Grip with mediam force (Magnet strength: 50%)\n'
    strAskForCommand += 'gs: Grip with strong force (Magnet strength: 100%)\n'
    strAskForCommand += '(0 - 100): Set magnet strength\n'

    strAskForCommand += '-->'

    return input(strAskForCommand)


def publisher():
    """ Main loop which requests new commands and
        publish them on the OnRobotMGOutput topic.
    """

    rospy.init_node(
        'OnRobotMGSimpleController',
        anonymous=True,
        log_level=rospy.DEBUG)
    pub = rospy.Publisher(
        'OnRobotMGOutput', OnRobotMGOutput, queue_size=1)
    command = OnRobotMGOutput()

    while not rospy.is_shutdown():
        command = genCommand(askForCommand(command), command)
        pub.publish(command)
        rospy.sleep(0.1)


if __name__ == '__main__':
    publisher()
