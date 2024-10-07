#!/usr/bin/env python
from roboticstoolbox.robot.ERobot import ERobot

class Kuka_iiwa(ERobot):
  def __init__(self):
        """ Kuka Robot Class """
        path = "kuka_iiwa.urdf"
        links, name, urdf_string, urdf_filepath = self.URDF_read(
            path, tld="../urdf"
        )

        super().__init__(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
        )

        self.q0 = [7.14937493770593e-09,-0.211797495415964,-2.17385137682851e-10,2.07588108651985,1.53275612769668e-09,-0.853117755863475,6.51826359458392e-09]


if __name__ == "__main__":  # pragma nocover
    robot = Kuka_iiwa()
    print(robot)