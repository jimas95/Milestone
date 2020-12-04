import csv
from robotControl import robotControler




#path
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_forward.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_sideways.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_turn.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/trajectory.csv


if __name__ == "__main__":
    print("Hello World!")
    robot = robotControler()
    robot.run(3)
