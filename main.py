import csv
from robotControl import robotControler
import sys





#path
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_forward.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_sideways.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/nextState_turn.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/trajectory.csv
# /home/jimas95/workSpace/449_Robotic_Manipulation/Milestone/data/pick_place.csv


if __name__ == "__main__":
    print("Hello World!")
    id = int(sys.argv[1])
    robot = robotControler()
    robot.run(id)
