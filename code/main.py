import csv
from robotControl import robotControler
import sys


if __name__ == "__main__":
    print("Hello World!")
    if len(sys.argv)!=2:
        print("Wrong argument, plz specify which case you want to Execute: 1...5")
    else:
        id = int(sys.argv[1])
        robot = robotControler()
        robot.run(id)
