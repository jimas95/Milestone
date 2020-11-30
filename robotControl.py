
import csv
import time
import functions

class robotControler:

    def __init__(self):
        print("init robot Controler")

        path = "input.csv"
        self.init_conditions = []
        self.get_input(path)
        functions.display_init_conditions(self.init_conditions)


    def get_input(self,path):
        """
        read initial conditions for csv file
        """
        print("Reading initial conditions.....")
        with open(path, newline='') as csvfile:
            spamreader = csv.reader(csvfile, delimiter=',', quotechar=',')

            for row in spamreader:
                temp_list = [float(item) for item in row]
                self.init_conditions.append(temp_list)
        time.sleep(1)
        print("Reading initial conditions Done")

