import numpy as np
import csv
import rospy
import keyboard
from common import TFcfg


tf_config = TFcfg()


def main():
    with open('ground_truth.csv', 'r') as csvfile:
        rdr = csv.reader(csvfile)
        for line in rdr:
            print(line)
    return

if __name__ == '__main__':
    main()