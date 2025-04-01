#!/usr/bin/env python
import csv
import rospy
import pandas as pd
import tf
import keyboard
from common import TFcfg

tf_config = TFcfg()
def tf_csv_write(writer, file_length):
    #tf write
    try:
        (trans, rot) = tf_config.listener.lookupTransform( '/base_link','/link_6_t', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return file_length
    #if error apear, need exception code 
    writer.writerow([file_length,str(trans[0]), str(trans[1]), str(trans[2])])
    print('%d ground_truth add: (%f, %f, %f)' % (file_length+1, trans[0], trans[1], trans[2]))
    file_length += 1
    return file_length

def main():
    rospy.init_node("CSV_Ground")
    rate = rospy.Rate(30)
    tf_config.set_tf_listener()
    print('print select mode(0: overwrite 1:Followwrite)')

    mode = input()
    file_length = None
    if(int(mode) == 0):
        with open('ground_truth.csv','w') as csvfile:
            file_length = 0
            wr = csv.writer(csvfile)
            flag = True
            while not rospy.is_shutdown():
                if keyboard.is_pressed('w') and flag:
                    file_length = tf_csv_write(wr, file_length)
                    flag = False
                elif keyboard.is_pressed('c'):
                    print('input mode')
                    flag = True
                elif keyboard.is_pressed('q'):
                    print('\nwriting Stop')
                    break
                rate.sleep()
    elif(int(mode) == 1):
        df = pd.read_csv('ground_truth.csv')
        file_length = len(df)
        with open('ground_truth.csv','a') as csvfile:
            wr = csv.writer(csvfile)
            flag = True
            while not rospy.is_shutdown():
                if keyboard.is_pressed('w') and flag:
                    file_length = tf_csv_write(wr, file_length)
                    flag = False
                elif keyboard.is_pressed('c'):
                    print('input mode')
                    flag = True
                elif keyboard.is_pressed('q'):
                    print('\nwriting Stop')
                    break
                rate.sleep()
    return

if __name__ == '__main__':
    main()