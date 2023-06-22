#!/usr/bin/env python
# Jinyi - June 2023

import os
import pandas as pd

import rospy
from std_msgs.msg import String

# CSV file locations
baseDir = '../ECG_Dataset/ecg-bp-013/'
ecgDir = baseDir + 'vivalnk_vv330_ecg/'
hrDir = baseDir + 'vivalnk_vv330_heart_rate/'
rrDir = baseDir + 'vivalnk_vv330_resp_rate/'

def loadData():
    # Create list of CSV files for each data type
    csvFilesECG = []
    for root, dirs, files in os.walk(ecgDir):
        for file in files:
         if '.csv' in file:
             csvFilesECG.append(os.path.join(root,file))

    csvFilesHR = []
    for root, dirs, files in os.walk(hrDir):
        for file in files:
            if '.csv' in file:
                csvFilesHR.append(os.path.join(root,file))

    csvFilesRR = []
    for root, dirs, files in os.walk(rrDir):
        for file in files:
            if '.csv' in file:
                csvFilesRR.append(os.path.join(root,file))

    # Create dataframe for each data type from all CSV files
    bioDataECG = pd.concat([pd.read_csv(file) for file in csvFilesECG], ignore_index=True)
    bioDataHR = pd.concat([pd.read_csv(file) for file in csvFilesHR], ignore_index=True)
    bioDataRR = pd.concat([pd.read_csv(file) for file in csvFilesRR], ignore_index=True)

    # Combine all readings for each time stamp
    bioDataMerged = bioDataECG.merge(bioDataHR.merge(bioDataRR, on=['time','userId','projectId']), on=['time','userId','projectId'])

    print(bioDataMerged)
    return bioDataMerged

def publish(bioData):
    # Create new publisher
    pub = rospy.Publisher('/biodata', String, queue_size=10)

    # Initialize publishing node
    rospy.init_node('bio_node', anonymous=True)

    # Publish rate = 10 Hz
    rate = rospy.Rate(10)

    # Track number of rows of data published
    row = 0
    while not rospy.is_shutdown():
        # Stop when all data published
        if row > len(bioData.index):
            print('All data published, exiting...')
            break

        # Publish only if there is a subscriber
        if pub.get_num_connections() > 0:
            # Send given row in json format
            data =  bioData.loc[row].to_json()
            row += 1

            # Print data to cli
            rospy.loginfo(data)

            # Publish data to topic
            pub.publish(data)

        # Sleep according to publish rate
        rate.sleep()

if __name__ == '__main__':
    # Load data in memory to be ready for a subscriber without delay
    bioData = loadData()

    try:
        # Begin publishing logic
        publish(bioData)
    except rospy.ROSInterruptException as e:
        raise e
