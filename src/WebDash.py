#!/usr/bin/env python
# Jinyi - June 2023

from dash import Dash, html, dcc, Input, Output, dash_table

import pandas as pd
from std_msgs.msg import String

import json
import threading
import subprocess
import time
import os

import base64
import matplotlib
import matplotlib.pyplot as plt

import rospy
import tf

# Set asset path outside of src
assetPath = os.getcwd() + '../assets/'

# Define Dash server object
web_dash = Dash(__name__,assets_folder=assetPath)

# Flag to track current image file name
imgFileFlag = 0

# Track X,Y coordinates of turtlebot
XPOS = 0
YPOS = 0

# Initialize empty dataframe to hold biodata and define data to be read
bioData = pd.DataFrame()
columnList = [{'name': 'projectId', 'id': 'projectId'},
             {'name': 'userId', 'id': 'userId'},
             {'name': 'time', 'id': 'time'},
             {'name': 'ecg', 'id': 'ecg'},
             {'name': 'hr', 'id': 'hr'},
             {'name': 'rr', 'id': 'rr'}]

# Dashboard layout
web_dash.layout = html.Div(children=[
    # Title and line breaks
    html.H1(children='ROS-Dash'),

    html.Div(children='Jinyi - June 2023'),
    html.Br(),

    # Map image
    html.Img(id='map-img'),
    # Timer for periodic updating of map image
    dcc.Interval(
        id='interval-map',
        interval=10000, # 10 seconds
        n_intervals=0,
        max_intervals=7000),

    html.Br(),
    html.Button('Load Bio Data', id='load-btn', style=dict()),

    # Timer for periodic updating of bio data dataframe
    dcc.Interval(
        id='interval',
        interval=100,
        n_intervals=0,
        max_intervals=70000),

    # Table for bio data dataframe
    html.Br(),
        dash_table.DataTable(
                    id = 'bio-table',
                    data = bioData.to_dict('records'),
                    columns=columnList)
])

# Callback function after Load Bio Data buttonpress
@web_dash.callback(
        Output(component_id='load-btn', component_property='style'),
        Input('load-btn','n_clicks')
)
def subscribe_bio_data(n_clicks):
    if n_clicks is not None:
        # Spawn new thread to subscribe to biodata publisher
        bioSubscriberThread = threading.Thread(target=bio_thread_entrance, args=())
        bioSubscriberThread.daemon = True
        bioSubscriberThread.start()

        # Hide button
        return dict(display='none')
    else:
        return dict()
# Thread to subscribe to biodata publisher
def bio_thread_entrance():
    print('Entering bio subscribe thread')
    sub = rospy.Subscriber('/biodata', String, bio_callback)

# Subscriber callback to store new bio data
def bio_callback(data):
    global bioData

    if bioData.empty:
        # Convert new biodata into dictionary from JSON string that is published
        bioData = pd.DataFrame(json.loads(data.data),index=[0])
    else:
        # Convert new biodata into dictionary from JSON string that is published and concatenate to existing dataframe
        bioData = pd.concat([bioData, pd.DataFrame(json.loads(data.data),index=[0])], ignore_index=True)
    rospy.loginfo(data.data)

# Callback to periodically update biodata display table
@web_dash.callback(
        Output('bio-table','data'),
        [Input('interval', 'n_intervals')])
def updateTable(n):
    global bioData, columnList
    return bioData.to_dict('records')

# Callback to periodically update map image
@web_dash.callback(
        Output('map-img','src'),
        [Input('interval-map', 'n_intervals')])
def updateMap(n):
    global imgFileFlag

    # Spanw new thread to get most recent map from /map topic
    mapThread = threading.Thread(target=map_update_thread, args=(imgFileFlag,))
    mapThread.daemon = True
    mapThread.start()

    # Give it time to write to file
    time.sleep(1)

    mapFilename = '../assets/map_img.pgm'

    # Alternate the name of the image file to load to force Dash to reload
    if imgFileFlag == 1:
        figFilename = '../assets/fig.png'
        imgFileFlag = 0
    else:
        figFilename = '../assets/fig1.png'
        imgFileFlag = 1

    # Read map image
    inData = matplotlib.image.imread(mapFilename)

    # Translation of Odometry data to pixel
    # Range of Odometry data is -2.3 to 2.3
    # Origin of map in image file is (200,180)
    xCoord = int(XPOS/4.6*110 + 200)
    yCoord = int(-1 * YPOS/4.6*100 + 180)

    # Clear old robot position
    plt.clf()
    # Draw robot position using a point
    plt.plot(xCoord,yCoord, marker='o', color='red')
    # Limit axis for clearer map image
    plt.axis([100, 275, 250, 100])
    # Draw underlying map
    plt.imshow(inData)
    # Save figure to file for Dash to load
    plt.savefig(figFilename)

    # Load file and convert to HTML compatible base 64
    encoded_image = base64.b64encode(open(figFilename, 'rb').read())
    # Send image file location to HTML image object
    return 'data:image/png;base64,{}'.format(encoded_image.decode())

# Separate thread to save the latest /map to file
def map_update_thread(imgFileFlag):
    subprocess.run(['rosrun', 'map_server', 'map_saver', '-f', '../assets/map_img'])
    return

# Thread to listen for transformation of coordinates between robot odometry and map
def odom_thread_entrance():
    global XPOS, YPOS

    listener = tf.TransformListener()
    rate = rospy.Rate(1) # 1 Hz
    while not rospy.is_shutdown():
        try:
            # Get latest transform values and store into X and Y positions
            (transform,rotation) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            XPOS = transform[0]
            YPOS = transform[1]

            # Log for debugging
            rospy.loginfo('XPOS = ' + str(XPOS))
            rospy.loginfo('YPOS = ' + str(YPOS))
        except:
            continue
        rate.sleep()
    return

if __name__ == '__main__':
    # Create rospy node
    rospy.init_node('bio_map_subscriber', anonymous=True)

    # Spawn thread to get periodic odometry data
    odomThread = threading.Thread(target=odom_thread_entrance, args=())
    odomThread.daemon = True
    odomThread.start()

    # Start web server
    # Use dev_tools_hot_reload as False to prevent the page from reloading after image is updated
    web_dash.run_server(debug=False, dev_tools_hot_reload=False)
