import bagpy
from bagpy import bagreader
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
import numpy as np

# -- Syntakas Akis - Example usage of bagpy package

#Read your bagfile - Put it in same folder as the bagpy_example.py or use full path
b = bagreader('2016-06-01-14-57-13.bag')


print(b.topic_table)

#CONVERT BAG file TO DATAFRAME
# Read odometry data from bagfile and convert to Pandas Dataframe
odom = b.odometry_data()
odomdata = pd.read_csv(odom[0])


# GET COLUMN NAMES
print(odomdata.columns)

#PLOT
plt.plot(odomdata["Time"], odomdata["linear.x"])
plt.show()


#READ ONE TOPIC FROM BAGFILE AND CONVERT TO PANDAS DATAFRAME
LASER_MSG = b.message_by_topic('/catvehicle/front_laser_points')
df_laser = pd.read_csv(LASER_MSG)

print(df_laser)
#for more examples see github repo of bagpy. Some notebooks with examples will make everything clear



