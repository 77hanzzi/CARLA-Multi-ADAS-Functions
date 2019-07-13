import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from matplotlib.font_manager import FontProperties  

path = r'/home/hirain777/hanzzi_learn_carla/2019.05.09_184625_ID_3874.xls'
data_source = pd.read_excel(io = path, sheet_name= 'Raw_data')
label_font = {'family':'serif',
               'style':'italic',
               'weight':'normal',
                'color':'darkred',
                'size':10,
}
text_font = {'family':'serif',
               'style':'italic',
               'weight':'normal',
                'color':'black',
                'size':9,
}
def curise_speed_diagram():
    plt.style.use("seaborn-paper")    
    plt.figure(figsize=(13.5, 4.5))
    l1, = plt.plot(data_source['Ego_Speed'],c='navy',linewidth = 1.0)
    l2, = plt.plot(data_source['Ego_TargetSpeed'],c='green',linestyle = '-.',linewidth = 1.0)
    front_speed = data_source['Front_Speed']
    front_speed_time = [index for index in range(len(front_speed))]
    l3 = plt.scatter(front_speed_time, data_source['Front_Speed'],  c='orangered',marker = 'x',s=10)

    plt.xlabel('Time', fontdict=label_font)
    plt.ylabel('Speed', fontdict=label_font)
    plt.legend(handles = [l1, l2,l3], labels = ['Ego Speed', 'Target Speed', 'Front Car Speed'], loc = 'best')
    plt.savefig("test.png")
    plt.show()

def dynamic_speed_diagram():
    plt.style.use("seaborn-paper")    
    plt.figure(figsize=(13.5, 4.5))
    l1, = plt.plot(data_source['Ego_Speed'],c='navy',linewidth = 1.0)
    l2, = plt.plot(data_source['Ego_TargetSpeed'],c='green',linestyle = '-.',linewidth = 1.0)
    front_speed = data_source['Front_Speed']
    front_speed_time = [index for index in range(len(front_speed))]
    print (front_speed)
    l3 = plt.scatter(front_speed_time, data_source['Front_Speed'],  c='orangered',marker = 'x',s=10)
    
    plt.axvline(x= 18,color = 'black', ls='--',linewidth = 0.8)
    plt.axvline(x= 42,color = 'black', ls='--',linewidth = 0.8)
    plt.xlabel('Time', fontdict=label_font)
    plt.ylabel('Speed', fontdict=label_font)
    plt.legend(handles = [l1, l2,l3], labels = ['Ego Speed', 'Target Speed', 'Front Car Speed'], loc = 'best')
    plt.text (15.2,6.0,'Cut in', fontdict=text_font)
    plt.text (39.2,6.0,'Cut out', fontdict=text_font)
    plt.show()

def distance_diagram():
    plt.style.use("seaborn-paper")    
    relative_distance = data_source['Relative_Distance']
    target_distance = data_source['Target_Relative_Distance']
    l1, = plt.plot(relative_distance,'k')
    l2, = plt.plot( target_distance,'r-.')
    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Distance', fontdict=label_font)
    plt.legend(handles = [l1, l2,], labels = ['Current Distance', 'Target Distance'], loc = 'best')
    plt.show()   

def yaw_diagram():
    fig, ax1 = plt.subplots(figsize=(13.5, 4.5)) 
    plt.style.use("seaborn-paper")    
    ax2 = ax1.twinx()
    yaw = data_source['Ego_Rotation_Yaw']
    position = data_source['Ego_Location_y']
    l1, = ax1.plot(yaw,c='navy',linewidth = 1.0)
    l2, = ax2.plot(position,c='red',linestyle = '-.',linewidth = 1.0)
    ax1.set_xlabel('Time',fontdict=label_font)
    ax1.set_ylabel('Angle',fontdict=label_font)
    ax2.set_ylabel('Position',linestyle = '-',fontdict=label_font)
    plt.legend(handles = [l1, l2,], labels = ['Yaw', 'Position'], loc = 'best')
    plt.show() 

def right_lane_change():
    fig, ax1 = plt.subplots(figsize=(13.5, 4.5)) 
    plt.style.use("seaborn-paper")    
    ax2 = ax1.twinx()
    s_critical = data_source['Right_rear_car_S_critical']
    position = data_source['Ego_Location_y']
    distance = data_source['Right_rear_car_relative_distance']
    l1, = ax1.plot(s_critical,c='navy',linewidth = 1.0)
    l2, = ax1.plot(distance,c='green',linewidth = 1.0)
    l3, = ax2.plot(position,c='red',linestyle = '-.',linewidth = 1.0)
    ax1.set_xlabel('Time',fontdict=label_font)
    ax1.set_ylabel('Distance',fontdict=label_font)
    ax2.set_ylabel('Position',fontdict=label_font)
    plt.legend(handles = [l1, l2,l3], labels = ['Critical Distance','Relative Distance','Position'], loc = 'best')
    plt.show() 
    
def left_lane_change():
    fig, ax1 = plt.subplots(figsize=(13.5, 4.5)) 
    plt.style.use("seaborn-paper")    
    ax2 = ax1.twinx()
    s_critical = data_source['Left_rear_car_S_critical']
    position = data_source['Ego_Location_y']
    distance = data_source['Left_rear_car_relative_distance']
    l1, = ax1.plot(s_critical,c='navy',linewidth = 1.0)
    l2, = ax1.plot(distance,c='green',linewidth = 1.0)
    l3, = ax2.plot(position,c='red',linestyle = '-.',linewidth = 1.0)
    ax1.set_xlabel('Time',fontdict=label_font)
    ax1.set_ylabel('Distance',fontdict=label_font)
    ax2.set_ylabel('Position',fontdict=label_font)
    plt.legend(handles = [l1, l2,l3], labels = ['Critical Distance','Relative Distance','Position'], loc = 'best')
    plt.show() 

def overtake_diagram():
    fig, ax1 = plt.subplots(figsize=(13.5, 4.5)) 
    plt.style.use("seaborn-paper")    
    ax2 = ax1.twinx()
    # ax3 = ax1.twinx()
    s_critical_right = data_source['Right_rear_car_S_critical']
    position = data_source['Ego_Location_y']
    distance_right = data_source['Right_rear_car_relative_distance']
    overtakecar_speed = data_source['overtake_target_speed']
    overtakecar_speed_time = [index for index in range(len(overtakecar_speed))]
    
    l1, = ax1.plot(s_critical_right,c='navy',linewidth = 1.0)
    l2, = ax1.plot(distance_right,c='green',linewidth = 1.0)
    l3, = ax2.plot(position,c='red',linestyle = '-.',linewidth = 1.0)
    # l4 = ax2.scatter(overtakecar_speed_time, data_source['overtake_target_speed'],  c='orange',marker = 'x',s=10)
    
    ax1.set_xlabel('Time',fontdict=label_font)
    ax1.set_ylabel('Distance',fontdict=label_font)
    ax2.set_ylabel('Speed',fontdict=label_font)
    plt.legend(handles = [l1, l2,l3], labels = ['Critical Distance','Relative Distance','Position','Front Car Speed'], loc = 'best')
    plt.show() 

def location_diagram():
    frequency = 20
    raw_ego_x = data_source['Ego_Location_x']
    raw_ego_y = data_source['Ego_Location_y']

    raw_target_x = data_source['Target_location_x']
    raw_target_y = data_source['Target_location_y']

    display_ego_x = [raw_ego_x[x_index] for x_index in range(0,len(raw_ego_x),frequency)]
    display_ego_y = [raw_ego_y[y_index] for y_index in range(0,len(raw_ego_y),frequency)]

    display_target_x = [ raw_target_x [x_index] for x_index in range(0,len( raw_target_x),frequency)]
    display_target_y = [ raw_target_y [y_index] for y_index in range(0,len( raw_target_y),frequency)]
    print (plt.style.available)
    plt.style.use("seaborn-paper")

    l1 = plt.scatter(display_ego_x,display_ego_y,c='navy',marker='x',s=10)
    l2 = plt.scatter(display_target_x,display_target_y,c='green',marker='o',s=10)

    # l2, = plt.plot( target_distance,c='green',linestyle = '-.',linewidth = 1.0)
    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Position',fontdict=label_font)
    plt.legend(handles = [l1, l2 ], labels = ['Ego Location', 'Target Location'], loc = 'best')
    plt.show()

if __name__ == '__main__':
    # speed_diagram()
    # distance_diagram()
    # yaw_diagram()
    # left_lane_change()
    # overtake_diagram()
    location_diagram()