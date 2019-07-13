import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from matplotlib.font_manager import FontProperties  

path = r'/home/hirain777/hanzzi_learn_carla/2019.05.07_202316_ID_520.xls'
data_source = pd.read_excel(io = path, sheet_name= 'Raw_data')
label_font = {'family':'serif',
               'style':'italic',
               'weight':'normal',
                'color':'darkred',
                'size':10,
}

def light_detection():

    fig, ax1 = plt.subplots(figsize=(13.5, 4.5)) 
    plt.style.use("seaborn-paper")    
    ax2 = ax1.twinx()
    ax2.set_yticks([-1,0,1,2])
    l1, = ax1.plot(data_source['Ego_Speed'],c='navy' , linewidth = 1.0) 
    l2, = ax2.plot(data_source['Traffic_Light_State'], c='red',linestyle = '-.',linewidth = 1.0) 
    ax1.set_xlabel('Time',fontdict=label_font)
    ax1.set_ylabel('Speed',fontdict=label_font)
    ax2.set_ylabel('Traffic Light State',fontdict=label_font)
    plt.legend(handles = [l1, l2], labels = ['Ego Speed', 'Traffic Light State'], loc = 'upper left')

    plt.show()

def overall_speed_diagram():
    frequency = 1
    # indicator = (frequency - 1) / 2 if frequency % 2 != 0 else frequency/2
    plt.style.use("seaborn-paper")    
    plt.figure(figsize=(13.5, 4.5))
    
    ego_speed = [data_source['Ego_Speed'][index] for index in range(0,len(data_source['Ego_Speed']),frequency)]
    ego_target_speed = [data_source['Ego_TargetSpeed'][index] for index in range(0,len(data_source['Ego_TargetSpeed']),frequency)]
    front_speed = [data_source['Front_Speed'][index] for index in range(0,len(data_source['Front_Speed']),frequency)]
    speed_limit = [data_source['Speed_Limit_State'][index] for index in range(0,len(data_source['Speed_Limit_State']),frequency )]

    front_speed_time = [front_speed.index(speed) for speed in front_speed]

    l1, = plt.plot(ego_speed,c='navy',linewidth = 1.0)
    l2, = plt.plot(ego_target_speed,c='green',linestyle = '-.',linewidth = 1.0)
    l3 = plt.scatter(front_speed_time,front_speed, c='orangered',marker = 'x',s=10)
    l4, = plt.plot(speed_limit,c='goldenrod', linewidth = 1.0)

    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Speed',fontdict=label_font)
    plt.legend(handles = [l1, l2, l3,l4,], labels = ['Ego Speed', 'Target Speed', 'Front Car Speed','Speed Limit'], loc = 'upper left')

    plt.show()

def target_speed_selection():
    plt.figure(figsize=(13.5, 4.5))
    plt.style.use("seaborn-paper")    

    # l1, = plt.plot(data_source['Ego_Speed'],c='navy',linewidth = 1.0)
    l2, = plt.plot(data_source['Ego_TargetSpeed'],c='green',linestyle = '-.',linewidth = 1.0)
    l3, = plt.plot(data_source['Front_Speed'], c='magenta',linewidth = 1.2)
    l4, = plt.plot(data_source['Speed_Limit_State'],c='goldenrod', linewidth = 1.0)

    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Speed',fontdict=label_font)
    plt.legend(handles = [ l2, l3,l4, ], labels = [ 'Target 车速', '前车车速','道路限速'], loc = 'upper left')

    plt.show()

def speed_diagram():
    plt.figure(figsize=(13.5, 4.5))
    plt.style.use("seaborn-paper")    

    l1, = plt.plot(data_source['Ego_Speed'],c='navy',linewidth = 1.0)
    l2, = plt.plot(data_source['Ego_TargetSpeed'],c='green',linestyle = '-.',linewidth = 1.0)
    # l3, = plt.plot(data_source['Front_Speed'], c='magenta',linewidth = 1.2)
    # l4, = plt.plot(data_source['Speed_Limit_State'],c='goldenrod', linewidth = 1.0)

    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Speed',fontdict=label_font)
    plt.legend(handles = [l1, l2,], labels = ['Ego Speed', 'Target Speed'], loc = 'upper left')

    plt.show()

def distance_diagram():
    plt.style.use("seaborn-paper")    
    relative_distance = data_source['Relative_Distance']
    target_distance = data_source['Target_Relative_Distance']
    l1, = plt.plot(relative_distance,c='navy',linewidth = 1.0)
    l2, = plt.plot( target_distance,c='green',linestyle = '-.',linewidth = 1.0)
    plt.xlabel('Time',fontdict=label_font)
    plt.ylabel('Distance',fontdict=label_font)
    plt.legend(handles = [l1, l2,], labels = ['Current Distance', 'Target Distance'], loc = 'upper left')
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
    light_detection()
    # overall_speed_diagram()
    # target_speed_selection()
    # speed_diagram()
    # distance_diagram()
    # location_diagram()