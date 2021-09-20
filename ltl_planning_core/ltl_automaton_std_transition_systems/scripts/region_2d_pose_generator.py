#!/usr/bin/env python
import os
import yaml
import math
import rospkg
from tf.transformations import quaternion_from_euler

def distance_2d(ax, ay, bx, by):
    return math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)

def connect_if_exist(region_dict, region_name, region_to_connect_name):
    if region_to_connect_name in region_dict['state_models']['2d_pose_region']['nodes']:
        region_dict['state_models']['2d_pose_region']['nodes'][region_name]['connected_to'].update({region_to_connect_name: 'goto_'+region_to_connect_name})

def check_if_station_in_cell(x_station, y_station, radius, x_center_cell, y_center_cell, cell_side_length):
    if (((x_center_cell - cell_side_length/2 - radius) < x_station < (x_center_cell + cell_side_length/2 + radius))
    and ((y_center_cell - cell_side_length/2) < y_station < (y_center_cell + cell_side_length/2))):
        return True
    if (((y_center_cell - cell_side_length/2 - radius) < y_station < (y_center_cell + cell_side_length/2 + radius))
    and ((x_center_cell - cell_side_length/2) < x_station < (x_center_cell + cell_side_length/2))):
        return True
    if min(distance_2d(x_station, y_station, x_center_cell + cell_side_length/2, y_center_cell + cell_side_length/2),
           distance_2d(x_station, y_station, x_center_cell - cell_side_length/2, y_center_cell + cell_side_length/2),
           distance_2d(x_station, y_station, x_center_cell - cell_side_length/2, y_center_cell - cell_side_length/2),
           distance_2d(x_station, y_station, x_center_cell + cell_side_length/2, y_center_cell - cell_side_length/2)
           < radius):
        return True
    return False

def generate_regions_and_actions(region_definition_dict):
    region_2d_pose_ts_dict = {'state_dim': ["2d_pose_region"],
                              'state_models': {'2d_pose_region': {'ts_type': "2d_pose_region",
                                                                  'initial': '',
                                                                  'nodes': {}}},
                              'actions': {}}

    # Create stations
    for i in range(len(region_definition_dict['stations'])):
        station_dict = region_definition_dict['stations'][i]
        # Add node
        region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'].update({'s'+str(i):
                                                                                    {'attr': {'type': 'station',
                                                                                              'pose': [[station_dict['origin']['x'], station_dict['origin']['y']], [station_dict['origin']['yaw']]],
                                                                                              'radius': station_dict['radius'],
                                                                                              'angle_threshold': station_dict['angle_threshold'],
                                                                                              'dist_hysteresis': station_dict['dist_hysteresis'],
                                                                                              'angle_hysteresis': station_dict['angle_hysteresis']},
                                                                                     'connected_to': {'s'+str(i): 'goto_s'+str(i)}}}) # Initialize with self-loop
        # Add action
        station_quaternion = quaternion_from_euler(0, 0, station_dict['origin']['yaw']) # Get quaternion from yaw angle
        region_2d_pose_ts_dict['actions'].update({'goto_s'+str(i): {'type': 'move',
                                                                    'weight': 10,
                                                                    'guard': "1",
                                                                    'attr': {'region': 's'+str(i), 'pose': [[station_dict['origin']['x'], station_dict['origin']['y'], 0], station_quaternion.tolist()]}}})


    # Create cell regions
    cell_iter = 1
    # Iterate over lines on y-axis
    for cell_num_on_y in range(region_definition_dict['grid']['number_of_cells_y']):
        # y-coordinate is origin on y-axis plus square side length time number of squares plus side half-length (for square-center)
        y_coord = (region_definition_dict['grid']['origin']['y']
                + (cell_num_on_y * region_definition_dict['grid']['cell_side_length'])
                + (region_definition_dict['grid']['cell_side_length']/2))
        # Iterate over columns on y-axis
        # y-coordinate is origin on y-axis plus square side length time number of squares plus side half-length (for square-center)
        for cell_num_on_x in range(region_definition_dict['grid']['number_of_cells_x']):
            x_coord = (region_definition_dict['grid']['origin']['x']
                    + (cell_num_on_x * region_definition_dict['grid']['cell_side_length'])
                    + (region_definition_dict['grid']['cell_side_length']/2))

            # Add node
            region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'].update({'r'+str(cell_iter): {'attr': {'type': 'square',
                                                                                                                    'pose': [[x_coord, y_coord], [0]],
                                                                                                                    'length': region_definition_dict['grid']['cell_side_length'],
                                                                                                                    'hysteresis': region_definition_dict['grid']['cell_hysteresis']},
                                                                                                           'connected_to': {'r'+str(cell_iter): 'goto_r'+str(cell_iter)}}}) # Initialize with self-loop}})
            # Add action
            region_2d_pose_ts_dict['actions'].update({'goto_r'+str(cell_iter): {'type': 'move',
                                                                                'weight': 10,
                                                                                'attr': {'region': 'r'+str(cell_iter), 'pose': [[ x_coord, y_coord, 0], [0, 0, 0, 1]]}}})

            # Check if station is connected
            # Go through all regions
            for reg_key in region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'].keys():
                # if region is station, check if in cell
                if region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['attr']['type'] == 'station':
                    if check_if_station_in_cell(region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['attr']['pose'][0][0],
                                                region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['attr']['pose'][0][1],
                                                region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['attr']['radius']
                                                 + region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['attr']['dist_hysteresis'],
                                                x_coord, y_coord, region_definition_dict['grid']['cell_side_length']):
                        
                        # Connect station to cell
                        region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes'][reg_key]['connected_to'].update({'r'+str(cell_iter): 'goto_'+'r'+str(cell_iter)})
                        # Connect cell to region
                        region_2d_pose_ts_dict['state_models']['2d_pose_region']['nodes']['r'+str(cell_iter)]['connected_to'].update({str(reg_key): 'goto_'+str(reg_key)})

            # Increment
            cell_iter += 1


    # Add connection between cells
    cell_iter = 1
    # Iterate over column on y-axis
    for cell_num_on_y in range(region_definition_dict['grid']['number_of_cells_y']):
        # Iterate over rows on x-axis
        for cell_num_on_x in range(region_definition_dict['grid']['number_of_cells_x']):
            # If not first cell of the column
            if not cell_num_on_x == 0:
                # If previous region exist, add to connected to list
                connect_if_exist(region_2d_pose_ts_dict, 'r'+str(cell_iter), 'r'+str(cell_iter-1))
            # If not last cell of the column
            if not cell_num_on_x == (region_definition_dict['grid']['number_of_cells_x']-1):
                # If next region exist, add to connected to list
                connect_if_exist(region_2d_pose_ts_dict, 'r'+str(cell_iter), 'r'+str(cell_iter+1))
            # If region in previous line exist, add to connected to list
            connect_if_exist(region_2d_pose_ts_dict, 'r'+str(cell_iter), 'r'+str(cell_iter-region_definition_dict['grid']['number_of_cells_x']))
            # If region in next line exist, add to connected to list
            connect_if_exist(region_2d_pose_ts_dict, 'r'+str(cell_iter), 'r'+str(cell_iter+region_definition_dict['grid']['number_of_cells_x']))
            # Increment region number
            cell_iter += 1

    return region_2d_pose_ts_dict

def write_to_file(file_name="generated_2d_pose_region_ts", region_2d_pose_ts_dict={}):
    with open(os.path.join(rospkg.RosPack().get_path('ltl_automaton_std_transition_systems'),
                                                     'config',
                                                     'generated_ts',
                                                     file_name+'.yaml'),'w') as file:
        yaml.dump(region_2d_pose_ts_dict, file)

    print("-----------------------------------------------------------")
    print(" 2D pose region transition systems successfully generated!")
    print("-----------------------------------------------------------")
    print(" File can be find at %s" % os.path.join(rospkg.RosPack().get_path('ltl_automaton_std_transition_systems'),
                                                                             'config',
                                                                             'generated_ts',
                                                                             file_name+'.yaml'))

