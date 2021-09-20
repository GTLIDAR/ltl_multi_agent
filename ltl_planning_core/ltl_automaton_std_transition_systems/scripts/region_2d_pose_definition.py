#!/usr/bin/env python
import yaml
import math
import region_2d_pose_generator

def extract_point_coord_from_input(start_point_str):
    point_coord_txt = start_point_str.split(',')
    if len(point_coord_txt) == 2:
        point_coord = []
        # x coordinate
        point_coord.append(float(point_coord_txt[0]))
        # y coordinate
        point_coord.append(float(point_coord_txt[1]))
        return point_coord
    else:
        return None

def extract_station_from_input(station_str):
    station_txt = station_str.split(',')
    if len(station_txt) == 7:
        station_data = []
        # station x coordinate
        station_data.append(float(station_txt[0]))
        # station y coordinate
        station_data.append(float(station_txt[1]))
        # station yaw
        station_data.append(float(station_txt[2]))
        # station radius
        station_data.append(float(station_txt[3]))
        # station angle tolerance
        station_data.append(float(station_txt[4]))
        # station distance hysteresis
        station_data.append(float(station_txt[5]))
        # station angle hysteresis
        station_data.append(float(station_txt[6]))
        return station_data
    else:
        return None

# Check if station is inside grid dimension and if radius is not bigger than square
def check_station(grid_dict, station_data):
    # Check if station radius is smaller than square side lenght:
    if (station_data[3] > grid_dict['cell_side_length']) or (station_data[3] <=0):
        print("Station radius must be a positive value and inferior than cell side lenght (%f meters)" % grid_dict['cell_side_length'])
        return False
    # Check if angle tolerance is inferior or equal to pi radians
    if (station_data[4] > math.pi):
        print("Station angle tolerance must be inferior or equal to pi radians")
        return False
    # Check if distance hysteresis is smaller than square cell side length
    if (station_data[5] > grid_dict['cell_side_length']/2):
        print("Station distance hysteresis must be inferior than half of the cell side lenght (%f meters)" % (grid_dict['cell_side_length']/2))
        return False
    # Check if angle hysteresis is inferior or equal to pi radians
    if (station_data[6] > math.pi/2):
        print("Station angle hysteresis must be inferior or equal to half-pi radians")
        return False

    # Check if station is in grid dimension
    # Check if x is in grid
    if (station_data[0] > (grid_dict['origin']['x'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_x'])) or (station_data[0] < grid_dict['origin']['x']):
        print("Station center point is not in grid, please enter a point inside the grid limits (from (%f, %f) to (%f, %f))"
           % (grid_dict['origin']['x'],
              grid_dict['origin']['y'], 
             (grid_dict['origin']['x'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_x']),
             (grid_dict['origin']['y'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_y'])))
        return False
    # Check if y is in grid
    if (station_data[1] > (grid_dict['origin']['y'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_y'])) or (station_data[1] < grid_dict['origin']['y']):
        print("Station center point is not in grid, please enter a point inside the grid limits (from (%f, %f) to (%f, %f))"
           % (grid_dict['origin']['x'],
              grid_dict['origin']['y'], 
             (grid_dict['origin']['x'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_x']),
             (grid_dict['origin']['y'] + grid_dict['cell_side_length']*grid_dict['number_of_cells_y'])))
        return False

    # Else, station is okay
    return True



#=============================
#         Main Script
#=============================
# Import raw_input for python2, stick with input if using python3
try:
    import __builtin__
    input = getattr(__builtin__, 'raw_input')
except (ImportError, AttributeError):
    pass

print("-------------------------------")
print(" 2D pose region grid generator")
print("-------------------------------")
print("Please enter grid file name")
prompt = '> '
file_name = input(prompt)

# Base grid definition
print("Enter origin of grid on the format \"x,y\"")
prompt = '>'
start_point = input(prompt)
point_coord = extract_point_coord_from_input(start_point)
while not (point_coord):
    print("Entered value is not on the format \"x,y\", please enter origin of grid again")
    prompt = '>'
    start_point = input(prompt)
    point_coord = extract_point_coord_from_input(start_point)

print("Enter cell side lenght (in meters)")
cell_side_length = None
while not cell_side_length:
    prompt = '>'
    cell_side_length = float(input(prompt))
    if cell_side_length <= 0:
        print("Please enter a non-zero positive value")
        cell_side_length = None

print("Enter cell hysteresis (added distance for leaving cell, in meters and needs to be smaller than half cell side length)")
cell_hysteresis = None
while not cell_hysteresis:
    prompt = '>'
    cell_hysteresis = float(input(prompt))
    if (cell_hysteresis <= 0) or (cell_hysteresis >= cell_side_length/2):
        print("Please enter a non-zero positive value and smaller than half cell side length")
        cell_hysteresis = None

print("Enter how many cells on x-axis")
number_of_cells_x = None
while not number_of_cells_x:
    prompt = '>'
    number_of_cells_x = int(input(prompt))
    if number_of_cells_x < 1:
        print("Please enter a minimum value of 1 square on x-axis")
        number_of_cells_x = None

print("Enter how many cells on y-axis")
number_of_cells_y = None
while not number_of_cells_y:
    prompt = '>'
    number_of_cells_y = int(input(prompt))
    if number_of_cells_y < 1:
        print("Please enter a minimum value of 1 square on x-axis")
        number_of_cells_y = None

# Grid recap
print("----------------------------------------------------------")
print("Creating grid of %f meters (x-axis) by %f meters (y-axis) with origin (%f, %f), and %i square cells of side length of %f meters. Hysteresis for leaving cell of %f meters." 
      % ((number_of_cells_x * cell_side_length),
         (number_of_cells_y * cell_side_length),
          point_coord[0], point_coord[1],
          number_of_cells_x * number_of_cells_y,
          cell_side_length,
          cell_hysteresis))
print("----------------------------------------------------------")
# Create grid dictionary
grid_dict = {}
grid_dict.update({'origin': {'x': point_coord[0],
                             'y': point_coord[1]},
                  'cell_side_length': cell_side_length,
                  'cell_hysteresis': cell_hysteresis,
                  'number_of_cells_x': number_of_cells_x,
                  'number_of_cells_y': number_of_cells_y})


# Add stations
print("Enter stations on the format \"x, y, yaw, radius, angle tolerance, distance hysteresis, angle hysteresis\" (in meters and angle in rad), type \"end\" to stop entering stations")
inputing_stations = True
station_list = []
while inputing_stations:
    prompt = '>'
    keyboard_input = input(prompt)
    if not keyboard_input == "end":
        station = extract_station_from_input(keyboard_input)
        if station:
            if check_station(grid_dict, station):
                station_list.append(station)
        else:
            print("Entered value is not on the format \"x, y, yaw, radius, angle tolerance, distance hysteresis, angle hysteresis\" (in meters and angle in rad), please enter station again")
    else:
        inputing_stations = False

print("----------------------------------------------------------")
if len(station_list) == 0:
    print(" No station added")
    station_dicts = []
else:
    print(" Adding the following stations")
    station_dicts = []
    for i in range(len(station_list)):
        print("  - center: (%f, %f), yaw: %f rad, radius: %f meters, angle tolerance: %f, distance hysteresis: %f, angle hysteresis: %f" % (station_list[i][0],
                                                                                                                                            station_list[i][1],
                                                                                                                                            station_list[i][2],
                                                                                                                                            station_list[i][3],
                                                                                                                                            station_list[i][4],
                                                                                                                                            station_list[i][5],
                                                                                                                                            station_list[i][6]))
        station_dicts.append({'origin': {'x': station_list[i][0],
                                        'y': station_list[i][1],
                                        'yaw': station_list[i][2]},
                              'radius': station_list[i][3],
                              'angle_threshold': station_list[i][4],
                              'dist_hysteresis': station_list[i][5],
                              'angle_hysteresis': station_list[i][6],
                              })

print("----------------------------------------------------------")
print(" Enter agent initial position on the on the format \"x,y\"")
prompt = '>'
initial_position_str = input(prompt)
initial_position = extract_point_coord_from_input(initial_position_str)
while not (initial_position):
    print("Entered value is not on the format \"x,y\", please enter agent initial position again")
    prompt = '>'
    initial_position_str = input(prompt)
    initial_position = extract_point_coord_from_input(initial_position_str)


region_2d_dict = {}
region_2d_dict.update({'grid': grid_dict, 'stations': station_dicts, 'initial_position': initial_position})


# Generate and write ts to file
region_2d_pose_generator.write_to_file(file_name, region_2d_pose_generator.generate_regions_and_actions(region_2d_dict))