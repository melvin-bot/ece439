# -*- coding: utf-8 -*-

import numpy as np
from matplotlib import pyplot as plt
import rospy

# Get movement parameters from parameter server
pen_liftoff_distance = rospy.get_param("/pen_liftoff_distance")
pen_draw_distance = rospy.get_param("/pen_draw_distance")
max_speed_move = rospy.get_param("/max_speed_move")
max_speed_draw = rospy.get_param("/max_speed_draw")
max_accel_move = rospy.get_param("/max_accel_move")
max_accel_draw = rospy.get_param("/max_accel_draw")
canvas_size_x = rospy.get_param("/canvas_size_x")
canvas_size_y = rospy.get_param("/canvas_size_y")
canvas_center_x = rospy.get_param("/canvas_center_x")
canvas_center_y = rospy.get_param("/canvas_center_y")

# pen_liftoff_distance = 0.02
# pen_draw_distance = -0.002
# max_speed_move = 0.3
# max_speed_draw = 0.1
# max_accel_move = 1.0
# max_accel_draw = 0.3
# canvas_size_x = 0.1651
# canvas_size_y = 0.2286
# canvas_center_x = 0.06985
# canvas_center_y = -0.1016

# All-inclusive function to get waypoints from an SVG file.    
def convert_svg_to_waypoints(svg_file,
                             canvas_size_x=canvas_size_x,
                             canvas_size_y=canvas_size_y,
                             canvas_center_x=canvas_center_x,
                             canvas_center_y=canvas_center_y,
                             speed_move=max_speed_move,
                             speed_draw=max_speed_draw,
                             accel_move=max_accel_move,
                             accel_draw=max_accel_draw,
                             pen_liftoff_distance=pen_liftoff_distance,
                             pen_draw_distance=pen_draw_distance):
    svg_coords = parse_svg_for_paths(svg_file, speed_move, speed_draw, accel_move, accel_draw, pen_liftoff_distance, pen_draw_distance)
    scaled_coords = scale_coords_to_arena(svg_coords, canvas_size_x, canvas_size_y, canvas_center_x, canvas_center_y)
    
    return scaled_coords

# Simple function to parse the line paths in an SVG file (marked "d") into a set of absolute spatial coordinates. 
def parse_svg_for_paths(svg_file,
                        speed_move=max_speed_move,
                        speed_draw=max_speed_draw,
                        accel_move=max_accel_move,
                        accel_draw=max_accel_draw,
                        pen_liftoff_distance=pen_liftoff_distance,
                        pen_draw_distance=pen_draw_distance):
    # Find lines with 'd="' at the beginning: 
    d_lines = []
    ignore = 0    # control parameter to ignore "<clipPath>" sections
    with open(svg_file,'r') as svgfile:
        for line in svgfile:
            l = line.strip()
            if l[0:3]=='d="' and not ignore:
                d_lines.append(line.strip().strip(' <d="/>'))
            elif l[0:9] == '<clipPath':
                ignore = 1
            elif l[0:11] == '</clipPath>':
                ignore = 0
            
    # parse such lines to separate out anything delimited by space
    svg_coords = np.ndarray((0,5)).astype(float)
    d_line_origin = np.array([[0.,0.,pen_draw_distance,speed_draw,accel_draw]])
    current_point = np.array([[0.,0.,0.,0.]])
    #first_absolute = 1 
    for d_line in d_lines:
        chunks = d_line.split(' ')  # Split the string at spaces
        # Step through the chunks and parse out coordinates
        ii = 0
        while ii < len(chunks): 
            chunk = chunks[ii]
            if chunk[0].isalpha():  # If chunk is alphabetic, it's a setting telling us what's coming. 
                
                # Case of mode command determines absolute (upper) or relative (lower)
                if chunk.isupper():
                    absolute = 1
                else : 
                    absolute = 0
                
                # use the lowercase version to settle what mode we are in. 
                mode = chunk.lower()
                # Set the number of chunks to increment
                if mode == "m":
                    incr = 1
                    # first M or m is always absolute, but this is a separate setting from Mode. 
                    first_absolute = 1
                elif mode == "l":
                    incr = 1
                elif mode == "h":  # horizontal segment
                    incr = 1
                elif mode == "v":  # vertical segment
                    incr = 1
                elif mode == "c":  # skip the control points
                    incr = 3
                elif mode == "s":  # skip the control point
                    incr = 2
                elif mode == "z":  # closing "z" or "Z"
                    incr = 1
                    svg_coords = np.append(svg_coords, d_line_origin, axis=0)
                else:
                    incr = 1
                    
                ii += incr
                continue
            
            else: 
                xy = chunk.split(',')
                xy = [float(s) for s in xy]
                
                # Deal with horizontal and vertical lines. 
                # They have only one number, so we have to fill the other with 0.0. 
                if mode == "h" and len(xy)==1:
                    if absolute: 
                        xy = [xy[0],current_point[1]]
                    else:
                        xy = [xy[0],0.0]
                elif mode == "v" and len(xy)==1: 
                    if absolute: 
                        xy = [current_point[0],xy[0]]
                    else:
                        xy = [0.0,xy[0]]
                
                # Now either initiate or append the new coordinate. 
                if first_absolute:
                    # Retract, then move, then set down at new point
                    if len(svg_coords) > 0:
                        svg_coords = np.append(svg_coords, [[current_point[0],current_point[1],pen_liftoff_distance,speed_draw,accel_draw]],axis=0)
                    svg_coords = np.append(svg_coords, [[xy[0],xy[1],pen_liftoff_distance,speed_move,accel_move]],axis=0)                
                    svg_coords = np.append(svg_coords, [[xy[0],xy[1],pen_draw_distance,speed_draw,accel_draw]],axis=0)
                    # set first_absolute to zero (not anymore). 
                    first_absolute = 0
                    d_line_origin = np.array([[xy[0],xy[1],pen_draw_distance,speed_draw,accel_draw]])
                else:
                    if absolute:
                        svg_coords = np.append(svg_coords, [[xy[0],xy[1],pen_draw_distance,speed_draw,accel_draw]],axis=0)
                    else:
                        svg_coords = np.append(svg_coords, [current_point + np.array([xy[0],xy[1],0.,0.,0.])],axis=0)
                
                current_point = svg_coords[-1]
                
                if ii < len(chunks)-1:
                    if chunks[ii+1].isalpha():
                        incr = 1
                
                ii += incr
    
    svg_coords = np.append(svg_coords, np.array([[current_point[0],current_point[1],pen_liftoff_distance,speed_draw,accel_draw]]), axis=0)
                
    return svg_coords

# Function to scale the coordinates from SVG to a specific size of workspace (rectangle, dimensions dx and dy)            
def scale_coords_to_arena(coords,
                          canvas_size_x=canvas_size_x,
                          canvas_size_y=canvas_size_y,
                          canvas_center_x=canvas_center_x,
                          canvas_center_y=canvas_center_y):
    x_svg = coords[:,0]
    y_svg = coords[:,1]  # Note that Y is Down in SVG!
    z_svg = coords[:,2]
    speed_svg = coords[:,3]
    accel_svg = coords[:,4]
    
    x_min_svg = np.min(x_svg)
    x_max_svg = np.max(x_svg)
    x_range_svg = x_max_svg - x_min_svg
    x_center_svg = x_min_svg + x_range_svg / 2
    y_min_svg = np.min(y_svg)
    y_max_svg = np.max(y_svg)
    y_range_svg = y_max_svg - y_min_svg
    y_center_svg = y_min_svg + y_range_svg / 2
 
    # Shift and scale. Also Rearrange Coordinates
    svg_scaling_factor = min([canvas_size_x/x_range_svg, canvas_size_y/y_range_svg])
    x_scaled = (x_center_svg - x_svg) * svg_scaling_factor + canvas_center_x
    y_scaled = (y_center_svg - y_svg) * svg_scaling_factor + canvas_center_y   # Note that Y is Down in SVG!
    scaled_coords = np.vstack((x_scaled, y_scaled, z_svg, speed_svg, accel_svg)).T
    
    return scaled_coords
        



if __name__=='__main__':
    svg_file = 'jerry_draw/src/smiley.svg'
    scaled_coords = convert_svg_to_waypoints(svg_file)

    print(scaled_coords)
    
    # Plot it if you want to:
    fig = plt.figure()
    plt.plot(scaled_coords[:,0],scaled_coords[:,1],'o-')
    plt.axis('equal')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.show()