#!/usr/bin/env python3

#Author: Tony Jacob
#Part of RISE Project. 
#Utility functions for path_gen.py
#tony.jacob@uri.edu

import numpy as np
import cv2
import math

##Viz functions.
def compare_two_lists(list1:list, list2:list, height:int, width:int):
    """
    Viz function to plot the original canny and extracted edge

    
    Args:
        list1: List of points (raw_pixels)
        list2: List of points

    Returns:
        image: Image with points plotted.
    """
    image = np.zeros((height, width), dtype=np.uint8)
    # Iterate through the list of coordinates and draw circles
    
    for coordinates in list1:
        center = tuple(coordinates)
        cv2.circle(image, center, 1, 100)

    for coordinates in list2:
        center = tuple(coordinates)
        cv2.circle(image, center, 1, 255)
    
    #plot center
    cv2.circle(image, (height//2, width//2),1,255)
    return image

def compare_points_with_image(frame:np.array, points_list:list):
    """
    Viz function to plot list of points in existing image

    Args:
        frame: Image Array
        points_list: list of points

    Returns:
        frame: Image Array with points_list vizualised
    """
    for index, coordinates in enumerate(points_list):
        center = tuple(coordinates)
        
        if index == 0:
            cv2.circle(frame, center, 1, 255, 1)
        else:
            cv2.circle(frame, center, 1, 100, 1)
    return frame

#Rotation Matrix
def rotate_points(points:list, angle_radians:float):
    """
    Rotation Matrix
    """

    # Define the rotation matrix
    rotation_matrix = [
        [math.cos(angle_radians), math.sin(angle_radians)],
        [-math.sin(angle_radians), math.cos(angle_radians)]
    ]
    
    # Initialize a list to store rotated points
    x_list = []
    y_list = []

    
    # Apply rotation to each point
    for point in points:
        # Perform matrix multiplication to rotate the point
        rotated_x = rotation_matrix[0][0] * point[0] + rotation_matrix[0][1] * point[1]
        rotated_y = rotation_matrix[1][0] * point[0] + rotation_matrix[1][1] * point[1]
        
        # Append rotated point to the list
        x_list.append((rotated_x))
        y_list.append((rotated_y))
    
    return x_list, y_list

def find_cordinates_of_max_value(image:np.array):
    """
    Find cordinates of white pixels

    Args:
        image: Image after Canny edge detection.
    
    Returns:
        max_intensity_coordinates: [[x,y]] of edges.
    """
    max_intensity = np.max(image)
    if max_intensity != 0:
        # Get the coordinates of pixels with the maximum intensity
        max_intensity_coordinates = np.column_stack(np.where(image == max_intensity))
        # column_stack GIVES INVERTED [x,y] from that of image frame
        max_intensity_coordinates = [[y,x] for x,y in max_intensity_coordinates]
        return max_intensity_coordinates
    else:
        return None

#Fitting functions
def model_f(x, a:float, b:float, c:float):
    """
    The polynomial used to fit the points.
    """
    return a*x**2 + b* x**1 +c

def calculate_slope(x_coords:list, y_coords:list):
    """
    Function to calculate slope 
    based on https://doi.org/10.1016/j.ifacol.2015.10.262 
    to handle vertical slope situations

    Args:
        x_coords: list of x_cordinates in {E}
        y_coords: list of y_cordinates in {E}
    """
    # Ensure there are at least two points
    if len(x_coords) < 2 or len(y_coords) < 2:
        raise ValueError("At least two points are required to calculate a slope.")
    
    if len(x_coords) != len(y_coords):
        raise ValueError("The number of x and y coordinates must be the same.")
    
    s_xx = sum([(x - np.mean(x_coords))**2 for x in x_coords])
    s_yy = sum([(y - np.mean(y_coords))**2 for y in y_coords])
    s_xy = sum([(x - np.mean(x_coords))*(y - np.mean(y_coords)) for x,y in zip(x_coords,y_coords)])
    
    # print(s_xx, s_yy, s_xy)
    if s_xx >= s_yy:
        beta_1 = s_xy/s_xx
        beta_0 = np.mean(y_coords) - beta_1*np.mean(x_coords)
        return beta_1, beta_0

    elif s_xx < s_yy:
        beta_1 = s_xy/s_yy
        beta_0 = np.mean(x_coords) - beta_1*np.mean(y_coords)
        #Get it in relative to x
        beta_0 = beta_0/beta_1
        beta_1 = 1/beta_1
        #slope, intercept
        return beta_1, beta_0


def sum_angles_radians(*angles):
    """
    Wrap angles within -pi,pi.

    Args:
        *angles: any number of arguments of angles in radians
    """
    total = sum(angles)
    if total>math.pi:
        total = total - 2*math.pi
    if total< -math.pi:
        total = total + 2*math.pi

    return total