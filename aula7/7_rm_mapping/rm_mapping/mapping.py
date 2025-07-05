#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# based on Cyril Stachniss - Mobile Sensing and Robotics 1 course assignment

import numpy as np
import matplotlib.pyplot as plt
import bresenham as bh

# Plot grid map
def plot_gridmap(gridmap): 
    gridmap = np.array(gridmap, dtype=np.float64)
    plt.figure()
    plt.imshow(gridmap, cmap='Greys',vmin=0, vmax=1)
    plt.show()
    
# Initialize grid map with zeros
def init_gridmap(size, res):
    gridmap = np.zeros([int(np.ceil(size/res)), int(np.ceil(size/res))])
    return gridmap

# Convert real world coordinates to map coordinates
def world2map(pose, gridmap, map_res):
    origin = np.array(gridmap.shape)/2
    new_pose = np.zeros((pose.shape))
    new_pose[0:] = np.round(pose[0:]/map_res) + origin[0]
    new_pose[1:] = np.round(pose[1:]/map_res) + origin[1]
    return new_pose.astype(int)

# Convert a pose to a homogeneous transformation matrix
def v2t(pose):
    c = np.cos(pose[2])
    s = np.sin(pose[2])
    tr = np.array([[c, -s, pose[0]], [s, c, pose[1]], [0, 0, 1]])
    return tr    

# Convert distance data to 2D points
def ranges2points(ranges):
    # laser properties
    start_angle = -1.5708
    angular_res = 0.0087270
    max_range = 30
    # rays within range
    num_beams = ranges.shape[0]
    idx = (ranges < max_range) & (ranges > 0)
    # 2D points
    angles = np.linspace(start_angle, start_angle + (num_beams*angular_res), num_beams)[idx]
    points = np.array([np.multiply(ranges[idx], np.cos(angles)), np.multiply(ranges[idx], np.sin(angles))])
    # homogeneous points
    points_hom = np.append(points, np.ones((1, points.shape[1])), axis=0)
    return points_hom

# Convert sensor data to map coordinates
def ranges2cells(r_ranges, w_pose, gridmap, map_res):
    # ranges to points
    r_points = ranges2points(r_ranges)
    w_P = v2t(w_pose)
    w_points = np.matmul(w_P, r_points)
    # covert to map frame
    m_points = world2map(w_points, gridmap, map_res)
    m_points = m_points[0:2,:]
    return m_points

# Convert poses to map coordinates
def poses2cells(w_pose, gridmap, map_res):
    # covert to map frame
    m_pose = world2map(w_pose, gridmap, map_res)
    return m_pose  

def bresenham(x0, y0, x1, y1):
    l = np.array(list(bh.bresenham(x0, y0, x1, y1)))
    return l

# Convert porobabilities to log odds    
def prob2logodds(p):
    l = np.log(p/(1-p))
    return l

# Convert log odds to probabilities
def logodds2prob(l):
    p = 1 - 1/(1 + np.exp(l))
    return p

# Inverse sensor model  
def inv_sensor_model(cell, endpoint, prob_occ, prob_free):
    if tuple(cell) == tuple(endpoint):
        return prob_occ
    else:
        return prob_free

def grid_mapping_with_known_poses(ranges_raw, poses_raw, occ_gridmap, map_res, prob_occ, prob_free, prior):
    logodds_map = prob2logodds(occ_gridmap)

    poses_cells = poses2cells(poses_raw, occ_gridmap, map_res)

    for pose in poses_raw:
        rays = ranges2cells(ranges_raw, pose, occ_gridmap, map_res)

        for ray in rays:
            sensible_path = inv_sensor_model(ray, rays, prob_occ, prob_free)

            for cell in sensible_path:
                logodds_map = logodds_map + prob2logodds(sensible_path)

    return logodds2prob(logodds_map)

map_size = 100
map_res = 0.25

prior = 0.50
prob_occ = 0.90
prob_free = 0.35

# load data
ranges_raw = np.loadtxt("data/ranges.data", delimiter=',', dtype='float')
poses_raw = np.loadtxt("data/poses.data", delimiter=',', dtype='float')

# initialize gridmap
occ_gridmap = init_gridmap(map_size, map_res)+prior
plot_gridmap(occ_gridmap)

#Grid map output
gridmap = grid_mapping_with_known_poses(ranges_raw, poses_raw, occ_gridmap, map_res, prob_occ, prob_free, prior)

#Plot the grid map
plot_gridmap(gridmap)
