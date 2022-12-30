import open3d as o3d
import random
import numpy as np
import time

start = time.time()

# read demo point cloud provided by Open3D
pcd_point_cloud = o3d.data.PCDPointCloud()
pcd = o3d.io.read_point_cloud(pcd_point_cloud.path)

# RANSAC params
min_points = 3 # minimum number of points to create a plane
in_threshold = 0.02 # threshold of point distance to plane to be considered inlier
confidence = 0.90 # acceptable threshold of sampled points being inliers
max_iteration = 500 # maximum number of iterations for RANSAC algo before outputting best model

# Finding points using RANSAC
points = np.asarray(pcd.points)
total_points = len(points) # = 113662

# Init permanent variables to compare temp variables with later
best_in_points = []
best_eq = []
best_chosen_points = []

# Init temp variables for easy access later (need to be reset to 0 after for loop)
in_points = []
chosen_points = []
vec12 = []
vec23 = []
perpen_vec = []
d = 0
plane_eq = []
dist = 0
voted = 0

for num in range(max_iteration):
    # Grab min_points and put into chosen_points list
    for int in range(min_points):
        chosen_points.append(random.choice(points))
    
    # Create 2 vectors representing the minimum of 3 chosen points, regardless of min_points var value
    # Format of vectors is np.array([x,y,z])
    vec12 = np.array([chosen_points[1][0]-chosen_points[0][0], chosen_points[1][1]-chosen_points[0][1], chosen_points[1][2]-chosen_points[0][2]])
    vec23 = np.array([chosen_points[2][0]-chosen_points[1][0], chosen_points[2][1]-chosen_points[1][1], chosen_points[2][2]-chosen_points[1][2]])

    # Find perpendicular vector by finding cross product of vec12 and vec23, which can also be found using np.cross(vec12, vec23)
    perpen_vec = np.array([(vec12[1]*vec23[2])-(vec12[2]*vec23[1]), -(vec12[0]*vec23[2])-(vec12[2]*vec23[0]), (vec12[0]*vec23[1])-(vec12[1]*vec23[0])])
    perpen_vec = perpen_vec / np.linalg.norm(perpen_vec)

    # Fill in plane formula with (a,b,c) as perpendicular vector's (x,y,z) and arbitrarily chosen_points[1] to get d
    d = -np.sum(np.multiply(perpen_vec, chosen_points[1]))

    # Format: a(x-x0)+b(y-y0)+c(z-z0)=d
    plane_eq = [perpen_vec[0], perpen_vec[1], perpen_vec[2], d]

    # Finding distance of all points to place to determine inliers and outliers
    for point in points:
        dist = abs(point[0]*plane_eq[0]+point[1]*plane_eq[1]+point[2]*plane_eq[2]+d)/np.sqrt(perpen_vec[0]**2+perpen_vec[1]**2+perpen_vec[2]**2)
        if (dist <= in_threshold):
            in_points.append(point)
    
    # If we have more inliers now, change our best inliers outcome
    if len(best_in_points) < len(in_points):
        best_in_points = in_points
        best_eq = plane_eq
        best_chosen_points = chosen_points

    # If confidence was reached, break out of for loop
    if (confidence <= len(in_points)/total_points):
        break

    # Otherwise, reset temp variables for next iteration
    in_points = []
    chosen_points = []
    vec12 = []
    vec23 = []
    perpen_vec = []
    d = 0
    plane_eq = []
    dist = 0
    voted = 0

# Initialize empty point cloud to put best_in_points in because there is a difference in class
# final_pcd.points has type open3d.cpu.pybind.utility.Vector3dVector
# in_points has type open3d.cpu.pybind.geometry.PointCloud
final_pcd = o3d.geometry.PointCloud()
final_pcd.points = o3d.utility.Vector3dVector(np.array(best_in_points, dtype=object))
final_pcd.paint_uniform_color([1,0,0]) # Paint points red

print("Rated: " + str(round((len(best_in_points)/total_points*100), 2)) + "%")
end = time.time()
print("Time:", round(end - start, 2), "seconds")

# Visualize best fitted plane after RANSAC
o3d.visualization.draw_geometries([pcd, final_pcd],
zoom=1,
front=[0.4257, -0.2125, -0.8795],
lookat=[2.6172, 2.0475, 1.532],
up=[-0.0694, -0.9768, 0.2024]
)

