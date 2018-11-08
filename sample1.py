# examples/Python/Tutorial/Basic/pointcloud.py

import numpy as np
import open3d

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = open3d.read_point_cloud("d:/git/open3d/examples/TestData/fragment.ply")
    print(pcd)
    print(np.asarray(pcd.points))
    open3d.draw_geometries([pcd])

    print("Downsample the point cloud with a voxel of 0.05")
    downpcd = open3d.voxel_down_sample(pcd, voxel_size = 0.05)
    open3d.draw_geometries([downpcd])

    print("Recompute the normal of the downsampled point cloud")
    open3d.estimate_normals(downpcd, search_param = open3d.KDTreeSearchParamHybrid(
            radius = 0.1, max_nn = 30))
    open3d.draw_geometries([downpcd])

    print("Print a normal vector of the 0th point")
    print(downpcd.normals[0])
    print("Print the normal vectors of the first 10 points")
    print(np.asarray(downpcd.normals)[:10,:])
    print("")

    print("Load a polygon volume and use it to crop the original point cloud")
    vol = open3d.read_selection_polygon_volume("d:/git/open3d/examples/TestData/Crop/cropped.json")
    chair = vol.crop_point_cloud(pcd)
    open3d.draw_geometries([chair])
    print("")

    print("Paint chair")
    chair.paint_uniform_color([1, 0.706, 0])
    open3d.draw_geometries([chair])
    print("")