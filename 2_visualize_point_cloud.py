import open3d
import numpy as np


def visualize(point_cloud):
    vis = open3d.visualization.Visualizer()
    vis.create_window()

    vis.get_render_option().point_size = 1.0
    vis.get_render_option().background_color = np.ones(3)

    # draw origin
    axis_pcd = open3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
    vis.add_geometry(axis_pcd)

    vis.add_geometry(point_cloud)
    vis.run()


if __name__ == "__main__":
    pcd = open3d.io.read_point_cloud("./pcd/lidar__left__1656676877.20443.pcd")
    visualize(point_cloud=pcd)
