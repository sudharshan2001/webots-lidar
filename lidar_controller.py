
from controller import Robot
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

def main(robot):
    timestep = 32
    max_speed = 6.28
    
    lm = robot.getMotor('left wheel motor')
    rm = robot.getMotor('right wheel motor')

    lm.setPosition(float('inf'))
    rm.setPosition(float('inf'))
    
    lm.setVelocity(0.0)
    rm.setVelocity(0.0)
    
    lidar = robot.getLidar('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    vis = o3d.visualization.Visualizer()
    
    vis.create_window(width = 480, height = 480)
    
    pcd = o3d.geometry.PointCloud()
  
    while robot.step(timestep)!=-1:
    
        data  = lidar.getPointCloud()
        
        print(np.array(lidar.getLayerRangeImage(3)).shape)

        points = np.zeros((len(data), 3), dtype=np.float32)
        
        for i, p in enumerate(data):
            points[i][0] = p.x
            points[i][1] = p.y
            points[i][2] = p.z
            
        # print(dir(data[0]))
        
        print(np.min(points))
        print(np.max(points))

        pcd.points = o3d.utility.Vector3dVector(points)

        vis.clear_geometries()
        vis.add_geometry(pcd)
        
        vis.update_geometry(pcd)
        
        vis.poll_events()
        vis.update_renderer()
        
        lm.setVelocity(max_speed*0.25)
        rm.setVelocity(max_speed*0.25)
        
if __name__ == "__main__":
    robot = Robot()
    main(robot)
