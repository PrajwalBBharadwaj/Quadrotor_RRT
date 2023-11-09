import timeit
import numpy as np
from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon
from math import log
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle   
import shapely
from queue import Queue
from obstacles import Obstacles
from RRT import RRT
from replan import Replan
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from shapely.geometry import LinearRing
from shapely.plotting import plot_line, plot_points
from mpl_toolkits.mplot3d import Axes3D

def plot(nodes, obstacle_poly, goal1_poly, path=0, path_present=False, two_paths=False, sigma_separate=[]):
    '''
    FUnction to plot the nodes, trajectoris, the path and the robot motions

    Parameters
    ----------
    nodes : array
        The tree expanded.
    robot_pose : Point
        The current position of the robot.
    obstacle_pose : Point
        The current position of the obstacle in the environment.
    plot_no : int, optional
        Number of plot for indexing the plot for the purpose of saving. The default is 0.
    path : list(int)
        The list of path values represented by the indices of the nodes in the tree.
    path_present : boolean, optional
        To determine whether to print path on plot or not. The default is False.
    two_paths : boolean, optional
        To determine whetehr there are two paths to printed or not. The default is False.
    sigma_separate : int(list), optional
        Takes in a list of nodes to plot on the path on the map. The default is [].

    Returns
    -------
    None.

    '''
    # plt the grid space
    # plot the obstacles
    # plot the edges

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Set limits for all three axes
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(-10, 10)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    valid_nodes = np.argwhere(nodes[:,5]==1).flatten()

    for j in range(valid_nodes.shape[0]):
        i = valid_nodes[j]
        if nodes[i][5] == 1:
            parent = int(nodes[i][4])
            if (parent ==-1 or nodes[parent,5]!=1):
                continue
            x = [nodes[i][0], nodes[parent][0]]
            y = [nodes[i][1], nodes[parent][1]]
            z = [nodes[i][2], nodes[parent][2]]
            plt.plot(x, y, z, color='cyan', linewidth=1,zorder=-1)
    if path_present is True:
        points_x = []
        points_y = []
        points_z = []
        for i in path:
            points_x.append(nodes[i][0])
            points_y.append(nodes[i][1])
            points_z.append(nodes[i][2])
        ax.plot(points_x, points_y, points_z, color='red', linewidth=2)
        ax.scatter(points_x[0], points_y[0], points_z[0], color='blue', s=30, zorder=0)

        if two_paths:
            points_x = []
            points_y = []
            points_z = []
            for i in sigma_separate:
                points_x.append(nodes[i][0])
                points_y.append(nodes[i][1])
                points_z.append(nodes[i][2])
            ax.plot(points_x, points_y, points_z, color='green', linewidth=2)
            ax.scatter(points_x[0], points_y[0], points_z[0], color='purple', s=30, zorder=0)

    goal1_x, goal1_y, goal1_z = zip(*goal1_poly.exterior.coords)
    ax.plot(goal1_x, goal1_y, goal1_z, color='green', linewidth=2)
    ax.add_collection3d(Poly3DCollection(obstacle_poly, facecolors='yellow', linewidths=1, edgecolors='black', alpha=0.3))
    
    plt.show()
    plt.savefig('Plots_map1/Fig'+str(0))
    
##################################################################################################################
'''
Main Loop
Please run the function one at a time. To run another function, clear all the variables andre run with desired function
'''
 
def runDynamicObstacles():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))    
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 5] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1], rrt.nodes[int(path[i]), 2])

        if step == 3:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(3.5,-5.0,5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        if step == 8:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(7.5,-7.5, 5.0)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
            blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
            rrt.nodes[blocked_tree_indices, 5] = 0
    
        if step == 10:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(7.5,2.5,5.0)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        if step == 12:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(7.5,-7.5,5.0)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
            blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
            rrt.nodes[blocked_tree_indices, 5] = 0
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            recheck_goal = False
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle,True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True, True, sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                print('Reconnect failed... trying regrow')
                temp = -1*np.ones((3000,7))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,6] = np.arange(rrt.nodes.shape[0])
                
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
            plot_no += 1
    
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                i = 0
                path_len = len(path)
        step = step+1

def runReconnect():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 5] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1], rrt.nodes[int(path[i]), 2])

        if step == 3:
            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(6.5,2.5,5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle,True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt,sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                print('Reconnect failed trying regrow')
                temp = -1*np.ones((3000,7))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,6] = np.arange(rrt.nodes.shape[0])
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            
            plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
            plot_no+=1
                
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                i = 0
                path_len = len(path)
        step = step+1       
        
def runRegrow():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    obstacle_pose = Point(-5,5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 5] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1], rrt.nodes[int(path[i]), 2])

        if step == 3:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(3.5,-5.0,5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle, True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                j = 0
                print('Reconnect failed trying regrow')
                temp = -1*np.ones((3000,7))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,6] = np.arange(rrt.nodes.shape[0])
                
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            plot_no+=1
            if (plot_no > 3):
                plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
                
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                print('Latest Path Cost:')
                print(cost)
                i = 0
                path_len = len(path)
        step = step+1
        
# Code for testing (please ignore)
def RNN():
    
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1], rrt.nodes[int(path[i]), 2])

        if step == 3:

            rrt.nodes[rrt.nodes[:,5]==0,5]=1
            obstacle_pose = Point(7.5,2.5, 5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
          
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            rrt2 = RRT(rrt.obstacles)
            rrt2.p_current = 0
            rrt2.start = rrt.nodes[rrt.p_current,0:3]
            rrt2.nodes[0,0:3] = rrt2.start
            rrt2.nodes[0][3] = 0
            rrt2.nodes[0][4] = 0
            j = 0
            replan.prune_till_pcurr(rrt.p_current, rrt)
            replan.modify_tree_and_path(rrt,path)
            for i in range(6000):
                x = rrt2.sample()
                nearest_index = rrt2.nearest(x)
                if nearest_index is None:
                    continue
                x_new, cost_steer = rrt2.steer(x, rrt2.nodes[nearest_index][0:3])
                check = rrt2.is_in_collision(x_new,rrt2.nodes[nearest_index][0:3])
                if check is False:
                    near_indices = rrt2.get_near_list(x_new)
                    rrt2.connect(x_new, nearest_index, cost_steer,new=True)
                    if near_indices.size != 0:
                        best_index, cost = rrt2.nearest_from_list(x_new,near_indices,reconnect=False)
                        if best_index is not(None):
                            cost_steer = rrt2.get_dist(x_new,rrt2.nodes[best_index][0:3])
                            rrt2.connect(x_new, best_index,cost_steer,new=False)
                        rrt2.rewire(x_new,near_indices)
                    
                found, min_cost_idx = rrt2.check_goal()
            if found is True:
                path = rrt2.find_path(min_cost_idx)
                plot(rrt2.nodes,robot_pose,None,0,path,True)
                break
            break

        step = step+1
        
# main functioon to intial RRT* algorithm
map_obstacles = Obstacles()
obstacles = []

obstacle_vertices = [(-3.4, -4.6, 0), (5.6, -4.6, 0), (5.6, 9.6, 0), (-3.4, 9.6, 0),
                    (-3.4, -4.6, 3), (5.6, -4.6, 3), (5.6, 9.6, 3), (-3.4, 9.6, 3)]
obstacle_faces = [[0, 1, 2, 3], [4, 5, 6, 7],
                  [0, 1, 5, 4], [2, 3, 7, 6],
                  [0, 3, 7, 4], [1, 2, 6, 5]]
obstacle_poly = [[obstacle_vertices[i] for i in face] for face in obstacle_faces]

goal1_vertices = [(7, 8, 8),(8, 8, 9),(9, 8, 8), (8, 8, 7)]
goal1_edges = [0, 1, 2, 3, 0]
goal1_poly = Polygon(goal1_vertices)

for face in obstacle_poly:
    obstacles.append(Polygon(face))

map_obstacles.addNewObstacle(obstacles)
obstacles_list = map_obstacles.getObstacles()
print(obstacles_list)
cost_total = []

try:
    with open('rrt_map1.pkl', 'rb') as input:
        rrt = pickle.load(input)
except:
    
    print('Pickle file of RRT* data for map 1 not found---Running RRT*')
    rrt = RRT(obstacles_list)
    print(rrt.nodes.shape)
    rrt.start = np.array([-9,-9,-9])
    rrt.nodes[0][0:3] = rrt.start
    rrt.nodes[0][3] = 0
    rrt.nodes[0][4] = 0
    for i in range(8000):
        x = rrt.sample()
        nearest_index = rrt.nearest(x)
        if nearest_index is None:
            continue
        x_new, cost_steer = rrt.steer(x, rrt.nodes[nearest_index][0:3])
        check = rrt.is_in_collision(x_new,rrt.nodes[nearest_index][0:3])
        if check is True:
            print('Collision at index :',i, 'between', x_new, rrt.nodes[nearest_index][0:3])
        if check is False:
            near_indices = rrt.get_near_list(x_new)
            rrt.connect(x_new, nearest_index, cost_steer,new=True)
            if near_indices.size != 0:
                best_index, cost = rrt.nearest_from_list(x_new, near_indices, reconnect=False)
                if best_index is not(None):
                    cost_steer = rrt.get_dist(x_new,rrt.nodes[best_index][0:3])
                    rrt.connect(x_new, best_index,cost_steer,new=False)
                rrt.rewire(x_new,near_indices)

    # final (after 8,000 iterations)
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
        rrt.plot(path,path_present=True)
        path.reverse()
        print('Path Found: ', path)
        cost_total.append(rrt.get_cost(min_cost_idx))
        print('Cost : ', rrt.get_cost(min_cost_idx))
            
        with open('rrt_map1.pkl', 'wb') as output:
            pickle.dump(rrt, output, pickle.HIGHEST_PROTOCOL)
    else:
        print('Path not found')


###
rrt.nodes = np.hstack((rrt.nodes,np.arange(rrt.nodes.shape[0]).reshape(rrt.nodes.shape[0],1)))
print(rrt.nodes.shape)
    
found, min_cost_idx = rrt.check_goal()
if found is True:
    path = rrt.find_path(min_cost_idx)
else:
    print('path not found')
path.reverse()
rrt.nodes = rrt.nodes[0:rrt.nodes_num,:]
plot(rrt.nodes, obstacle_poly, goal1_poly, path, path_present= True)
replan = Replan()
 
# Function calls. Please run only one of them at a time. Wheb running one, comment out the other two calls
# runDynamicObstacles()
# runReconnect()
# runRegrow()