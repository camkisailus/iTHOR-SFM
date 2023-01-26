import numpy as np
# import math
import random

class Node():
    def __init__(self, parent, action, position, rotation):
        """
            Node used in path planning
            Members:
                parent (Node): node from which this node was visited(added)
                action (String): action that moves robot from parent TO this node
                position ([x, z]): list of floats representing position of this node
        """
        self.x = position[0]
        self.z = position[1]
        self.rotation = rotation
        self.parent = parent
        self.action = action
    
    def expand(self):
        if self.rotation == 0:
            forward_node = Node(parent=self, action="forward", position=[self.x, self.z + 0.25], rotation=self.rotation)
            backward_node = Node(parent=self, action="backward", position=[self.x, self.z - 0.25], rotation=self.rotation)
            right_node = Node(parent=self, action="right", position=[self.x + 0.25, self.z], rotation=90.0)
            left_node = Node(parent=self, action="left", position=[self.x - 0.25, self.z], rotation=270.0)
        
        elif self.rotation == 90:
            forward_node = Node(parent=self, action="forward", position=[self.x + 0.25, self.z], rotation=self.rotation)
            backward_node = Node(parent=self, action="backward", position=[self.x - 0.25, self.z], rotation=self.rotation)
            right_node = Node(parent=self, action="right", position=[self.x, self.z - 0.25], rotation=180.0)
            left_node = Node(parent=self, action="left", position=[self.x, self.z + 0.25], rotation=0.0)
        
        elif self.rotation == 180:
            forward_node = Node(parent=self, action="forward", position=[self.x, self.z - 0.25], rotation=self.rotation)
            backward_node = Node(parent=self, action="backward", position=[self.x, self.z + 0.25], rotation=self.rotation)
            right_node = Node(parent=self, action="right", position=[self.x - 0.25, self.z], rotation=270.0)
            left_node = Node(parent=self, action="left", position=[self.x + 0.25, self.z], rotation=90.0)
        
        elif self.rotation == 270:
            forward_node = Node(parent=self, action="forward", position=[self.x - 0.25, self.z], rotation=self.rotation)
            backward_node = Node(parent=self, action="backward", position=[self.x + 0.25, self.z], rotation=self.rotation)
            right_node = Node(parent=self, action="right", position=[self.x, self.z + 0.25], rotation=0.0)
            left_node = Node(parent=self, action="left", position=[self.x, self.z - 0.25], rotation=180.0)
        return [forward_node, backward_node, right_node, left_node]

    
    def __eq__(self, other):
        # Nodes are equivalent if they are at the same position
        return self.x == other.x and self.z == other.z
    
    def __ne__(self, other):
        return not self.__eq__(other)
    
    def __hash__(self):
        return hash((self.x, self.z))


class Navigation():
    def __init__(self, controller):
        reachable_poses = controller.step(
            action="GetReachablePositions"
        ).metadata["actionReturn"]
        self.reachable_poses = np.zeros((len(reachable_poses), 2))
        for i, pose in enumerate(reachable_poses):
            # y is verticle in this environment, so ignore it for planar navigation
            self.reachable_poses[i][0] = pose['x']
            self.reachable_poses[i][1] = pose['z']
        self.controller = controller
    
    def getRandomValidPose(self):
        valid_pose =  self.reachable_poses[random.randint(0,len(self.reachable_poses)-1), :]
        return {'x':valid_pose[0], 'z':valid_pose[1], 'yaw':0.0}

    def navTo(self, robot_pose, goal_pose, tolerance=0.05):
        """
            Plan path and navigate to goal

            Input: 
                robot_pose (dict) : {'x': float, 'z': float, 'yaw': float}
                goal_pose (dict): {'x': float, 'z': float}
            
            Returns:
                success (bool): indicates nav success
        """
        return self.planPath(robot_pose, goal_pose, tolerance)
        # return self.followPath(path)
        

    def planPath(self, robot_pose, goal_pose, tolerance=0.05):
        # print("Planning path to {}".format(goal_pose))
        goal = (goal_pose['x'], goal_pose['z'])
        goal_pose_np = np.tile(goal, (self.reachable_poses.shape[0],1))
        dists = np.sqrt(np.sum(np.square(np.abs(self.reachable_poses - goal_pose_np)), axis=1))
        if np.min(dists) > tolerance:
            ## Cannot get within tolerance of goal
            print("No reachable poses near the goal location.. Will not attempt to plan")
            pass
        else:
            # populate reachable node
            reachable_nodes = {}
            for i,pose in enumerate(self.reachable_poses):
                node = Node(parent=None, action=None, position=[pose[0], pose[1]], rotation=0.0)
                # print("Adding reachable node: ({}, {})".format(node.x, node.z))
                reachable_nodes[node] = i 
            goal_reached = False
            start_node = Node(parent=None, action=None, position=[robot_pose['x'], robot_pose['z']], rotation=robot_pose['yaw'])
            # print("Start in reachable? {}".format(start_node in reachable_nodes))
            goal_node = Node(parent=None, action=None, position=[goal[0], goal[1]], rotation=0.0)
            nodes_to_visit = [start_node]
            while not goal_reached and len(nodes_to_visit) > 0:
                cur_node = nodes_to_visit.pop(0)
                # print("Cur node: ({}, {})".format(cur_node.x, cur_node.z))
                next_possible_nodes = cur_node.expand()
                valid_possible_nodes = {}
                for i, node in enumerate(next_possible_nodes):
                    idx_to_action_dict = {0: 'forward', 1:'backward', 2:'right', 3:'left'}
                    if node in reachable_nodes:
                        valid_possible_nodes[idx_to_action_dict[i]] = node
                for action, node in valid_possible_nodes.items():
                    if node == goal_node:
                        goal_reached = True
                        goal_node.parent = cur_node
                        goal_node.action = action
                    elif node not in nodes_to_visit:
                        nodes_to_visit.append(node)

            if goal_reached:
                goal_rotation = goal_pose['yaw']
                print("Old Goal: ({}, {}, {})".format(goal_node.x, goal_node.z, goal_node.rotation))
                if goal_node.rotation != goal_rotation:
                    # Add in place rotations to match goal_rotation
                    print("Mismatch in goal_node and goal_rotation. node: {} goal: {}".format(goal_node.rotation, goal_rotation))
                    diff = goal_rotation - goal_node.rotation
                    num_turns = int(np.abs(diff/90))
                    print("Num turns: {}".format(num_turns))
                    if diff < 0:
                        # rotate left
                        node = Node(goal_node, "turn_left", (goal_node.x, goal_node.z), goal_node.rotation-90.0)
                        while node.rotation != goal_rotation:
                            node = Node(node, "turn_left", (node.x, node.z), node.rotation-90.0)
                    else: 
                        # rotate right
                        node = Node(goal_node, "turn_right", (goal_node.x, goal_node.z), goal_node.rotation+90.0)
                        while node.rotation != goal_rotation:
                            node = Node(node, "turn_right", (node.x, node.z), node.rotation+90.0)
                    
                    goal_node = node
                    print("New Goal: ({}, {}, {})".format(goal_node.x, goal_node.z, goal_node.rotation))
                # trace path
                # print("PATH FOUND!")
                cur_node = goal_node
                positions = [] # keep track of positions along path
                actions = [] # keep track of actions taken to reach goal
                while cur_node.parent is not None:
                    # print("Current Node Position: ({}, {}, {}) Action: {}".format(cur_node.x, cur_node.z, cur_node.rotation, cur_node.action))
                    positions.append((cur_node.x, cur_node.z, cur_node.rotation))
                    if cur_node.action == 'left':
                        actions.append('forward')
                        actions.append('turn_left')
                    
                    elif cur_node.action == 'right':
                        actions.append('forward')
                        actions.append('turn_right')
                    else:
                        actions.append(cur_node.action)
                    cur_node = cur_node.parent
                actions.reverse()
                positions.reverse()
                print(positions)
                print(actions)
                # print(positions)
                # pos_np = np.zeros((len(positions), 2))
                # for i,pos in enumerate(positions):
                #     pos_np[i,0] = pos[0]
                #     pos_np[i,1] = pos[1]
                
                # plt.scatter(pos_np[:,0], pos_np[:,1], color='red', label="Path")
                # plt.legend()
                # plt.show()
                return actions
    
