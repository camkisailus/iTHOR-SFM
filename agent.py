from navigation import Navigation
import numpy as np
import cv2
from particle_filters import ObjectParticleFilter, FrameParticleFilter
import utils
import math
import os

ROOT = os.path.dirname(os.path.realpath(__file__))


class State:
    def __init__(self, action_history=[]):
        self.action_history = action_history


class Agent:
    def __init__(self, controller, pose, frames, objects, trial_name):
        self.trial_name = trial_name
        os.mkdir(os.path.join(ROOT, "top_down", "trial_{}".format(trial_name)))
        os.mkdir(os.path.join(ROOT, "distributions", "trial_{}".format(trial_name)))
        self.controller = controller
        self.nav = Navigation(controller)
        self.cur_pose = pose
        self.idx = 0
        self.cam_idx = 0
        self.topdown_frames = []
        self.object_filters = {}
        for object in objects:
            self.object_filters[object] = ObjectParticleFilter(
                object.capitalize(), self.controller
            )
        # print("Loaded the following object filters")
        # for name, filter in self.object_filters.items():
        #     print("Name: {}".format(name))
        self.frame_filters = {}
        for frame in frames:
            filter = FrameParticleFilter(
                frame.name.capitalize(),
                preconditions=frame.preconditions,
                core_frame_elements=frame.core_frame_elements,
                controller=self.controller,
            )
            for frame_element in frame.core_frame_elements:
                filter.addFrameElementFilter(
                    frame_element, self.object_filters[frame_element]
                )
            self.frame_filters[frame.name] = filter
        for frame in frames:
            if frame.preconditions is not None:
                for precondition in frame.preconditions:
                    self.frame_filters[frame.name].addPreconditionFilter(
                        precondition, self.frame_filters[precondition]
                    )
        # print("Loaded the following frame filters")
        # for filter in self.frame_filters.values():
        #     print(filter)

        self.state = State()
        self.frameElements = [label for label in self.object_filters.keys()]
        # self.observeSurroundings()

    def updateFilters(self):
        for objFilter in self.object_filters.values():
            objFilter.updateFilter()
        for frameFilter in self.frame_filters.values():
            frameFilter.updateFilter(self.state)

    # def check_point_in_fov(self, x1, y1, xp, yp, fov, visibility_dist): #x1, y1 is robot position, considering rotation switch x1, y1

    #     angle = fov/2
    #     range_x = visibility_dist/math.cos(2*angle*math.pi/360)

    #     #left vertex
    #     x2 = x1 - range_x
    #     y2 = y1 + visibility_dist

    #     #right vertex
    #     x3 = x1 + range_x
    #     y3 = y1 + visibility_dist

    #     #perform tests
    #     t1 = (x2-x1)*(yp-y1)-(y2-y1)*(xp-x1)
    #     t2 = (x3-x2)*(yp-y2)-(y3-y2)*(xp-x2)
    #     t3 = (x1-x3)*(yp-y3)-(y1-y3)*(xp-x3)
        
    #     if (t1<0 and t2<0 and t3<0) or (t1>0 and t2>0 and t3>0):
    #         return True
    #     else:
    #         return False

    def make_region(self, x1, y1, fov, visibility_dist): #x1, y1 is robot position, considering rotation switch x1, y1

        angle = fov/2
        range_x = visibility_dist/math.cos(2*angle*math.pi/360)

        #left vertex
        x2 = x1 - range_x
        y2 = y1 + visibility_dist

        #right vertex
        x3 = x1 + range_x
        y3 = y1 + visibility_dist

        return [x1, y1, x2, y2, x3, y3]



    def handleObservation(self, observation):
        objectSeen = set()
        for obj_id, interactablePoses in observation.items():
            try:
                self.object_filters[utils.cleanObjectID(obj_id)].addObservation(
                    interactablePoses
                )
                objectSeen.add(utils.cleanObjectID(obj_id))
            except KeyError:
                pass
        for filter in self.object_filters.values():
            if filter.label not in objectSeen:
                filter.addNegativePose(observation["robot_pose"])

            # +z is up, +x is right, 0degrees = +z, 90degrees = +x, 180degrees = -z, 270degrees = -x 
                visibility_dist = 1.5
                fov = 90
                angle = fov/2
                range1 = visibility_dist/math.cos(2*angle*math.pi/360)

                robot_cur_pos = observation["robot_pose"]
                # interactablePoses = observation[filter.label]
                # interactablePoses_in_view = []
                
                if robot_cur_pos["yaw"] == 0:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["z"] <= inter_pose[1] <= robot_cur_pos["z"] + visibility_dist) and (inter_pose[0] - range1 <= inter_pose[0] <= inter_pose[0] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["x"], robot_cur_pos["z"], inter_pose[0], inter_pose[1], fov, visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(robot_cur_pos["x"], robot_cur_pos["z"], fov, visibility_dist)
                    filter.add_negative_region(triangle_region)

                elif robot_cur_pos["yaw"] == 180:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["z"] - visibility_dist <= inter_pose[1] <= robot_cur_pos["z"]) and (inter_pose[0] - range1 <= inter_pose[0] <= inter_pose[0] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["x"], robot_cur_pos["z"], inter_pose[0], inter_pose[1], fov, -1*visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)     
                    triangle_region = self.make_region(robot_cur_pos["x"], robot_cur_pos["z"], fov, -1*visibility_dist)
                    filter.add_negative_region(triangle_region)

                elif robot_cur_pos["yaw"] == 90:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["x"] <= inter_pose[0] <= robot_cur_pos["x"] + visibility_dist) and (inter_pose[1] - range1 <= inter_pose[1] <= inter_pose[1] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["z"], robot_cur_pos["x"], inter_pose[0], inter_pose[1], fov, visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(robot_cur_pos["z"], robot_cur_pos["x"], fov, visibility_dist)
                    triangle_region = [triangle_region[1], triangle_region[0], triangle_region[3], triangle_region[2], triangle_region[5], triangle_region[4]]
                    filter.add_negative_region(triangle_region)            

                elif robot_cur_pos["yaw"] == 270:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["x"] - visibility_dist <= inter_pose[0] <= robot_cur_pos["x"]) and (inter_pose[1] - range1 <= inter_pose[1] <= inter_pose[1] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["z"], robot_cur_pos["x"], inter_pose[0], inter_pose[1], fov, -1*visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(robot_cur_pos["z"], robot_cur_pos["x"], fov, -1*visibility_dist)
                    triangle_region = [triangle_region[1], triangle_region[0], triangle_region[3], triangle_region[2], triangle_region[5], triangle_region[4]]
                    filter.add_negative_region(triangle_region)                        
                                
                
                # for ip_in_view in interactablePoses_in_view():
                #     filter.addNegativePose(ip_in_view)
                            

                         
                    

    def saveDistributions(self, filterName=None):
        # print("Saving Distributions!!!!!!!!!!!!")
        if filterName is None:
            for filter in self.object_filters.values():
                filter.saveDistribution(self.trial_name)
            for filter in self.frame_filters.values():
                filter.saveDistribution(self.trial_name)
        else:
            try:
                self.object_filters[filterName].saveDistribution(self.trial_name)
            except KeyError:
                pass
            try:
                self.frame_filters[filterName].saveDistribution(self.trial_name)
            except KeyError:
                pass

    def pickup(self, object_id):
        event = self.controller.step(
            action="PickupObject",
            objectId=object_id,
            forceAction=False,
            manualInteract=False,
        )
        if event.metadata["lastActionSuccess"]:
            return True
        else:
            print(event)
            return False

    def slice(self, object_id):
        # print("Attempting to slice {}".format(utils.cleanObjectID(object_id)))
        return self.controller.step(
            action="SliceObject", objectId=object_id, forceAction=False
        ).metadata["lastActionSuccess"]

    def done(self):
        self.controller.step(action="Done")

    def observeSurroundings(self):
        self.saveDistributions()
        for i in range(4):
            # Quick observation of surroundings
            self.stepPath("turn_right")
            self.processRGB()
            self.saveTopDown()
        self.updateFilters()
        self.saveDistributions()

    def execute(self, frame_name):
        frameFilter = None
        for filter in self.frame_filters.values():
            # print(filter.label)
            if filter.label.lower() == frame_name.lower():
                # print("Grabbing highest particle from {}".format(filter.label))
                frameFilter = filter
                # currentBestEstimate, weight = filter.getHighestWeightedParticle()
                # top_k = filter.getTopKWeightedParticles()
                top_k = filter.getMaxWeightParticles()

        # currentBestEstimate = top_k[0]
        # print(
        #     "currentBestEstimate: ({}, {}, {}) Weight: ({})".format(
        #         currentBestEstimate[0],
        #         currentBestEstimate[1],
        #         currentBestEstimate[2],
        #         weight,
        #     )
        # )

        if frame_name == "Grasp_Tomato":
            obj_id, _ = self.processRGB(object_name="Tomato", return_poses=True)
            print("Obj_id = {}".format(obj_id))
            if self.pickup(obj_id):
                print("Successful!!!!")
        elif frame_name == "Grasp_Knife":
            obj_id, _ = self.processRGB(object_name="Knife", return_poses=True)
            print("Obj_id = {}".format(obj_id))
            if self.pickup(obj_id):
                print("Successful!!!!")
        elif frame_name == "Slice_Tomato":
            knifeGrasped = False
            topKIndex = 0
            while not knifeGrasped:
                currentBestEstimate = top_k[topKIndex][0]
                print("curBestEst: {}".format(currentBestEstimate))
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                # print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                for step in path:
                    self.stepPath(step)
                    object_detection_msg = self.processRGB()
                    self.handleObservation(object_detection_msg)
                obj_id, _ = self.processRGB(object_name="Knife", return_poses=True)
                if obj_id is not None:
                    print(
                        "[AGENT]: Image at ({}, {}, {}) saw knife with id: {}".format(
                            navGoal["x"], navGoal["z"], navGoal["yaw"], obj_id
                        )
                    )
                else:
                    print(
                        "[AGENT]: No knife seen at ({}, {}, {})".format(
                            navGoal["x"], navGoal["z"], navGoal["yaw"]
                        )
                    )
                    print("[AGENT]: Updating Filters and saving Distributions")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
                    continue
                if self.pickup(obj_id):
                    print("[AGENT]: Grasped the knife!")
                    self.state.action_history.append("Grasp_Knife")
                    # print("Updated the action history")
                    self.updateFilters()
                    self.saveDistributions()
                    knifeGrasped = True
                else:
                    print("[AGENT]: Grasp Knife Failed")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
            top_k = frameFilter.getMaxWeightParticles()
            topKIndex = 0
            tomatoSliced = False
            while not tomatoSliced and topKIndex <= 50:
                try:
                    currentBestEstimate = top_k[topKIndex][0]
                except IndexError:
                    print("[AGENT]: Unable to slice tomato with all given poses")
                    return False
                # print("Trying {} pose to slice".format(topKIndex))
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                # print("navGoal is {}".format(navGoal))
                # print("curPose is {}".format(self.cur_pose))
                path = self.nav.planPath(self.cur_pose, navGoal)
                for step in path:
                    self.stepPath(step)
                    object_detection_msg = self.processRGB()
                    self.handleObservation(object_detection_msg)
                # self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Tomato", return_poses=True)
                if obj_id is None:
                    print("[AGENT]: Updating Filters and saving Distributions")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
                else:
                    if self.slice(obj_id):
                        print("[AGENT]: Successfully sliced tomato with a knife")
                        return True
                    else:
                        print("[AGENT]: Slice Failed")
                        self.updateFilters()
                        self.saveDistributions()
                        top_k = frameFilter.getMaxWeightParticles()
        elif frame_name == "Slice_Apple":
            knifeGrasped = False
            topKIndex = 0
            while not knifeGrasped:
                try:
                    currentBestEstimate = top_k[topKIndex][0]
                except IndexError:
                    # out of bounds --> all max weight particles failed
                    return False
                # print(currentBestEstimate)
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                # print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                for step in path:
                    self.stepPath(step)
                    object_detection_msg = self.processRGB()
                    self.handleObservation(object_detection_msg)
                # self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Knife", return_poses=True)
                if obj_id is not None:
                    print(
                        "[AGENT]: Image at ({}, {}, {}) saw knife with id: {}".format(
                            navGoal["x"], navGoal["z"], navGoal["yaw"], obj_id
                        )
                    )
                else:
                    print(
                        "[AGENT]: No knife seen at ({}, {}, {})".format(
                            navGoal["x"], navGoal["z"], navGoal["yaw"]
                        )
                    )
                    print("[AGENT]: Updating Filters and saving Distributions")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
                    continue
                # print("Obj_id = {}".format(obj_id))
                if self.pickup(obj_id):
                    print("[AGENT]: Grasped the knife!")
                    self.state.action_history.append("Grasp_Knife")
                    # print("Updated the action history")
                    self.updateFilters()
                    self.saveDistributions()
                    knifeGrasped = True
                else:
                    print("[AGENT]: Grasp Knife Failed")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
            top_k = frameFilter.getMaxWeightParticles()
            topKIndex = 0
            appleSliced = False
            while not appleSliced and topKIndex <= 50:
                try:
                    currentBestEstimate = top_k[topKIndex][0]
                except IndexError:
                    print("[AGENT]: Unable to slice apple with all given poses")
                    return False
                # print("Trying {} pose to slice".format(topKIndex))
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                # print("navGoal is {}".format(navGoal))
                # print("curPose is {}".format(self.cur_pose))
                path = self.nav.planPath(self.cur_pose, navGoal)
                for step in path:
                    self.stepPath(step)
                    object_detection_msg = self.processRGB()
                    self.handleObservation(object_detection_msg)
                # self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Apple", return_poses=True)
                if obj_id is None:
                    print("[AGENT]: Updating Filters and saving Distributions")
                    self.updateFilters()
                    self.saveDistributions()
                    top_k = frameFilter.getMaxWeightParticles()
                else:
                    if self.slice(obj_id):
                        print("[AGENT]: Successfully sliced apple with a knife")
                        return True
                    else:
                        print("[AGENT]: Slice Failed")
                        self.updateFilters()
                        self.saveDistributions()
                        top_k = frameFilter.getMaxWeightParticles()

    def searchFor(self, object_name):
        # explore randomly
        obj_found = False
        filter_idx = 0
        i = 0
        while not obj_found:
            print("Random Exploration #: {}".format(i))
            goal = self.nav.getRandomValidPose()
            path = self.nav.planPath(self.cur_pose, goal)
            for step in path:
                # print("Stepping Path")
                self.stepPath(step)
                # print("Processing RGB")
                obj_id, reachable_poses = self.processRGB(
                    object_name=object_name, return_poses=True
                )
                # print("Updating Filter")
                self.updateFilters()
                # print("Saving Distribution")
                self.saveDistributions()
                filter_idx += 1
                if reachable_poses is not None:
                    obj_found = True
                    print("Total of {} poses found".format(len(reachable_poses)))
                    for i, pose in enumerate(reachable_poses):
                        print("Trying to reach from {}th pose".format(i))
                        # print(reachable_poses[0])
                        goal = {
                            "x": reachable_poses[i]["x"],
                            "z": reachable_poses[i]["z"],
                            "yaw": reachable_poses[i]["rotation"],
                        }
                        path = self.nav.planPath(self.cur_pose, goal)
                        for step in path:
                            self.stepPath(step)
                            self.processRGB()
                            self.updateFilters()
                            self.saveDistributions()
                        if self.slice(obj_id):
                            print("Successfully sliced the object")
                            self.done()
                            return True
                        # self.followPath(path)
                        # if self.pickup(obj_id):
                        #     print("Picked up object!!")
                        #     return True
                        # print("Failed to pickup object")
                        # print(obj_id
                        # for obj in self.controller.last_event.metadata['objects']:
                        #     if obj['objectId'] == obj_id:
                        #         if obj['isPickedUp']:
                        #             return True
                        # if self.controller.last_event.metadata["objects"][obj_id]['isPickedUp']:
                        #     return True

            i += 1
        return False

    def processRGB(self, object_name=None, return_poses=False):
        """
        Process the latest RGB img

        Args:
            object_name (str): String of clean object id. If provided, this returns the object_id and interactable_poses
            return_poses (bool): If True, return interactable poses
        """
        # rgb = self.controller.last_event.third_party_camera_frames[0]
        # cv2.imwrite("/home/cuhsailus/Desktop/Research/22_academic_year/thor_test/{}.png".format(self.cam_idx), rgb)
        # self.cam_idx += 1
        self.idx += 1
        obj_dets = self.controller.last_event.instance_detections2D
        try:
            cur_robot_pose = {
                "x": self.controller.last_event.metadata["agent"]["position"]["x"],
                "z": self.controller.last_event.metadata["agent"]["position"]["z"],
                "yaw": self.controller.last_event.metadata["agent"]["rotation"]["y"],
            }
            object_detection_msg = {
                "robot_pose": cur_robot_pose
            }  # dict(object_id: [interactable_poses]) dict mapping object id to all interactable poses
            # print("Looking for {}".format(self.frameElements))
            for obj_id, loc in obj_dets.items():
                if utils.cleanObjectID(obj_id) in self.frameElements:
                    print("Observed {}".format(utils.cleanObjectID(obj_id)))
                    interactable_poses = self.controller.step(
                        action="GetInteractablePoses",
                        objectId=obj_id,
                        rotations=[0, 90, 180, 270],
                        horizons=[0],
                        standings=[True],
                    ).metadata["actionReturn"]
                    # if interactable_poses is not None:
                    #     object_detection_msg[obj_id] = interactable_poses
        except:
            pass
        self.handleObservation(object_detection_msg)
        # print(object_detection_msg)
        if return_poses:
            # print("Returning poses!")
            # return poses only for object of interest
            for obj_id, poses in object_detection_msg.items():
                if utils.cleanObjectID(obj_id) == object_name:
                    return obj_id, poses
        elif object_name is None:
            # return entire detection msg
            return object_detection_msg
        return None, None

    def goTo(self, goal):
        """
        Plan  to the goal

        Input:
            goal (dict): {'x':float, 'z':float, 'yaw':float}

        Returns:
            success (bool): True if navigation succeeds, false otherwise
        """
        if goal == "random":
            goal = self.nav.getRandomValidPose()
            path = self.nav.planPath(self.cur_pose, goal)
        else:
            path = self.nav.planPath(self.cur_pose, goal)
        return self.followPath(path)

    def stepPath(self, step):
        # print(step)
        if step == "forward":
            # print("Stepping Forward")
            self.controller.step(action="MoveAhead", moveMagnitude=0.25)
        elif step == "backward":
            # print("Stepping Backward")
            self.controller.step(action="MoveBack", moveMagnitude=0.25)
        elif step == "turn_right":
            # print("Turning Right")
            self.controller.step(action="RotateRight", degrees=90)
        elif step == "turn_left":
            # print("Turning Left")
            self.controller.step(action="RotateLeft", degrees=90)
        self.cur_pose = {
            "x": self.controller.last_event.metadata["agent"]["position"]["x"],
            "z": self.controller.last_event.metadata["agent"]["position"]["z"],
            "yaw": round(self.controller.last_event.metadata["agent"]["rotation"]["y"]),
        }
        self.saveTopDown()

    def saveTopDown(self):
        topdown_img = self.controller.last_event.third_party_camera_frames[0][
            :, :, ::-1
        ]
        self.topdown_frames.append(topdown_img)
        cv2.imwrite(
            "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/top_down/trial_{}/{}.png".format(
                self.trial_name, self.cam_idx
            ),
            topdown_img,
        )
        self.cam_idx += 1

    def followPath(self, path):
        for i, step in enumerate(path):
            # print("Step: {}".format(i))
            self.stepPath(step)
        return True
