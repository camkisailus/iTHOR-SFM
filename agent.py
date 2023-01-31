from navigation import Navigation
import numpy as np
import cv2
from moviepy.editor import ImageSequenceClip
from particle_filters import ObjectParticleFilter, FrameParticleFilter
import utils


class State:
    def __init__(self, action_history=[]):
        self.action_history = action_history


class Agent:
    def __init__(self, controller, pose, frames, objects):
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
        print("Loaded the following object filters")
        for name, filter in self.object_filters.items():
            print("Name: {}".format(name))
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
            # if frame.preconditions is not None:
            #     for precondition in frame.preconditions:
            #     filter.addPreconditionFilter(precondition, )
            self.frame_filters[frame.name] = filter
        for frame in frames:
            if frame.preconditions is not None:
                for precondition in frame.preconditions:
                    self.frame_filters[frame.name].addPreconditionFilter(
                        precondition, self.frame_filters[precondition]
                    )
        print("Loaded the following frame filters")
        for filter in self.frame_filters.values():
            print(filter)

        # self = SemanticFrameMapping(
        #     self.objectFilters, self.frameFilters, self.controller
        # )
        self.state = State()
        self.frameElements = [label for label in self.object_filters.keys()]
        self.observeSurroundings()

    def updateFilters(self):
        for objFilter in self.object_filters.values():
            objFilter.updateFilter()
        for frameFilter in self.frame_filters.values():
            frameFilter.updateFilter(self.state)

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

    def saveDistributions(self, filterName=None):
        print("Saving Distributions!!!!!!!!!!!!")
        if filterName is None:
            for filter in self.object_filters.values():
                filter.saveDistribution()
            for filter in self.frame_filters.values():
                filter.saveDistribution()
        else:
            try:
                self.object_filters[filterName].saveDistribution()
            except KeyError:
                pass
            try:
                self.frame_filters[filterName].saveDistribution()
            except KeyError:
                pass

    def makeVideo(self):
        for i in range(10):
            # add padding to end of video
            self.topdown_frames.append(self.topdown_frames[-1])
        clip = ImageSequenceClip(self.topdown_frames, fps=10)
        clip.write_videofile("trajectory.mp4")

    def pickup(self, object_id):
        return self.controller.step(
            action="PickupObject",
            objectId=object_id,
            forceAction=False,
            manualInteract=True,
        ).metadata["lastActionSuccess"]

    def slice(self, object_id):
        print("Attempting to slice {}".format(utils.cleanObjectID(object_id)))
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
                print("Grabbing highest particle from {}".format(filter.label))
                frameFilter = filter
                # currentBestEstimate, weight = filter.getHighestWeightedParticle()
                top_k = filter.getTopKWeightedParticles()

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
                currentBestEstimate = top_k[topKIndex]
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Knife", return_poses=True)
                print("Obj_id = {}".format(obj_id))
                if self.pickup(obj_id):
                    print("Grasped the knife!")
                    self.state.action_history.append("Grasp_Knife")
                    print("Updated the action history")
                    self.updateFilters()
                    self.saveDistributions()
                    knifeGrasped = True
            top_k = frameFilter.getTopKWeightedParticles()
            topKIndex = 0
            tomatoSliced = False
            while not tomatoSliced:
                try:
                    currentBestEstimate = top_k[topKIndex]
                except IndexError:
                    print(
                        "[EXECUTION FAILED]: Unable to slice tomato with all given poses"
                    )
                    return False
                print("Trying {} pose to slice".format(topKIndex))
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Tomato", return_poses=True)
                if obj_id is None:
                    pass
                else:
                    if self.slice(obj_id):
                        print("Successfully sliced tomato with a knife")
                        return True
                    else:
                        print("Slice Failed")
        elif frame_name == "Slice_Apple":
            knifeGrasped = False
            topKIndex = 0
            while not knifeGrasped:
                currentBestEstimate = top_k[topKIndex]
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Knife", return_poses=True)
                print("Obj_id = {}".format(obj_id))
                if self.pickup(obj_id):
                    print("Grasped the knife!")
                    self.state.action_history.append("Grasp_Knife")
                    print("Updated the action history")
                    self.updateFilters()
                    self.saveDistributions()
                    knifeGrasped = True
            top_k = frameFilter.getTopKWeightedParticles()
            topKIndex = 0
            appleSliced = False
            while not appleSliced:
                try:
                    currentBestEstimate = top_k[topKIndex]
                except IndexError:
                    print(
                        "[EXECUTION FAILED]: Unable to slice apple with all given poses"
                    )
                    return False
                print("Trying {} pose to slice".format(topKIndex))
                topKIndex += 1
                navGoal = {
                    "x": currentBestEstimate[0],
                    "z": currentBestEstimate[1],
                    "yaw": currentBestEstimate[2],
                }
                print("navGoal is {}".format(navGoal))
                path = self.nav.planPath(self.cur_pose, navGoal)
                self.followPath(path)
                obj_id, _ = self.processRGB(object_name="Apple", return_poses=True)
                if obj_id is None:
                    pass
                else:
                    if self.slice(obj_id):
                        print("Successfully sliced apple with a knife")
                        return True
                    else:
                        print("Slice Failed")

            # currentBestEstimate, weight = filter.getHighestWeightedParticle()
            # print(
            #     "currentBestEstimate: ({}, {}, {}) Weight: ({})".format(
            #         currentBestEstimate[0],
            #         currentBestEstimate[1],
            #         currentBestEstimate[2],
            #         weight,
            #     )
            # )
            # navGoal = {
            #     "x": currentBestEstimate[0],
            #     "z": currentBestEstimate[1],
            #     "yaw": currentBestEstimate[2],
            # }
            # self.followPath(self.nav.planPath(self.cur_pose, navGoal))

            # print("Tomato obj_id = {}".format(obj_id))
            # if self.slice(obj_id):
            #     print("Successfully sliced the tomato w a knife!")
            # else:
            #     print("Slice failed")

        # for step in path:
        #     self.stepPath(step)
        #     self.processRGB()
        #     self.updateFilters(self.state)
        #     self.saveDistributions(filter_name="Grasp_Tomato")

        # check for new highest weight
        # newBestEstimate, newWeight = frameFilter.getHighestWeightedParticle()
        # if (
        #     currentBestEstimate[0] != newBestEstimate[0]
        #     and currentBestEstimate[1] != newBestEstimate[1]
        #     and currentBestEstimate[2] != newBestEstimate[2]
        # ):
        #     print("New Best Estimate!!!!")
        #     currentBestEstimate = newBestEstimate
        #     weight = newWeight
        #     print(
        #         "currentBestEstimate: ({}, {}, {}) Weight: {}".format(
        #             currentBestEstimate[0],
        #             currentBestEstimate[1],
        #             currentBestEstimate[2],
        #             weight,
        #         )
        #     )
        #     break

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
                    # print("Observed {}".format(utils.cleanObjectID(obj_id)))
                    interactable_poses = self.controller.step(
                        action="GetInteractablePoses",
                        objectId=obj_id,
                        rotations=[0, 90, 180, 270],
                        horizons=[-30, 0],
                        standings=[True],
                    ).metadata["actionReturn"]
                    if interactable_poses is not None:
                        object_detection_msg[obj_id] = interactable_poses
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
            return self.followPath(path)
        else:
            path = self.nav.navTo(self.cur_pose, goal)
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
            "yaw": self.controller.last_event.metadata["agent"]["rotation"]["y"],
        }
        self.saveTopDown()

    def saveTopDown(self):
        topdown_img = self.controller.last_event.third_party_camera_frames[0][
            :, :, ::-1
        ]
        self.topdown_frames.append(topdown_img)
        cv2.imwrite(
            "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/top_down/{}.png".format(
                self.cam_idx
            ),
            topdown_img,
        )
        self.cam_idx += 1

    def followPath(self, path):
        for i, step in enumerate(path):
            # print("Step: {}".format(i))
            self.stepPath(step)
        return True
