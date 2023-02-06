from navigation import Navigation
import numpy as np
import cv2
from particle_filters import ObjectParticleFilter, FrameParticleFilter
import utils
import math
import os

ROOT = os.path.dirname(os.path.realpath(__file__))


class State:
    def __init__(self, pose, action_history=[]):
        self.action_history = action_history
        self.robot_cur_pose = pose
        self.objectInGripper = ""
        self.objectsSliced = []


class Agent:
    def __init__(self, controller, pose, frames, objects, trial_name, mode):
        self.saveCount = 0
        self.trial_name = trial_name
        self.mode = mode
        if mode == "sfm":
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
        print("Loaded the following frame filters")
        for filter in self.frame_filters.values():
            print(filter)
        # exit()

        self.state = State(self.cur_pose)
        self.frameElements = [label for label in self.object_filters.keys()]
        if mode == "oracle":
            self.oracle = {}
            self.mode = "oracle"
            # Has a map of all obj_ids to interactable poses
            for obj in controller.last_event.metadata["objects"]:
                for frameElement in self.frameElements:
                    if frameElement in obj["objectType"]:
                        print("Getting poses for: {}".format(obj["objectType"]))
                        interactable_poses = self.controller.step(
                            action="GetInteractablePoses",
                            objectId=obj["objectId"],
                            rotations=[0, 90, 180, 270],
                            horizons=[0],
                            standings=[True],
                        ).metadata["actionReturn"]
                        print(len(interactable_poses))
                        try:
                            self.oracle[frameElement].append(interactable_poses)
                        except KeyError:
                            self.oracle[frameElement] = interactable_poses
            for frameElement, poses in self.oracle.items():
                print("FrameElement: {} nPoses: {}".format(frameElement, len(poses)))
        self.retries = 5
        if self.mode == "sfm":
            self.observeSurroundings()

    def updateFilters(self):
        for objFilter in self.object_filters.values():
            objFilter.updateFilter(self.state)
        for frameFilter in self.frame_filters.values():
            frameFilter.updateFilter(self.state)

    def make_region(
        self, x1, y1, fov, visibility_dist
    ):  # x1, y1 is robot position, considering rotation switch x1, y1

        angle = fov / 2
        range_x = visibility_dist / math.cos(2 * angle * math.pi / 360)

        # left vertex
        x2 = x1 - range_x
        y2 = y1 + visibility_dist

        # right vertex
        x3 = x1 + range_x
        y3 = y1 + visibility_dist

        return [x1, y1, x2, y2, x3, y3]

    def handleObservation(self, observation):
        objectSeen = set()
        for obj_id, interactablePoses in observation.items():
            if obj_id == "robot_pose":
                continue
            try:
                self.object_filters[utils.cleanObjectID(obj_id)].addObservation(
                    interactablePoses
                )
                # if "Slice" in obj_id:
                #     print("[AGENT]: Added observation of {} to {}".format(obj_id, self.object_filters[utils.cleanObjectID(obj_id)].label))
                objectSeen.add(utils.cleanObjectID(obj_id))
            except KeyError:
                print("[AGENT]: KeyError when adding {} to filters".format(utils.cleanObjectID(obj_id)))
                exit()
                pass
        for filter in self.object_filters.values():
            if filter.label not in objectSeen:
                filter.addNegativePose(observation["robot_pose"])

                # +z is up, +x is right, 0degrees = +z, 90degrees = +x, 180degrees = -z, 270degrees = -x
                visibility_dist = 1.5
                fov = 90
                angle = fov / 2
                range1 = visibility_dist / math.cos(2 * angle * math.pi / 360)

                robot_cur_pos = observation["robot_pose"]
                # interactablePoses = observation[filter.label]
                # interactablePoses_in_view = []

                if robot_cur_pos["yaw"] == 0:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["z"] <= inter_pose[1] <= robot_cur_pos["z"] + visibility_dist) and (inter_pose[0] - range1 <= inter_pose[0] <= inter_pose[0] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["x"], robot_cur_pos["z"], inter_pose[0], inter_pose[1], fov, visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(
                        robot_cur_pos["x"], robot_cur_pos["z"], fov, visibility_dist
                    )
                    filter.add_negative_region(triangle_region)

                elif robot_cur_pos["yaw"] == 180:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["z"] - visibility_dist <= inter_pose[1] <= robot_cur_pos["z"]) and (inter_pose[0] - range1 <= inter_pose[0] <= inter_pose[0] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["x"], robot_cur_pos["z"], inter_pose[0], inter_pose[1], fov, -1*visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(
                        robot_cur_pos["x"],
                        robot_cur_pos["z"],
                        fov,
                        -1 * visibility_dist,
                    )
                    filter.add_negative_region(triangle_region)

                elif robot_cur_pos["yaw"] == 90:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["x"] <= inter_pose[0] <= robot_cur_pos["x"] + visibility_dist) and (inter_pose[1] - range1 <= inter_pose[1] <= inter_pose[1] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["z"], robot_cur_pos["x"], inter_pose[0], inter_pose[1], fov, visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(
                        robot_cur_pos["z"], robot_cur_pos["x"], fov, visibility_dist
                    )
                    triangle_region = [
                        triangle_region[1],
                        triangle_region[0],
                        triangle_region[3],
                        triangle_region[2],
                        triangle_region[5],
                        triangle_region[4],
                    ]
                    filter.add_negative_region(triangle_region)

                elif robot_cur_pos["yaw"] == 270:
                    # for inter_pose in  interactablePoses:
                    #     if (robot_cur_pos["x"] - visibility_dist <= inter_pose[0] <= robot_cur_pos["x"]) and (inter_pose[1] - range1 <= inter_pose[1] <= inter_pose[1] + range1):
                    #         if self.check_point_in_fov(robot_cur_pos["z"], robot_cur_pos["x"], inter_pose[0], inter_pose[1], fov, -1*visibility_dist):
                    #             interactablePoses_in_view.append(inter_pose)
                    triangle_region = self.make_region(
                        robot_cur_pos["z"],
                        robot_cur_pos["x"],
                        fov,
                        -1 * visibility_dist,
                    )
                    triangle_region = [
                        triangle_region[1],
                        triangle_region[0],
                        triangle_region[3],
                        triangle_region[2],
                        triangle_region[5],
                        triangle_region[4],
                    ]
                    filter.add_negative_region(triangle_region)

                # for ip_in_view in interactablePoses_in_view():
                #     filter.addNegativePose(ip_in_view)

    def saveDistributions(self, filterName=None):
        print("Saving Distributions count = {}".format(self.saveCount))
        self.saveCount += 1
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
            self.state.objectInGripper = utils.cleanObjectID(object_id)
            print("[AGENT]: Grasped obj is now: {}".format(self.state.objectInGripper))
            return True
        else:
            # print(object_id)
            # print(event)
            return False

    def slice(self, object_id):
        # print("Attempting to slice {}".format(utils.cleanObjectID(object_id)))
        return self.controller.step(
            action="SliceObject", objectId=object_id, forceAction=False
        ).metadata["lastActionSuccess"]

    def drop(self):
        return self.controller.step(
            action="DropHandObject", forceAction=False
        ).metadata["lastActionSuccess"]

    def put(self, target=None):
        if target == None:
            # find a countertop or table to place object
            counterTarget = self.objectIdFromRGB("CounterTop")
            if counterTarget is None:
                tableTarget = self.objectIdFromRGB("Table")
                if tableTarget is None:
                    print("[AGENT]: Could not find counter or table to put object onto.")
                    return False
                else:
                    target = tableTarget
            else:
                target = counterTarget

        return self.controller.step(
            action="PutObject", objectId=target, forceAction=False, placeStationary=False
        ).metadata["lastActionSuccess"]

    def done(self):
        self.controller.step(action="Done")

    def observeSurroundings(self):
        print("[AGENT]: Observing the surroundings")
        self.saveDistributions()
        for i in range(4):
            # Quick observation of surroundings
            self.stepPath("turn_right")
            self.processRGB()
            # self.saveTopDown()
        self.updateFilters()
        self.saveDistributions()

    def oracle_execute(self, frame_name):
        print("[AGENT]: Running Oracle Execution")
        if frame_name == "Slice_Tomato":
            try:
                poses = self.oracle["Knife"]
            except KeyError:
                return False
            i = 0
            knifeGrasped = False
            while i < self.retries:
                try:
                    pose = poses[i]
                except IndexError:
                    # out of possible poses
                    return False
                goal = {"x": pose["x"], "z": pose["z"], "yaw": pose["rotation"]}
                suc = self.goTo(goal)
                if not suc:
                    poses.pop(i)
                    continue
                else:
                    i += 1
                obj_id = self.processRGB(object_name="Knife")
                print("[AGENT]: Obj id: {}".format(obj_id))
                if self.pickup(obj_id):
                    print("[AGENT]: Grasped the knife!")
                    knifeGrasped = True
                    break
                else:
                    print(["[AGENT]: Failed to Grasp Knife"])

            if not knifeGrasped:
                print(
                    "[AGENT]: Failed to grasp knife after {} retries".format(
                        self.retries
                    )
                )
                return False
            tomatoSliced = False
            try:
                poses = self.oracle["Tomato"]
            except:
                return False
            i = 0
            while i < self.retries:
                try:
                    pose = poses[i]
                except IndexError:
                    return False
                goal = {"x": pose["x"], "z": pose["z"], "yaw": pose["rotation"]}
                suc = self.goTo(goal)
                if not suc:
                    poses.pop(i)
                    continue
                else:
                    i += 1

                obj_id = self.processRGB(object_name="Tomato")
                print("[AGENT]: Obj id: {}".format(obj_id))
                if self.slice(obj_id):
                    print("[AGENT]: Sliced tomato!")
                    tomatoSliced = True
                    break
                else:
                    print("[AGENT]: Failed to slice tomato")
            if not tomatoSliced:
                print(
                    "[AGENT]: Failed to slice tomato after {} retries".format(
                        self.retries
                    )
                )
                return False
            else:
                return True

    def graspObject(self, object: str, filter: FrameParticleFilter) -> bool:
        print("[AGENT]: Entering graspObj({}, {})".format(object, filter.label))
        if self.state.objectInGripper != "":
            if object == self.state.objectInGripper:
                print("[AGENT]: Already Grasping a {}".format(object))
            else:
                # drop object and continue
                print(
                    "[AGENT]: Am currently holding {} will drop so that I can attempt to grasp {}".format(
                        self.state.objectInGripper, object
                    )
                )
                if self.put():
                    self.state.action_history.remove("Grasp_{}".format(self.state.objectInGripper))
                    self.state.objectInGripper = ""
                else:
                    return False

        objGrasped = False
        topKParticles = filter.getMaxWeightParticles()
        attempts = 0
        while not objGrasped and attempts < self.retries:
            currentBestEstimate = topKParticles[0][0]
            navGoal = {
                "x": currentBestEstimate[0],
                "z": currentBestEstimate[1],
                "yaw": currentBestEstimate[2],
            }
            print(
                "[AGENT]: Current best estimate: ({}, {})".format(
                    navGoal["x"], navGoal["z"]
                )
            )
            path = self.nav.planPath(self.cur_pose, navGoal)
            for step in path:
                self.stepPath(step)
                object_detection_msg = self.processRGB()
                self.handleObservation(object_detection_msg)
            # obj_id = self.processRGB(object_name=object)
            obj_id = self.objectIdFromRGB(object_name=object)
            if obj_id is not None:
                print(
                    "[AGENT]: Image at ({}, {}, {}) saw {} with id: {}".format(
                        navGoal["x"], navGoal["z"], navGoal["yaw"], object, obj_id
                    )
                )
            else:
                print(
                    "[AGENT]: No {} seen at ({}, {}, {})".format(
                        object, navGoal["x"], navGoal["z"], navGoal["yaw"]
                    )
                )
                print("[AGENT]: Updating Filters and saving Distributions")
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
                continue
            if self.pickup(obj_id):
                print("[AGENT]: Grasped the {}!".format(object))
                self.state.action_history.append("Grasp_{}".format(object))
                objGrasped = True
                return True
            else:
                print("[AGENT]: Grasp {} Failed".format(object))
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
        if not objGrasped:
            print(
                "[AGENT]: Grasp {} failed after {} retries".format(object, self.retries)
            )
            return False

    def sliceObj(self, object: str, filter: FrameParticleFilter) -> bool:
        print("[AGENT]: Entering sliceObj({}, {})".format(object, filter.label))
        objSliced = False
        topKParticles = filter.getMaxWeightParticles()
        attempts = 0
        while not objSliced and attempts < self.retries:
            currentBestEstimate = topKParticles[0][0]
            navGoal = {
                "x": currentBestEstimate[0],
                "z": currentBestEstimate[1],
                "yaw": currentBestEstimate[2],
            }
            print("[AGENT]: Current best estimate: ({}, {})".format(navGoal['x'], navGoal['z']))
            path = self.nav.planPath(self.cur_pose, navGoal)
            # print(pat//h)
            for step in path:
                # print("[AGENT]: Stepping path and handling RGB")
                self.stepPath(step)
                object_detection_msg = self.processRGB()
                self.handleObservation(object_detection_msg)
            obj_id = self.objectIdFromRGB(object_name=object)
            # obj_id = self.processRGB(object_name=object)
            if obj_id is not None:
                print(
                    "[AGENT]: Image at ({}, {}, {}) saw {} with id: {}".format(
                        navGoal["x"], navGoal["z"], navGoal["yaw"], object, obj_id
                    )
                )
            else:
                print(
                    "[AGENT]: No {} seen at ({}, {}, {})".format(
                        object, navGoal["x"], navGoal["z"], navGoal["yaw"]
                    )
                )
                print("[AGENT]: Updating Filters and saving Distributions")
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
                continue
            if self.slice(obj_id):
                print("[AGENT]: Sliced {}!".format(object))
                self.state.action_history.append("Slice_{}".format(object))
                # print("Updated the action history")
                self.updateFilters()
                self.saveDistributions()
                objSliced = True
                return True
            else:
                print("[AGENT]: Slice {} Failed".format(object))
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
        if not objSliced:
            print(
                "[AGENT]:Slice {} failed after {} retries".format(object, self.retries)
            )
            return False

    def putObject(self, object: str, target: str, filter: FrameParticleFilter) -> bool:
        print(
            "[AGENT]: Entering putObject({}, {}, {})".format(
                object, target, filter.label
            )
        )
        objectPut = False
        topKParticles = filter.getMaxWeightParticles()
        attempts = 0
        while not objectPut and attempts < self.retries:
            currentBestEstimate = topKParticles[0][0]
            navGoal = {
                "x": currentBestEstimate[0],
                "z": currentBestEstimate[1],
                "yaw": currentBestEstimate[2],
            }
            path = self.nav.planPath(self.cur_pose, navGoal)
            for step in path:
                self.stepPath(step)
                object_detection_msg = self.processRGB()
                self.handleObservation(object_detection_msg)
            # obj_id = self.processRGB(object_name=target)
            obj_id = self.objectIdFromRGB(object_name=target)
            if obj_id is not None:
                print(
                    "[AGENT]: Image at ({}, {}, {}) saw {} with id: {}".format(
                        navGoal["x"], navGoal["z"], navGoal["yaw"], object, obj_id
                    )
                )
            else:
                print(
                    "[AGENT]: No {} seen at ({}, {}, {})".format(
                        object, navGoal["x"], navGoal["z"], navGoal["yaw"]
                    )
                )
                print("[AGENT]: Updating Filters and saving Distributions")
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
                continue
            if self.put(obj_id):
                print("[AGENT]: Put {} on {}!".format(object, target))
                self.state.action_history.append("Put_{}_on_{}".format(object, target))
                self.state.objectInGripper = ""
                self.state.action_history.remove("Grasp_{}".format(object))
                # print("Updated the action history")
                self.updateFilters()
                self.saveDistributions()
                objectPut = True
                return True
            else:
                print("[AGENT]: Put {} on {} Failed".format(object, target))
                self.updateFilters()
                self.saveDistributions()
                topKParticles = filter.getMaxWeightParticles()
                attempts += 1
        if not objectPut:
            print(
                "[AGENT]: Put {} on {} failed after {} retries".format(
                    object, target, self.retries
                )
            )
            return False

    def execute(self, frame_name: str):
        if self.mode == "oracle":
            return self.oracle_execute(frame_name)
        frameFilter = self.frame_filters[frame_name]
        try:
            uncompletedPreconditions = [
                pre
                for pre in frameFilter.preconditions
                if pre not in self.state.action_history
            ]
            print(
                "[AGENT]: Uncompleted preconditions for {} are: {}".format(
                    frame_name, uncompletedPreconditions
                )
            )
            for precondition in uncompletedPreconditions:
                if self.execute(precondition):
                    print(
                        "[AGENT]: Action history: {}".format(self.state.action_history)
                    )
                    print("[AGENT]: Updating filters and saving distributions after successful execution")
                    self.updateFilters()
                    self.saveDistributions()
                else:
                    print("[AGENT]: Precondition {} failed".format(precondition))
                    return False
        except TypeError:
            # if preconditions is None
            print("[AGENT]: No preconditions for {}".format(frame_name))

        if frame_name.split("_")[0] == "Grasp":
            return self.graspObject(frame_name.split("_", 1)[1], frameFilter)
        elif frame_name.split("_")[0] == "Slice":
            if self.sliceObj(frame_name.split("_")[1], frameFilter):
                # get positions of object slices
                self.processRGB()
                self.updateFilters()
                return True
        elif frame_name.split("_")[0] == "Put":
            obj = frame_name.split("_")[1]  # obj to put
            receptacle = frame_name.split("_")[-1]  # receptacle
            return self.putObject(obj, receptacle, frameFilter)

        return False

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

    def objectIdFromRGB(self, object_name:str):
        obj_dets = self.controller.last_event.instance_detections2D
        for obj_id, loc in obj_dets.items():
            if object_name in utils.cleanObjectID(obj_id):
                print("[AGENT]: Observed at {} with id {}".format(object_name, obj_id))
                return obj_id
        # did not find object
        return None
    
    def processRGB(self, object_name=None, verbose=False):
        """
        Process the latest RGB img

        Args:
            object_name (str): String of clean object id. If provided, this returns the object_id otherwise (object_name == None) the entire detection msg is returned

        """
        # rgb = self.controller.last_event.third_party_camera_frames[0]
        # cv2.imwrite("/home/cuhsailus/Desktop/Research/22_academic_year/thor_test/{}.png".format(self.cam_idx), rgb)
        # self.cam_idx += 1
        self.idx += 1
        obj_dets = self.controller.last_event.instance_detections2D
        if verbose:
            print(obj_dets)
        elementToIDMap = {}
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
                for frameElement in self.frameElements:
                    if frameElement in utils.cleanObjectID(
                        obj_id
                    ):  # in self.frameElements:
                        if (
                            "Slice" in utils.cleanObjectID(obj_id)
                            and "Slice" not in frameElement
                        ):
                            continue
                        interactable_poses = self.controller.step(
                            action="GetInteractablePoses",
                            objectId=obj_id,
                            rotations=[0, 90, 180, 270],
                            horizons=[0],
                            standings=[True],
                        ).metadata["actionReturn"]
                        # if frameElement == "BreadSlice":
                        #     print("[AGENT]: Saw a bread slice!")
                        if interactable_poses is not None:
                            elementToIDMap[frameElement] = obj_id
                            object_detection_msg[frameElement] = interactable_poses
                            # try:
                            #     object_detection_msg[frameElement].append(interactable_poses)
                            # except KeyError:
                            #     object_detection_msg[frameElement] = interactable_poses
                            # print("[AGENT]: Adding poses for {} to frameElement {} ".format(obj_id, frameElement))
        except:
            pass
        # print(object_detection_m/sg)
        if self.mode == "sfm":
            self.handleObservation(object_detection_msg)
        # print(object_detection_msg)
        if object_name is not None:
            for frameElement, _ in object_detection_msg.items():
                if frameElement == object_name:
                    return elementToIDMap[frameElement]
            # Did not observe object of interest
            return None
        else:
            # return entire detection msg
            return object_detection_msg

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
        self.followPath(path)
        # check we reached goal
        # assert((self.cur_pose['x'] == goal['x'] and self.cur_pose['z'] == goal['z'] and self.cur_pose['yaw'] == goal['yaw']))
        if not (
            self.cur_pose["x"] == goal["x"]
            and self.cur_pose["z"] == goal["z"]
            and self.cur_pose["yaw"] == goal["yaw"]
        ):
            return False
        return True

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
        self.state.robot_cur_pose = self.cur_pose
        assert self.controller.last_event.metadata["agent"]["cameraHorizon"] == 0
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
