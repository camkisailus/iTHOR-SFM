from navigation import Navigation
import numpy as np
import cv2
from moviepy.editor import ImageSequenceClip
from particle_filters import ObjectParticleFilter, FrameParticleFilter
from SFM import SemanticFrameMapping
import utils

class State:
    def __init__(self, action_history=[]):
        self.action_history = action_history
    
class Agent:
    def __init__(self, controller, pose):
        self.controller = controller
        self.nav = Navigation(controller)
        self.cur_pose = pose
        self.idx = 0
        self.cam_idx = 0
        self.topdown_frames = []
        self.pillow_pf = ObjectParticleFilter("Pillow", self.controller)
        self.grasp_pillow_pf = FrameParticleFilter(
            "Grasp_Pillow",
            preconditions=None,
            core_frame_elements=["Pillow"],
            controller=controller,
        )
        self.grasp_pillow_pf.addFrameElementFilter("Pillow", self.pillow_pf)
        self.objectFilters = [self.pillow_pf]
        self.frameFilters = [self.grasp_pillow_pf]
        self.sfm = SemanticFrameMapping(
            self.objectFilters, self.frameFilters, self.controller
        )
        self.state = State()

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
                self.sfm.updateFilters(self.state)
                # print("Saving Distribution")
                self.sfm.saveDistributions()
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
                            self.sfm.updateFilters(self.state)
                            self.sfm.saveDistributions()
                        # self.followPath(path)
                        if self.pickup(obj_id):
                            print("Picked up object!!")
                            return True
                        print("Failed to pickup object")
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
            for obj_id, loc in obj_dets.items():
                # if object_name is not None and obj_id.split("|")[0] == object_name:
                # tlx = loc[0]
                # tly = loc[1]
                # brx = loc[2]
                # bry = loc[3]
                # bbox_center = (int((tlx + brx) / 2), int((tly + bry) / 2))
                # query = self.controller.step(
                #     action="GetCoordinateFromRaycast",
                #     x=bbox_center[0] / 300,
                #     y=bbox_center[1] / 300,
                # )
                # world_coords = query.metadata["actionReturn"]
                # print(
                #     "{} located at ({})".format(obj_id.split("|")[0], world_coords)
                # )
                interactable_poses = self.controller.step(
                    action="GetInteractablePoses",
                    objectId=obj_id,
                    horizons=[0],
                    standings=[True],
                ).metadata["actionReturn"]
                if interactable_poses is not None:
                    object_detection_msg[obj_id] = interactable_poses
                # if utils.cleanObjectID(obj_id) == object_name:
                #     print("Interactable_poses: {}".format(interactable_poses))
                # return obj_id, interactable_poses  # reachable poses of object
        except:
            pass
        self.sfm.handleObservation(object_detection_msg)
        # print(object_detection_msg)
        if return_poses:
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
            # print("Num steps: {}".format(len(path)))
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
            # print("Stepping Right")
            self.controller.step(action="RotateRight", degrees=90)
        elif step == "turn_left":
            # print("Stepping Left")
            self.controller.step(action="RotateLeft", degrees=90)
        self.cur_pose = {
            "x": self.controller.last_event.metadata["agent"]["position"]["x"],
            "z": self.controller.last_event.metadata["agent"]["position"]["z"],
            "yaw": self.controller.last_event.metadata["agent"]["rotation"]["y"],
        }
        # print("Cur robot pose: ({}, {}, {})".format(self.cur_pose['x'], self.cur_pose['z'], self.cur_pose['yaw']))
        # rgb =
        # print("Saving Img")
        # topdown_img = self.controller.last_event.third_party_camera_frames[0][
        #     :, :, ::-1
        # ]
        # self.topdown_frames.append(topdown_img)
        # cv2.imwrite(
        #     "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/top_down/{}.png".format(
        #         self.cam_idx
        #     ),
        #     topdown_img,
        # )
        # self.cam_idx += 1
        # self.processRGB()

    def followPath(self, path):
        for i, step in enumerate(path):
            # print("Step: {}".format(i))
            self.stepPath(step)
        return True
