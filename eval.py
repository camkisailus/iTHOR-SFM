from ai2thor.controller import Controller
import os
import json
import glob
from agent import Agent
import utils
from multiprocessing import Process
import random

class ThorEnv:
    def __init__(self, scene_description: dict, goal_description: str, trial_id: str):
        # self.floorPlan =
        self.scene_desc = scene_description
        self.goal_desc = goal_description
        self.frames, self.objects = utils.loadFramesFromALFRED(goal_description)
        # print("Frames:")
        # for frame in self.frames:
        #     print("\t{}".format(frame))
        # print("Objects: {}".format(self.objects))
        # event = self.controller.step(
        #     action="SetObjectPoses", objectPoses=scene_description["object_poses"]
        # )
        # print(event.metadata)
        self.trial_name = "{}_{}".format(goal_description, trial_id)
       
    
    def init_scene(self):
        print(
            "[DRIVER]: Initializing ThorEnv with FloorPlan {}".format(
                self.scene_desc["floor_plan"]
            )
        )
        self.controller = Controller(
            agentMode="default",
            visibilityDistance=1.0,
            scene=self.scene_desc["floor_plan"],
            # step sizes
            gridSize=0.25,
            snapToGrid=True,
            rotateStepDegrees=90,
            # image modalities
            renderDepthImage=False,
            renderInstanceSegmentation=True,
            # camera properties
            width=300,
            height=300,
            fieldOfView=90,
        )
        event = self.controller.step(action="GetMapViewCameraProperties")
        event = self.controller.step(
            action="AddThirdPartyCamera", agentId=0, **event.metadata["actionReturn"]
        )
        self.controller.step(
            action="TeleportFull",
            position=dict(
                x=self.scene_desc["init_action"]["x"],
                y=self.scene_desc["init_action"]["y"],
                z=self.scene_desc["init_action"]["z"],
            ),
            rotation=dict(x=0, y=self.scene_desc["init_action"]["rotation"], z=0),
            horizon=0,
            standing=True,
        )
        agent_metadata = self.controller.last_event.metadata["agent"]
        agent_pose = {
            "x": agent_metadata["position"]["x"],
            "z": agent_metadata["position"]["z"],
            "yaw": round(agent_metadata["rotation"]["y"]),
        }
        # print(
        #     "Initial agent_pose: ({}, {}, {})".format(
        #         agent_pose["x"], agent_pose["z"], agent_pose["yaw"]
        #     )
        # )
        self.ag = Agent(
            self.controller,
            agent_pose,
            self.frames,
            self.objects,
            trial_name=self.trial_name,
            mode="sfm",
            verbose=False
        )
        self.actionToExecute = self.frames[-1].name


def evaluate(env:ThorEnv):
    env.init_scene()
    suc, reason = env.ag.execute(env.actionToExecute) 
    env.controller.step(
        action="Done"
    )
    if suc:
        print("Trial {}: PASS".format(env.trial_name))
        return True
    else:
        print("Trial {}: FAIL Reason: {}".format(env.trial_name, reason))
        return False


files = glob.glob(
    "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/**/ann_*.json",
    recursive=True,
)
# random.shuffle(files)
# files = ["/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-218/trial_T20190908_152137_254608/pp/ann_0.json"]
# envs = []
successes = 0
fails = 0
trials = 0
i = 0
count = 0
metagroups = set(["pick_and_place_simple"])
for file in files:
    goal_desc = file.split("/")[9]
    if "trial" in goal_desc:
        continue
    if "look_at_obj_in_light" in file:
        # count += 1
        with open(file) as f:
            print("Opening {}".format(file))
            try:
                data = json.load(f)
            except:
                print("Failed")
                continue
            scene_desc = data["scene"]
            env = ThorEnv(scene_desc, goal_desc, i)
            i+=1
            if evaluate(env):
                successes += 1
            else:
                fails += 1
            trials += 1
# print(count)
print("nTrials: {}".format(trials))
print("nSuccesses: {}".format(successes))
print("nFails: {}".format(fails))
print("Success Rate: {}%".format((successes/trials)*100))

            
            # envs.append(ThorEnv(scene_desc, goal_desc, i))
        #     i+=1
        # if len(envs)>=5:
        #     break
# processes = []
# for env in envs:
#     p = Process(target=evaluate, args=(env,))
#     p.start()
#     processes.append(p)
# for p in processes:
#     p.join()
# print("All Processes Done!")
