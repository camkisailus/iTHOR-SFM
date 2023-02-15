from ai2thor.controller import Controller
from ai2thor.platform import CloudRendering
import os
import json
import glob
from agent import Agent
import utils
from multiprocessing import Process
import random
import concurrent.futures
import argparse 

class ThorEnv:
    def __init__(self, scene_description: dict, goal_description: str, trial_id: str):
        # self.floorPlan =
        print(goal_description)
        self.scene_desc = scene_description
        self.goal_desc = goal_description
        self.frames, self.objects = utils.loadFramesFromALFRED(goal_description)
        print("Frames:")
        for frame in self.frames:
            print("\t{}".format(frame))
        print("Objects: {}".format(self.objects))
        # event = self.controller.step(
        #     action="SetObjectPoses", objectPoses=scene_description["object_poses"]
        # )
        # print(event.metadata)
        self.trial_name = "{}_{}".format(goal_description, trial_id)    
    
    def init_scene(self):
        # print(
        #     "[DRIVER]: Initializing ThorEnv with FloorPlan {} experiment is {}".format(
        #         self.scene_desc["floor_plan"], self.goal_desc
        #     )
        # )
        self.controller = Controller(
            agentMode="default",
            visibilityDistance=1.5,
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
            # platform=CloudRendering
        )
        # objs = []
        # for obj in self.controller.last_event.metadata["objects"]:
        #     if "Sink" in obj['objectType']:
        #         print(obj['objectType'])
        #         print(obj['objectId'])
        #         print(utils.cleanObjectID(obj['objectId']))
        #     objs.append(obj["objectType"])
        # for obj in sorted(objs):
        #     if obj == "SinkBasin"
        #     print(obj)
        # exit(s)
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
        self.ag = Agent(
            self.controller,
            agent_pose,
            self.frames,
            self.objects,
            trial_name=self.trial_name,
            mode="oracle",
            verbose=False
        )
        self.actions = ["Put_AppleSliced_on_Microwave"]
        ## TODO: Make this for all 7 metagroups. 
        # This tells the evaluator which frames to execute AND in which order
        # self.actions = []
        # for frame in self.frames:

        #     if "Clean" in frame.name:
        #         self.actions.insert(0, frame.name)
        #     elif "Put" in frame.name:
        #         self.actions.insert(1, frame.name)
            # if "Heat" in frame.name:
            #     self.actions.insert(0, frame.name)
            # elif "Put" in frame.name:
            #     self.actions.insert(1, frame.name)
        # assert(len(self.actions) == 2)
        # self.actionToExecute = self.frames[-1].name


def evaluate(env:ThorEnv):
    env.init_scene()
    if env.ag.state.action_history != []:
        env.ag.state.action_history = []
        # raise RuntimeError("[EVALUATE]: Agent history = {}".format(env.ag.state.action_history))
    assert(env.ag.state.action_history == [])
    for action in env.actions:
        # print("Executing {}".format(action))
        suc, reason = env.ag.execute(action)
        if not suc:
            break 
    env.controller.step(
        action="Done"
    )
    if suc:
        print("Trial {}: PASS".format(env.trial_name))
        return True
    else:
        print("Trial {}: FAIL Reason: {}".format(env.trial_name, reason))
        return False
   

def evaluate_threaded(chunk):
    # files = [
    #     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-HandTowel-None-CounterTop-421/trial_T20190909_145525_802579/pp/ann_1.json",
    #     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-ButterKnife-None-SideTable-28/trial_T20190908_133040_811710/pp/ann_1.json"

    # ]
    # chunks = [
    #     "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/pick_and_place_simple/pick_and_place_simple_chunk_.txt"
    # ]
    chunk_file = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/pick_and_place_simple/pick_and_place_simple_chunk_{}.txt".format(chunk)
    # chunk_file= "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/pick_and_place_simple/test_{}.txt".format(chunk)
    # files = [line.rstrip() for line in open()]
    with open(chunk_file) as tasks:
        files = [line.rstrip() for line in tasks]
    with concurrent.futures.ThreadPoolExecutor() as executor:
        for file in files:
            if "Sliced" in file:
                # skip these for now
                continue
            futures = []
            with open(file) as f:
                try:
                    data = json.load(f)
                except:
                    raise RuntimeError("JSON parse failed")
                scene_desc = data["scene"]
                goal_desc = file.split("/")[9]
                # env = ThorEnv(scene_description=scene_desc, goal_description=goal_desc, trial_id=0)
                futures.append(executor.submit(evaluate, env=ThorEnv(scene_description=scene_desc, goal_description=goal_desc, trial_id=0)))
        for future in concurrent.futures.as_completed(futures):
            # print(future.result)
            pass

if __name__ == "__main__":
    # parser = argparse.ArgumentParser()
    # parser.add_argument("--chunk")
    # args = parser.parse_args()
    # chunk_file = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/pick_and_place_simple/pick_and_place_simple_chunk_{}.txt".format(args.chunk)
    # with open(chunk_file) as tasks:
    #     files = [line.rstrip() for line in tasks]
    # for file in files:
    #     with open(file) as f:
    #         if "Sliced" in file:
    #             # skip these for now
    #             continue
    #         try:
    #             data = json.load(f)
    #         except:
    #             raise RuntimeError("JSON parse failed for {}".format(file))
    #         scene_desc = data['scene']
    #         goal_desc = file.split("/")[9]
    #         env = ThorEnv(scene_description=scene_desc, goal_description=goal_desc, trial_id=0)
    #         print("Evaluating: {}".format(goal_desc))
    #         evaluate(env)


    # print(args.chunk)
    # evaluate_threaded(chunk=args.chunk)
    # print("Hello world")
    # files = [
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-HandTowel-None-CounterTop-421/trial_T20190909_145525_802579/pp/ann_1.json",
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-ButterKnife-None-SideTable-28/trial_T20190908_133040_811710/pp/ann_1.json",
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-HandTowel-None-BathtubBasin-423/trial_T20190909_044455_524924/pp/ann_1.json",
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-SoapBottle-None-Toilet-413/trial_T20190909_042928_067491/pp/ann_1.json",
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-Pillow-None-ArmChair-225/trial_T20190908_142357_753395/pp/ann_1.json",
    #         # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-ToiletPaper-None-ToiletPaperHanger-421/trial_T20190906_182536_996833/pp/ann_1.json", 
    #         "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-Vase-None-Safe-204/trial_T20190919_000336_714640/pp/ann_1.json"
    #     ]
    # files = ["/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_heat_then_place_in_recep-BreadSliced-None-DiningTable-27/trial_T20190908_140003_184319/pp/ann_0.json"]
    files = [
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_with_movable_recep-AppleSliced-Pot-CounterTop-4/trial_T20190909_050345_143442/pp/ann_0.json",
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_with_movable_recep-AppleSliced-Plate-Fridge-14/trial_T20190908_081858_699793/pp/ann_0.json",
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_with_movable_recep-Apple-Pot-Fridge-26/trial_T20190907_150522_970867/pp/ann_0.json",
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_with_movable_recep-Apple-Pan-DiningTable-18/trial_T20190908_055822_495768/pp/ann_0.json",

    ]
    for file in files:
        goal_desc = file.split("/")[9]
        with open(file) as f:
            # print("Opening ")
            try:
                data = json.load(f)
            except:
                # print("Loading {} ".format(f))
                raise RuntimeError("Foobar")
            scene_desc = data["scene"]
            env = ThorEnv(scene_desc, goal_desc, 0)
            print("#"*20)
            # if evaluate(env):
            #     print("WOOHOO!")
            # else:
            #     print("AWWWW MAN :(")