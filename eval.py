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
# print("UHHHHHH")
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
        # print(
        #     "[DRIVER]: Initializing ThorEnv with FloorPlan {} experiment is {}".format(
        #         self.scene_desc["floor_plan"], self.goal_desc
        #     )
        # )
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
            # platform=CloudRendering
        )
        # for obj in self.controller.last_event.metadata["objects"]:
        #     print(obj["objectType"])
        # exit()
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
            mode="sfm",
            verbose=True
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

def evaluate_threaded():
    files = [
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-HandTowel-None-CounterTop-421/trial_T20190909_145525_802579/pp/ann_1.json",
        "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-ButterKnife-None-SideTable-28/trial_T20190908_133040_811710/pp/ann_1.json"

    ]
    # with open("look_at_obj_in_light_chunk_14.txt") as tasks:
    #     files = [line.rstrip() for line in tasks]
    with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
        for file in files:
            futures = []
            with open(file) as f:
                try:
                    data = json.load(f)
                except:
                    raise RuntimeError("JSON parse failed")
                scene_desc = data["scene"]
                goal_desc = file.split("/")[9]
                env = ThorEnv(scene_description=scene_desc, goal_description=goal_desc, trial_id=0)
                futures.append(executor.submit(evaluate, env=env))
        for future in concurrent.futures.as_completed(futures):
            pass

if __name__ == "__main__":
    # print("Hello world")
    files = [
            "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-HandTowel-None-CounterTop-421/trial_T20190909_145525_802579/pp/ann_1.json",
            "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple-ButterKnife-None-SideTable-28/trial_T20190908_133040_811710/pp/ann_1.json"
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
            if evaluate(env):
                print("WOOHOO!")
            else:
                print("AWWWW MAN")



# files = glob.glob(
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/**/ann_*.json",
#     recursive=True,
# )
# random.shuffle(files)
# files = ["/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-218/trial_T20190908_152137_254608/pp/ann_0.json"]
# envs = []
# successes = 0
# fails = 0
# trials = 0
# i = 0
# count = 0
# metagroups = set(["pick_and_place_simple"])
# look_at_obj_in_light = set()
# pick_and_place_simple = set()
# pick_two_and_place = set()
# pick_heat_and_place = set()
# pick_cool_and_place = set()
# pick_clean_and_place = set()
# pick_and_place_movable = set()
# task_queue = []
# for file in files:
#     goal_desc = file.split("/")[9]
#     if "trial" in goal_desc:
#         continue
#     if "look_at_obj_in_light" in file:
#         #/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-213/trial_T20190909_144511_768997/pp/ann_2.json
#         count+=1
#         if goal_desc not in look_at_obj_in_light:
#             look_at_obj_in_light.add(goal_desc)
#             # task_queue.append(file)
#     elif "pick_and_place_simple" in file:
#         if goal_desc not in pick_and_place_simple:
#             pick_and_place_simple.add(goal_desc)
#             task_queue.append(file)
#     elif "pick_two_obj_and_place" in file:
#         if goal_desc not in pick_two_and_place:
#             pick_two_and_place.add(goal_desc)
#     elif "pick_heat_then_place" in file:
#         if goal_desc not in pick_heat_and_place:
#             pick_heat_and_place.add(goal_desc)
#     elif "pick_cool_then_place" in file:
#         if goal_desc not in pick_cool_and_place:
#             pick_cool_and_place.add(goal_desc)
#     elif "pick_clean_then_place" in file:
#         if goal_desc not in pick_clean_and_place:
#             pick_clean_and_place.add(goal_desc)
#     elif "pick_and_place_with_movable" in file:
#         if goal_desc not in pick_and_place_movable:
#             pick_and_place_movable.add(goal_desc)
    # elif 
        # experiments.add(file.split("/")[])
        # count += 1
        # with open(file) as f:
        #     print("Opening {}".format(file))
        #     try:
        #         data = json.load(f)
        #     except:
        #         print("Failed")
        #         continue
        #     scene_desc = data["scene"]
        #     env = ThorEnv(scene_desc, goal_desc, i)
        #     i+=1
        #     if evaluate(env):
        #         successes += 1
        #     else:
        #         fails += 1
        #     trials += 1
# file = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Box-None-FloorLamp-212/trial_T20190908_193427_340509/pp/ann_1.json"
# files = [
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Statue-None-FloorLamp-221/trial_T20190907_082527_808720/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Watch-None-FloorLamp-202/trial_T20190906_185327_967653/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-209/trial_T20190907_212742_693540/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Box-None-FloorLamp-212/trial_T20190908_193427_340509/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-225/trial_T20190909_103849_934231/pp/ann_1.json"
# ]
# files = [
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-220/trial_T20190907_073800_789341/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-205/trial_T20190909_025208_030542/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-BasketBall-None-DeskLamp-304/trial_T20190906_174726_702879/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CellPhone-None-FloorLamp-219/trial_T20190908_044113_026049/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-TennisRacket-None-DeskLamp-328/trial_T20190907_083746_459297/pp/ann_1.json"
# ]
# files = [
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Statue-None-FloorLamp-221/trial_T20190907_082527_808720/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Watch-None-FloorLamp-202/trial_T20190906_185327_967653/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-209/trial_T20190907_212742_693540/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-Box-None-FloorLamp-212/trial_T20190908_193427_340509/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-225/trial_T20190909_103849_934231/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-220/trial_T20190907_073800_789341/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CreditCard-None-FloorLamp-205/trial_T20190909_025208_030542/pp/ann_1.json",
#     "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-BasketBall-None-DeskLamp-304/trial_T20190906_174726_702879/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-CellPhone-None-FloorLamp-219/trial_T20190908_044113_026049/pp/ann_1.json",
#     # "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_in_light-TennisRacket-None-DeskLamp-328/trial_T20190907_083746_459297/pp/ann_1.json"
# ]
# print(goal_desc)
# print(scene_desc)

# print(count)
# print(experiments)
# for exp in experiments:
#     print(exp)
# look_at_obj_in_light = set()
# pick_and_place_simple = set()
# pick_two_and_place = set()
# pick_heat_and_place = set()
# pick_cool_and_place = set()
# pick_clean_and_place = set()
# pick_and_place_movable = set()
# with open("pick_and_place_simple.txt", 'w+') as f:
#     for task in task_queue:
#         f.write(task+"\n")

# print("{} experiments of type look_at_obj_in_light".format(len(look_at_obj_in_light)))
# i = 0
# envs = []
# for task in task_queue:
#     goal_desc = task.split("/")[9]
#     with open(task) as f:
        # try:
        #     data = json.load(f)
        # except:
        #     print("Loading {} ".format(f))
        #     continue
        # scene_desc = data["scene"]
        # env = ThorEnv(scene_desc, goal_desc, i)
        # envs.append(env)
        # i += 1
    
# processes = []
# for env in envs:
#     p = Process(target=evaluate, args=(env,))
#     p.start()
#     processes.append(p)
# for p in processes:
#     p.join()
# print(len(envs))

# print(len(task_queue))
# print("{} experiments of type pick_and_place_simple".format(len(pick_and_place_simple)))
# print("{} experiments of type pick_two_and_place".format(len(pick_two_and_place)))
# print("{} experiments of type pick_heat_and_place".format(len(pick_heat_and_place)))
# print("{} experiments of type pick_cool_and_place".format(len(pick_cool_and_place)))
# print("{} experiments of type pick_clean_and_place".format(len(pick_clean_and_place)))
# print("{} experiments of type pick_and_place_movable".format(len(pick_and_place_movable)))
# i = 0