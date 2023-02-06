from ai2thor.controller import Controller
# from ai2thor.platform import CloudRendering
from navigation import Navigation
from agent import Agent
import matplotlib.pyplot as plt
import random
import argparse
import utils
import os

ROOT = os.path.dirname(os.path.realpath(__file__))
BAD_SCENES = [
    "FloorPlan23",
    "FloorPlan306",
    "FloorPlan209",
    "FloorPlan325",
    "FloorPlan210",
    "FloorPlan15",
    "FloorPlan206",
    "FloorPlan9",
    "FloorPlan323",
]



def testObsv(agent):
    print("Saving Initial Poses")
    agent.saveDistributions()
    # agent.processRGB()
    # agent.updateFilters()
    # agent.saveDistributions()
    # valid_rotations = [0, 90, 180, 270]
    # goal = {
    #         "x": -1,
    #         "z": 3,
    #         "yaw": 90,
    #     }
    goal = agent.nav.getRandomValidPose()
    goal['yaw'] = 0.0
    print("Goal: {}".format(goal))
    agent.goTo(goal)
    print("Processing RGB")
    agent.processRGB()
    print("Updating filters")
    agent.updateFilters()
    print("Saving Final Distributions")
    agent.saveDistributions()
    

def testNav(agent):
    """
    Test Nav Algorithm.
    On set of all bedrooms, kitchens and livings rooms we reach 99.2% of all possible poses
    """
    reachable_poses = controller.step(action="GetReachablePositions").metadata[
        "actionReturn"
    ]
    denom = len(reachable_poses)
    num = 0
    valid_rotations = [0, 90, 180, 270]
    random.shuffle(reachable_poses)
    for i, position in enumerate(reachable_poses):
        goal = {
            "x": position["x"],
            "z": position["z"],
            "yaw": random.choice(valid_rotations),
        }
        agent.goTo(goal)
        # agent_metadata = controller.last_event.metadata["agent"]
        # x = agent_metadata["position"]["x"]
        # z = agent_metadata["position"]["z"]
        # yaw = agent_metadata["rotation"]["y"]
        x = agent.cur_pose["x"]
        z = agent.cur_pose["z"]
        yaw = agent.cur_pose["yaw"]
        if goal["x"] == x and goal["z"] == z and goal["yaw"] == yaw:
            # print("Test {}/{}: PASS".format(i + 1, denom))
            num += 1
        else:
            print("Test {}/{}: FAIL".format(i + 1, denom))
            print(
                "\t Goal: ({},{},{}) Final Robot Pose: ({}, {}, {})".format(
                    goal["x"], goal["z"], goal["yaw"], x, z, yaw
                )
            )
    print("Success Rate: {}%".format((num / denom) * 100))
    return num, denom


if __name__ == "__main__":
    floorPlan, frames, objects, actions = utils.loadExperimentConfig(
        os.path.join(ROOT, "experiment_configs", "exp_1.yaml")
    )
    scene = "FloorPlan3_physics"
    # for frame in frames:
    #     print(frame)
    # print("#"*20)
    # for object in objects:
    #     print(object)

    controller = Controller(
        agentMode="default",
        visibilityDistance=1.0,
        scene=scene,
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
    # for obj in controller.last_event.metadata["objects"]:
    #     if "Tomato" in obj['objectType']:
    #         print(obj['objectId'])
    #         print(controller.step(
    #                         action="GetInteractablePoses",
    #                         objectId=obj['objectId'],
    #                         rotations=[0, 90, 180, 270],
    #                         horizons=[0],
    #                         standings=[True],
    #                     ).metadata["actionReturn"])
    #         print("Tomato is located at ({}, {})".format(obj['position']['x'], obj['position']['z']))
    #     elif "Knife" == obj['objectType']:
    #         print("Knife is located at ({}, {})".format(obj['position']['x'], obj['position']['z']))
    # exit()
    # kitchens = controller.ithor_scenes(
    #     include_kitchens=True,
    #     include_living_rooms=False,
    #     include_bedrooms=False,
    #     include_bathrooms=False
    # )
    # random.shuffle(kitchens)
    success = 0
    total = 0
    # kitchen_scene = "FloorPlan2_physics"
    # for i, kitchen_scene in enumerate(kitchens):
    print("[DRIVER]: Starting Trial with FloorPlan {}".format(scene))
    # controller.reset(scene=kitchen_scene)
    # setup topdown view cam
    event = controller.step(action="GetMapViewCameraProperties")
    event = controller.step(
        action="AddThirdPartyCamera", agentId=0, **event.metadata["actionReturn"]
    )
    agent_metadata = controller.last_event.metadata["agent"]
    agent_pose = {
        "x": agent_metadata["position"]["x"],
        "z": agent_metadata["position"]["z"],
        "yaw": round(agent_metadata["rotation"]["y"]),
    }
    # print(agent_pose)
    # ag = Agent(controller, agent_pose, frames, objects)
    # testObsv(ag)
    # # for action in actions:
    # #     print("Executing {}".format(action))
    #     ag.execute(action)
    # # print("Done")
    controller.step(
        action="Teleport",
        position=dict(
            x=agent_pose["x"], y=agent_metadata["position"]["y"], z=agent_pose["z"]
        ),
        rotation=dict(x=0, y=agent_pose["yaw"], z=0),
    )
    # try:
    # ag = Agent(controller, agent_pose, frames, objects, trial_name="gPlate_{}_oracle".format(scene), mode='oracle')
    ag = Agent(controller, agent_pose, frames, objects, trial_name="test2CascadingPreconditions_{}".format(scene), mode='sfm')
    # suc, tot = testNav(ag)
    # success+= suc
    # total += tot
    # continue
    # testObsv(ag)
    for action in actions:
        print("[DRIVER]: Executing {}".format(action))
        suc = ag.execute(action)
    if suc:
        success += 1
        print("[DRIVER]: Success")
    else:
        print("[DRIVER]: Failure")
    # print("Success: {}".format(success))
    # print("Total: {}".format(total))
    # suc_rate = (success/total)*100
    # print("Overall success rate: {}".format(suc_rate))
    # print("Total Success Rate: {:.4f}%".format((success/len(kitchens))*100))
    # ag.goTo(goal={'x':-4.0, 'z':-1.0, 'yaw':0.0})
    # ag.makeVideo()
    # testNav(ag)
    controller.step(
        action="Done"
    )
    while True:
        pass
