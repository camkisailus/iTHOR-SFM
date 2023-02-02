from ai2thor.controller import Controller
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
    # for frame in frames:
    #     print(frame)
    # print("#"*20)
    # for object in objects:
    #     print(object)

    controller = Controller(
        agentMode="default",
        visibilityDistance=1.5,
        scene="FloorPlan27_physics",
        # step sizes
        gridSize=0.25,
        snapToGrid=True,
        rotateStepDegrees=90,
        # image modalities
        renderDepthImage=True,
        renderInstanceSegmentation=True,
        # camera properties
        width=300,
        height=300,
        fieldOfView=90,
    )
    # kitchens = controller.ithor_scenes(
    #     include_kitchens=True,
    #     include_living_rooms=False,
    #     include_bedrooms=False,
    #     include_bathrooms=False
    # )
    # random.shuffle(kitchens)
    success = 0
    total = 0
    kitchen_scene = "FloorPlan27_physics"
    # for i, kitchen_scene in enumerate(kitchens):
    print("[DRIVER]: Starting Trial with FloorPlan {}".format(kitchen_scene))
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
    controller.step(
        action="Teleport",
        position=dict(
            x=agent_pose["x"], y=agent_metadata["position"]["y"], z=agent_pose["z"]
        ),
        rotation=dict(x=0, y=agent_pose["yaw"], z=0),
    )
    # try:
    ag = Agent(controller, agent_pose, frames, objects, trial_name="foobar")
    # suc, tot = testNav(ag)
    # success+= suc
    # total += tot
    # continue
    for action in actions:
        print("Executing {}".format(action))
        suc = ag.execute(action)
    # except Exception as e:
    #     print("[ERROR]: {}...Floorplan was {}".format(e, kitchen_scene))
    #     suc = False

    # if suc:
    #     print("Trial {}/{}: PASS".format(i, len(kitchens)))
    #     success += 1
    # else:
    #     print("Trial {}/{}: FAIL".format(i, len(kitchens)))
    # print("Success: {}".format(success))
    # print("Total: {}".format(total))
    # suc_rate = (success/total)*100
    # print("Overall success rate: {}".format(suc_rate))
    # print("Total Success Rate: {:.4f}%".format((success/len(kitchens))*100))
    exit()

    # ag.goTo(goal={'x':-4.0, 'z':-1.0, 'yaw':0.0})
    # ag.makeVideo()
    # testNav(ag)
    exit()
    while True:
        pass
