from ai2thor.controller import Controller
from navigation import Navigation
from agent import Agent
import matplotlib.pyplot as plt
import random
import argparse
import utils
import os

ROOT = os.path.dirname(os.path.realpath(__file__))


def testNav(agent):
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
        agent_metadata = controller.last_event.metadata["agent"]
        x = agent_metadata["position"]["x"]
        z = agent_metadata["position"]["z"]
        yaw = agent_metadata["rotation"]["y"]
        if goal["x"] == x and goal["z"] == z and goal["yaw"] == yaw:
            print("Test {}/{}: PASS".format(i + 1, denom))
            num += 1
        else:
            print("Test {}/{}: FAIL".format(i + 1, denom))
            print(
                "\t Goal: ({},{},{}) Final Robot Pose: ({}, {}, {})".format(
                    goal["x"], goal["z"], goal["yaw"], x, z, yaw
                )
            )
    print("Success Rate: {}%".format((num / denom) * 100))


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
        scene=floorPlan,
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
    # setup topdown view cam
    event = controller.step(action="GetMapViewCameraProperties")
    event = controller.step(
        action="AddThirdPartyCamera", agentId=0, **event.metadata["actionReturn"]
    )
    agent_metadata = controller.last_event.metadata["agent"]
    agent_pose = {
        "x": agent_metadata["position"]["x"],
        "z": agent_metadata["position"]["z"],
        "yaw": agent_metadata["rotation"]["y"],
    }
    print(agent_pose)
    ag = Agent(controller, agent_pose, frames, objects)
    for action in actions:
        print("Executing {}".format(action))
        ag.execute(action)
    # print("Done")
    # ag.goTo(goal={'x':-4.0, 'z':-1.0, 'yaw':0.0})
    # ag.makeVideo()
    # testNav(ag)
    exit()
    while True:
        pass
