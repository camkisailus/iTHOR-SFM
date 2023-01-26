from ai2thor.controller import Controller
from navigation import Navigation
from agent import Agent
import matplotlib.pyplot as plt
import random


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

    controller = Controller(
        agentMode="default",
        visibilityDistance=1.5,
        scene="FloorPlan212",
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

    controller.step(
        action="Teleport",
        position=dict(
            x=agent_metadata["position"]["x"],
            y=agent_metadata["position"]["y"],
            z=agent_metadata["position"]["z"],
        ),
        rotation=dict(x=0, y=270, z=0),
        horizon=30,
        standing=True,
    )
    agent_metadata = controller.last_event.metadata["agent"]
    agent_pose = {
        "x": agent_metadata["position"]["x"],
        "z": agent_metadata["position"]["z"],
        "yaw": agent_metadata["rotation"]["y"],
    }
    print(agent_pose)
    ag = Agent(controller, agent_pose)
    ag.searchFor(object_name="Pillow")
    ag.goTo(goal="random")
    # ag.makeVideo()
    # testNav(ag)
    while True:
        pass

    exit()
