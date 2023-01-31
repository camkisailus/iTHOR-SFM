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
        scene="FloorPlan3",
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
    # controller.step(
    #     action='SetObjectPoses',
    #     objectPoses=[
    #         {
    #             "objectName":"Knife"
    #         }
    #     ]
    # )
    # setup topdown view cam
    event = controller.step(action="GetMapViewCameraProperties")
    event = controller.step(
        action="AddThirdPartyCamera", agentId=0, **event.metadata["actionReturn"]
    )

    agent_metadata = controller.last_event.metadata["agent"]

    # controller.step(
    #     action="Teleport",
    #     position=dict(
    #         x= -0.25,#agent_metadata["position"]["x"],
    #         y= 0, #agent_metadata["position"]["y"],
    #         z=1.25 #agent_metadata["position"]["z"],
    #     ),
    #     rotation=dict(x=0, y=270, z=0),
    #     horizon=30,
    #     standing=True,
    # )
    # {'x': -0.25, 'z': 1.25, 'yaw': 270.0}
    agent_metadata = controller.last_event.metadata["agent"]
    agent_pose = {
        "x": agent_metadata["position"]["x"],
        "z": agent_metadata["position"]["z"],
        "yaw": agent_metadata["rotation"]["y"],
    }
    print(agent_pose)
    ag = Agent(controller, agent_pose)
    ag.observeSurroundings()
    ag.execute("Slice_Apple")
    # ag.searchFor(object_name="Tomato")
    # goal_pose = {
    #     'x': -3.75,
    #     'z':1.25,
    #     'yaw':0.0
    # }
    # ag.goTo(goal=goal_pose)
    print("Done")
    # ag.goTo(goal={'x':-4.0, 'z':-1.0, 'yaw':0.0})
    # ag.makeVideo()
    # testNav(ag)
    while True:
        pass

    exit()
