from ai2thor.controller import Controller
from navigation import Navigation
from agent import Agent
import matplotlib.pyplot as plt
import random

def testNav(agent):
    reachable_poses =  controller.step(
        action="GetReachablePositions"
    ).metadata["actionReturn"]
    denom = len(reachable_poses)
    num = 0
    random.shuffle(reachable_poses)
    for position in reachable_poses:
        goal = {'x':position['x'], 'z':position['z']}
        agent.goTo(goal)
        agent_metadata = controller.last_event.metadata['agent']
        if goal['x'] == agent_metadata['position']['x'] and goal['z'] == agent_metadata['position']['z']:
            num += 1
        break
    # print("Success Rate: {}%".format((num / denom)*100))

if __name__ == '__main__':

    controller = Controller(
        agentMode="default",
        visibilityDistance=1.5,
        scene="FloorPlan219",

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
        fieldOfView=90
    )
    # setup topdown view cam
    event = controller.step(action="GetMapViewCameraProperties")
    event = controller.step(action="AddThirdPartyCamera", agentId=0, **event.metadata["actionReturn"])
    # event = controller.step(
    #     action="AddThirdPartyCamera",
    #     position=dict(x=0, y=2, z=0),
    #     rotation=dict(x=0, y=0, z=0),
    #     fieldOfView=90
    # )

# event.third_party_camera_frames
    agent_metdata = controller.last_event.metadata['agent']
    agent_pose = {
        'x': agent_metdata['position']['x'],
        'z':agent_metdata['position']['z'],
        'yaw':agent_metdata['rotation']['y']
    }
    ag = Agent(controller, agent_pose)
    ag.searchFor(object_name="Newspaper")
    ag.goTo(goal="random")
    ag.makeVideo()
    # testNav(ag)
    while True:
        pass

    exit()
