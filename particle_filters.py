import numpy as np
import matplotlib.pyplot as plt
from copy import deepcopy
import utils


class ParticleFilter:
    def __init__(self, label, controller):
        self.label = label
        self.valid_poses = controller.step(action="GetReachablePositions").metadata[
            "actionReturn"
        ]
        nPoses = len(self.valid_poses)
        self.nParticles = nPoses*4
        self.particles = np.zeros([self.nParticles, 3])  # 4 rotations for each x,z pose
        self.weights = (1 / (self.nParticles)) * np.ones(
            (self.nParticles)
        )  # initialize weights
        idx = 0
        for pose in self.valid_poses:
            for rot in [0, 90, 180, 270]:
                self.particles[idx, 0] = pose["x"]
                self.particles[idx, 1] = pose["z"]
                self.particles[idx, 2] = rot
                idx += 1

    def showParticles(self):
        print("Particles in {}".format(self.label))
        for i, particle in enumerate(self.particles):
            print(
                "\tPose: ({}, {}, {})...Weight: {:.4f}\n".format(
                    particle[0], particle[1], particle[2], self.weights[i]
                )
            )

    def getHighestWeightedParticle(self):
        return self.particles[np.argmax(self.weights)]

    def saveDistribution(self, fname):
        cm = plt.cm.get_cmap("winter")
        fig = plt.figure()
        axs = fig.gca()
        weightMap = np.clip(self.weights / np.max(self.weights), 0, 1)
        sc = axs.scatter(
            self.particles[:, 0], self.particles[:, 1], c=weightMap, cmap=cm, alpha=weightMap
        )
        axs.set_title("{} Distribution @ step {}".format(self.label, fname))
        fig.colorbar(sc)
        fig.savefig(
            "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/distributions/{}_{}.png".format(
                self.label, fname
            )
        )
        plt.close(fig)

class ObjectParticleFilter(ParticleFilter):
    def __init__(self, label, controller):
        super().__init__(label, controller)
        self.observations = (
            []
        )  # list of interactable poses ({'x', 'z', 'yaw'}) for object
        self.negative_regions = []  # list of boundaries (min_x, max_x, min_z, max_z)
        self.negative_poses = []

    def addObservation(self, interactablePoses):
        for pose in interactablePoses:
            self.observations.append(
                {"x": pose["x"], "z": pose["z"], "yaw": pose["rotation"]}
            )

    def addNegativePose(self, pose):
        self.negative_poses.append(pose)

    def handleObservation(self, observation_msg):
        observed_obj = False
        for obj_id, interactable_poses in observation_msg.items():
            if utils.cleanObjectID(obj_id) == self.label:
                # print("Adding interactable poses to filter")
                observed_obj = True
                # Detected object
                for pose in interactable_poses:
                    self.observations.append(
                        {"x": pose["x"], "z": pose["z"], "yaw": pose["rotation"]}
                    )
        if not observed_obj:
            # Add negative region around robot
            # print("Adding negative robot pose ({}, {})".format(observation_msg['robot_pose']['x'], observation_msg['robot_pose']['z']))
            robot_pose = observation_msg["robot_pose"]
            visibility = 1.5  # m
            fov = 90  # degrees
            if robot_pose not in self.negative_poses:
                self.negative_poses.append(robot_pose)

    def assignWeight(self, particle):
        # for region in self.negative_regions:
        #     min_x, max_x, min_z, max_z = region[0], region[1], region[2], region[3]
        #     if min_x <= particle[0] <= max_x and min_z <= particle[1] <= max_z:
        #         return 0.0
        # print("Particle at ({}, {}, {})".format(particle[0], particle[1], particle[2]))
        for pose in self.negative_poses:
            if particle[0] == pose["x"] and particle[1] == pose["z"]:
                # print("Negative Pose")
                return 0.0
        max_phi = -1e9
        for interactable_pose in self.observations:
            if (
                particle[0] == interactable_pose["x"]
                and particle[1] == interactable_pose["z"]
                and particle[2] == interactable_pose["yaw"]
            ):
                # print("Perfect Match!")
                return 10
            else:  # particle close to interactable pose
                linearDist = np.sqrt(
                    (interactable_pose['x'] - particle[0]) ** 2
                    + (interactable_pose['z'] - particle[1]) ** 2
                )
                angularDist = np.abs(interactable_pose['yaw'] - particle[2]) % 360
                if angularDist > 180:
                    angularDist = 360 - angularDist
                linearStepErr = linearDist / 0.25  # 0.25m per robot step
                angularStepErr = angularDist / 90  # 90deg per robot step
                phi = np.exp(-1*(linearStepErr + angularStepErr))
                if phi > max_phi:
                    max_phi = phi
        return max_phi

    def updateFilter(self):
        for i, particle in enumerate(self.particles):
            weight = self.assignWeight(particle)
            self.weights[i] += weight
        self.weights /= np.sum(self.weights)


class FrameParticleFilter(ParticleFilter):
    def __init__(self, label, preconditions, core_frame_elements, controller):
        """
        Particle Filter for Semantic Frames

        Args:
            label (str) : frame name
            preconditions (list[str]) : ordered list of preconditions
            core_frame_elements (list[str]): ordered list of core_frame_elements
            controller (Controller): ai2-thor controller object
        """
        super().__init__(label, controller)
        self.frame_element_filters = {
            frame_element: None for frame_element in core_frame_elements
        }
        self.frame_elements = core_frame_elements
        if preconditions:
            self.precondition_filters = {
                precondition: None for precondition in preconditions
            }
            self.preconditions = preconditions
            self.next_precondition = preconditions[0]
        else:
            self.preconditions = False
        self.controller = controller

    def __str__(self):
        string = "Label: {}\n\tnParticles: {}\n\tFrame Elements:\n".format(
            self.label, self.nParticles
        )
        frameElemString = ""
        for frame_elem in self.frame_element_filters.keys():
            frameElemString += "\t\t{}\n".format(frame_elem)
        string += frameElemString
        string += "\tPreconditions:"
        preconditionString = ""
        if self.preconditions:
            for precondtion in self.precondition_filters.keys():
                preconditionString += "\t\t{}\n".format(precondtion)
        else:
            preconditionString = "\n\t\tNone"
        string += preconditionString
        return string

    def addFrameElementFilter(self, label, filter):
        self.frame_element_filters[label] = filter

    def addPreconditionFilter(self, label, filter):
        self.precondition_filters[label] = filter

    def contextPotential(self, particle, state):
        i = 0
        if self.preconditions:
            # print("We have preconditions")
            for precondition in self.preconditions:
                if precondition in state.action_history:
                    i += 1
                else:
                    break
            # print("i = {}".format(i))
            potential = 1.0
            while i < len(self.preconditions):
                subtaskFilter = self.precondition_filters[self.preconditions[i]]
                # print("Subtask filter name: {}".format(subtaskFilter.label))
                for j in range(subtaskFilter.nParticles):
                    otherParticle = subtaskFilter.particles[j, :]
                    linearDist = np.sqrt(
                        (otherParticle[0] - particle[0]) ** 2
                        + (otherParticle[1] - otherParticle[1]) ** 2
                    )
                    angularDist = np.abs(particle[2] - otherParticle[2]) % 360
                    if angularDist > 180:
                        angularDist = 360 - angularDist
                    linearStepErr = linearDist % 0.25  # 0.25m per robot step
                    angularStepErr = angularDist % 90  # 90deg per robot step
                    phi = np.exp(-10 * (linearStepErr + angularStepErr))
                    potential += phi
                i += 1
            return potential
        else:
            return 1.0

    def measurementPotential(self, particle, state):
        i = 0
        # print("Particle at ({}, {}, {})".format(particle[0], particle[1], particle[2]))
        if self.preconditions:
            for precondition in self.preconditons:
                if precondition not in state.action_history:
                    break
                else:
                    i += 1
        potential = 0
        coreElementWeightModifier = 1
        while i < len(self.frame_elements):
            coreElementFilter = self.frame_element_filters[self.frame_elements[i]]
            for j in range(coreElementFilter.nParticles):
                otherParticle = coreElementFilter.particles[j, :]
                if(
                    otherParticle[0] == particle[0]
                    and otherParticle[1] == particle[1]
                    and otherParticle[2] == particle[2]
                ):
                    potential += coreElementFilter.weights[j]
                else:
                    potential+=0
                # print("otherParticle at ({}, {}, {})".format(otherParticle[0], otherParticle[1], otherParticle[2]))
                # print("otherParticle weight = {}".format(coreElementFilter.weights[j]))
                # linearDist = np.sqrt(
                #     (otherParticle[0] - particle[0]) ** 2
                #     + (otherParticle[1] - particle[1]) ** 2
                # )
                # angularDist = np.abs(otherParticle[2] - particle[2]) % 360
                # if angularDist > 180:
                #     angularDist = 360 - angularDist
                # linearStepErr = linearDist / 0.25  # 0.25m per robot step
                # angularStepErr = angularDist / 90  # 90deg per robot step
                # # print("Linear step err: {}".format(linearStepErr))
                # # print("Angular step err: {}".format(angularStepErr))
                # # print("Total Err: {}".format(linearStepErr + angularStepErr))
                # phi = np.exp(-10 * (linearStepErr + angularStepErr))
                # # potential += (1 / (coreElementWeightModifier**2)) * (
                # #     phi * coreElementFilter.weights[j]
                # # )
                # potential += (1 / (coreElementWeightModifier**2)) * (
                #     phi * coreElementFilter.weights[j]
                # )
                # print("Potential = {}".format(potential))
            i += 1
            coreElementWeightModifier += 1
        # print("Measurement Potential = {}".format(potential))
        return potential

    def assignWeight(self, particle, state):
        measurement = self.measurementPotential(particle, state)
        context = self.contextPotential(particle, state)
        # print("Measurement: {}... Context: {}".format(measurement, context))
        # assert(measurement >= 1.0 and context >= 1.0)
        return measurement * context

    def updateFilter(self, state):
        for i in range(self.nParticles):
            weight = self.assignWeight(self.particles[i, :], state)
            self.weights[i] += weight
        self.weights /= np.sum(self.weights)

class State:
    def __init__(self, action_history=[]):
        self.action_history = action_history

if __name__ == "__main__":
    pillow_pf = ObjectParticleFilter("Pillow", None)  # self.controller)
    grasp_pillow_pf = FrameParticleFilter(
        "Grasp_Pillow",
        preconditions=None,
        core_frame_elements=["Pillow"],
        controller=None,
    )
    state = State()
    grasp_pillow_pf.addFrameElementFilter("Pillow", pillow_pf)
    print("Before Observation")
    pillow_pf.showParticles()
    grasp_pillow_pf.showParticles()
    pillow_pf.addObservation([{'x':0.0, 'z':0.0, 'rotation':0.0}])
    print("After Observation")
    for i in range(10000):
        pillow_pf.updateFilter()
        grasp_pillow_pf.updateFilter(state)
    pillow_pf.showParticles()
    grasp_pillow_pf.showParticles()
    # pillow_pf.updateFilter()
    # pillow_pf.showParticles()
    # grasp_pillow_pf.updateFilter(state)
    # grasp_pillow_pf.showParticles()


