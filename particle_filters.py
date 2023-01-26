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
        self.particles = np.zeros([nPoses, 3])  # 4 rotations for each x,z pose
        self.nParticles = nPoses
        self.weights = (1 / (nPoses)) * np.ones((nPoses))  # initialize weights
        idx = 0
        for pose in self.valid_poses:
            for rot in [0]:#, 90, 180, 270]:
                self.particles[idx, 0] = pose["x"]
                self.particles[idx, 1] = pose["z"]
                self.particles[idx, 2] = rot
                idx += 1
        

    def getHighestWeightedParticle(self):
        return self.particles[np.argmax(self.weights)]

    def saveDistribution(self, fname):
        max_weight = np.max(self.weights)
        min_weight = np.min(self.weights)
        # zero_weight_particles = np.where(self.weights == 0)[0]
        # assert(self.particles.shape == (self.nParticles, 3))
        # print("nParticles w 0 weight = {}".format(len(zero_weight_particles)))
        # for idx in zero_weight_particles:
        #     print("Particle at ({}, {})".format(self.particles[idx,0], self.particles[idx,1]))
        if max_weight == min_weight:
            # print("max == min")
            alphas = 0.1 *np.ones_like(self.weights)
        else:
            alphas = (self.weights[:] - min_weight)/(max_weight - min_weight)
            # print("Max alpha: {}... Min Alpha: {}".format(np.min(alphas), np.max(alphas)))
            # print(np.where(self.weights == 0)[0])
            # print("#####################")
            # print(np.where(alphas == 0)[0])
            # assert(np.where(self.weights==0) == np.where(alphas == 0))
            # idxs = np.where(self.weights == max_weight)
            # print("Max particles")
            # for idx in idxs:
            #     print(self.particles[idx])
        # alphas = np.clip(self.weights + 0.1, 0, 1)
            # print("Max particles: {}".format(np.where(self.weights == max_weight, self.weights)))
        # print("Alpha range: ({}, {})".format(np.min(alphas), np.max(alphas)))
        # print("Max alpha: {}".format(np.max(alphas)))
        fig = plt.figure()
        axs = fig.gca()
        axs.scatter(self.particles[:,0], self.particles[:,1], color='red', alpha=alphas)
        # print("Saving Fig {}.png".format(fname))
        # fig.show()
        fig.savefig("/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/distributions/{}.png".format(fname))
        plt.close(fig)
    # def resampleParticles(self):
    #     temp_p = deepcopy(self.particles)
    #     temp_w = deepcopy(self.weights)
    #     idx = np.arange(0, self.nParticles)
    #     samples = np.random.choice(idx, int(self.nParticles*0.8), replace=True, p=temp_w)
    #     for i in range(len(samples)):
    #         self.particles[i] = temp_p[samples[i]]
    #     for i in range(len(samples), self.nParticles-2):
    #         self.particles[i] = self.


class ObjectParticleFilter(ParticleFilter):
    def __init__(self, label, controller):
        super().__init__(label, controller)
        self.observations = []  # list of interactable poses ({'x', 'z', 'yaw'}) for object
        self.negative_regions = []  # list of boundaries (min_x, max_x, min_z, max_z)
        self.negative_poses = []
    def handleObservation(self, observation_msg):
        observed_obj = False
        for obj_id, interactable_poses in observation_msg.items():
            if utils.cleanObjectID(obj_id) == self.label:
                print("Adding interactable poses to filter")
                observed_obj = True
                # Detected object
                for pose in interactable_poses:
                    self.observations.append({
                        'x': pose['x'],
                        'z':pose['z'],
                        'yaw':pose['rotation']}
                    )
        if not observed_obj:
            # Add negative region around robot
            print("Adding negative robot pose ({}, {})".format(observation_msg['robot_pose']['x'], observation_msg['robot_pose']['z']))
            robot_pose = observation_msg["robot_pose"]
            visibility = 1.5  # m
            fov = 90  # degrees
            self.negative_poses.append(robot_pose)

    def assignWeight(self, particle):
        # for region in self.negative_regions:
        #     min_x, max_x, min_z, max_z = region[0], region[1], region[2], region[3]
        #     if min_x <= particle[0] <= max_x and min_z <= particle[1] <= max_z:
        #         return 0.0
        for pose in self.negative_poses:
            if (particle[0] == pose['x'] and particle[1] == pose['z']):
                # print("Particle at ({}, {}) got 0 weight".format(particle[0], particle[1]))
                return 0.0
        for interactable_pose in self.observations:
            # print(interactable_pose)
            if (
                particle[0] == interactable_pose["x"]
                and particle[1] == interactable_pose["z"]
                #and particle[2] == interactable_pose["yaw"]
            ):
                return 1.0
        return 0.2

    def updateFilter(self):
        for i, particle in enumerate(self.particles):
            weight = self.assignWeight(particle)
            self.weights[i] = weight
        self.weights /= np.sum(self.weights)
        # print(np.sum(self.weights))
        # assert(np.sum(self.weights) == 1)


class FrameParticleFilter(ParticleFilter):
    def __init__(self, label, controller):
        super().__init__(label, controller)
