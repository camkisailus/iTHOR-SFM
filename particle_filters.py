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
        self.weights = (1 / (self.nParticles)) * np.ones((self.nParticles))  # initialize weights
        idx = 0
        for pose in self.valid_poses:
            for rot in [0, 90, 180, 270]:
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
        # if max_weight == min_weight:
        #     # print("max == min")
        #     alphas = 0.1 *np.ones_like(self.weights)
        # else:
        #     alphas = (self.weights[:] - min_weight)/(max_weight - min_weight)
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
        cm = plt.cm.get_cmap('RdYlGn')
        fig = plt.figure()
        axs = fig.gca()
        weightMap = [float(i)/np.max(self.weights) for i in self.weights]
        sc = axs.scatter(self.particles[:,0], self.particles[:,1], c=weightMap, cmap=cm, alpha=0.2)
        axs.set_title("{} Distribution @ step {}".format(self.label, fname))
        fig.colorbar(sc)
        # print("Saving Fig {}.png".format(fname))
        # fig.show()
        fig.savefig("/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/distributions/{}_{}.png".format(self.label, fname))
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

    def addObservation(self, interactablePoses):
        for pose in interactablePoses:
            self.observations.append({
                'x': pose['x'],
                'z':pose['z'],
                'yaw':pose['rotation']}
            )
    
    def addNegativePose(self, pose):
        self.negative_poses.append(pose)

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
        for pose in self.negative_poses:
            if (particle[0] == pose['x'] and particle[1] == pose['z']):
                return 0.0
        for interactable_pose in self.observations:
            if (
                particle[0] == interactable_pose["x"]
                and particle[1] == interactable_pose["z"]
                and particle[2] == interactable_pose["yaw"]
            ):
                return 1.0
            elif(
                False # particle close to interactable pose
            ):
                pass
        
        return 0.3

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
        self.frame_element_filters = {frame_element:None for frame_element in core_frame_elements}
        self.frame_elements = core_frame_elements
        if preconditions:
            self.precondition_filters = {precondition:None for precondition in preconditions}
            self.preconditions = preconditions
            self.next_precondition = preconditions[0]
        else:
            self.preconditions = False
        self.controller = controller
    
    def __str__(self):
        string = "Label: {}\n\tnParticles: {}\n\tFrame Elements:\n".format(self.label, self.nParticles)
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
        string+= preconditionString
        return string
    
    def addFrameElementFilter(self, label, filter):
        self.frame_element_filters[label] = filter
    
    def addPreconditionFilter(self, label, filter):
        self.precondition_filters[label] = filter
    
    def contextPotential(self, particle, state):
        i = 0
        if self.preconditions:
            for precondition in self.preconditions:
                if precondition in state.action_history:
                    i += 1
                else:
                    break
            potential = 1.0
            while i < len(self.preconditions):
                subtaskFilter = self.precondition_filters[self.preconditions[i]]
                for j in range(subtaskFilter.nParticles):
                    otherParticle = subtaskFilter.particles[j,:]
                    linearDist = np.sqrt((otherParticle[0]-particle[0])**2 + (otherParticle[1]-otherParticle[1])**2)
                    angularDist = np.abs(particle[2] - otherParticle[2]) % 360
                    if angularDist > 180:
                        angularDist = 360 - angularDist
                    linearStepErr = linearDist % 0.25 # 0.25m per robot step
                    angularStepErr = angularDist % 90 # 90deg per robot step
                    phi = np.exp(-10*(linearStepErr+angularStepErr))
                    potential += phi
                i+=1
            return potential
        else:
            return 1.0
                    
    def measurementPotential(self, particle, state):
        i = 0
        if self.preconditions:
            for precondition in self.preconditons:
                if precondition not in state.action_history:
                    break
                else:
                    i+=1
        potential = 1
        coreElementWeightModifier = 1
        while i < len(self.frame_elements):
            coreElementFilter = self.frame_element_filters[self.frame_elements[i]]
            for j in range(coreElementFilter.nParticles):
                otherParticle = coreElementFilter.particles[j, :]
                linearDist = np.sqrt((otherParticle[0]-particle[0])**2 + (otherParticle[1]-otherParticle[1])**2)
                angularDist = np.abs(particle[2] - otherParticle[2]) % 360
                if angularDist > 180:
                    angularDist = 360 - angularDist
                linearStepErr = linearDist % 0.25 # 0.25m per robot step
                angularStepErr = angularDist % 90 # 90deg per robot step
                phi = np.exp(-10*(linearStepErr + angularStepErr))
                potential += (1/(coreElementWeightModifier**2))*(phi*coreElementFilter.weights[j])
            i+=1
            coreElementWeightModifier+=1
        return potential
                
    def assignWeight(self, particle, state):
        measurement = self.measurementPotential(particle, state)
        context = self.contextPotential(particle, state)
        assert(measurement >= 1.0 and context >= 1.0)
        return measurement*context
    
    def updateFilter(self, state):
        for i in range(self.nParticles):
            weight = self.assignWeight(self.particles[i,:], state)
            self.weights[i] += weight
        self.weights /= np.sum(self.weights)

