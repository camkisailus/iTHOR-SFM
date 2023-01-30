import numpy as np
import utils


class SemanticFrameMapping:
    def __init__(self, object_filters, frame_filters, controller):
        self.controller = controller
        self.objectFilters = {
            obj_filter.label: obj_filter for obj_filter in object_filters
        }
        self.frameFilters = {
            frame_filter.label: frame_filter for frame_filter in frame_filters
        }
        print("SFM Filters:")
        for filter in self.frameFilters.values():
            print(filter)
        self.distIdx = 0
    
    def getFrameElements(self):
        return [label for label in self.objectFilters.keys()]

    def saveDistributions(self):
        for filter in self.objectFilters.values():
            filter.saveDistribution(self.distIdx)
        for filter in self.frameFilters.values():
            filter.saveDistribution(self.distIdx)
        self.distIdx += 1

    def displayHighestWeighted(self):
        for filter in self.objectFilters.values():
            print("Filter label: {}".format(filter.label))
            idxs = np.argwhere(filter.weights == np.amax(filter.weights)).flatten().tolist()
            # idxs = np.argmax(filter.weights)
            print(idxs)
            for idx in idxs:
                particle = filter.particles[idx]
                print(
                    "\tPose: ({}, {}, {})...Weight: {:.4f}".format(
                        particle[0], particle[1], particle[2], filter.weights[idx]
                    )
                )
        for filter in self.frameFilters.values():
            print("Filter label: {}".format(filter.label))
            idxs = np.argwhere(filter.weights == np.amax(filter.weights)).flatten().tolist()
            for idx in idxs:
                particle = filter.particles[idx]
                print(
                    "\tPose: ({}, {}, {})...Weight: {:.4f}".format(
                        particle[0], particle[1], particle[2], filter.weights[idx]
                    )
                )
        print("#"*20)
        

    def updateFilters(self, state):
        for filter in self.objectFilters.values():
            filter.updateFilter()
        for filter in self.frameFilters.values():
            filter.updateFilter(state)

    def handleObservation(self, observation):
        objectsSeen = []
        for obj_id, interactable_poses in observation.items():
            try:
                self.objectFilters[utils.cleanObjectID(obj_id)].addObservation(
                    interactable_poses
                )
                objectsSeen.append(utils.cleanObjectID(obj_id))
            except KeyError:
                # object is not being tracked
                pass
        for filter in self.objectFilters.values():
            if filter.label not in objectsSeen:
                filter.addNegativePose(observation["robot_pose"])
