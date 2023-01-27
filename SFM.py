import numpy as np
import utils

class SemanticFrameMapping:
    def __init__(self, object_filters, frame_filters, controller):
        self.controller = controller
        self.objectFilters = {
            obj_filter.label: obj_filter for obj_filter in object_filters
        }
        self.frameFitlers = {
            frame_filter.label: frame_filter for frame_filter in frame_filters
        }
        print("SFM Filters:")
        for filter in self.frameFitlers.values():
            print(filter)
        self.distIdx = 0
    
    def saveDistributions(self):
        for filter in self.objectFilters.values():
            filter.saveDistribution(self.distIdx)
        self.distIdx += 1
        # for filt

    def updateFilters(self, state):
        for filter in self.objectFilters.values():
            filter.updateFilter()
        # for filter in self.frameFitlers.values():
        #     filter.updateFilter(state)

    def handleObservation(self, observation):
        objectsSeen = []
        for obj_id, interactable_poses in observation.items():
            try:
                self.objectFilters[utils.cleanObjectID(obj_id)].addObservation(interactable_poses)
                objectsSeen.append(utils.cleanObjectID(obj_id))
            except KeyError:
                # object is not being tracked
                pass
        for filter in self.objectFilters.values():
            if filter.label not in objectsSeen:
                filter.addNegativePose(observation['robot_pose'])
        
