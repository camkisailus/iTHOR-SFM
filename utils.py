import yaml
import os

ROOT = os.path.dirname(os.path.realpath(__file__))


class Frame:
    def __init__(self, info_dict):
        self.name = info_dict["name"]
        self.description = info_dict["description"]
        self.core_frame_elements = []
        self.optional_frame_elements = []
        for frame_element in info_dict["frame_elements"]:
            if frame_element["is_core"]:
                self.core_frame_elements.append(frame_element["name"])
            else:
                self.optional_frame_elements.append(frame_element["name"])
        self.preconditions = []
        try:
            preconditions = info_dict["preconditions"]
            for precondition in preconditions:
                self.preconditions.append(precondition["name"])
        except KeyError:
            pass

    def __str__(self):
        string = "Label: {}\n\tFrame Elements:\n".format(self.name)
        frameElemString = ""
        for frame_elem in self.core_frame_elements:
            frameElemString += "\t\t{}\n".format(frame_elem)
        string += frameElemString
        string += "\tPreconditions:\n"
        preconditionString = ""
        if self.preconditions:
            for precondtion in self.preconditions:
                preconditionString += "\t\t{}\n".format(precondtion)
        else:
            preconditionString = "\t\tNone"
        string += preconditionString
        return string


def cleanObjectID(objectID):
    if "Sliced" in objectID:
        # form of objectID for sliced objects is: Apple|-01.75|+01.37|-01.16|AppleSliced_0
        return objectID.split("|")[-1].split("_")[0]
    else:
        # form of objectID for not sliced objects is: Apple|-01.75|+01.37|-01.16
        return objectID.split("|")[0]


def loadExperimentConfig(config_path):
    frames = []
    actions = []
    with open(config_path, "r") as stream:
        config = yaml.safe_load(stream)
        floorPlan = config["floor_plan"]
        for action in config["execute"]:
            actions.append(action["name"])
        for i, frame in enumerate(config["frames"]):
            frame_name = frame["name"].lower()
            frame_path = os.path.join(ROOT, "frames", frame_name + "_sf.yaml")
            with open(frame_path, "r") as fstream:
                frames.append(Frame(yaml.safe_load(fstream)))
    objects = set()
    for frame in frames:
        for frameElement in frame.core_frame_elements:
            objects.add(frameElement)
    return floorPlan, frames, objects, actions
