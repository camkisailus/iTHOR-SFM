import yaml
import os

ROOT = os.path.dirname(os.path.realpath(__file__))
OPENABLE_RECEPS = ["Safe", "Cabinet", "Fridge", "Microwave", "GarbageCan", "Drawer"]


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
    if "Sliced" in objectID or "Basin" in objectID:
        # form of objectID for sliced objects is: Apple|-01.75|+01.37|-01.16|AppleSliced_0
        return objectID.split("|")[-1].split("_")[0]
    else:
        # form of objectID for not sliced objects is: Apple|-01.75|+01.37|-01.16
        return objectID.split("|")[0]


def loadFramesFromALFRED(goal_description):
    # print("Goal Desc: {}".format(goal_description))
    # print("pick_and_place_with_moveable_recep" in goal_description)
    frames = []
    objects = set()
    if "pick_and_place_with_movable_recep" in goal_description:
        # form is pick_and_place_with_moveable_recep-{obj}-{mrecep}-{target}-floorPlan
        # Frames needed:
        #   grasp_obj
        #   put_obj_on_mrecep
        #   put_mrecep_on_target
        slice = False
        obj = goal_description.split("-")[1]
        if "Sliced" in obj:
            slice = True
            grasp_knife_frame = {
                "name": "Grasp_Knife",
                "description": "Grasp a knife",
                "frame_elements": [{"name": "Knife", "is_core": True}],
            }
            slice_obj_frame = {
                "name": "Slice_{}".format(obj.replace("Sliced", "")),
                "description": "Slice a {} with a Knife",
                "frame_elements": [
                    {"name": "Knife", "is_core": True},
                    {"name": obj.replace("Sliced", ""), "is_core": True},
                ],
                "preconditions": [{"name": "Grasp_Knife"}],
            }
            frames.append(Frame(grasp_knife_frame))
            frames.append(Frame(slice_obj_frame))
        mrecep = goal_description.split("-")[2]
        target = goal_description.split("-")[3]
        grasp_obj_frame = {
            "name": "Grasp_{}".format(obj),
            "description": "Grasp a {}".format(obj),
            "frame_elements": [{"name": obj, "is_core": True}],
        }
        frames.append(Frame(grasp_obj_frame))

        grasp_mrecep_frame = {
            "name": "Grasp_{}".format(mrecep),
            "description": "Grasp a {}".format(mrecep),
            "frame_elements": [{"name": mrecep, "is_core": True}],
        }
        frames.append(Frame(grasp_mrecep_frame))

        put_obj_on_mrecep_frame = {
            "name": "Put_{}_on_{}".format(obj, mrecep),
            "description": "Put a {} on a {}".format(obj, mrecep),
            "frame_elements": [
                {"name": obj, "is_core": True},
                {"name": mrecep, "is_core": True},
            ],
            "preconditions": [{"name": "Grasp_{}".format(obj)}],
        }
        frames.append(Frame(put_obj_on_mrecep_frame))
        if target in OPENABLE_RECEPS:
            open_target_frame = {
                "name": "Open_{}".format(target),
                "description": "Open a {}".format(target),
                "frame_elements": [{"name": target, "is_core": True}],
            }
            frames.append(Frame(open_target_frame))
            if slice:
                put_mrecep_on_target_frame = {
                    "name": "Put_{}_with_{}_on_{}".format(mrecep, obj, target),
                    "description": "Put a {} with a {} on/in a {}".format(mrecep, obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": mrecep, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Slice_{}".format(obj.replace("Sliced", ""))},
                        {"name": "Open_{}".format(target)},
                        {"name": "Put_{}_on_{}".format(obj, mrecep)},
                        {"name": "Grasp_{}".format(mrecep)},
                    ],
                }

            else:
                put_mrecep_on_target_frame = {
                    "name": "Put_{}_with_{}_on_{}".format(mrecep, obj, target),
                    "description": "Put a {} with a {} on/in a {}".format(mrecep, obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": mrecep, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Open_{}".format(target)},
                        {"name": "Put_{}_on_{}".format(obj, mrecep)},
                        {"name": "Grasp_{}".format(mrecep)},
                    ],
                }
        
        else:
            if slice:
                put_mrecep_on_target_frame = {
                    "name": "Put_{}_with_{}_on_{}".format(mrecep, obj, target),
                    "description": "Put a {} with a {} on/in a {}".format(mrecep, obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": mrecep, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Slice_{}".format(obj.replace("Sliced", ""))},
                        {"name": "Put_{}_on_{}".format(obj, mrecep)},
                        {"name": "Grasp_{}".format(mrecep)},
                    ],
                }

            else:
                put_mrecep_on_target_frame = {
                    "name": "Put_{}_with_{}_on_{}".format(mrecep, obj, target),
                    "description": "Put a {} with a {} on/in a {}".format(mrecep, obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": mrecep, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Put_{}_on_{}".format(obj, mrecep)},
                        {"name": "Grasp_{}".format(mrecep)},
                    ],
                }
        frames.append(Frame(put_mrecep_on_target_frame))

        for frame in frames:
            for frameElement in frame.core_frame_elements:
                objects.add(frameElement)
    elif "look_at_obj_in_light" in goal_description:
        # form is look_at_obj_in_light-{obj}-None-{target}-{FloorPlan}
        # Grasp obj
        obj = goal_description.split("-")[1]
        if "Sliced" in obj:
            raise TypeError("This is not setup to deal with sliced objects yet")
        grasp_obj_frame = {
            "name": "Grasp_{}".format(obj),
            "description": "Grasp a {}".format(obj),
            "frame_elements": [{"name": obj, "is_core": True}],
        }
        frames.append(Frame(grasp_obj_frame))
        target = goal_description.split("-")[3]
        look_at_obj_under_target_frame = {
            "name": "Look_at_{}_under_{}".format(obj, target),
            "description": "Look at a {} under the {}".format(obj, target),
            "frame_elements": [
                {"name": obj, "is_core": True},
                {"name": target, "is_core": True},
            ],
            "preconditions": [{"name": "Grasp_{}".format(obj)}],
        }
        frames.append(Frame(look_at_obj_under_target_frame))
        for frame in frames:
            for frameElement in frame.core_frame_elements:
                objects.add(frameElement)
    elif "pick_and_place_simple" in goal_description:
        # form is pick_and_place_simple-{obj}-None-{target}-{floorPlan}
        obj = goal_description.split("-")[1]
        slice = False
        if "Sliced" in obj:
            slice = True
            grasp_knife_frame = {
                "name": "Grasp_Knife",
                "description": "Grasp a knife",
                "frame_elements": [{"name": "Knife", "is_core": True}],
            }
            slice_obj_frame = {
                "name": "Slice_{}".format(obj.replace("Sliced", "")),
                "description": "Slice a {} with a Knife",
                "frame_elements": [
                    {"name": "Knife", "is_core": True},
                    {"name": obj.replace("Sliced", ""), "is_core": True},
                ],
                "preconditions": [{"name": "Grasp_Knife"}],
            }
            frames.append(Frame(grasp_knife_frame))
            frames.append(Frame(slice_obj_frame))
        # elif "Knife" in obj:
        #     obj = "*Knife"
        target = goal_description.split("-")[3]
        # if target == "BathtubBasin":
        #     target = "Bathtub"
        # if target == "SinkBasin":
        #     target= "Sink"
        if target in OPENABLE_RECEPS:
            # We will need to open it first to place the object
            open_target_frame = {
                "name": "Open_{}".format(target),
                "description": "Open a {}".format(target),
                "frame_elements": [{"name": target, "is_core": True}],
            }
            frames.append(Frame(open_target_frame))
            if slice:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
        else:
            if slice:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
        grasp_obj_frame = {
            "name": "Grasp_{}".format(obj),
            "description": "Grasp a {}".format(obj),
            "frame_elements": [{"name": obj, "is_core": True}],
        }
        frames.append(Frame(grasp_obj_frame))
        frames.append(Frame(put_obj_on_target_frame))
        for frame in frames:
            for frameElement in frame.core_frame_elements:
                objects.add(frameElement)
    elif "pick_heat" in goal_description:
        # form is pick_heat_then_place_in_recep-{obj}-None-{target}-11
        obj = goal_description.split("-")[1]
        target = goal_description.split("-")[3]
        # if target == "BathtubBasin":
        #     target = "Bathtub"
        # if target == "SinkBasin":
        #     target = "Sink"
        slice = False
        if "Sliced" in obj:
            slice = True
            grasp_knife_frame = {
                "name": "Grasp_Knife",
                "description": "Grasp a knife",
                "frame_elements": [{"name": "Knife", "is_core": True}],
            }
            slice_obj_frame = {
                "name": "Slice_{}".format(obj.replace("Sliced", "")),
                "description": "Slice a {} with a Knife",
                "frame_elements": [
                    {"name": "Knife", "is_core": True},
                    {"name": obj.replace("Sliced", ""), "is_core": True},
                ],
                "preconditions": [{"name": "Grasp_Knife"}],
            }
            frames.append(Frame(grasp_knife_frame))
            frames.append(Frame(slice_obj_frame))
        open_microwave_frame = {
            "name": "Open_Microwave",
            "description": "Open a Microwave",
            "frame_elements": [{"name": "Microwave", "is_core": True}],
        }
        frames.append(Frame(open_microwave_frame))
        if slice:
            grasp_obj_frame = {
                "name": "Grasp_{}".format(obj),
                "description": "Grasp a {}".format(obj),
                "frame_elements": [{"name": obj, "is_core": True}],
            }
            frames.append(Frame(grasp_obj_frame))
            heat_obj_frame = {
                "name": "Heat_{}".format(obj),
                "description": "Heat {} in a microwave".format(obj),
                "frame_elements": [
                    {"name": obj, "is_core": True},
                    {"name": "Microwave", "is_core": True},
                ],
                "preconditions": [
                    {"name": "Slice_{}".format(obj.replace("Sliced", ""))},
                    {"name": "Open_Microwave"},
                    {"name": "Grasp_{}".format(obj)},
                ],
            }
            frames.append(Frame(heat_obj_frame))
            if target in OPENABLE_RECEPS:
                open_target_frame = {
                "name": "Open_{}".format(target),
                "description": "Open a {}".format(target),
                "frame_elements": [{"name": target, "is_core": True}],
                }
                frames.append(Frame(open_target_frame))
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }

        else:
            grasp_obj_frame = {
                "name": "Grasp_{}".format(obj),
                "description": "Grasp a {}".format(obj),
                "frame_elements": [{"name": obj, "is_core": True}],
            }
            frames.append(Frame(grasp_obj_frame))
            heat_obj_frame = {
                "name": "Heat_{}".format(obj),
                "description": "Heat {} in a microwave".format(obj),
                "frame_elements": [
                    {"name": obj, "is_core": True},
                    {"name": "Microwave", "is_core": True},
                ],
                "preconditions": [
                    {"name": "Open_Microwave"},
                    {"name": "Grasp_{}".format(obj)},
                ],
            }
            frames.append(Frame(heat_obj_frame))
            if target in OPENABLE_RECEPS:
                open_target_frame = {
                "name": "Open_{}".format(target),
                "description": "Open a {}".format(target),
                "frame_elements": [{"name": target, "is_core": True}],
                }
                frames.append(Frame(open_target_frame))
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }
        frames.append(Frame(put_obj_on_target_frame))
        for frame in frames:
            for frameElement in frame.core_frame_elements:
                objects.add(frameElement)

    elif "pick_clean" in goal_description:

        # grasp obj, put(obj, sink), turnOn(faucet), turnOff(faucet)
        obj = goal_description.split("-")[1]
        target = goal_description.split("-")[3]
        # if target == "BathtubBasin":
        #     target = "Bathtub"
        # if target == "SinkBasin":
        #     target = "Sink"
        slice = False
        if "Sliced" in obj:
            slice = True
            grasp_knife_frame = {
                "name": "Grasp_Knife",
                "description": "Grasp a knife",
                "frame_elements": [{"name": "Knife", "is_core": True}],
            }
            slice_obj_frame = {
                "name": "Slice_{}".format(obj.replace("Sliced", "")),
                "description": "Slice a {} with a Knife",
                "frame_elements": [
                    {"name": "Knife", "is_core": True},
                    {"name": obj.replace("Sliced", ""), "is_core": True},
                ],
                "preconditions": [{"name": "Grasp_Knife"}],
            }
            frames.append(Frame(grasp_knife_frame))
            frames.append(Frame(slice_obj_frame))
        if slice:
            grasp_obj_frame = {
                "name": "Grasp_{}".format(obj),
                "description": "Grasp a {}".format(obj),
                "frame_elements": [{"name": obj, "is_core": True}],
            }
            frames.append(Frame(grasp_obj_frame))
            clean_obj_frame = {
                "name": "Clean_{}".format(obj),
                "description": "Clean {} in the sink".format(obj),
                "frame_elements": [
                    {"name": obj, "is_core": True},
                    {"name": "SinkBasin", "is_core": True},
                ],
                "preconditions": [{"name":"Slice_{}".format(obj.replace("Sliced", ""))},{"name": "Grasp_{}".format(obj)}],
            }
            frames.append(Frame(clean_obj_frame))
            if target in OPENABLE_RECEPS:
                open_target_frame = {
                    "name": "Open_{}".format(target),
                    "description": "Open a {}".format(target),
                    "frame_elements": [{"name": target, "is_core": True}],
                }
                frames.append(Frame(open_target_frame))
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }

            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [                        
                        {
                            "name": "Slice_{}".format(obj.replace("Sliced", ""))
                        },
                        {"name": "Grasp_{}".format(obj)}],
                }
            frames.append(Frame(put_obj_on_target_frame))

        else:
            grasp_obj_frame = {
                "name": "Grasp_{}".format(obj),
                "description": "Grasp a {}".format(obj),
                "frame_elements": [{"name": obj, "is_core": True}],
            }
            frames.append(Frame(grasp_obj_frame))
            clean_obj_frame = {
                "name": "Clean_{}".format(obj),
                "description": "Clean {} in the sink".format(obj),
                "frame_elements": [
                    {"name": obj, "is_core": True},
                    {"name": "SinkBasin", "is_core": True},
                ],
                "preconditions": [{"name": "Grasp_{}".format(obj)}],
            }
            frames.append(Frame(clean_obj_frame))
            if target in OPENABLE_RECEPS:
                open_target_frame = {
                    "name": "Open_{}".format(target),
                    "description": "Open a {}".format(target),
                    "frame_elements": [{"name": target, "is_core": True}],
                }
                frames.append(Frame(open_target_frame))
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [
                        {"name": "Open_{}".format(target)},
                        {"name": "Grasp_{}".format(obj)},
                    ],
                }

            else:
                put_obj_on_target_frame = {
                    "name": "Put_{}_on_{}".format(obj, target),
                    "description": "Put a {} on a {}".format(obj, target),
                    "frame_elements": [
                        {"name": obj, "is_core": True},
                        {"name": target, "is_core": True},
                    ],
                    "preconditions": [{"name": "Grasp_{}".format(obj)}],
                }
            frames.append(Frame(put_obj_on_target_frame))
        for frame in frames:
            for frameElement in frame.core_frame_elements:
                objects.add(frameElement)

    return frames, objects


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
