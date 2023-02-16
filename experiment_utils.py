import os
import glob
import pandas 

filepath = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/experiment_configs/pp_simple_experiments.txt"
root = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/"
def split(list, chunksize=20):
    for i in range(0, len(list), chunksize):
        yield list[i:i+chunksize]
def chunkExperiments():
    with open(filepath) as f:
        lines = f.read().splitlines()
        chunks = list(split(lines))
        for i,chunk in enumerate(chunks):
            writeFile = os.path.join(root, "pp_simple_chunk_{}.txt".format(i))
            with open(writeFile, "w+") as wf:
                for line in chunk:
                    wf.write(line+"\n")

def grabConfigs():
    pick_heat_path = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_heat_*/*/pp/*.json"
    files = glob.glob(pick_heat_path)
    pick_heat_seen = set()
    pick_heat_files = []
    for file in files:
        trial = file.split("/")[10]
        if trial not in pick_heat_seen:
            pick_heat_seen.add(trial)
            pick_heat_files.append(file)
    print("{} files in pick_heat".format(len(pick_heat_files)))
    with open(os.path.join(root, "pp_heat_experiments.txt"), "w+") as wf:
        for file in pick_heat_files:
            wf.write(file+"\n")
    
    pick_clean_path = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_clean_*/*/pp/*.json"
    files = glob.glob(pick_clean_path)
    pick_clean_files = []
    pick_clean_seen = set()
    for file in files:
        trial = file.split("/")[10]
        if trial not in pick_clean_seen:
            pick_clean_seen.add(trial)
            pick_clean_files.append(file)
    print("{} files in pick_clean".format(len(pick_clean_files)))
    with open(os.path.join(root, "pp_clean_experiments.txt"), "w+") as wf:
        for file in pick_clean_files:
            wf.write(file+"\n")
    
    pick_w_move_path = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_with_movable_*/*/pp/*.json"
    files = glob.glob(pick_w_move_path)
    pick_w_move_seen = set()
    pick_w_move_files = []
    for file in files:
        trial = file.split("/")[10]
        if trial not in pick_w_move_seen:
            pick_w_move_seen.add(trial)
            pick_w_move_files.append(file)
    print("{} files in pick_w_movable".format(len(pick_w_move_files)))
    with open(os.path.join(root, "pp_move_experiments.txt"), "w+") as wf:
        for file in pick_w_move_files:
            wf.write(file+"\n")
    
    pick_simple_path = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/pick_and_place_simple*/*/pp/*.json"
    files = glob.glob(pick_simple_path)
    pick_simple_seen = set()
    pick_simple_files = []
    for file in files:
        trial = file.split("/")[10]
        if trial not in pick_simple_seen:
            pick_simple_seen.add(trial)
            pick_simple_files.append(file)
    print("{} files in pick_simple".format(len(pick_simple_files)))
    with open(os.path.join(root, "pp_simple_experiments.txt"), "w+") as wf:
        for file in pick_simple_files:
            wf.write(file+"\n")

    look_at_path = "/home/cuhsailus/Desktop/Research/22_academic_year/alfred/data/json_2.1.0_copy/look_at_obj_*/*/pp/*.json"
    files = glob.glob(look_at_path)
    look_at_seen = set()
    look_at_files = []
    for file in files:
        trial = file.split("/")[10]
        if trial not in look_at_seen:
            look_at_seen.add(trial)
            look_at_files.append(file)
    print("{} files in look_at".format(len(look_at_files)))
    with open(os.path.join(root, "look_at_experiments.txt"), "w+") as wf:
        for file in look_at_files:
            wf.write(file+"\n")
def createSummary():
    data = {}
    for i in range(40):
        fname = "look_at_results_{}.txt".format(i)
        print("Loading {}".format(fname))
        fpath = os.path.join(root, fname)
        # if not os.path.exists(fpath):
        #     break
        with open(fpath) as f:
            lines = f.read().splitlines()
            print("{} lines found".format(len(lines)))
            for line in lines:
                full_trial_name = line.split(" ")[1]
                result = line.split(" ")[2]
                try:
                    data['trial_names'].append(full_trial_name)
                    data['result'].append(result)
                except KeyError:
                    data['trial_names'] = [full_trial_name]
                    data['result'] = [result]
                # print("{}: {}".format(full_trial_name, result))
    df = pandas.DataFrame(data=data)
    df.to_csv(os.path.join(root, "look_at_full_results_summary.csv"), sep=",")
    # print(df)
chunkExperiments()
# createSummary()