import os
import pandas 

filepath = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/pick_and_place_simple.txt"
root = "/home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/"
def split(list, chunksize=20):
    for i in range(0, len(list), chunksize):
        yield list[i:i+chunksize]
def chunkExperiments():
    with open(filepath) as f:
        lines = f.read().splitlines()
        chunks = list(split(lines))
        for i,chunk in enumerate(chunks):
            writeFile = os.path.join(root, "pick_and_place_simple_chunk_{}.txt".format(i))
            with open(writeFile, "w+") as wf:
                for line in chunk:
                    wf.write(line+"\n")
def createSummary():
    data = {}
    for i in range(20):
        fname = "pick_and_place_simple_{}_results.txt".format(i)
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
    df.to_csv(os.path.join(root, "pick_and_place_simple_summary.csv"), sep=",")
    print(df)

createSummary()
