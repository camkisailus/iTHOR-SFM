#! /usr/bin/zsh
chunk="0"
# for chunk_num in 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29
while [ $chunk -lt 1 ]
do 
    echo "Executing chunk:" $chunk "at" $(date +%H:%M:%S)
    /home/cuhsailus/anaconda3/envs/THOR/bin/python /home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/eval.py --chunk $chunk 
    echo "Done with chunk:" $chunk "at" $(date +%H:%M:%S)
    chunk=$[ $chunk + 1 ]
done