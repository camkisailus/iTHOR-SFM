#! /usr/bin/zsh
for chunk_num in 9 10 11 12 13 14 15 16 17 18 19
do 
    /home/cuhsailus/anaconda3/envs/THOR/bin/python /home/cuhsailus/Desktop/Research/22_academic_year/iTHOR-SFM/eval.py --chunk $chunk > pick_and_place_simple_{$chunk_num}_results.txt
done