#!/bin/bash
for chunk_num in 9 10 11 12 13 14 15 16 17 18 19
do 
    python3 eval.py --chunk $chunk_num > pick_and_place_simple_{$chunk_num}_results.txt
done