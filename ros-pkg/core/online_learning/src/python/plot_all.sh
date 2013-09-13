#!/bin/bash

for type in car pedestrian bicyclist; do
#    rosrun online_learning plot_perclass_accuracy.py $type
    rosrun online_learning plot_perclass_pr.py $type
done