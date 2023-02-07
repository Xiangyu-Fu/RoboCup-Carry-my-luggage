#!/bin/bash
echo "Now starting the human_following.launch script"
source ~/venv/bin/activate
roslaunch hsrb_human_following human_following.launch allow_growth:=true