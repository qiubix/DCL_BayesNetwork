#!/bin/sh

tmux has-session -t thesis-dev
if [ $?  != 0 ]
then
  tmux new-session -s thesis-dev -n vim -d
  tmux send-keys -t thesis-dev 'cd ~/DCL/BayesNetwork/build' C-m
  tmux send-keys -t thesis-dev 'vim' C-m
  tmux new-window -n console -t thesis-dev
  tmux send-keys -t thesis-dev:2 'cd ~/DCL/BayesNetwork' C-m
  tmux new-window -n discode -t thesis-dev
  tmux send-keys -t thesis-dev:3 'cd ~/DCL/BayesNetwork/build' C-m
  tmux new-window -n browse -t thesis-dev
  tmux send-keys -t thesis-dev:4 'cd ~/DCL/' C-m
  tmux select-window -t thesis-dev:1
fi
#tmux -2 attach -t thesis-dev
