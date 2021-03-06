#!/bin/sh

alias tmux='TERM=xterm-256color /home/$USER/bin/tmux -2'
echo "Check whether session thesis-dev exists"
tmux has-session -t thesis-dev
if [ $? != 0 ]
then
  echo "Session thesis-dev doesn't exist. Creating new one"
  tmux new-session -s thesis-dev -n vim -d
  tmux send-keys -t thesis-dev 'cd ~/src/DCL/BayesNetwork/build' C-m
  tmux send-keys -t thesis-dev 'vim' C-m

  tmux new-window -n console -t thesis-dev
  tmux send-keys -t thesis-dev:2 'cd ~/src/DCL/BayesNetwork' C-m

  tmux new-window -n test -t thesis-dev
  tmux split-window -h
  tmux send-keys -t thesis-dev:3.1 'cd ~/src/DCL/BayesNetwork/build' C-m
  tmux send-keys -t thesis-dev:3.2 'cd ~/src/DCL/BayesNetwork/build/test' C-m
  tmux send-keys -t thesis-dev:3.1 'make test' C-m
  tmux send-keys -t thesis-dev:3.2 'find | grep Test$ | sh' C-m
  tmux send-keys -t thesis-dev:3.2 'find | grep Test$' C-m

  tmux new-window -n discode -t thesis-dev
  tmux send-keys -t thesis-dev:4 'cd ~/src/DCL/BayesNetwork/build' C-m

  tmux new-window -n browse -t thesis-dev
  tmux send-keys -t thesis-dev:5 'cd ~/src/DCL/' C-m

  echo "Select starting window"
  tmux select-window -t thesis-dev:1
fi
echo "Attaching to thesis-dev session..."
tmux attach -t thesis-dev
