#!/bin/bash

# I have four servos driving four 'legs' on a robot.  This script makes
# the legs go though something like a walking motion.  It actually just
# skids back and fore on the spot at the moment, becuase it needs knees
# and lower legs before it can really lift up its feet and take a real
# step forward.  This is just a quick demo though..

ss()
{
  echo $1 > /dev/servoblaster
}

p[0]=132
p[1]=164
p[2]=150
p[3]=170

step=2
cnt=0
while true; do
  for i in 0 1 2 3; do
    ss $i=${p[$i]}
    p[$i]=$[ p[$i] + step ]
    sleep 0.05
  done
  cnt=$[ cnt + step ]
  if [ $step -gt 0 ]; then
    if [ $cnt -ge 20 ]; then
      step=-2
    fi
  else
    if [ $cnt -lt -20 ]; then
      step=2
    fi
  fi
done
