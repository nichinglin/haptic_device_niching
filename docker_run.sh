#!/usr/bin/env bash
export repo=haptic_device_niching

if [ "$1" == "same" ]; then
  docker exec -it unity_ros bash
else
  docker run --name unity_ros --rm -it --net=host --privileged \
             -v /dev:/dev \
             -v /etc/localtime:/etc/localtime:ro \
             -v /var/run/docker.sock:/var/run/docker.sock \
             -v /home/$USER/$repo:/root/$repo \
             -v /home/$USER/.bashrc:/root/.bashrc \
             $repo:laptop
fi
