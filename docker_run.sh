#!/usr/bin/env bash
export repo=haptic_device_niching

if [ "$1" == "same" ]; then
  docker exec -it $repo bash
else
  docker run --name $repo --rm -it --net=host --privileged \
             -v /dev:/dev \
             -v /etc/localtime:/etc/localtime:ro \
             -v /var/run/docker.sock:/var/run/docker.sock \
             -v /home/$USER/$repo:/root/$repo \
             -v /home/$USER/.bashrc:/root/.bashrc \
             nichinglin/$repo:laptop
fi

