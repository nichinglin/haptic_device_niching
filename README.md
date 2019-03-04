# haptic_device_niching

This is a tutorial for Haptic device connect to Laptop.

## Setup

Your will need to install git, docker, and some depandency

### Install Git
```
sudo apt-get install git
```
### Install Docker
```
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install docker-ce
```
*** Run Docker without sudo ***
```
sudo -i
sudo usermod -aG docker $USER
```

### Download
```
# setup user and email
git config --globle user.email "yourmail@gmail.com"
git config --globle user.name "yourname"
docker login # login with our email and name
# download
docker pull nichinglin/haptic_device_niching
git clone https://github.com/nichinglin/haptic_device_niching.git
```

## How To Run
```
# terminal 1
source ~/haptic_device_niching/docker_run.sh
source ~/haptic_device_niching/unity.sh [master_ip]
roslaunch rosbridge_server rosbridge_websocket.launch
# terminal 2
source ~/haptic_device_niching/docker_run.sh same
source ~/haptic_device_niching/unity.sh [master_ip]
rostopic list
```
(add same to any terminal except the first docker terminal)

## Others

### How to build your Docker
```
cd ~/haptic_device_niching/docker/
docker build --rm -t [your_docker_image_name] . --no-cache
# example:
# docker build --rm -t haptic_device_niching:laptop . --no-cache
```
### How to Push to your docker hub
```
docker tag [image] [dockerhub_name]/[image]:[tag]
docker push [dockerhub_name]/[image]:[tag]
```
