# haptic_device_niching

This is a tutorial for Haptic device connect to Laptop.

## Setup

Your will need to install git, docker, and some depandency

### Install Git
``` bash
sudo apt-get install git
```
### Install Docker
``` bash
sudo apt-get update
sudo apt-get install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo apt-key fingerprint 0EBFCD88
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
sudo apt-get update
sudo apt-get install docker-ce
```
*** Run Docker without sudo ***
``` bash
sudo -i
sudo usermod -aG docker $USER
```

### Download
``` bash
# setup user and email
git config --globle user.email "yourmail@gmail.com"
git config --globle user.name "yourname"
docker login # login with our email and name
# download
docker pull nichinglin/haptic_device_niching
git clone https://github.com/nichinglin/haptic_device_niching.git
```

## How To Run
``` bash
# open bluetooth and connect to device
sudo service bluetooth start
sudo bluetoothctl
power on
agent on
default-agent
scan on
...
scan off
# find device bluetooth mac address (I use 20:15:04:30:61:95)
pair 20:15:04:30:61:95
# [agent] Enter PIN code: 1234
connect 20:15:04:30:61:95
exit

cd ~/haptic_device_niching/python
python KeyBinder.py -a [device_id]
# example: python KeyBinder.py -a 9
# press 'q' to quit KeyBinder.py
```
(add same to any terminal except the first docker terminal)

## Others

### How to build your Docker
``` bash
cd ~/haptic_device_niching/docker/
docker build --rm -t [your_docker_image_name] . --no-cache
# example:
# docker build --rm -t nichinglin/haptic_device_niching:laptop . --no-cache
```
### How to Push to your docker hub
``` bash
docker tag [image] [dockerhub_name]/[image]:[tag]
docker push [dockerhub_name]/[image]:[tag]
```
