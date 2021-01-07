# Supaero Robotics, software set up

This setup is provided as a [Docker](https://www.docker.com/) image and a [VirtualBox](https://www.virtualbox.org/)
image. We expect better performances with Docker, but VirtualBox is easier to use and more bullet-proof.

## Setting up with Docker

1. Install docker engine [https://docs.docker.com](https://docs.docker.com/engine/install/ubuntu/#installation-methods) 
1. run
```bash
docker run --rm -p 7000:7000 -p 8888:8888 \
-v summer-school:/home/student \
-it memoryofmotion/summer-school
```
1. Several links are produced by the terminal. Open the one that starts with `127.0.0.1` in your web browser.
1. An minimal example is produced in ~/hello_talos.ipynb: try it to check your install and say hi to Talos!
1. Notebooks for the tutorials are in ~/supaero2021 .


### Update the notebooks

The VirtualBox or Docker images might not contain the latest version of the notebooks so please pull the latest modifications before the tutorials. To do that :
1. run
```bash
docker run --rm -v summer-school:/home/student \
-it memoryofmotion/summer-school \
git -C summer-school pull --rebase \
--recurse-submodules --ff-only
```

## Setting up with VirtualBox

1. Download the virtual machine using [https://frama.link/supaero2021-vbox](this link).
1. If you don't already have a virtualization system, download [https://www.virtualbox.org/](https://www.virtualbox.org/).
1. Unzip and load the virtual machine into your virtualization system (the virtual machine is in an open format and should be read by any kind of virtualization system).
1. Start the VirtualBox Image. If you start the VirtualBox for the first time, everything should be already running with the test notebook ~/summer-school/setup/load_talos.ipynb. Say Hi to Talos!
1. Otherwise, you can start by running ```jupyter notebook``` in ~/supaero2021 . Notebooks for the tutorials are in this folder.

### Update the notebooks

The VirtualBox or Docker images might not contain the latest version of the notebooks.
At the begining of each class, just pull the latest modifications before the tutorials. To do that :
1. If running in one of the terminals, kill jupyter notebook using ctrl+C.
1. Pull changes from the repository:
```bash
cd ~/supaero2021
git pull
git submodule update --init
```
1. restart ```jupyter notebook```


## Native installation

*If you feel adventurous, you can also try to setup the softwares directly on your computer. But you're on your own! Due
to limited time, no support will be provided by the Summer School. You can ask for help in the project's issue trackers,
but without time guarantees. On Ubuntu 18.04, running setup.sh as root should do most of the work, though.*

The following dependencies are needed:

- To be install using APT on Ubuntu 18.04 or follow [https://stack-of-tasks.github.io/pinocchio/download.html](Pinocchio documentation).

- You also need a bunch of other smaller packages around pinocchio. Once pinocchio is there, apt robotpkg-py36-pinocchio robotpkg-py36-qt4-gepetto-viewer-corba robotpkg-example-robot-data robotpkg-osg-dae robotpkg-py35-example-robot-data

- Finally, we need some standard Python packages (also on apt): ipython3 python3-pip freeglut3 python3-matplotlib python3-pil.imagetk jupyter python3-scipy python3-matplotlib jupyter python3-scipy python3-numpy 
