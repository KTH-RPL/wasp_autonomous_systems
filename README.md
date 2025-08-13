# WASP Autonomous Systems Course

In this course, we will be using [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html). ROS 2 Jazzy works best on [Ubuntu](https://ubuntu.com/) 24.04; therefore, we will be using [Multipass](https://canonical.com/multipass) to create an Ubuntu 24.04 virtual machine (VM). In the VM, you will be running everything ROS-related as well as your own code. We will also utilize the [Webots](https://cyberbotics.com/) simulator, to simulate different robots and sensors. Webots will be installed natively (i.e., on the host and **not** in the VM), for optimal performance, and we will show you how to interface it with the VM. Lastely, to visualize data and configure parameters, we will be using [Foxglove](https://foxglove.dev/), which can be installed natively or be used through a web browser ([requires Chrome v111+](https://docs.foxglove.dev/docs/visualization#system-requirements)).

## Supported Systems

Based on the software needed and their system requirements: [Multipass](https://documentation.ubuntu.com/multipass/en/latest/how-to-guides/install-multipass/), [Webots](https://cyberbotics.com/doc/guide/system-requirements), and [Foxglove](https://docs.foxglove.dev/docs/visualization#system-requirements) (ROS 2 Jazzy is not included here since it will be installed in a VM using Multipass) the list of supported systems are as follows:

- Linux: Only x86-64 systems. Ubuntu 22.04 or 24.04 recommended.
- Mac: Both x86-64 and AArch64 systems. macOS 13 and above.
- Windows: Only x86-64 systems. Windows 10 and 11.

## Virtualization

As we will use a VM, you need to enable virtualization. It is usually done in your systems UEFI, EFI, or BIOS; look for settings labelled _Virtualization_, _Virtualization Technology_, _VT-x_, _VT-d_, _Extended Page Tables_, _EPT_, _Vanderpool_, _AMD-V_, _SVM_, _SVM Mode_, _VMX_, etc.

### Check if Virtualization is Enabled

You can check if virtualization is enabled following the instructions below for your operating system.

#### Linux

---

Run the command:

```sh
lscpu | grep "Virtualization"
```

If you see output similar such as:

```sh
Virtualization: AMD-V
```

on AMD platform or

```sh
Virtualization: VT-x
```

on Intel platform, then virtualization should be enabled.

If the above did not work you can also try:

```sh
grep --color -E "vmx|svm" /proc/cpuinfo
```

if you see output then virtualization should be enabled.

#### Apple silicon Mac

---

Should be enabled.

#### Intel Mac

---

Run the command:

```sh
sysctl -a | grep -o VMX
```

If you see `VMX` in the output then it should be enabled.

#### Windows

---

Run the command:

```sh
systeminfo | findstr /i "virtualization"
```

If you see output such as

```sh
Virtualization Enabled In Firmware: Yes
```

then it should be enabled.

### How to Enable Virtualization

If virtualization is not enabled on your computer, please follow the instructions below for your OS to enable it. You may need to ask IT support at your school to allow you to enable it.

#### Linux

---

It differs depending on your computer. Below we link to some of the most popular brands instructions.

##### Acer

[How to Enable Virtualization Technology on Acer Products](https://community.acer.com/kb/articles/14750)

##### Asus

AMD CPU
[\[Notebook\] How to enable or disable AMD Virtualization (AMD-V™) technology?](https://www.asus.com/support/faq/1043992/)

Intel CPU (laptop) [\[Notebook\] How to enable or disable Intel® Virtualization Technology (VT-x)?](https://www.asus.com/support/faq/1043181/)

Intel CPU (desktop) [\[Motherboard\] How to enable Intel(VMX) Virtualization Technology in the BIOS](https://www.asus.com/support/faq/1043786/)

##### Dell

[How To Enable or Disable Hardware Virtualization on Dell Computers](https://www.dell.com/support/kbdoc/en-uk/000195978/how-to-enable-or-disable-hardware-virtualization-on-dell-systems?lwp=rt)

##### HP

[HP PCs - Enable Virtualization Technology in the BIOS](https://support.hp.com/us-en/document/ish_5637142-5637191-16)

##### Lenovo

[How to enable Virtualization Technology on Lenovo PC computers](https://support.lenovo.com/se/en/solutions/ht500006)

##### Microsoft

Virtualization is already enabled on Surface devices.

##### Other

If your PC manufacturer is not listed, you will need to find the instructions on your own. They should (hopefully be on their website).

#### macOS

---

Could only find information saying it should be enabled. Please let us know if this was not the case for you so we can update this.

#### Windows

---

Please see [Microsoft's instructions](https://support.microsoft.com/en-us/windows/enable-virtualization-on-windows-c5578302-6e43-4b4b-a449-8ced115f58e1).

## Installation

Next, we will go through how to install all the software you need for the course. At the bottom you will find videos where we go through the steps on Linux, Mac, and Windows. If you have any problems during the installation of any of the software or want to follow the official guide, we also include them at the bottom.

### Ubuntu 22.04 and 24.04

Install Multipass:

```sh
snap install multipass
```

Install Webots **R2025a**:

```sh
curl -L -O https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install ./webots_2025a_amd64.deb
```

Install Foxglove:

```sh
curl -L -O https://get.foxglove.dev/desktop/latest/foxglove-studio-latest-linux-amd64.deb
sudo apt install ./foxglove-studio-latest-linux-amd64.deb
```

### Mac

Install Multipass:

1. [Download the Multipass installer](https://canonical.com/multipass/download/macos).
2. Run the downloaded installer and follow the guided procedure.

Install Webots **R2025a**:

```sh
curl -L -O https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a.dmg
open webots-R2025a.dmg
```

Install Foxglove:

1. [Download Foxglove](https://get.foxglove.dev/desktop/latest/foxglove-latest-mac-universal.dmg).
2. Double click to install.

### Windows 10 and 11

Install Multipass:

1. [Download the Multipass installer](https://canonical.com/multipass/download/windows).
2. Run the downloaded installer and follow the guided procedure.

Install Webots **R2025a**:

1. Download [webots_R2025a_setup.exe](https://github.com/cyberbotics/webots/releases/download/R2025a/webots-R2025a_setup.exe).
2. Double click on the file.
3. Follow the installation instructions.

Install Foxglove:

1. [Download Foxglove](https://get.foxglove.dev/desktop/latest/foxglove-latest-win.exe).
2. Double click to install.

### Official Instructions

[Installation instructions for Multipass](https://documentation.ubuntu.com/multipass/en/latest/how-to-guides/install-multipass/).

[Installation instructions for Webots](https://cyberbotics.com/doc/guide/installing-webots).

[Installation instructions for Foxglove](https://foxglove.dev/download).

## Post-Installation

Now that all the software is installed, we need to create and configure the VM. After this step you should be ready to do the assignments.

### Create Ubuntu 24.04 VM with ROS 2 Jazzy and Course Packages (Using Multipass)

We have prepared a [Cloud-init](https://cloud-init.io/) file that will install and setup everything you need inside the VM/instance. To setup the VM/instance you first need to create a shared directory. This directory will be accessable from both the host (your OS) and the guest (the VM/instance). We will use this directory for Webots communication and to share a couple of [Rosbags](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html).

---

**NOTE:** In the VM we have enabled SSH password authentication to make it easier for you to get started. It is not recommended to have password authentication enabled (especially without a root password). If you want to disable SSH password authentication you can change the cloud-init file and set `ssh_pwauth: false` and remove the whole `chpasswd` block. Then after having setup the VM you can add your SSH key (that you need to generate if you have none) by running the command:

```sh
multipass exec wasp -- bash -c "echo `cat <KEY_ON_HOST>.pub` >> ~/.ssh/authorized_keys"
```

---

So fist step, create a directory somewhere on your computer that will be the shared directory. We will refer to this directory (the absolute path to it) as `<SHARED_DIR>`. Second, run the following command to setup the VM and make sure to change the `<SHARED_DIR>` with the absolute path to the directory that you just created:

```sh
multipass launch 24.04 --name wasp --cpus 4 --memory 4G --disk 10G --timeout 1800 --cloud-init https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/refs/heads/ht25/wasp.yaml -v --mount <SHARED_DIR>:/home/ubuntu/shared
```

This will create a VM using Ubuntu 24.04 as base, with name _wasp_, 4 CPUs, 4 GB of memory, 10 GB of disk, maximum initialization time of 30 minutes (1800 seconds), using the Cloud init [wasp.yaml](https://raw.githubusercontent.com/KTH-RPL/wasp_autonomous_systems/refs/heads/ht25/wasp.yaml) file, with more verbose output (can increase the output by changing `-v` to `-vv`, `-vvv`, and `-vvvv` for more verbose output, or remove for minimum output), and mount the host directory `<SHARED_DIR>` (make sure the directory already exists) to the guest (VM/instance) directory `/home/ubuntu/shared`. You can of course change these values. We recommend at least 10 GB of disk.

It is possible to change number of CPU(s), memory, and disk afterwards as well. As well as add or delete mounts.

### Verify the VM Setup is Completed

You can check the state of the VM by running the command:

```sh
multipass info wasp
```

You can check to make sure all ROS packages (last step of the setup) have been built and installed by running the command:

```sh
multipass exec -d ros2_ws/install wasp -- ls
```

if the output is:

```sh
COLCON_IGNORE             local_setup.bash  setup.zsh
_local_setup_util_ps1.py  local_setup.ps1   wasp_autonomous_systems
_local_setup_util_sh.py   local_setup.sh    wasp_autonomous_systems_interfaces
assignment_1              local_setup.zsh   webots_ros2_control
assignment_2              setup.bash        webots_ros2_driver
assignment_3              setup.ps1         webots_ros2_importer
assignment_4              setup.sh          webots_ros2_msgs
```

then everything is as it should and you can continue. Otherwise, you should wait or perhaps something went wrong. The important part here is that you see the four packages `assignment_1`, `assignment_2`, `assignment_3`, and `assignment_4`.

### Make Webots use Shared Directory

To enable ROS inside the VM to communicate with the natively running Webots simulator, we need to tell Webots where the shared directory is located. To do this, run:

```sh
# Create a Webots directory inside the shared directory
multipass exec wasp -- sh -c "mkdir /home/ubuntu/shared/webots"
# Export the directory mapping so Webots know about it
multipass exec wasp -- sh -c "echo 'export WEBOTS_SHARED_FOLDER=<SHARED_DIR>:/home/ubuntu/shared/webots' >> ~/.bashrc"
```

using the same absolute path to the host shared directory `<SHARED_DIR>` as during the installation.

### Restart the VM

During the setup of the VM a lot of packages was installed. To ensure all of them work as expect it is recommended to restart the VM. You can do this by running the command:

```sh
multipass exec wasp -- sudo reboot
```

When it has booted up again, you can run:

```sh
multipass exec wasp -- cloud-init status
```

If everything is as it should, you should see the output:

```sh
status: done
```

## Videos

### Linux

TODO: Video showing the steps using Ubuntu 24.04.

### Mac

TODO: Video showing the steps using macOS 15.

### Windows

TODO: Video showing the steps using Windows 11.

## Tested Platforms

We have tested the instructions and verified acceptable performance on the following platforms:

| Type | OS | Arch | CPU | RAM (GB) | GPU | VM CPU(s) | VM Memory (GB) | VM Disk (GB) |
| -------- | -------- | ------- | ------- | ------- | ------- | ------- | ------- | ------- |
| Desktop | Ubuntu 24.04 | x86-64 | Intel Core i7-8700K | 64 | Nvidia Titan X | 4 | 4  | 10 |
| Laptop | macOS 15.6 | AArch64 | Apple M3 | 16 | Apple M3 | 4 | 4 | 10 |
| Desktop | Ubuntu 24.04 | x86-64 | AMD Ryzen 5 3600X | 32 | Nvidia GeForce GTX 1660 Super | 4 | 4 | 10 |
| Laptop | Windows 10 | x86-64 | Intel Core i7-8550U | 16 | Nvidia GeForce MX150 | 4 | 4 | 10 |
| Laptop | Ubuntu 24.04 | x86-64 | Intel Core i7-8550U | 16 | Nvidia GeForce MX150 | 4 | 4 | 10 |
| Desktop | Windows 11 | x86-64 | Intel Core i7-6700K | 32 | Nvidia GeForce GTX 1650 | 4 | 4 | 10 |
| Laptop | macOS 15.6 | x86-64 | Intel Core i | 16 | Intel Iris Plus Graphics | 4 | 4 | 10 |
