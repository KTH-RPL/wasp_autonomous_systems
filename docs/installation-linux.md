## Install Pixi 
You find full instructions [here](https://pixi.prefix.dev/latest/installation/).
Run the following in a terminal
```
curl -fsSL https://pixi.sh/install.sh | sh
```
You might want to set up [autocomplete](https://pixi.prefix.dev/latest/installation/#autocompletion) smoother operation of Pixi.

**Note:** Restart the terminal 

## Install Webots R2025a 
```
curl -L -O https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt install ./webots_2025a_amd64.deb
```
Do not worry if you see the message:
```
N: Download is performed unsandboxed as root file 
'<SOMETHING>/webots_2025a_amd64.deb' couldn't be 
accessed by user '_apt'. - pkgAcquire::Run
(13: Permission denied)
```
that is expected.

