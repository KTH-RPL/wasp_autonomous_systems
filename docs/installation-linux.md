## Install a C/C++ compiler
`pixi run build` needs a system C/C++ toolchain - Webots' vendored controller library
Makefile calls `gcc`/`make` directly rather than the conda-provided compiler. On a
minimal/fresh install this often isn't there yet:
```
sudo apt install build-essential
```

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

On a minimal install (which doesn't pull in desktop-environment audio packages the way
a full Ubuntu desktop install does), Webots may fail to start with:
```
error while loading shared libraries: libsndio.so.7: cannot open shared object file
```
Fix:
```
sudo apt install libsndio7.0
```

## TLS certificate errors during `pixi install`/`pixi run build`
On some machines (seen on a managed/institutional Linux install) Pixi fails with an
error like `invalid peer certificate: UnknownIssuer` when downloading package data.
This means Pixi could not find your system's CA certificate bundle. Fix, run once
before retrying the command that failed:
```
export SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt
```
This only needs to be set for commands that actually talk to the network (an initial
`pixi run build`/`pixi install`, or a later `pixi update`) - once packages are
downloaded, everyday `pixi run <task>` commands just use what's already installed and
don't need it. If you find yourself needing it repeatedly, add the line to your
`~/.bashrc` (or equivalent) instead of retyping it each time.

