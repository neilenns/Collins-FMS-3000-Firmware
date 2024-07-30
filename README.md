# Collins FMS 3000 MobiFlight firmware

This repository is a highly customized version of the [MobiFlight firmware](https://github.com/MobiFlight/MobiFlight-FirmwareSource) designed to work with the [Collins FMS 3000 PCB](https://github.com/neilenns/Collins-FMS-3000).

The code is a good example of the minimal methods required for a board to get recognized by [MobiFlight](http://www.mobiflight.com/). It also demonstrates dynamically generating a list of connected devices (69 buttons and one "LED" for brightness control) without the need to store the configuration in EEPROM.

The 69 buttons are handled using TI TCA8418 keyboard driver. Backlight LEDs are managed using an [ISSI IS31FL3733B matrix LED driver](https://www.lumissil.com/assets/pdf/core/IS31FL3733B_DS.pdf) and features a startup animation with completion detection via interrupts.

## Building the repository

This repo is designed to work with [Visual Studio Code](https://code.visualstudio.com/Download) and [PlatformIO](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide). Clone the repo, open the folder in Visual Studio Code, and you will get prompted to install the necessary extensions. Build using the `PlatformIO: Build` command.

A devcontainer configuration is also provided for use with [VSCode Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack) and [GitHub Codespaces](https://github.com/features/codespaces).
