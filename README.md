LiDAR GTA V
============================
*A plugin for Grand Theft Auto V that generates a labeled LiDAR point cloud form the game environment.*

<img src="/samples/LiDAR Sample - Traffic.png">

## Requirements

* [Grand Theft Auto V](https://store.steampowered.com/app/271590/Grand_Theft_Auto_V/)
* [ScriptHookV by Alexander Blade](http://www.dev-c.com/gtav/scripthookv/)
* [VAutodrive](https://www.gta5-mods.com/scripts/vautodrive) (optional)
* [GTA 5 LauncherBypass](https://www.gta5-mods.com/tools/gtavlauncherbypass) (Optional)

## Installation

1. Install [ScriptHookV](http://www.dev-c.com/gtav/scripthookv/) following its accompanying instructions.
2. Copy-paste *LiDAR GTA V.asi* and the *LiDAR GTA V* folder found in [*bin/Release*](https://github.com/UsmanJafri/LiDAR-GTA-V/tree/master/LiDAR%20GTA%20V/bin/Release) into your GTA-V directory (the folder containing *GTAV.exe*)
3. Install [VAutodrive](https://www.gta5-mods.com/scripts/vautodrive) for automated driving and log data over the night.
4. Install [GTA 5 LauncherBypass](https://www.gta5-mods.com/tools/gtavlauncherbypass) to bypass the loading screen and go directly into story mode. Trust me you will need this. 
5. If you are seeing the error below when building the solution try changing the VS configuration to **Release** Mode and **x64** configuration. 
>LINK : fatal error LNK1561: entry point must be defined

## How to use

1. Navigate to *#your_game_directory#/LiDAR GTA V/LIDAR GTA V.cfg* and edit the parameters as needed. The parameters include: Horizontal FOV, Vertical FOV, Range, Density and Output Filename.
2. If everything works correctly, the game should directly take you to story mode. 
3. Get inside a car of your choice manually. 
4. Press G to start the automatic driving script from VAutodrive. 
5. Press F6 to start logging the data. Note this will store one every 30/40 seconds. Watch ur storage. 
6. Output files are point cloud data and the respective labels like [KITTI](http://www.cvlibs.net/datasets/kitti/) LiDAR dataset. 
7. These files are then processed with the python blank_point_remover.py file. 
8. This will generate point cloud files with labels of objects in it similar to KITTI. 

## Version dependencies

* You will need Visual Studio 2017 or higher to open the project.
* Any contributions to the project are welcomed, it's recommended to use GitHub [Pull requests](https://help.github.com/articles/using-pull-requests/).
* This script was tested on GTA 5 version 1.0.1737.6. 

## Contributions
* Modified the point cloud data to have similar values like real LiDAR data. Shifted the world origin to source of ray cast. 
* Added 3D bounding box labels similar to KITTI for vehicles with rotations.
* Added funtionality for logging large amount of data with automated driving. 

## TODO 
* Fix the bug for wrong rotations of the bounding boxes for some of the LiDAR samples.
* The generated files contain excessive repetitive data. Optimize it. 
* Add the camera snapshots for each LiDAR snapshot as well. 
* Shift this plugin to client server based implementation similar to  [DeepGTA 5](https://github.com/aitorzip/DeepGTAV).

## License 
* This repository is build on top of the repository from the author @[UsmanJafri](https://github.com/UsmanJafri/LiDAR-GTA-V) and is only intended for educational research purpose. 


## Acknowledgements
* [Native Trainer](http://www.dev-c.com/gtav/scripthookv/) (a ScriptHookV sample) was used as base project to import configuration.
* Some functions were inspired from [DeepGTA 5](https://github.com/aitorzip/DeepGTAV) as well. 
