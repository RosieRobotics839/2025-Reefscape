# A Guide to our Kraken Orchestra

## Overview

This system allows users to control music playback on the robot via WPILib's Shuffleboard interface. It utilizes drop down choosers for selecting a song, as well as for starting and stopping music. The music playback is powered by the Talon FX Orchestra feature, which uses `.chrp` files.

## Understanding the Interface

### What is a Drop Down Chooser?

A drop down chooser is a menu that allows users to select an option from a list. In this system, we use drop down choosers in Shuffleboard for:

- Selecting a song (`songChooser`, displayed as `Song Selector` in Shuffleboard)
- Controlling playback with `Start` and `Stop` buttons (displayed as `Music Control` in Shuffleboard)

Both options are presented as drop down menus, where you select a command and execute it accordingly. Make sure to expand the boxes to see the full list of options available.

### What is the Deploy Directory?

The deploy directory is a folder within the robot's codebase where essential files, such as `.chrp` music files, are stored. It is located in the `src/main/deploy/` directory of the project. When the robot runs, it accesses files from this location to play music. Any new `.chrp` files must be placed here to be recognized by the system.

## User Guide

### Opening Shuffleboard

1. Open WPILib VS Code.
2. Click on the WPILib icon in the sidebar and select `Start Tool`.
3. Choose `Shuffleboard` from the list of tools.
4. Once Shuffleboard opens, navigate to the `Music` section.

### Selecting and Playing a Song

1. Open Shuffleboard and go to the `Music` section.
2. Choose a song from the `Song Selector` drop down (internally referenced as `songChooser`).
3. Select `Start` from the `Music Control` drop down to play the selected song.
4. If the song is already playing and you attempt to start it again, it will continue playing uninterrupted.
5. To switch to a different song, first select `Stop` from the `Music Control` drop down, then choose a new song and select `Start`.

### Important Notes

- `.chrp` files take time to load before playback begins, so there may be a slight delay after selecting `Start`.
- Only one song can play at a time due to hardware limitations.

## Uploading New Songs

To add new songs to the system, follow these steps:

1. **Obtain a MIDI file that contains the main track of the song.
2. **Convert the MIDI file to a `.chrp` file using Phoenix Tuner tools.
3. **Add the .chrp file to the deploy directory (src/main/deploy/) of your robot code.
4. **Register the song in `Robot.java` using the following format:
   ```java
   songChooser.addOption("Song Name", Filesystem.getDeployDirectory() + "/" + "filename.chrp");
   ```

## Future Enhancements

Currently, only a single track can be played at a time. Once we have multiple motors available, we can modify the code to support multiple melodies/tracks for a song, distributing different instrument parts across the Kraken motors on our robot.

## Troubleshooting

- **Song not playing?** Ensure the `.chrp` file is correctly placed in the deploy directory and is properly registered in `Robot.java`.
- **Long loading times?** This is normal due to `.chrp` file processing delays.
- **Can't switch songs?** Stop the currently playing song first, then select and start a new one.

