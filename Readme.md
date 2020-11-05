# RECORDER PACKAGE

## Table of contents
* [General info](#general-info)
* [Maintainers](#maintainers)
* [Technologies](#technologies)
* [Dependent UR's packages](#dependent-urs-packages)
* [Setup](#setup)
* [Description](#description)
    * [ Modes](#modes)
    * [ Triggers](#triggers)
    * [ Time of recording](#time-of-recording)
    * [ Topics recorded](#topics-recorded)
    * [ Tagging](#tagging)
* [Tests and debugging](#tests-and-debugging)
* [To do](#to-do)

<br>

-------


## General info
This package is a recorder of topics used in case of unforeseen events like ghostings and turnovers (path replanning).

<br>

-------	
## Maintainers:
* Krzysztof Michalski (topics.h)
* Marcin Gajewski (recorder_node.cpp)

<br>

-------
## Technologies
Project is created with:
* Ubuntu 18.04
* ROS Melodic
-------

<br>

## Dependent UR's packages
* ghostbuster

-------

<br>

## Setup
To run this project, first make a configuration in a launch file according to your environment and needs:

        rosed recorder recorder.launch

<br>

Parameters:

|Name  | Default value (given in the recorder_node, lines 84-100)|
|------------ | -------------|
| before_trigger  | 10.0 (s) |
| after_trigger | 10.0 (s) |
| output_dir | homepath + "/catkin_ws/src/recorder/bags" |
| file1_dir | homepath + "/catkin_ws/src/recorder/topics.txt" |
| file2_dir | homepath + "/catkin_ws/src/recorder/topics_recovery.txt" |


<br><br>
Description:
* before_trigger - time of recorded buffer before trigger  
* after_trigger - time of recorded buffer after trigger
* output_dir - directory where the bags will be saved. Absolute path.

* file1_dir - file location from which list of topics for normal mode is loaded. Absolute path.
* file2_dir - file location from which list of topics for recovery mode is loaded. Absolute path.



<br>
Then recorder can be launched by:

        roslaunch recorder recorder.launch
<br>



-------
## Description
The package is designated to recording unforeseen events during robots work. It allows to get information about possible problems ( for example with vision systems). 

<br>

### Modes
Recorder is sensible to 3 modes of robot work (Normal, Recovery, Off):
* BAG_NORMAL - normal list of topics is recorded
* BAG_RECOVERY - in recovery mode, an extended list of topics is recorded
* BAG_OFF - recording is stopped

<br>

### Triggers
Recorder is sensible to 2 types of events:
* Ghosting - artefact hanging in the air, appearing suddenly in front of a robot, causing replanning
* Turnover - sudden trajectory replanning, caused by some disturbances 

<br>

### Time of recording
Topics are recorded 'beforeTrigger' seconds before, using buffers and 'afterTrigger' seconds after the trigger. If during the recording time another trigger occurs, the time is extended 'afterTrigger' seconds. 

<br>

### Topics recorded 
List of topics to record in the normal mode is included in topics.txt file, while for recovry mode in topics_recovery.txt.

<br>

### Tagging
Files are tagged in the following way:

    anomaly_ + (timestamp in Unix time) + (_event tag) + (mode tag)

Events' tags are as follow:

* G - ghosting

* T - turnover

Modes' tags are as follow:

* N - normal mode

* R - recovery mode

<br>

When a bag is recorded for example because of a ghosting in the normal mode of a robot, a the bag is named as follow:

    anomaly_1602850722_GN.bag 

When two events occur, the time will be extended, so will be the tag:

    anomaly_1602850722_GN_TN.bag

When a change of robot's state occurs, the time will be extended, so will be the tag:

    anomaly_1602850722_GN_GR.bag

<br>

Basing on this approach we can easily get an information about the content of the bag.

More information about tagging (including code description) there: https://unitedrobots.atlassian.net/wiki/spaces/TD/pages/1394737153/How+to+rename+tag+files+basing+on+content+on+example+of+bags.

<br>

-------
## Tests and debugging
Very useful tool for checking work of the package is node for manual triggering. 
It can be run by 
`````
rosrun recorder recorder_keyboard_trigger_node
`````

Triggers as well as mode changing can be simulated manually.

Commands:
* 'g' - trigger ghosting
* 't' - trigger turnover
* 'n' - change mode to normal
* 'r' - change mode to recovery
* 'o' - change mode to off
* 'q' - quit node

<br>

--------
## To do
* buffering from Krzysiek
* efficiency -maybe just copying from buffer after recording?
* what about changing robot mode during recording? Propose just tag - done
* what about tags saying how many seconds from the start it was to trigger/changing mode - rather not
* subgoals trigger
* tests on a robot
* launches and configs - done


---------
Last edit: the 23th Oct 2020



