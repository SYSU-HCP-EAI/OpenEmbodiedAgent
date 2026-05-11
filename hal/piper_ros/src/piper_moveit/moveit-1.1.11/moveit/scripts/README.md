# Utility Scripts for MoveIt

## README Markdown Buildfarm Table Generator

This Python script will generate Github-style Markdown table with Jenkins badges for every package recursively from the current folder.
It will order all packages alphabetically, but start with all MoveIt packages first.
For generating an updated README for a new ROS distribution:
First update the `create_readme_table.py` script and add/remove the current MoveIt-supported ROS distros and their corresponding Ubuntu distribution in the specified Python dictionary.
Then, in the main MoveIt `README.md` file, remove the "## ROS Buildfarm" header and the following table.
Finally, from your catkin workspace with every MoveIt package, run the `create_readme_table.py` script.
For example:

    cd ~/ws_moveit/src
    python moveit/moveit/scripts/create_readme_table.py >> moveit/README.md

## HTML Table Generator

This python script will generate an html page with useful data about every package in the current folder the script is run in.
Therefore, to summarize moveit, run within a catkin workspace.
For example:

    cd ~/ws_moveit/src
    python moveit/moveit/scripts/create_maintainer_table.py

The generated page should automatically open in your web browser.
