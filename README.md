# pcl_proc

Package for path generation and path following.

### Requirements
Clone this [fork](https://github.com/GSO-soslab/navigation2) of nav2. This is needed for costmaps.

### Launch files

`filter.launch.py` applies the distance filter and intensity filter on the MSIS pointclouds. <br>
`costmaps.launch.py` creates costmap from the filtered pointclouds.<br>
`path_gen.launch.py` generates path from the costmap. <br>
`wp_admin.launch.py` spins up the Waypoint Administrator. <br>

[Video](https://drive.google.com/file/d/1_OjvJ9xO-Ar6HdSANdNQEcmdRiWNHEWJ/view?usp=drive_link)
