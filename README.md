# 2022-group-02 | Algorithm for steering wheel angle

## Description

This algorithm is data driven and is integrated with [https://github.com/chalmers-revere/opendlv-vehicle-view](opendlv-vehicle-view). The algorithm uses object detection in the data from the video feed from [https://github.com/chalmers-revere/opendlv-vehicle-view](opendlv-vehicle-view) along side with other sensor readings to compute an accurate steering angle output close to what the actual steering wheel angle was at the time of the video feed.


## Installation
Latest release: 

![version](https://img.shields.io/badge/version-1.1.0-blue)

## Usage
The system is ran alongside with [opendlv-vehicle-view](https://github.com/chalmers-revere/opendlv-vehicle-view) and [h264decoder](https://github.com/chalmers-revere/opendlv-video-h264-decoder) (to extract h264 frames into a shared memory area to provide access to ARGB pixels in the frames). By using shared memory, the system is able to detect cones of different color and detemine the heading direction including the needed steering wheel angles for the wheels. 

The algorithm produces an output in the terminal for each video frame containing the sample timestamp together with the calculated steering wheel angle.


## Authors and acknowledgment
**Authors and contributors:**

<img src="https://avatars.githubusercontent.com/u/71592942?s=40&v=4" alt="Profile Picture Caisesiume" width="20"/> Caisesiume [(GitHub)](https://github.com/Caisesiume) [(GitLab)](https://git.chalmers.se/simonar)

<img src="https://avatars.githubusercontent.com/u/71591829?v=4" alt="Profile Picture JohanAxell" width="20"/> JohanAxell [(GitHub)](https://github.com/johanaxell) [(GitLab)](https://git.chalmers.se/johanaxe)

<img src="https://avatars.githubusercontent.com/u/81258179?v=4" alt="Profile Picture Jidarv" width="20"/> Jidarv [(GitHub)](https://github.com/Jidarv)[(GitLab)](https://git.chalmers.se/jidarv)

<img src="https://avatars.githubusercontent.com/u/81112288?v=4" alt="Profile Picture RobbanGit" width="20"/> RobbanGit [(GitHub)](https://github.com/RobbanGit) [(GitLab)](https://git.chalmers.se/robinhan)

<img src="https://avatars.githubusercontent.com/u/72571860?v=4" alt="Profile Picture Asiya-Ismail" width="20"/> Asiya [(GitHub)](https://github.com/Asiya-Ismail)[(GitLab)](https://git.chalmers.se/asiya)

**Acknowledgment:**

Thanks to University of Gothenburg, Chalmers University of Technology and Christian Berger for setting up this project possiblility.


## License
For this project a [MIT License](https://git.chalmers.se/courses/dit638/students/2022-group-02/-/blob/main/LICENSE) applies.

## Project status
The current project state:

The project development is in its ending phase. There might only be occational updates to this repository.


## Collaboration with the team members

See Code-of-Conduct.

