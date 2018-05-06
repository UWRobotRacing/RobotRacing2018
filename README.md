# UW Robotics Team
## Robot Racing
                                                            
### Description:
This repository contains the current competition code (2018). This code will be used for the IARRC 2018. This code currently handles LIDAR mapping, trajectory rollout, PID speed control, lane detection and traffic light detection.

A list of what we're currently working on is in the issues section of this repo.

### Competition Rules:
Please review the competition rules before pushing any code :)
* https://drive.google.com/file/d/1iF5CkpvjLxLN1k5ueg4_vOaK9uYlgP9x/view

### Coding Standards:
For C++ code and Python Scripts we follow the Google coding standards linked below
* https://google.github.io/styleguide/cppguide.html
* https://google.github.io/styleguide/pyguide.html

Please fill out this header above all functions (Doxygen style)
``` cpp
/** @brief Writes the current position of the cursor
 *         into the arguments row and col.
 *  @param row The address to which the current cursor
 *         row will be written.
 *  @param col The address to which the current cursor
 *         column will be written.
 *  @return Void.
 */
void get_cursor(int* row, int* col);
```
Please fill this header out above every source code file(including launch files and xml files)
```
/** @file console.h
 *  @brief Function prototypes for the console driver.
 *
 *  This contains the prototypes for the console
 *  driver and eventually any macros, constants,
 *  or global variables you will need.
 *
 *  @author YOUR NAME (GITHUB_USERNAME)
 *  @author Toni Ogunmade(oluwatoni)
 *  @competition IARRC 2018
 */
```

### Moving forward:
This code base should be integrated with the overall UWRobotics code such that modularity can be achieved?

### Contributers for the current code:
* Toni Ogunmade(Software Lead)
* Jack Xu
* Angela Gu
* Brian Tran
* Adrian Malaran

### Past Contributers (2014-2017):
* Matthew Post(Software Lead)
* Praveen Dorairaj
* Ivy Xing
* Aditya Matam
* Ee Ern Low
* Sirui Song
* Michael Smart
* Jungwook Lee(Software Lead)
* Raymond Kuo(Software Lead)
* Jakub Dworakowski(Software Lead)
* Greg Varty
* Archie Lee
* Shalin Upadhyay
* Ning Zhao
* Jason Leung
* Jamie Kim
