**Car Warning System using Real-Time Traffic Light Detection**

**Implemented Features:**
- Red traffic light detection using HSV color space analysis and basic object detection
- Generic CAN Bus driver temporarily implemented for Preet's SJOne Board

**Demo Prerequisites:**
- OpenCV development environment
- SJOne Board and CAN Bus external controller (MCP2515 + MCP2551)

**How to Install OpenCV for Linux:**
- Installation prerequisites:
    - Git
    - CMake
    - GNU Make
    - MinGW
    - Recommended: Eclipse IDE with CDT4 (C++ Development Tooling)

- Installation procedures:

    1. Clone the OpenCV repository in a workspace; For example:

        `mkdir ~/cv_workspace`

        `cd ~/cv_workspace`

        `git clone https://github.com/Itseez/opencv.git`

        `cd opencv`

    2.  Build the OpenCV source in another directory; for the cmake command, provide a path to the source directory; For     example:

        `mkdir release`

        `cd release`

        `cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=/usr/local ~/cv_workspace/opencv`

    3. Finialize the build

        `make`
        
        `make install`
        
    4. Recommended: Set up Eclipse IDE for development
        1. Create a new C++ project
        2. Modify the newly created project settings
            - Add external include path: /usr/local/include
            - Add external dependencies as a path for the linker: /usr/local/lib

**Running an OpenCV C++ Program in Eclipse IDE**

1. Download the provided source code and import it to the created Eclipse project
    - OR: create a new source file in the created Eclipse project
2. Build the source code then run

