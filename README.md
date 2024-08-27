# Electronics Xplore
## For those setting up the project for the first time
You need to do the following things before the project will build for Rpi/JETSON/Desktop:
* Within a terminal write `git submodule update --init --remote`: to pull the sub-repository BroCo (stored in a different repository)
* Then write `git submodule add git@github.com:EPFLXplore/ERC_SE_CustomMessages.git src/custom_msg` : to pull the custom messages.
* Within 'ERC_EL\src\BRoCo\include\Build\Build.h': 
	* **Uncomment:** #define BUILD_WITH_CAN_SOCKET_DRIVER
	* **Comment:** #define BUILD_WITH_FDCAN

