#!/bin/bash

#AUV (Autonomus Underwater Vehicle) Container Controller V3.1.0 - Developed by Robert Fudge
#DO NOT run this script as root, user environment variables are used, it will prompt for
#authentication when needed

#ASCII escape formatting sequences
RESET="\033[0m"
BOLD="\033[1m"
DIM="\033[2m"
ITALIC="\033[3m"
UNDERLINE="\033[4m"
BLINK="\033[5m"

#ASCII foreground formatting sequences
FG_BLACK="\033[30m"
FG_RED="\033[31m"
FG_GREEN="\033[32m"
FG_YELLOW="\033[33m"
FG_BLUE="\033[34m"
FG_MAGENTA="\033[35m"
FG_CYAN="\033[36m"
FG_WHITE="\033[37m"

#ASCII background formatting sequences
BG_BLACK="\033[40m"
BG_RED="\033[41m"
BG_GREEN="\033[42m"
BG_YELLOW="\033[43m"
BG_BLUE="\033[44m"
BG_MAGENTA="\033[45m"
BG_CYAN="\033[46m"
BG_WHITE="\033[47m"

#Log File
INIT_LOG=".status/logs/init.log"
BUILD_LOG=".status/logs/build.log"
START_LOG=".status/logs/start.log"

#Display program intro
echo -e "${BOLD}${FG_BLUE}AUV Container Controller (Desktop) REV 3.1, Program developed by Rob Fudge${RESET}"

#Ensure status directory exists
mkdir -p .status/logs

#Parse command line arguments
while getopts "hi:b:s:dnl:" options; do
    case ${options} in
        #Handle help case - Working (TESTED)
        h)
            echo -e "${FG_CYAN}AUV Container Controller Help Interface"
            echo -e "Valid commands are listed below:"
            echo -e "   -h - Displays the help window"
            echo -e "   -i - Initializes the system for project building, specify either x86_64 or aarch64"
            echo -e "   -b - Builds the container"
            echo -e "   -s - Starts the container with GPU virtualization"
            echo -e "   -d - Displays the status of the project and associated containers"
            echo -e "   -n - Starts a new terminal instance in the running container"
            echo -e "   -l - Views specified log, pass in either i, b, or s to specify log"
            echo -e ""
            echo -e "Please do not modify the .status folder, as the files contained within are used to keep track of which sub-scripts have been run"

            ;;

        #Handles initialization
        i)
            touch .status/initRunning
            echo -e "${FG_CYAN}Initializing system for project compilation${RESET}" | tee -a ${INIT_LOG}

            #Print time to log file
            echo -e "\n"Start Time: $(date)"\n" > ${INIT_LOG}

            #Ensure certificates, curl and wget are installed
            sudo apt-get install ca-certificates curl wget | tee -a ${INIT_LOG}

            #Instructions specific to x86_64
            if [ "${OPTARG}" == "x86_64" ]; then
                echo -e "${FG_CYAN}Target Selected: x86_64${RESET}" | tee -a ${INIT_LOG}

                echo -e "${FG_CYAN}Removing old versions of Nvidia CUDA Toolkit${RESET}" | tee -a ${INIT_LOG}
                sudo apt-get remove --purge -y '^nvidia-.*' | tee -a ${INIT_LOG}
                sudo apt-get install ubuntu-desktop | tee -a ${INIT_LOG}

                #Install the Nvidia CUDA Toolkit and Drivers
                #https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network
                echo -e "${FG_CYAN}Installing Nvidia CUDA Toolkit and drivers${RESET}" | tee -a ${INIT_LOG}
                wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb | tee -a ${INIT_LOG}
                sudo dpkg -i cuda-keyring_1.1-1_all.deb | tee -a ${INIT_LOG}
                sudo apt-get update | tee -a ${INIT_LOG}
                sudo apt-get install -y nvidia-driver-555-open cuda-drivers-555 | tee -a ${INIT_LOG}
                sudo apt-get install -y cuda-toolkit-12-5 cuda-compat-12-5 | tee -a ${INIT_LOG}

                #Modify .bashrc
                echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc 
                echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

                rm -f ./cuda-keyring_1.1-1_all.deb

                echo -e "${FG_GREEN}Finished installing Nvidia CUDA Toolkit and drivers${RESET}" | tee -a ${INIT_LOG}

                #Install the ZED SDK (Not sure how good this works as can't reboot mid-script)
                #https://www.stereolabs.com/developers/release#82af3640d775
                echo -e "${FG_CYAN}Installing ZED SDK${RESET}"
                wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22  | tee -a ${INIT_LOG}
                chmod +x ./ubuntu22  | tee -a ${INIT_LOG}
                ./ubuntu22 -- silent  | tee -a ${INIT_LOG}

                rm -f ./ubuntu22

                echo -e "${FG_GREEN}ZED SDK Installed${RESET}" | tee -a ${INIT_LOG}

                #Install Docker Engine
                #https://docs.docker.com/engine/install/ubuntu/
                echo -e "${FG_CYAN}Installing Docker Engine${RESET}" | tee -a ${INIT_LOG}
                sudo install -m 0755 -d /etc/apt/keyrings
                sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
                sudo chmod a+r /etc/apt/keyrings/docker.asc
                echo \
                    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
                    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
                    sudo tee -a /etc/apt/sources.list.d/docker.list > /dev/null
                sudo apt-get update  | tee -a ${BUILD_LOG}
                sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin | tee -a ${BUILD_LOG}

                echo -e "${FG_CYAN}Docker Engine Installed${RESET}" | tee -a ${INIT_LOG}

                #Install Foxglove Studio
                #https://foxglove.dev/download
                echo -e "${FG_CYAN}Installing Foxglove Studio Visualizer${RESET}" | tee -a ${INIT_LOG}
                sudo snap install foxglove-studio  | tee -a ${BUILD_LOG}
                echo -e "${FG_CYAN}Foxglove Studio Installed${RESET}" | tee -a ${INIT_LOG}

            elif [ "${OPTARG}" == "x86_64_WSL" ]; then
                wget https://developer.download.nvidia.com/compute/cuda/repos/wsl-ubuntu/x86_64/cuda-keyring_1.1-1_all.deb
                sudo dpkg -i cuda-keyring_1.1-1_all.deb
                sudo apt-get update
                sudo apt-get -y install cuda-toolkit-12-5

                #Modify .bashrc
                echo 'export PATH=/usr/local/cuda/bin:$PATH' >> ~/.bashrc 
                echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH' >> ~/.bashrc

                rm -f ./cuda-keyring_1.1-1_all.deb

                echo -e "${FG_GREEN}Finished installing Nvidia Cuda Toolkit and drivers${RESET}" | tee -a ${INIT_LOG}

                #Install the ZED SDK (Not sure how good this works as can't reboot mid-script)
                #https://www.stereolabs.com/developers/release#82af3640d775
                echo -e "${FG_CYAN}Installing ZED SDK${RESET}"
                wget https://download.stereolabs.com/zedsdk/4.1/cu121/ubuntu22  | tee -a ${INIT_LOG}
                chmod +x ./ubuntu22  | tee -a ${INIT_LOG}
                ./ubuntu22 -- silent  | tee -a ${INIT_LOG}

                rm -f ./ubuntu22

                echo -e "${FG_GREEN}ZED SDK Installed${RESET}" | tee -a ${INIT_LOG}

                #Install Docker Engine
                #https://docs.docker.com/engine/install/ubuntu/
                echo -e "${FG_CYAN}Installing Docker Engine${RESET}" | tee -a ${INIT_LOG}
                sudo install -m 0755 -d /etc/apt/keyrings
                sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
                sudo chmod a+r /etc/apt/keyrings/docker.asc
                echo \
                    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
                    $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
                    sudo tee -a /etc/apt/sources.list.d/docker.list > /dev/null
                sudo apt-get update  | tee -a ${BUILD_LOG}
                sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin | tee -a ${BUILD_LOG}

                echo -e "${FG_CYAN}Docker Engine Installed${RESET}" | tee -a ${INIT_LOG}

                #Install Foxglove Studio
                #https://foxglove.dev/download
                echo -e "${FG_CYAN}Installing Foxglove Studio Visualizer${RESET}" | tee -a ${INIT_LOG}
                sudo snap install foxglove-studio  | tee -a ${BUILD_LOG}
                echo -e "${FG_CYAN}Foxglove Studio Installed${RESET}" | tee -a ${INIT_LOG}


            #Instructions specific to aarch64
            elif [ "${OPTARG}" == "aarch64" ]; then
                echo -e "${FG_CYAN}Target Selected: aarch64${RESET}" | tee -a ${INIT_LOG}

                #Install Drivers for Intel Wireless 8265/8275 (backport), driver removed from 22.04 by default
                sudo apt-get install -y iwlwifi-modules python3-pip | tee -a ${BUILD_LOG}

                #Ensure device is configured for maximum performance
                #https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/compute/index.html
                sudo /usr/bin/jetson_clocks | tee -a ${INIT_LOG}

                #Install the Nvidia CUDA Toolkit
                #https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=aarch64-jetson&Compilation=Native&Distribution=Ubuntu&target_version=22.04&target_type=deb_network
                echo -e "${FG_CYAN}Installing Nvidia CUDA Toolkit${RESET}" | tee -a ${INIT_LOG}
                wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb | tee -a ${INIT_LOG}
                sudo dpkg -i cuda-keyring_1.1-1_all.deb
                sudo apt-get update | tee -a ${INIT_LOG}
                sudo apt-get install -y cuda-toolkit-12-2 cuda-compat-12-2 | tee -a ${INIT_LOG}

                #Install the ZED SDK (Not sure how good this works as can't reboot mid-script)
                #https://www.stereolabs.com/developers/release#82af3640d775
                wget https://download.stereolabs.com/zedsdk/4.1/l4t36.3/jetsons | tee -a ${INIT_LOG}
                chmod +x ./jetsons

                pip3 install onnx

                ./jetsons -- silent | tee -a ${INIT_LOG}

                rm -f ./jetsons

                #Install Foxglove Studio
                #https://get.foxglove.dev/desktop/latest/foxglove-studio-2.5.1-linux-arm64.deb
                echo -e "${FG_CYAN}Installing Foxglove Studio Visualizer${RESET}" | tee -a ${INIT_LOG}
                wget https://get.foxglove.dev/desktop/latest/foxglove-studio-2.6.0-linux-arm64.deb
                sudo dpkg -i foxglove-studio-2.6.0-linux-arm64.deb  | tee -a ${BUILD_LOG}
                echo -e "${FG_CYAN}Foxglove Studio Installed${RESET}" | tee -a ${INIT_LOG}

            #Doesn't install any software, just gather dependencies
            elif [ "${OPTARG}" == "no_arch" ]; then
                echo -e "${FG_CYAN}No architecture selected, resetting dependencies only${RESET}"
                rm -rf dependencies
                mkdir -p dependencies/configuration_files

                cd ./dependencies
                git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git | tee -a ${INIT_LOG}
                git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
                git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git
                git clone https://github.com/Robotic-Decision-Making-Lab/ardusub_driver.git
                git clone -b ros2 https://github.com/mavlink/mavros

                cp ./zed-ros2-wrapper/zed_wrapper/config/common.yaml ./configuration_files/zed/zed-common.yaml
                cp ./zed-ros2-wrapper/zed_wrapper/config/zed2i.yaml ./configuration_files/zed/zed-zed2i.yaml
                cp ./ardusub_driver/ardusub_teleop/config/joy_teleop.yaml ./configuration_files/ardusub/joy_teleop.yaml

                exit 0

            #Handle the case where a user specifies invalid architecture
            else
                echo -e "${FG_RED}Error (FATAL): Invalid target selected${RESET}" | tee -a ${INIT_LOG}
                exit 1

            fi

            #Grab dependencies
            echo -e "${FG_CYAN}Cloning dependencies from GitHub${RESET}" | tee -a ${INIT_LOG}
            mkdir -p ./dependencies/configuration_files
            cd ./dependencies
            git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git | tee -a ${INIT_LOG}
            git clone --recursive https://github.com/stereolabs/zed-ros2-wrapper.git
            git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git
            git clone https://github.com/Robotic-Decision-Making-Lab/ardusub_driver.git
            git clone -b ros2 https://github.com/mavlink/mavros

            cp zed-ros2-wrapper/zed_wrapper/config/common.yaml ./configuration_files/zed/common.yaml
            cp zed-ros2-wrapper/zed_wrapper/config/zed2i.yaml ./configuration_files/zed/zed2i.yaml
            cp ardusub_driver/ardusub_teleop/config/joy_teleop.yaml ./configuration_files/ardusub/joy_teleop.yaml

            cd ..

            #Install the Nvidia Container Toolkit
            #https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
            echo -e "${FG_CYAN}Installing Nvidia Container Toolkit${RESET}" | tee -a ${INIT_LOG}
            curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
                && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
                sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                sudo tee -a /etc/apt/sources.list.d/nvidia-container-toolkit.list
            sudo apt-get update
            sudo apt-get install -y nvidia-container-toolkit | tee -a ${BUILD_LOG}           #V1.15.0

            #Link Nvidia Container Toolkit to Docker
            sudo nvidia-ctk runtime configure --runtime=docker | tee -a ${BUILD_LOG}
            sudo systemctl restart docker

            echo -e "${FG_CYAN}Nvidia Container Toolkit Installed${RESET}" | tee -a ${INIT_LOG}
            
            #Fix Docker permissions
            #https://docs.docker.com/engine/install/linux-postinstall/
            echo -e "${FG_CYAN}Cleaning Up${RESET}" | tee -a ${INIT_LOG}
            sudo usermod -aG docker ${USER}

            #Add user to dialout and video groups
            sudo usermod -aG dialout ${USER}
            sudo usermod -aG video ${USER}
            
            #Notify user script is done and finish resetting docker daemon
            echo -e "${FG_GREEN}Initialization Done!${RESET}" | tee -a ${INIT_LOG}
            rm -f .status/initRunning
            touch .status/initDone
            
            #Reboot in 10 seconds
            echo -e "${FG_YELLOW}Rebooting in 10 seconds${RESET}" | tee -a ${INIT_LOG}
            sleep 10

            if [ "${OPTARG}" == "x86_64" ]; then
                sudo reboot
            else
                sudo /usr/sbin/nvpmodel -m 0 --force
                sudo reboot
            fi

            ;;

        #Handles building the container for the target system
        b)
            touch .status/buildRunning
            echo -e "${FG_CYAN}Building container for AUV${RESET}" | tee -a ${BUILD_LOG}

            #Print time to log file
            echo -e "\n"Start Time: $(date)"\n" > ${BUILD_LOG}
            
            #Symlink dockerfiles into dependencies (not manually refreshing so delete old hardlink first)
            cd ./dependencies/isaac_ros_common/docker/
            rm -f Dockerfile.auv-x86_64
            rm -f Dockerfile.auv-aarch64
            ln ../../../containers/Dockerfile.auv-x86_64 Dockerfile.auv-x86_64
            ln ../../../containers/Dockerfile.auv-aarch64 Dockerfile.auv-aarch64

            cd scripts
            rm -f auv-entrypoint.sh
            ln ../../../../configuration/auv-entrypoint.sh auv-entrypoint.sh
            cd ../../../../

            #Ensure user has access to the script
            chmod +x ./dependencies/isaac_ros_common/scripts/build_image_layers.sh

            if [ "${OPTARG}" == "x86_64" ]; then
                echo -e "${FG_CYAN}Target Selected: x86_64${RESET}" | tee -a ${BUILD_LOG}
                ./dependencies/isaac_ros_common/scripts/build_image_layers.sh --image_key x86_64.ros2_humble.user.auv-x86_64 --build_arg USERNAME=auv-container --image_name auv-container:x86_64 | tee -a ${BUILD_LOG}

            elif [ "${OPTARG}" == "aarch64" ]; then
                echo -e "${FG_CYAN}Target Selected: aarch64${RESET}" | tee -a ${BUILD_LOG}
                ./dependencies/isaac_ros_common/scripts/build_image_layers.sh --image_key aarch64.ros2_humble.user.auv-aarch64 --build_arg USERNAME=auv-container --image_name auv-container:aarch64 | tee -a ${BUILD_LOG}

            else
                echo -e "${FG_RED}Error (FATAL): Invalid target selected${RESET}" | tee -a ${BUILD_LOG}
                exit 2

            fi

            echo -e "${FG_GREEN}Build Done!${RESET}" | tee -a ${BUILD_LOG}
            rm -f .status/buildRunning
            touch .status/buildDone

            ;;

        #Starts the container on the target system
        s)
            touch .status/containerRunning
            echo -e "${FG_CYAN}Starting container${RESET}" | tee -a ${START_LOG}

            #Print time to log file
            echo -e "\n"Start Time: $(date)"\n" > ${START_LOG}

            if realpath -q /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00 | grep -q "/dev/"; then
                echo -e "${FG_CYAN}GNSS detected by host system${RESET}" | tee -a ${START_LOG}
                export GNSS_DIR=$(realpath /dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00)
            else
                echo -e "${FG_MAGENTA}Error (WARNING): GNSS device not detected, global telemetry will not be available${RESET}" | tee -a ${START_LOG}
            fi

            if lsusb | grep -q "ID 2b03:f880 STEREOLABS ZED 2i"; then
                echo -e "${FG_CYAN}ZED2i detected by host system" | tee -a ${START_LOG}
            else
                echo -e "${FG_MAGENTA}Error (WARNING): ZED2i not detected, system is blind to environment${RESET}" | tee -a ${START_LOG}
            fi

            #Create shared volumes for source code and data
            mkdir -p share
            mkdir -p share/source_vol
            mkdir -p share/data_vol

            docker volume create --driver local --opt type="none" --opt device="${PWD}/share/source_vol" --opt o="bind" "auv_source_vol" > /dev/null
            docker volume create --driver local --opt type="none" --opt device="${PWD}/share/data_vol" --opt o="bind" "auv_data_vol" > /dev/null

            #Need to disable authentication for access to windowing system
            xhost +

            #Run the build script for the specified architecture
            if [ "${OPTARG}" == "x86_64" ];then
                echo -e "${FG_CYAN}Target Selected: x86_64${RESET}" | tee -a ${START_LOG}
                docker run --entrypoint /usr/local/bin/scripts/auv-entrypoint.sh --privileged --runtime=nvidia --network=host --rm -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/usr/local/zed/settings:/usr/local/zed/settings:rw" --volume="/usr/local/zed/resources:/usr/local/zed/resources:rw"  --volume="${PWD}/dependencies/configuration_files/zed:/home/auv-container/ros_ws/src/zed-ros2-wrapper/zed_wrapper/config:rw" --volume="auv_source_vol:/home/auv-container/ros_ws/src/auv-kernel:rw" --volume="auv_data_vol:/home/ros-container/ros_ws/data:rw" "auv-container:x86_64" ${GNSS_DIR}

            elif [ "${OPTARG}" == "aarch64" ]; then
                echo -e "${FG_CYAN}Target Selected: aarch64${RESET}" | tee -a ${START_LOG}
                docker run --entrypoint /usr/local/bin/scripts/auv-entrypoint.sh --privileged --runtime=nvidia --network=host --rm -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="/usr/local/zed/settings:/usr/local/zed/settings:rw" --volume="/usr/local/zed/resources:/usr/local/zed/resources:rw" --volume="${PWD}/dependencies/configuration_files/zed:/home/auv-container/ros_ws/src/zed-ros2-wrapper/zed_wrapper/config:rw" --volume="auv_source_vol:/home/ros-container/ros_ws/src/auv-kernel:rw" --volume="auv_data_vol:/home/auv-container/ros_ws/data:rw" "auv-container:aarch64" ${GNSS_DIR}

            else
                echo -e "${FG_RED}Error (FATAL): Invalid target selected${RESET}" | tee -a ${START_LOG}
                xhost -
                exit 3

            fi

            #Re-enable authentication status
            xhost -

            ;;

        #Display the AUV project's status
        d)
            #Display status of the initialization script state
            if ! test -f .status/initRunning; then
                echo -e "Initialization Script Status (-i): ${FG_YELLOW}Standby${RESET}"
            else
                echo -e "Initialization Script Status (-i): ${FG_GREEN}Running${RESET}"
            fi

            #Display status of the initialization phase
            if ! test -f .status/initDone; then
                echo -e "Initialization Status (-i): ${FG_RED}Required${RESET}"
            else
                echo -e "Initialization Status: ${FG_GREEN}Done${RESET}"
            fi

            #Display status of the build script
            if ! test -f .status/buildRunning; then
                echo -e "Build Script Status (-b): ${FG_YELLOW}Standby${RESET}"
            else
                echo -e "Build Script Status (-b): ${FG_GREEN}Running${RESET}"
            fi

            #Display status of the build phase
            if ! test -f .status/buildDone; then
                echo -e "Build Status (-b): ${FG_RED}Not Built${RESET}"
            else
                echo -e "Buiild Status: ${FG_GREEN}Container Built${RESET}"
            fi

            #Display whether container is running
            if ! test -f .status/containerRunning; then
                echo -e "Container Running (-s): ${FG_RED}No${RESET}"
            else
                echo -e "Container Running (-s): ${FG_GREEN}Yes${RESET}"
            fi

            ;;

        #Create a new terminal instance for the running container
        n)
            CONTAINER=$(docker container ls -la --format "{{.Names}}")

            if [ "${CONTAINER}" != "" ]; then
                echo -e "${FG_CYAN}Starting new terminal instance${RESET}"
                docker exec -it ${CONTAINER} /bin/bash
                echo -e "${FG_CYAN}Closing terminal instance${RESET}"

            else
                echo -e "${FG_RED}Error (FATAL): No container is running, please start one before continuing${RESET}"
                exit 4
            fi

            ;;

        #Print specified log
        l)
            if [ "${OPTARG}" == "i" ]; then
                cat ${INIT_LOG}

            elif [ "${OPTARG}" == "b" ]; then
                cat ${BUILD_LOG}

            elif [ "${OPTARG}" == "s" ]; then
                cat ${START_LOG}

            else
                echo -e "${FG_RED}Error (FATAL): Could not determine which log ${OPTARG} corresponds to${RESET}"
            fi

            ;;
    esac
done
        
