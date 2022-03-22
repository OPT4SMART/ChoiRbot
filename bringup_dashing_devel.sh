#!/bin/bash

#set nocasematch option
shopt -s nocasematch
if [[ $(lshw -C display | grep vendor) =~ Nvidia ]]; then
    nvidiacard=1
else
    nvidiacard=0
fi
shopt -u nocasematch

#Helper functions
function retFileNameFromGraphicsCard()
{
    if [ $nvidiacard -eq 1 ]; then
        echo "docker-compose_gpu.yml"
    elif [ $nvidiacard -eq 0 ]; then
        echo "docker-compose.yml"
    fi
}

function helpFunction()
{
    echo ""
    echo "Usage: $0 -c <build/run>"
    echo "-c configuration specifying if the container should be built or run"
    exit 1
}

while getopts "c:" opt
do
    case "$opt" in
        c)
            cmdargs=$OPTARG
            ;;
        \?) #Invalid Option
            helpFunction
            ;;
    esac
done

if [[ $cmdargs == "build" ]]; then
    filename=$(retFileNameFromGraphicsCard)
    docker-compose -f $filename build

elif [[ $cmdargs == "run" ]]; then
    filename=$(retFileNameFromGraphicsCard)

    docker-compose -f $filename up &

    sleep 10
    #Check if the docker-compose process is running
    SECONDS=0
    while true;
    do
        if ! pgrep -x "docker-compose" > /dev/null; then
            duration=$SECONDS
            if [ $duration -ge 30 ]; then
                echo "Error"
                exit 1
            fi
        else
            break
        fi

    done
    #Enable container to access the X server of the host
    xhost +local:`docker inspect --format='{{ .Config.Hostname }}' $(docker ps -l -q)`
    sleep 10

    echo "The container is running"
    echo "Press Ctrl-C to stop the container"
    #NOTE: Will this use up a lot of system resources.
    #Could stop the bash script but the user must manually kill the docker-compose process
    #when he/she stops using the container.
    sleep infinity

else
    helpFunction
fi
