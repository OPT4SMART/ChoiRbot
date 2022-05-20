
## Instructions to bringup the docker environment(ROS 2 dashing with gazebo-11)

- Tested on docker version 20.10.7
- Requires docker compose v1.28.0 or higher


**Install the following packages if you have a dedicated nvidia graphics card**

-  Add the GPG key:
	
	```
	distribution=$(. /etc/os-release;echo $ID$VERSION_ID) \
   	&& curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-	key add - \
   	&& curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
   	```
  
- Install the nvidia-docker2 package 
  
  	```
  	sudo apt-get update
  	```
  	
  	```
  	sudo apt-get install -y nvidia-docker2
  	```
   
- Restart the Docker daemon
     
   	```
   	sudo systemctl restart docker
   	``` 

**Instructions to build the image and run the container**

- Place the Dockerfile, the docker-compose.yml file  and the docker-compose_gpu.yml file inside ~/your_workspace/. **NOTE:** Your entire src/ folder will be mounted as a volume in the container.

- To build the image (changed the filename to docker-compose.yml if your system doesn't have an nvidia gpu):

	```
	docker-compose -f docker-compose_gpu.yml build
	```
	
	NOTE:  This step might be time consuming.

- To run the container:
	```
	cd  ~/your_workspace/
	```

	```
    docker-compose -f docker-compose_gpu.yml up -d
	 ```
	 
	 Wait for the docker containers to run. In another terminal 
	 
	 ```
	 docker-compose exec app /bin/bash
	 ```
