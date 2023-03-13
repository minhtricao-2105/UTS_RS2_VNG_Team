The Package Need To Be Installed Inside The Ros Workspace Of Your Computer In Order To Run This Code. Below Is Some Step To Do It:

If you want to use your own python or C++ file to control, first you need to create your own Ros Package: 

For me, it’s a good idea to put your package inside the ros workplace (catkin/src), I have tested that, and it won't conflict another package inside that workplace. 

First, Go To Your Ros Workplace: 

	cd ~/catkin_ws/src 

	catkin_create_pkg my_python_package rospy 

	cd my_python_package 

	mkdir scripts 

	touch scripts/my_python_program.py 

	chmod +x scripts/my_python_program.py 

Then rebuild your catkin workplace 

How to run the package: 

	rosrun my_python_package my_python_program.py 
