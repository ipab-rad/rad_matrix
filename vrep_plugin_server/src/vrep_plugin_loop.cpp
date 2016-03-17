// Copyright 2006-2015 Coppelia Robotics GmbH. All rights reserved.
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
//
// -------------------------------------------------------------------
// THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
//
// You are free to use/modify/distribute this file for whatever purpose!
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.2.2 Rev1 on September 5th 2015

#include "../include/v_repLib.h"
#include "../include/vrep_plugin_server/vrep_plugin_loop.h"
#include "../include/vrep_plugin_server/ros_server.h"

#include "ros/ros.h"
#include <iostream>

#define PLUGIN_VERSION 1

LIBRARY vrepLib; // the V-REP library that we will dynamically load and bind

ROSServer ros_server; // The ros server

// This is the plugin start routine (called just once, just after the plugin was loaded):
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,
                                        int reservedInt) {
	// Dynamically load and bind V-REP functions:
	// ******************************************
	// 1. Figure out this plugin's directory:
	char curDirAndFile[1024];
	getcwd(curDirAndFile, sizeof(curDirAndFile));

	std::string currentDirAndPath(curDirAndFile);
	// 2. Append the V-REP library's name:
	std::string temp(currentDirAndPath);
#ifdef _WIN32
	temp += "\\v_rep.dll";
#elif defined (__linux)
	temp += "/libv_rep.so";
#elif defined (__APPLE__)
	temp += "/libv_rep.dylib";
#endif

	// 3. Load the V-REP library:
	vrepLib = loadVrepLibrary(temp.c_str());
	if (vrepLib == NULL) {
		std::cout <<
		          "Error, could not find or correctly load the V-REP library. Cannot start 'rosSkeleton' plugin.\n";
		return (0); // Means error, V-REP will unload this plugin
	}
	if (getVrepProcAddresses(vrepLib) == 0) {
		std::cout <<
		          "Error, could not find all required functions in the V-REP library. Cannot start 'rosSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return (0); // Means error, V-REP will unload this plugin
	}
	// ******************************************

	// Check the version of V-REP:
	// ******************************************
	int vrepVer;
	simGetIntegerParameter(sim_intparam_program_version, &vrepVer);
	if (vrepVer < 30102) { // if V-REP version is smaller than 3.01.02
		std::cout <<
		          "Sorry, your V-REP copy is somewhat old. Cannot start 'rosSkeleton' plugin.\n";
		unloadVrepLibrary(vrepLib);
		return (0); // Means error, V-REP will unload this plugin
	}
	// ******************************************


	// Initialize the ROS part:
	if (!ros_server.initialize()) {
		std::cout << "ROS master is not running. Cannot start 'rosSkeleton' plugin.\n";
		return (0); //If the master is not running then the plugin is not loaded.
	}

	return (PLUGIN_VERSION); // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

// This is the plugin end routine (called just once, when V-REP is ending, i.e. releasing this plugin):
VREP_DLLEXPORT void v_repEnd() {
	ros_server.shutDown();	// shutdown the ROS_server
	unloadVrepLibrary(vrepLib); // release the library
}

// This is the plugin messaging routine (i.e. V-REP calls this function very often, with various messages):
VREP_DLLEXPORT void* v_repMessage(int message, int* auxiliaryData,
                                  void* customData, int* replyData) {
	// This is called quite often. Just watch out for messages/events you want to handle
	// Keep following 4 lines at the beginning and unchanged:
	int errorModeSaved;
	simGetIntegerParameter(sim_intparam_error_report_mode, &errorModeSaved);
	simSetIntegerParameter(sim_intparam_error_report_mode,
	                       sim_api_errormessage_ignore);
	void* retVal = NULL;

	// Here we can intercept many messages from V-REP (actually callbacks). Only the most important messages are listed here:

	if (message == sim_message_eventcallback_instancepass) {
		// This message is sent each time the scene was rendered (well, shortly after) (very often)
		// When a simulation is not running, but you still need to execute some commands, then put some code here

		ros_server.instancePass();
	}

	if (message == sim_message_eventcallback_mainscriptabouttobecalled) {
		// Main script is about to be run (only called while a simulation is running (and not paused!))
		//
		// This is a good location to execute simulation commands

		ros_server.mainScriptAboutToBeCalled();
	}

	if (message == sim_message_eventcallback_simulationabouttostart) {
		// Simulation is about to start

		ros_server.simulationAboutToStart();
	}

	if (message == sim_message_eventcallback_simulationended) {
		// Simulation just ended

		ros_server.simulationEnded();
	}

	// Keep following unchanged:
	simSetIntegerParameter(sim_intparam_error_report_mode,
	                       errorModeSaved); // restore previous settings
	return (retVal);
}

