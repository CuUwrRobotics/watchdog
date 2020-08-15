#include "WdMain.h"

int main(int argc, char *argv[]) {
	atexit(stopWatchdog); // Allows for a peaceful death

	// Get ROS connected first
	ros::init(argc, argv, "watchdog");
	ros::NodeHandle n;
	// How the watchdog gets pet
	ros::Subscriber wd_petting_msg = n.subscribe("pet_dog_msg", 1000,
	                                             petCallback);
	// How the watchdog outputs it's status
	wd_status_msg = n.advertise <watchdog::watchdog_status_msg> (
		"watchdog_status_msg", 0);

	// Get watchdog config from the system environment variables
	const char *mainConfigFileName = std::getenv("ros_configuration_file");
	if (mainConfigFileName == nullptr) { // invalid to assign nullptr to std::string
		error_printf("No such environment variable 'ros_configuration_file'!");
		exit(EXIT_FAILURE);
	}
	printf("Reading main config file %s\n", mainConfigFileName);

	// Get the location of the watchdog YAML config file
	YAML::Node mainConfigYAML = YAML::LoadFile(mainConfigFileName);
	wd_conf_file_name =
		mainConfigYAML["watchdog_config_file"].as <std::string> ();

	// ROS_INFO("CONFIG STRING: %s", yamlConfigFile);
	wd_conf_file = YAML::LoadFile(wd_conf_file_name);
	printf("Loading watchdog config file %s\n", wd_conf_file_name.c_str());
	readWatchdogConfig();
	// If no nodes were read, exit.
	if (nodes.size() == 0) {
		printf("Did not get any nodes to watch. Leaving.");
		exit(EXIT_SUCCESS);
	}
	// Start the actual watchdog
	runContinuousWatchdog();

	return 0;
} // main

/**
 * Reads all nodes in the config file. Each node must be assigned a priority and a
 * name to check.
 * example config YAML file:
 *
 # - priority: 0 # highest
 #   name: "system_critical_node"
 #   time: 1000 # time in ms before fault
 # - priority: 4 # watchdog won't look for this
 #   name: "unimportant_node"
 #   time: 1000 # time in ms before fault
 *
 * Priority ranges from 0 - 3.
 */

void readWatchdogConfig(){
	uint8_t priority = PRIORITY_IGNORE;
	uint8_t nodesIgnored = 0;
	uint8_t nodeErrors = 0;
	CheckableNode tempNode;
	for (int nodeConfig = 0; nodeConfig < wd_conf_file.size(); nodeConfig++) { // For each nodeConfig
		if (nodeConfig >= MAX_NODES_TO_WATCH) {
			error_printf("Got too many nodes: %d, max=%d. Will not look for more.",
			             nodeConfig, MAX_NODES_TO_WATCH);
			break;
		}
		// get priority
		tempNode.priority = wd_conf_file[0]["priority"].as <int> ();

		// Check for invalid priority
		if (tempNode.priority != PRIORITY_CRITICAL &&
		    tempNode.priority != PRIORITY_HIGH &&
		    tempNode.priority != PRIORITY_MED &&
		    tempNode.priority != PRIORITY_LOW &&
		    tempNode.priority != PRIORITY_IGNORE) {
			nodeErrors++;
			error_printf("Got a bad priority for node config %d: %d", nodeConfig,
			             tempNode.priority);
			continue; // don't worry about the rest of the data
		}
		if (tempNode.priority == PRIORITY_IGNORE) {
			nodesIgnored++;
			// warn_printf("Ignoring node config # %d", nodeConfig);
			continue; // don't worry about the rest of the data
		}

		tempNode.nodeName = wd_conf_file[nodeConfig]["name"].as <std::string> ();
		tempNode.timeBeforeExp =
			wd_conf_file[nodeConfig]["time"].as <int> ();

		// Instert the node in the vector. Order doesn't matter.
		nodes.insert(nodes.begin(), tempNode);
	} // For each nodeConfig in wd_conf_file

	// Print out the data
	printf("%sConfig completed. Results: %s\n", COLOR_GOLD, COLOR_NONE);
	uint8_t critical = 0, high = 0, medium = 0, low  = 0; // ignore and error are already counted
	for (int i = 0; i < nodes.size(); i++) {
		switch (nodes.at(i).priority) {
		case PRIORITY_CRITICAL:
			critical++;
			break;
		case PRIORITY_HIGH:
			high++;
			break;
		case PRIORITY_MED:
			medium++;
			break;
		case PRIORITY_LOW:
			low++;
			break;
		default: break;
		} // switch
	}
	printf("\tCritical:\t%d\n", critical);
	printf("\tHigh:    \t%d\n", high);
	printf("\tMedium:  \t%d\n", medium);
	printf("\tLow:     \t%d\n", low);
	if (nodesIgnored != 0)
		printf("\t%sIgnored:  \t%d%s\n", COLOR_YELLOW, nodesIgnored, COLOR_NONE);
	if (nodeErrors != 0)
		printf("\t%sErrors:   \t%d%s\n", COLOR_L_RED, nodeErrors, COLOR_NONE);
} // readWatchdogConfig

void stopWatchdog(){
	// Humanely kills watchdog
	printf("Stopping watchdog.\n");
	ros::shutdown();
} // stopWatchdog

void printFaults() {
	printf("Total Faults: %d\n", totalFaults);
} // printFaults

void petCallback(const watchdog::pet_dog_msg &msg) {
	for (int i = 0; i < nodes.size(); i++) {
		// reset the counter for this node
		if (strcmp(nodes.at(i).nodeName.c_str(), msg.petterName.c_str()) == 0) {
			nodes.at(i).petCounter = 0;
			if (nodes.at(i).fault)
				printf("%s%s%s pet the watchdog after expiring.%s\n",
				       COLOR_WHITE,
				       nodes.at(i).nodeName.c_str(),
				       COLOR_L_GREEN, COLOR_NONE);
			nodes.at(i).fault = false;
		}
	}
	// printf("Counter reset for: %s\n", msg.petterName.c_str());
} // callback

bool allNodesOk() {
	totalFaults = 0; // reset global fault counter
	for (int i = 0; i < nodes.size(); i++) {
		if (!nodes.at(i).fault) { // Skip nodes that have already faulted
			// Check if there is a fault (time elapsed > max time)
			nodes.at(i).fault = (nodes.at(i).petCounter > nodes.at(i).timeBeforeExp);
			// If there's a fault, flag it, otherwise keep incrementing
			if (nodes.at(i).fault) {
				totalFaults++;
				printf("%s%s%s watchdog timer has expired!%s\n",
				       COLOR_WHITE,
				       nodes.at(i).nodeName.c_str(),
				       COLOR_L_RED, COLOR_NONE);
			}	else nodes.at(i).petCounter += TIME_BETWEEN_CHECKS_MS;
		} else totalFaults++;
	}
	return (totalFaults == 0);
} // allNodesOk

void runContinuousWatchdog() {
	watchdog::watchdog_status_msg status;
	// Make sure ROS is running first
	if (!ros::ok()) {
		error_printf("ROS is not ready yet! Waiting for ROS to initialize.");
		while (!ros::ok()) {}
	}
	printf("%sStarted watching for messages from nodes.%s\n",
	       COLOR_GOLD, COLOR_NONE);
	ros::Duration loop_wait((float)TIME_BETWEEN_CHECKS_MS / 1000); // loops every 100 ms
	printFaults();
	while (ros::ok()) {
		ros::spinOnce();

		allNodesOk(); // Check and increemnt timer on all nodes, while counting totalFaults

		// Publish the watchdog status to the message
		status.faults = totalFaults;
		wd_status_msg.publish(status);

		// Print total faults to console if there are changes
		if (totalFaults != totalPreviousFaults) {
			totalPreviousFaults = totalFaults;
			printFaults();
		}

		loop_wait.sleep(); // Wait the rest of the 100 ms
	}
} // runContinuousWatchdog
