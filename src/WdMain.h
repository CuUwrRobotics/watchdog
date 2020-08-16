#ifndef WD_MAIN_H
#define WD_MAIN_H

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <vector>

#include "watchdog/pet_dog_msg.h"
#include "watchdog/watchdog_status_msg.h"

// WATCHDOG STATUS MESSAGE ////////////////////////////////////////////////////
ros::Publisher wd_status_msg; // Message publisher for the status message
uint8_t totalFaults = 0; // The number of faults ot pack in the message
uint8_t totalPreviousFaults = 0;

// FUNCTION DEFINITIONS ///////////////////////////////////////////////////////
void readWatchdogConfig();

void stopWatchdog();

void runContinuousWatchdog();

void petCallback(const watchdog::pet_dog_msg &msg);

// CONFIGURATIONS /////////////////////////////////////////////////////////////
static std::string wd_conf_file_name;
YAML::Node wd_conf_file; // Assigned and read at initialization

// NODES TO WATCH /////////////////////////////////////////////////////////////
// Prevents balooning of memory in case config file is broken
#define MAX_NODES_TO_WATCH	32
// Node priorities
#define PRIORITY_CRITICAL		0
#define PRIORITY_HIGH				1
#define PRIORITY_MED				2
#define PRIORITY_LOW				3
#define PRIORITY_IGNORE			4

// Stores info about each node to check
struct CheckableNode {
	// All times in ms
	uint8_t priority;
	bool fault;
	std::string nodeName;
	int petCounter; // Counts how long it's been sincethe last pet
	int timeBeforeExp; // How high the petCounter can go before the watchdog expires
};

// Holds all nodes to be checked
std::vector < CheckableNode > nodes;

#define TIME_BETWEEN_CHECKS_MS 100 // in ms

// PRINTERS ///////////////////////////////////////////////////////////////////
// Prints pretty errors and warnings
#define warn_printf(a, ...) \
	printf("\e[1;33mInternal Warning: " a "\e[0;37m [from %s (in %s:%d)]\e[0m\n" \
	       __VA_OPT__(, ) __VA_ARGS__,  \
	       __FUNCTION__, __FILE__, __LINE__)
#define error_printf(a, ...) \
	printf("\e[1;31mInternal Error: " a "\e[1;30m [from %s (in %s:%d)]\e[0m\n" \
	       __VA_OPT__(, ) __VA_ARGS__,  \
	       __FUNCTION__, __FILE__, __LINE__)

// COLORS /////////////////////////////////////////////////////////////////////
// const char COLOR_BLACK[]    = "\e[0;30m";
// const char COLOR_D_GRAY[]   = "\e[1;30m";
// const char COLOR_RED[]      = "\e[0;31m";
const char COLOR_L_RED[]    = "\e[1;31m";
// const char COLOR_GREEN[]    = "\e[0;32m";
const char COLOR_L_GREEN[]  = "\e[1;32m";
const char COLOR_GOLD[]     = "\e[0;33m";
const char COLOR_YELLOW[]   = "\e[1;33m";
// const char COLOR_BLUE[]     = "\e[0;34m";
// const char COLOR_L_BLUE[]   = "\e[1;34m";
// const char COLOR_PURPLE[]   = "\e[0;35m";
// const char COLOR_L_PURPLE[] = "\e[1;35m";
// const char COLOR_CYAN[]     = "\e[0;36m";
// const char COLOR_L_CYAN[]   = "\e[1;36m";
// const char COLOR_L_GRAY[]   = "\e[0;37m";
const char COLOR_WHITE[]    = "\e[1;38m";
const char COLOR_NONE[]     = "\e[0m";

#endif /* end of include guard: WD_MAIN_H */
