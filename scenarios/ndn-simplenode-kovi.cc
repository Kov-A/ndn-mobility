/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Copyright (c) 2014 Waseda University
 * Author: Jairo Eduardo Lopez <jairo@ruri.waseda.jp>
 *
 * ndn-mobility-random.cc
 *  Random walk Wifi Mobile scenario for ndnSIM
 *
 * Special thanks to University of Washington for initial templates
 */

// Standard C++ modules
#include <algorithm>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iterator>
#include <iostream>
#include <string>
#include <sys/time.h>
#include <vector>

// Random modules
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int_distribution.hpp>
#include <boost/random/variate_generator.hpp>

// ns3 modules
#include <ns3-dev/ns3/applications-module.h>
#include <ns3-dev/ns3/bridge-helper.h>
#include <ns3-dev/ns3/csma-module.h>
#include <ns3-dev/ns3/core-module.h>
#include <ns3-dev/ns3/mobility-module.h>
#include <ns3-dev/ns3/network-module.h>
#include <ns3-dev/ns3/point-to-point-module.h>
#include <ns3-dev/ns3/wifi-module.h>

// ndnSIM modules
#include <ns3-dev/ns3/ndnSIM-module.h>
#include <ns3-dev/ns3/ndnSIM/utils/tracers/ipv4-rate-l3-tracer.h>
#include <ns3-dev/ns3/ndnSIM/utils/tracers/ipv4-seqs-app-tracer.h>

using namespace ns3;
using namespace boost;
using namespace std;

namespace br = boost::random;

typedef struct timeval TIMER_TYPE;
#define TIMER_NOW(_t) gettimeofday (&_t,NULL);
#define TIMER_SECONDS(_t) ((double)(_t).tv_sec + (_t).tv_usec*1e-6)
#define TIMER_DIFF(_t1, _t2) (TIMER_SECONDS (_t1)-TIMER_SECONDS (_t2))

char scenario[250] = "NDNMobilityRandom";

NS_LOG_COMPONENT_DEFINE (scenario);

// Number generator
br::mt19937_64 gen;

// Obtains a random number from a uniform distribution between min and max.
// Must seed number generator to ensure randomness at runtime.
int obtain_Num(int min, int max) {
    br::uniform_int_distribution<> dist(min, max);
    return dist(gen);
}

std::vector<Ptr<Node> > getVector(NodeContainer node) {

	uint32_t size = node.GetN ();

	std::vector<Ptr<Node> > nodemutable;

	// Copy the Node pointers into a mutable vector
	for (uint32_t i = 0; i < size; i++) {
		nodemutable.push_back (node.Get(i));
	}

	NS_LOG_INFO ("getVector: returning Node vector");

	return nodemutable;
}

// Randomly picks toAsig nodes from a vector that has nodesAvailable in size
std::vector<Ptr<Node> > assignNodes(std::vector<Ptr<Node> > nodes, int toAsig, int nodesAvailable) {

	char buffer[250];

	sprintf(buffer, "assignNodes: to assign %d, left %d", toAsig, nodesAvailable);

	NS_LOG_INFO (buffer);

	std::vector<Ptr<Node> > assignedNodes;

	uint32_t assignMin = nodesAvailable - toAsig;

	// Apply Fisher-Yates shuffle
	for (uint32_t i = nodesAvailable; i > assignMin; i--)
	{
		// Get a random number
		int toSwap = obtain_Num (0, i);
		// Push into the client container
		assignedNodes.push_back (nodes[toSwap]);
		// Swap the obtained number with the last element
		std::swap (nodes[toSwap], nodes[i]);
	}

	return assignedNodes;
}

// Obtains a random list of num_clients clients and num_server server from a NodeContainer
tuple<std::vector<Ptr<Node> >, std::vector<Ptr<Node> > > assignClientsandserver(NodeContainer nodes, int num_clients, int num_server) {

	char buffer[250];

	// Get the number of nodes in the simulation
	uint32_t size = nodes.GetN ();

	sprintf(buffer, "assignClientsandserver, we have %d nodes, will assign %d clients and %d server", size, num_clients, num_server);

	NS_LOG_INFO (buffer);

	// Check that we haven't asked for a scenario where we don't have enough Nodes to fulfill
	// the requirements
	if (num_clients + num_server > size) {
		NS_LOG_INFO("assignClientsandServer, required number bigger than container size!");
		return tuple<std::vector<Ptr<Node> >, std::vector<Ptr<Node> > > ();
	}

	std::vector<Ptr<Node> > nodemutable = getVector(nodes);

	std::vector<Ptr<Node> > ClientContainer = assignNodes(nodemutable, num_clients, size-1);

	std::vector<Ptr<Node> > ServerContainer = assignNodes(nodemutable, num_server, size-1-num_clients);

	return tuple<std::vector<Ptr<Node> >, std::vector<Ptr<Node> > > (ClientContainer, ServerContainer);
}

// Returns a randomly picked num of Nodes from nodes Container
std::vector<Ptr<Node> > assignWithinContainer (NodeContainer nodes, int num)
{
	char buffer[250];

	// Get the number of nodes in the simulation
	uint32_t size = nodes.GetN ();

	sprintf(buffer, "assignWithinContainer, we have %d nodes, will assign %d", size, num);

	NS_LOG_INFO (buffer);

	if (num > size) {
		NS_LOG_INFO("assignWithinContainer, required number bigger than container size!");
		return std::vector<Ptr<Node> >();
	}

	std::vector<Ptr<Node> > nodemutable = getVector(nodes);

	return assignNodes(nodemutable, num, size-1);

}

// Function to get a complete Random setup
tuple<std::vector<Ptr<Node> >, std::vector<Ptr<Node> > > assignCompleteRandom(int num_clients, int num_server) {

	// Obtain all the node used in the simulation
	NodeContainer global = NodeContainer::GetGlobal ();

	return assignClientsandserver(global, num_clients, num_server);
}

int main (int argc, char *argv[])
{
	// These are our scenario arguments
	// uint32_t sectors = 9;                       // Number of wireless sectors
	uint32_t aps = 2;					        // Number of wireless access nodes in a sector
	uint32_t mobile = 1;				        // Number of mobile terminals
	uint32_t server = 1;				        // Number of server in the network
	uint32_t xaxis = 100;                       // Size of the X axis
	uint32_t yaxis = 100;                       // Size of the Y axis
	int posCC = -1;                             // Establish which node will be client
	double sec = 0.0;                           // Movement start
        double distance = 100;                      // Distance from the AP to the node
	double waitint = 1.0;                       // Wait at AP
	double travelTime = 3.0;                    // Travel time within APs
	double walkSpeed = 4;                     // Speed of walk
	bool traceFiles = false;                    // Tells to run the simulation with traceFiles
	bool smart = false;                         // Tells to run the simulation with SmartFlooding
	bool bestr = false;                         // Tells to run the simulation with BestRoute
	bool walk = true;                           // Do random walk at walking speed
	bool car = false;                           // Do random walk at car speed
	char results[250] = "results";              // Directory to place results
	char posFile[250] = "rand-hex.txt";          // File including the positioning of the nodes

	// Variable for buffer
	char buffer[250];
      
	CommandLine cmd;
	cmd.AddValue ("mobile", "Number of mobile terminals in simulation", mobile);
	cmd.AddValue ("server", "Number of server in the simulation", server);
	cmd.AddValue ("results", "Directory to place results", results);
        cmd.AddValue ("distance", "Distance separating the node and AP", distance);
	cmd.AddValue ("start", "Starting second", sec);
	cmd.AddValue ("waitint", "Wait interval between APs", waitint);
	cmd.AddValue ("travel", "Travel time between APs", travelTime);
	cmd.AddValue ("pos", "Position ", posCC);
	cmd.AddValue ("trace", "Enable trace files", traceFiles);
	cmd.AddValue ("smart", "Enable SmartFlooding forwarding", smart);
	cmd.AddValue ("bestr", "Enable BestRoute forwarding", bestr);
	cmd.AddValue ("walk", "Enable random walk at walking speed", walk);
	cmd.AddValue ("car", "Enable random walk at car speed", car);
	
	cmd.Parse (argc,argv);

	// Node definitions for mobile terminals (consumers)
	NodeContainer mobileTerminalContainer;
	mobileTerminalContainer.Create(mobile);

	uint32_t mtId = mobileTerminalContainer.Get (0)->GetId();

        // Container for server (producer) nodes
	NodeContainer serverNodes;
	serverNodes.Create(server);

	// Wireless access Nodes
	NodeContainer wirelessContainer;
	wirelessContainer.Create(aps);
	
	// Container for all NDN capable nodes
	NodeContainer allNdnNodes;
	allNdnNodes.Add (serverNodes);
	allNdnNodes.Add (wirelessContainer);

	// Container for all nodes without NDN specific capabilities
	NodeContainer allUserNodes;
	allUserNodes.Add (mobileTerminalContainer);
        //allUserNodes.Add (serverNodes);

	// Make sure to seed our random
	gen.seed (std::time (0) + (long long)getpid () << 32);

	NS_LOG_INFO ("Placing server nodes");
	MobilityHelper serverStations;

	Ptr<ListPositionAllocator> initialCenter = CreateObject<ListPositionAllocator> ();

	for (int i = 0; i < server; i++)
	{
		Vector pos (100, 100, 0.0);
		initialCenter->Add (pos);
	}

	serverStations.SetPositionAllocator(initialCenter);
	serverStations.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	serverStations.Install(serverNodes);


	NS_LOG_INFO ("Placing wireless access nodes");
	MobilityHelper wirelessStations;

	Ptr<ListPositionAllocator> initialWireless = CreateObject<ListPositionAllocator> ();

	for (int i = 1; i <= aps; i++)
	{
		Vector pos (100-50.0*i, 50.0*i, 0.0);
		initialWireless->Add (pos);
	}

	wirelessStations.SetPositionAllocator(initialWireless);
	wirelessStations.SetMobilityModel("ns3::ConstantPositionMobilityModel");
	wirelessStations.Install(wirelessContainer);
        

	NS_LOG_INFO ("Placing mobile node");
	MobilityHelper mobileStations;

	Ptr<ListPositionAllocator> initialMobile = CreateObject<ListPositionAllocator> ();

	initialMobile->Add(Vector(50, 150, 0.0));
        mobileStations.SetPositionAllocator(initialMobile);
        mobileStations.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobileStations.Install(mobileTerminalContainer);
	sprintf(buffer, "0|%d|0|%d", xaxis, yaxis);

	string bounds = string(buffer);
	
	if (walk)
	{
		NS_LOG_INFO("Random walk at human walking speed - 1.4m/s");
		sprintf(buffer, "ns3::ConstantRandomVariable[Constant=%f]", walkSpeed);
	}

	string speed = string(buffer);
 /*
	mobileStations.SetPositionAllocator(initialMobile);
	mobileStations.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
	                             "Mode", StringValue ("Distance"),
	                             "Distance", StringValue ("15"),
	                             "Speed", StringValue (speed),
	                             "Bounds", StringValue (bounds));

	mobileStations.Install(mobileTerminalContainer);
*/
	// Connect the server to the lone core node

        vector <NetDeviceContainer> ptpWLANCenterDevices;

	PointToPointHelper p2p_100mbps5ms;
	p2p_100mbps5ms.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
	p2p_100mbps5ms.SetChannelAttribute ("Delay", StringValue ("5ms"));
	NetDeviceContainer ptpServerlowerNdnDevices;
	ptpServerlowerNdnDevices.Add (p2p_100mbps5ms.Install (serverNodes.Get (0), wirelessContainer.Get(0)));
	ptpServerlowerNdnDevices.Add (p2p_100mbps5ms.Install (serverNodes.Get (0), wirelessContainer.Get(1)));
	
	NS_LOG_INFO ("Creating Wireless cards");

	// Use the Wifi Helper to define the wireless interfaces for APs
	WifiHelper wifi = WifiHelper::Default ();
	wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");

	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
	wifiChannel.AddPropagationLoss ("ns3::ThreeLogDistancePropagationLossModel");
	wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");

	// All interfaces are placed on the same channel. Makes AP changes easy. Might
	// have to be reconsidered for multiple mobile nodes
	YansWifiPhyHelper wifiPhyHelper = YansWifiPhyHelper::Default ();
	wifiPhyHelper.SetChannel (wifiChannel.Create ());
	wifiPhyHelper.Set("TxPowerStart", DoubleValue(16.0206));
	wifiPhyHelper.Set("TxPowerEnd", DoubleValue(1));

	// Add a simple no QoS based card to the Wifi interfaces
	NqosWifiMacHelper wifiMacHelper = NqosWifiMacHelper::Default ();

	// Create SSIDs for all the APs
	std::vector<Ssid> ssidV;

	NS_LOG_INFO ("Creating ssids for wireless cards");

	for (int i = 0; i < aps; i++)
	{
		ssidV.push_back (Ssid ("ap-" + boost::lexical_cast<std::string>(i)));
	}

	NS_LOG_INFO ("Assigning mobile terminal wireless cards");

	NS_LOG_INFO ("Assigning AP wireless cards");
	std::vector<NetDeviceContainer> wifiAPNetDevices;
	for (int i = 0; i < aps; i++)
	{
		wifiMacHelper.SetType ("ns3::ApWifiMac",
						   "Ssid", SsidValue (ssidV[i]),
						   "BeaconGeneration", BooleanValue (true),
						   "BeaconInterval", TimeValue (Seconds (0.1)));

		wifiAPNetDevices.push_back (wifi.Install (wifiPhyHelper, wifiMacHelper, wirelessContainer.Get (i)));
	}

	// Create a Wifi station type MAC
	wifiMacHelper.SetType("ns3::StaWifiMac",
			"Ssid", SsidValue (ssidV[0]),
			"ActiveProbing", BooleanValue (true));

	NetDeviceContainer wifiMTNetDevices = wifi.Install (wifiPhyHelper, wifiMacHelper, mobileTerminalContainer);
        
	NetDeviceContainer wifiMTNetDevices2 = wifi.Install (wifiPhyHelper, wifiMacHelper, mobileTerminalContainer);


	char routeType[250];

	// Now install content stores and the rest on the middle node. Leave
	// out clients and the mobile node
	NS_LOG_INFO ("Installing NDN stack on routers");
	ndn::StackHelper ndnHelperRouters;
        
        
	// Decide what Forwarding strategy to use depending on user command line input
	if (smart) {
		sprintf(routeType, "%s", "smart");
		NS_LOG_INFO ("NDN Utilizing SmartFloInstalling NDN stack on routersoding");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::SmartFlooding::PerOutFaceLimits", "Limit", "ns3::ndn::Limits::Window");
	} else if (bestr) {
		sprintf(routeType, "%s", "bestr");
		NS_LOG_INFO ("NDN Utilizing BestRoute");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::BestRoute::PerOutFaceLimits", "Limit", "ns3::ndn::Limits::Window");
	} else {
		sprintf(routeType, "%s", "flood");
		NS_LOG_INFO ("NDN Utilizing Flooding");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::Flooding::PerOutFaceLimits", "Limit", "ns3::ndn::Limits::Window");
	} 
     // Creating Pcap for mobiles and server node
		
        wifiPhyHelper.EnablePcap ("mobilenode", mobileTerminalContainer, true);

/*
        // Decide what Forwarding strategy to use depending on user command line input
	if (smart) {
		sprintf(routeType, "%s", "smart");
		NS_LOG_INFO ("NDN Utilizing SmartFloInstalling NDN stack on routersoding");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::SmartFlooding");
	} else if (bestr) {
		sprintf(routeType, "%s", "bestr");
		NS_LOG_INFO ("NDN Utilizing BestRoute");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
	} else {
		sprintf(routeType, "%s", "flood");
		NS_LOG_INFO ("NDN Utilizing Flooding");
		ndnHelperRouters.SetForwardingStrategy ("ns3::ndn::fw::Flooding");
	}
*/
	// Set the Content Stores
	ndnHelperRouters.SetContentStore ("ns3::ndn::cs::Freshness::Lru", "MaxSize", "1000");
	ndnHelperRouters.SetDefaultRoutes (true);
	// Install on ICN capable routers
	ndnHelperRouters.Install (allNdnNodes);

	// Create a NDN stack for the clients and mobile node
	ndn::StackHelper ndnHelperUsers;
	// These nodes have only one interface, so BestRoute forwarding makes sense
	ndnHelperUsers.SetForwardingStrategy ("ns3::ndn::fw::BestRoute");
	// No Content Stores are installed on these machines
	ndnHelperUsers.SetContentStore ("ns3::ndn::cs::Nocache");
	ndnHelperUsers.SetDefaultRoutes (true);
	ndnHelperUsers.Install (allUserNodes);

	NS_LOG_INFO ("Installing Producer Application");
	// Create the producer on the mobile node
	ndn::AppHelper producerHelper ("ns3::ndn::Producer");
	producerHelper.SetPrefix ("/waseda/sato");
	producerHelper.SetAttribute("StopTime", TimeValue (Seconds(sec)));
	producerHelper.Install (serverNodes);

	NS_LOG_INFO ("Installing Consumer Application");
	// Create the consumer on the randomly selected node
	ndn::AppHelper consumerHelper ("ns3::ndn::ConsumerCbr");
	consumerHelper.SetPrefix ("/waseda/sato");
	consumerHelper.SetAttribute ("Frequency", DoubleValue (147.6));
	consumerHelper.SetAttribute("StartTime", TimeValue (Seconds(travelTime /2)));
	consumerHelper.SetAttribute("StopTime", TimeValue (Seconds(sec-1)));
	consumerHelper.Install (mobileTerminalContainer);

	sprintf(buffer, "Ending time! %f", sec-1);
	NS_LOG_INFO(buffer);

	// If the variable is set, print the trace files
	if (traceFiles) {
		// Filename
		char filename[250];

		// File ID
		char fileId[250];

		// Create the file identifier
		sprintf(fileId, "%s-%02d-%03d-%03d.txt", routeType, mobile, server, aps);

		// Print server nodes to file
		sprintf(filename, "%s/%s-server-%d", results, scenario, fileId);
		NS_LOG_INFO ("Installing tracers");
		// NDN Aggregate tracer
		sprintf (filename, "%s/%s-aggregate-trace-%s", results, scenario, fileId);
		ndn::L3AggregateTracer::InstallAll(filename, Seconds (1.0));

		// NDN L3 tracer
		sprintf (filename, "%s/%s-rate-trace-%s", results, scenario, fileId);
		ndn::L3RateTracer::InstallAll (filename, Seconds (1.0));

		// NDN App Tracer
		sprintf (filename, "%s/%s-app-delays-%s", results, scenario, fileId);
		ndn::AppDelayTracer::InstallAll (filename);

		// L2 Drop rate tracer
		sprintf (filename, "%s/%s-drop-trace-%s", results, scenario, fileId);
		L2RateTracer::InstallAll (filename, Seconds (0.5));

		// Content Store tracer
		sprintf (filename, "%s/%s-cs-trace-%s", results, scenario, fileId);
		ndn::CsTracer::InstallAll (filename, Seconds (1));
	}

	NS_LOG_INFO ("Ready for execution!");
	
	Simulator::Stop (Seconds (20.0));
	Simulator::Run ();
	Simulator::Destroy ();
}

