#include "ns3/lte-helper.h"
#include "ns3/epc-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/lte-module.h"
#include "ns3/applications-module.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/radio-bearer-stats-calculator.h"
#include "ns3/lte-global-pathloss-database.h"
#include "ns3/log.h"
#include "ns3/lte-amc.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/lte-ue-net-device.h"

#include "ns3/itu-inh-propagation-loss-model.h" 
#include <iomanip>
#include <string>
#include <iostream>
#include <algorithm>

using namespace ns3;
/*This file, combined with the ns-3-LBT model available for download at https://www.nsnam.org/~tomh/ns-3-lbt-documents/html/lbt-wifi-coexistence.html allows the simulation of a 2-tier NB-IoT network, and was used in the paper

Pol Serra i Lidon et al., "Two-tier Architecture for NB-IoT: Improving Coverage and Load Balancing," submitted to ICT 2019.
*/
NS_LOG_COMPONENT_DEFINE ("NB-IoT");

void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon);

int main (int argc, char *argv[])
{
        uint16_t numberOfNodes = 2500;
        double simTime = 30;
        double interPacketIntervalOne = 24000;
	double interPacketIntervalTwo = 2000;
	double interPacketIntervalThree = 1000;
        uint32_t pacchetto = 12*20;	
									
	CommandLine cmd;
	cmd.AddValue("numberOfNodes", "Number of eNodeBs + UE pairs", numberOfNodes);
  	cmd.AddValue("simTime", "Total duration of the simulation [s])", simTime);
 	cmd.AddValue("interPacketIntervalOne", "Inter packet interval two [ms])", interPacketIntervalOne);
 	cmd.AddValue("interPacketIntervalTwo", "Inter packet interval one [ms])", interPacketIntervalTwo);
 	cmd.AddValue("interPacketIntervalThree", "Inter packet interval one [ms])", interPacketIntervalThree);
  	cmd.Parse (argc, argv);

	Time::SetResolution (Time::NS);

	IntegerValue runValue;
  	GlobalValue::GetValueByName ("RngRun", runValue);

	std::ostringstream tag;
  	tag << "_rngRun"  << std::setw (3) << std::setfill ('0')  << runValue.Get () ;

	//Configure the LTE+EPC system

	Ptr<LteHelper> lteHelper = CreateObject<LteHelper> ();
   	Ptr<PointToPointEpcHelper>  epcHelper = CreateObject<PointToPointEpcHelper> ();
  	lteHelper->SetEpcHelper (epcHelper);
  	//epcHelper->Initialize ();

  	lteHelper->SetSchedulerType("ns3::PfFfMacScheduler");
  	Config::SetDefault ("ns3::LteAmc::AmcModel", EnumValue (LteAmc::PiroEW2010)); 
  	Config::SetDefault ("ns3::LteEnbRrc::DefaultTransmissionMode", UintegerValue (0)); // 0=SISO; 1=SIMO; 2=MIMO OPEN 
											   //LOOP; 3=MIMO CLOSED LOOP; 
											   //4=MIMO MULTI-USER
  	
  	
	ConfigStore inputConfig;
  	inputConfig.ConfigureDefaults ();

	cmd.Parse(argc, argv);
	
	
	// Create the PGW

  	Ptr<Node> pgw = epcHelper->GetPgwNode ();

	// Create a single RemoteHost
  	
	NodeContainer remoteHostContainer;
  	remoteHostContainer.Create (1);
  	Ptr<Node> remoteHost = remoteHostContainer.Get (0);
  	InternetStackHelper internet;
  	internet.Install (remoteHostContainer);
	
	// Create the Internet
  	
	PointToPointHelper p2ph;
  	p2ph.SetDeviceAttribute ("DataRate", DataRateValue (DataRate ("100Gb/s")));
  	p2ph.SetDeviceAttribute ("Mtu", UintegerValue (1500));
  	p2ph.SetChannelAttribute ("Delay", TimeValue (Seconds (0.010)));
  	NetDeviceContainer internetDevices = p2ph.Install (pgw, remoteHost);
  	Ipv4AddressHelper ipv4h;
  	ipv4h.SetBase ("1.0.0.0", "255.255.255.0");
  	Ipv4InterfaceContainer internetIpIfaces = ipv4h.Assign (internetDevices);
  	// interface 0 is localhost, 1 is the p2p device
  	Ipv4Address remoteHostAddr = internetIpIfaces.GetAddress (1);

  	Ipv4StaticRoutingHelper ipv4RoutingHelper;
  	Ptr<Ipv4StaticRouting> remoteHostStaticRouting = ipv4RoutingHelper.GetStaticRouting (remoteHost->GetObject<Ipv4> ());
  	remoteHostStaticRouting->AddNetworkRouteTo (Ipv4Address ("7.0.0.0"), Ipv4Mask ("255.255.255.0"), 1);

	//Create UEs and eNB, with mobility model

	NodeContainer ueNodesOne;
	NodeContainer ueNodesTwo;
	NodeContainer ueNodesThree;
  	NodeContainer enbNodes1;
	NodeContainer enbNodes2;
  	enbNodes1.Create(15);
	enbNodes2.Create(15);
        ueNodesOne.Create(0.1*numberOfNodes);
        ueNodesTwo.Create(0.8*numberOfNodes);
        ueNodesThree.Create(0.1*numberOfNodes);

	Config::SetDefault ("ns3::LteEnbRrc::SrsPeriodicity", UintegerValue (320));
	Config::SetDefault ("ns3::LteUePhy::TxPower", DoubleValue (23.0));
	Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (43.0));


	//macro mobility

	Ptr<ListPositionAllocator> positionAlloc1One = CreateObject<ListPositionAllocator> ();
	positionAlloc1One->Add (Vector(1.0, 0.17, 23.0));
	MobilityHelper mobilityENB1One;
	mobilityENB1One.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB1One.SetPositionAllocator (positionAlloc1One);
	mobilityENB1One.Install (enbNodes1.Get(0));
	
	Ptr<ListPositionAllocator> positionAlloc1Two = CreateObject<ListPositionAllocator> ();
	positionAlloc1Two->Add (Vector(-0.5, 0.86, 23.0));
	MobilityHelper mobilityENB1Two;
	mobilityENB1Two.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB1Two.SetPositionAllocator (positionAlloc1Two);
	mobilityENB1Two.Install (enbNodes1.Get(1));
	
	Ptr<ListPositionAllocator> positionAlloc1Three = CreateObject<ListPositionAllocator> ();
	positionAlloc1Three->Add (Vector(0.17, -0.98, 23.0));
	MobilityHelper mobilityENB1Three;
	mobilityENB1Three.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB1Three.SetPositionAllocator (positionAlloc1Three);
	mobilityENB1Three.Install (enbNodes1.Get(2));
	
	
	
	
	Ptr<ListPositionAllocator> positionAlloc2One = CreateObject<ListPositionAllocator> ();
	positionAlloc2One->Add (Vector(-331.1, 697.34, 23.0));
	MobilityHelper mobilityENB2One;
	mobilityENB2One.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB2One.SetPositionAllocator (positionAlloc2One);
	mobilityENB2One.Install (enbNodes1.Get(3));
	
	Ptr<ListPositionAllocator> positionAlloc2Two = CreateObject<ListPositionAllocator> ();
	positionAlloc2Two->Add (Vector(-332.0, 698.0, 23.0));
	MobilityHelper mobilityENB2Two;
	mobilityENB2Two.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB2Two.SetPositionAllocator (positionAlloc2Two);
	mobilityENB2Two.Install (enbNodes1.Get(4));
	
	Ptr<ListPositionAllocator> positionAlloc2Three = CreateObject<ListPositionAllocator> ();
	positionAlloc2Three->Add (Vector(-332.76, 696.35, 23.0));
	MobilityHelper mobilityENB2Three;
	mobilityENB2Three.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB2Three.SetPositionAllocator (positionAlloc2Three);
	mobilityENB2Three.Install (enbNodes1.Get(5));
	
	
	
	
	Ptr<ListPositionAllocator> positionAlloc3One = CreateObject<ListPositionAllocator> ();
	positionAlloc3One->Add (Vector(-677.5, -769.13, 23.0));
	MobilityHelper mobilityENB3One;
	mobilityENB3One.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB3One.SetPositionAllocator (positionAlloc3One);
	mobilityENB3One.Install (enbNodes1.Get(6));
	
	Ptr<ListPositionAllocator> positionAlloc3Two = CreateObject<ListPositionAllocator> ();
	positionAlloc3Two->Add (Vector(-679.0, -770.0, 23.0));
	MobilityHelper mobilityENB3Two;
	mobilityENB3Two.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB3Two.SetPositionAllocator (positionAlloc3Two);
	mobilityENB3Two.Install (enbNodes1.Get(7));
	
	Ptr<ListPositionAllocator> positionAlloc3Three = CreateObject<ListPositionAllocator> ();
	positionAlloc3Three->Add (Vector(-677.5, -770.86, 23.0));
	MobilityHelper mobilityENB3Three;
	mobilityENB3Three.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB3Three.SetPositionAllocator (positionAlloc3Three);
	mobilityENB3Three.Install (enbNodes1.Get(8));
	
	
	
	
	Ptr<ListPositionAllocator> positionAlloc4One = CreateObject<ListPositionAllocator> ();
	positionAlloc4One->Add (Vector(570.98, -377.17, 23.0));
	MobilityHelper mobilityENB4One;
	mobilityENB4One.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB4One.SetPositionAllocator (positionAlloc4One);
	mobilityENB4One.Install (enbNodes1.Get(9));
	
	Ptr<ListPositionAllocator> positionAlloc4Two = CreateObject<ListPositionAllocator> ();
	positionAlloc4Two->Add (Vector(569.1, -376.5, 23.0));
	MobilityHelper mobilityENB4Two;
	mobilityENB4Two.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB4Two.SetPositionAllocator (positionAlloc4Two);
	mobilityENB4Two.Install (enbNodes1.Get(10));
	
	Ptr<ListPositionAllocator> positionAlloc4Three = CreateObject<ListPositionAllocator> ();
	positionAlloc4Three->Add (Vector(569.83, -377.98, 23.0));
	MobilityHelper mobilityENB4Three;
	mobilityENB4Three.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB4Three.SetPositionAllocator (positionAlloc4Three);
	mobilityENB4Three.Install (enbNodes1.Get(11));
	
	
	
	
	Ptr<ListPositionAllocator> positionAlloc5One = CreateObject<ListPositionAllocator> ();
	positionAlloc5One->Add (Vector(-1066.66, -79.1, 23.0));
	MobilityHelper mobilityENB5One;
	mobilityENB5One.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB5One.SetPositionAllocator (positionAlloc5One);
	mobilityENB5One.Install (enbNodes1.Get(12));
	
	Ptr<ListPositionAllocator> positionAlloc5Two = CreateObject<ListPositionAllocator> ();
	positionAlloc5Two->Add (Vector(-1067.98, -79.83, 23.0));
	MobilityHelper mobilityENB5Two;
	mobilityENB5Two.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB5Two.SetPositionAllocator (positionAlloc5Two);
	mobilityENB5Two.Install (enbNodes1.Get(13));
	
	Ptr<ListPositionAllocator> positionAlloc5Three = CreateObject<ListPositionAllocator> ();
	positionAlloc5Three->Add (Vector(-1066.35, -80.77, 23.0));
	MobilityHelper mobilityENB5Three;
	mobilityENB5Three.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityENB5Three.SetPositionAllocator (positionAlloc5Three);
	mobilityENB5Three.Install (enbNodes1.Get(14));




	//mobility of small cells

	Ptr<ListPositionAllocator> positionAlloc1Onee = CreateObject<ListPositionAllocator> ();
	positionAlloc1Onee->Add (Vector(-600.1, 200.2, 0.1));
	MobilityHelper mobilityOne;
	mobilityOne.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityOne.SetPositionAllocator (positionAlloc1Onee);
	mobilityOne.Install (enbNodes2.Get(0));
	
	Ptr<ListPositionAllocator> positionAlloc1Twoo = CreateObject<ListPositionAllocator> ();
	positionAlloc1Twoo->Add (Vector(-1000, 700, 0.1));
	MobilityHelper mobilityTwo;
	mobilityTwo.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityTwo.SetPositionAllocator (positionAlloc1Twoo);
	mobilityTwo.Install (enbNodes2.Get(1));
	
	Ptr<ListPositionAllocator> positionAlloc1Threee = CreateObject<ListPositionAllocator> ();
	positionAlloc1Threee->Add (Vector(600.3, 600.3, 0.1));
	MobilityHelper mobilityThree;
	mobilityThree.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityThree.SetPositionAllocator (positionAlloc1Threee);
	mobilityThree.Install (enbNodes2.Get(2));
	
	Ptr<ListPositionAllocator> positionAlloc2Onee = CreateObject<ListPositionAllocator> ();
	positionAlloc2Onee->Add (Vector(-750.2, 600, 0.1));
	MobilityHelper mobilityFour;
	mobilityFour.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityFour.SetPositionAllocator (positionAlloc2Onee);
	mobilityFour.Install (enbNodes2.Get(3));
	
	Ptr<ListPositionAllocator> positionAlloc2Twoo = CreateObject<ListPositionAllocator> ();
	positionAlloc2Twoo->Add (Vector(-1100.3, -750.1, 0.1));
	MobilityHelper mobilityFive;
	mobilityFive.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityFive.SetPositionAllocator (positionAlloc2Twoo);
	mobilityFive.Install (enbNodes2.Get(4));
	
	Ptr<ListPositionAllocator> positionAlloc2Threee = CreateObject<ListPositionAllocator> ();
	positionAlloc2Threee->Add (Vector(-450.2, -100, 0.1));
	MobilityHelper mobilitySix;
	mobilitySix.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilitySix.SetPositionAllocator (positionAlloc2Threee);
	mobilitySix.Install (enbNodes2.Get(5));
	
	Ptr<ListPositionAllocator> positionAlloc3Onee = CreateObject<ListPositionAllocator> ();
	positionAlloc3Onee->Add (Vector(650.5, 100.2, 0.1));
	MobilityHelper mobilitySeven;
	mobilitySeven.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilitySeven.SetPositionAllocator (positionAlloc3Onee);
	mobilitySeven.Install (enbNodes2.Get(6));
	
	Ptr<ListPositionAllocator> positionAlloc3Twoo = CreateObject<ListPositionAllocator> ();
	positionAlloc3Twoo->Add (Vector(-1000.2, -500.1, 0.1));
	MobilityHelper mobilityEight;
	mobilityEight.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityEight.SetPositionAllocator (positionAlloc3Twoo);
	mobilityEight.Install (enbNodes2.Get(7));
	
	Ptr<ListPositionAllocator> positionAlloc3Threee= CreateObject<ListPositionAllocator> ();
	positionAlloc3Threee->Add (Vector(200.5, -800.1, 0.1));
	MobilityHelper mobilityNine;
	mobilityNine.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityNine.SetPositionAllocator (positionAlloc3Threee);
	mobilityNine.Install (enbNodes2.Get(8));

	Ptr<ListPositionAllocator> positionAlloc4Onee = CreateObject<ListPositionAllocator> ();
	positionAlloc4Onee->Add (Vector(-200, -750.1,0.1));
	MobilityHelper mobilityTen;
	mobilityTen.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityTen.SetPositionAllocator (positionAlloc4Onee);
	mobilityTen.Install (enbNodes2.Get(9));
	
	Ptr<ListPositionAllocator> positionAlloc4Twoo = CreateObject<ListPositionAllocator> ();
	positionAlloc4Twoo->Add (Vector(300.4, 780.2,0.1));
	MobilityHelper mobilityEleven;
	mobilityEleven.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityEleven.SetPositionAllocator (positionAlloc4Twoo);
	mobilityEleven.Install (enbNodes2.Get(10));
	
	Ptr<ListPositionAllocator> positionAlloc4Threee = CreateObject<ListPositionAllocator> ();
	positionAlloc4Threee->Add (Vector(0.2, 500.5, 0.1));
	MobilityHelper mobilityTwelve;
	mobilityTwelve.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityTwelve.SetPositionAllocator (positionAlloc4Threee);
	mobilityTwelve.Install (enbNodes2.Get(11));
	
	Ptr<ListPositionAllocator> positionAlloc5Onee = CreateObject<ListPositionAllocator> ();
	positionAlloc5Onee->Add (Vector(-1100.3, 400.8, 0.1));
	MobilityHelper mobilityThirteen;
	mobilityThirteen.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityThirteen.SetPositionAllocator (positionAlloc5Onee);
	mobilityThirteen.Install (enbNodes2.Get(12));
	
	Ptr<ListPositionAllocator> positionAlloc5Twoo = CreateObject<ListPositionAllocator> ();
	positionAlloc5Twoo->Add (Vector(600.5, -800.6, 0.1));
	MobilityHelper mobilityFourteen;
	mobilityFourteen.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityFourteen.SetPositionAllocator (positionAlloc5Twoo);
	mobilityFourteen.Install (enbNodes2.Get(13));
	
	Ptr<ListPositionAllocator> positionAlloc5Threee = CreateObject<ListPositionAllocator> ();
	positionAlloc5Threee->Add (Vector(0.7, -450, 0.1));
	MobilityHelper mobilityFifteen;
	mobilityFifteen.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
	mobilityFifteen.SetPositionAllocator (positionAlloc5Threee);
	mobilityFifteen.Install (enbNodes2.Get(14));



//NOW THE UEs



	MobilityHelper mobilityUEOne;
	mobilityUEOne.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
 	mobilityUEOne.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
		"X",StringValue ("ns3::UniformRandomVariable[Min=-1200.0|Max=800.0]"),
		"Y",StringValue ("ns3::UniformRandomVariable[Min=-900.0|Max=800.0]"),
		"Z",StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"));

	MobilityHelper mobilityUETwo;
	mobilityUETwo.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
 	mobilityUETwo.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
		"X",StringValue ("ns3::UniformRandomVariable[Min=-1200.0|Max=800.0]"),
		"Y",StringValue ("ns3::UniformRandomVariable[Min=-900.0|Max=800.0]"),
		"Z",StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"));

	MobilityHelper mobilityUEThree;
	mobilityUEThree.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
 	mobilityUEThree.SetPositionAllocator ("ns3::RandomBoxPositionAllocator",
		"X",StringValue ("ns3::UniformRandomVariable[Min=-1200.0|Max=800.0]"),
		"Y",StringValue ("ns3::UniformRandomVariable[Min=-900.0|Max=800.0]"),
		"Z",StringValue ("ns3::UniformRandomVariable[Min=1.0|Max=1.0]"));
	
  	mobilityUEOne.Install (ueNodesOne);
  	mobilityUETwo.Install (ueNodesTwo);	
  	mobilityUEThree.Install (ueNodesThree);

  	Ptr<MobilityModel> modelENB1 = enbNodes1.Get(0)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB2 = enbNodes1.Get(1)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB3 = enbNodes1.Get(2)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB4 = enbNodes1.Get(3)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB5 = enbNodes1.Get(4)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB6 = enbNodes1.Get(5)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB7 = enbNodes1.Get(6)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB8 = enbNodes1.Get(7)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB9 = enbNodes1.Get(8)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB10 = enbNodes1.Get(9)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB11 = enbNodes1.Get(10)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB12 = enbNodes1.Get(11)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB13 = enbNodes1.Get(12)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB14 = enbNodes1.Get(13)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB15 = enbNodes1.Get(14)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB16 = enbNodes2.Get(0)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB17 = enbNodes2.Get(1)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB18 = enbNodes2.Get(2)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB19 = enbNodes2.Get(3)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB20 = enbNodes2.Get(4)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB21 = enbNodes2.Get(5)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB22 = enbNodes2.Get(6)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB23 = enbNodes2.Get(7)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB24 = enbNodes2.Get(8)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB25 = enbNodes2.Get(9)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB26 = enbNodes2.Get(10)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB27 = enbNodes2.Get(11)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB28 = enbNodes2.Get(12)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB29 = enbNodes2.Get(13)->GetObject<MobilityModel>();
	Ptr<MobilityModel> modelENB30 = enbNodes2.Get(14)->GetObject<MobilityModel>();
	
	
        //lteHelper->SetPathlossModelAttribute ("Environment", EnumValue (Urban));
        //lteHelper->SetPathlossModelAttribute ("CitySize", EnumValue (Large));
        //lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (20.0));

	//Install LTE protocol stack on the enB
	
	NetDeviceContainer enbDevs; // = lteHelper->InstallEnbDevice (enbNodes1);
  	NetDeviceContainer enbDevs2;


  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (10));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (20.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (0)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (120));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));	
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (1)));
  	
  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (280));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (20.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (2)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (350));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (20.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (3)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (90));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (4)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (220));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (5)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (6)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (180));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (7)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (300));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (8)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (350));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (9)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (150));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (10)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (220));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (11)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (10));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (20.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (12)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (140));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (13)));

  	lteHelper->SetEnbAntennaModelType ("ns3::CosineAntennaModel");
 	lteHelper->SetEnbAntennaModelAttribute ("Orientation", DoubleValue (300));
  	lteHelper->SetEnbAntennaModelAttribute ("Beamwidth",   DoubleValue (60));
  	lteHelper->SetEnbAntennaModelAttribute ("MaxGain",     DoubleValue (15.0));
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (24300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuR1411NlosOverRooftopPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (800e6));
	lteHelper->SetPathlossModelAttribute ("RooftopLevel", DoubleValue (15.0));
	enbDevs.Add ( lteHelper->InstallEnbDevice (enbNodes1.Get (14)));

//Here the isotropics

	Config::SetDefault ("ns3::LteEnbPhy::TxPower", DoubleValue (23.0)); // change the power to small cells

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (0)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (1)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (2)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (3)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (4)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (5)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (6)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (7)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (8)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (9)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (10)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (11)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (12)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (13)));

	lteHelper->SetAttribute ("PathlossModel", StringValue ("ns3::ItuInhPropagationLossModel"));
	lteHelper->SetPathlossModelAttribute ("Frequency", DoubleValue (2100e6));
	lteHelper->SetEnbAntennaModelType ("ns3::IsotropicAntennaModel");
	lteHelper->SetEnbDeviceAttribute ("UlEarfcn", UintegerValue (18300));
  	lteHelper->SetEnbDeviceAttribute ("UlBandwidth", UintegerValue (12));
	enbDevs2.Add ( lteHelper->InstallEnbDevice (enbNodes2.Get (14)));


  	
  	  	
  	NetDeviceContainer ueDevsOne = lteHelper->InstallUeDevice (ueNodesOne);
	NetDeviceContainer ueDevsTwo = lteHelper->InstallUeDevice (ueNodesTwo);
  	NetDeviceContainer ueDevsThree = lteHelper->InstallUeDevice (ueNodesThree);	
	
	// Install the IP stack on the UEs

  	internet.Install (ueNodesOne);
  	internet.Install (ueNodesTwo);
  	internet.Install (ueNodesThree);  	
  	Ipv4InterfaceContainer ueIpIfaceOne;
  	Ipv4InterfaceContainer ueIpIfaceTwo;
  	Ipv4InterfaceContainer ueIpIfaceThree;
  	ueIpIfaceOne = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevsOne));
  	ueIpIfaceTwo = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevsTwo));
  	ueIpIfaceThree = epcHelper->AssignUeIpv4Address (NetDeviceContainer (ueDevsThree));

  	// Assign IP address to UEs, and install applications

  	for (uint32_t u = 0; u < ueNodesOne.GetN (); ++u)
   	 {
	     	 Ptr<Node> ueNodeOne = ueNodesOne.Get (u);

	      	// Set the default gateway for the UE

	      	Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodeOne->GetObject<Ipv4> ());
	      	ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    	}
	
	for (uint32_t u = 0; u < ueNodesTwo.GetN (); ++u)
   	 {
	     	 Ptr<Node> ueNodeTwo = ueNodesTwo.Get (u);

	      	// Set the default gateway for the UE

	      	Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodeTwo->GetObject<Ipv4> ());
	      	ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    	}
	
	for (uint32_t u = 0; u < ueNodesThree.GetN (); ++u)
   	 {
	     	 Ptr<Node> ueNodeThree = ueNodesThree.Get (u);

	      	// Set the default gateway for the UE

	      	Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNodeThree->GetObject<Ipv4> ());
	      	ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    	}

	// Attach one UE per eNodeB
	
	for (uint32_t u = 0; u < ueNodesOne.GetN (); ++u) 	 
	{
		Ptr<MobilityModel> modelNodeOne = ueNodesOne.Get(u)->GetObject<MobilityModel>();
		double distance1 = modelNodeOne->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeOne->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeOne->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeOne->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeOne->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeOne->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeOne->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeOne->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeOne->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeOne->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeOne->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeOne->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeOne->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeOne->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeOne->GetDistanceFrom(modelENB15);
		double distance16 = modelNodeOne->GetDistanceFrom(modelENB16);
		double distance17 = modelNodeOne->GetDistanceFrom(modelENB17);
		double distance18 = modelNodeOne->GetDistanceFrom(modelENB18);
		double distance19 = modelNodeOne->GetDistanceFrom(modelENB19);
		double distance20 = modelNodeOne->GetDistanceFrom(modelENB20);
		double distance21 = modelNodeOne->GetDistanceFrom(modelENB21);
		double distance22 = modelNodeOne->GetDistanceFrom(modelENB22);
		double distance23 = modelNodeOne->GetDistanceFrom(modelENB23);
		double distance24 = modelNodeOne->GetDistanceFrom(modelENB24);
		double distance25 = modelNodeOne->GetDistanceFrom(modelENB25);
		double distance26 = modelNodeOne->GetDistanceFrom(modelENB26);
		double distance27 = modelNodeOne->GetDistanceFrom(modelENB27);
		double distance28 = modelNodeOne->GetDistanceFrom(modelENB28);
		double distance29 = modelNodeOne->GetDistanceFrom(modelENB29);
		double distance30 = modelNodeOne->GetDistanceFrom(modelENB30);
		
		double distancearray1[] = {distance1, distance2, distance3, distance4, distance5, distance6,  // distance to macro
		distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14,
		distance15};
		double distancearray2[] = {distance16, distance17, distance18, distance19, distance20, distance21, distance22,
		distance23, distance24, distance25, distance26, distance27, distance28, distance29, distance30};

		double dist1 = distancearray1[0];
		double dist2 = distancearray2[0]; 

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray1[i] < dist1 ){
            		dist1 = distancearray1[i];
			}
		}

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray2[i] < dist2 ){
            		dist2 = distancearray2[i];
			}
		}
		
		std::cout <<"dist1: "<<dist1<<", y dist2: "<<dist2<<std::endl;
		if (dist2 < dist1)
		{
			if(dist2<150)
			{
				lteHelper->AttachToClosestEnb (ueDevsOne.Get(u), enbDevs2);
			std::cout << "Indoor closest and in range"<<std::endl;
			}
			else
			{
				lteHelper->AttachToClosestEnb (ueDevsOne.Get(u), enbDevs);
			std::cout << "Outdoor, closer to Indoor but not enough"<<std::endl;
			}			
		}
      		else
      		{
      		lteHelper->AttachToClosestEnb (ueDevsOne.Get(u), enbDevs);
		std::cout << "Outdoor closest"<<std::endl;
      		}

	}
for (uint32_t v = 0; v < ueNodesTwo.GetN (); ++v) 	 
	{
		Ptr<MobilityModel> modelNodeOne = ueNodesTwo.Get(v)->GetObject<MobilityModel>();
		double distance1 = modelNodeOne->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeOne->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeOne->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeOne->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeOne->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeOne->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeOne->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeOne->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeOne->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeOne->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeOne->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeOne->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeOne->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeOne->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeOne->GetDistanceFrom(modelENB15);
		double distance16 = modelNodeOne->GetDistanceFrom(modelENB16);
		double distance17 = modelNodeOne->GetDistanceFrom(modelENB17);
		double distance18 = modelNodeOne->GetDistanceFrom(modelENB18);
		double distance19 = modelNodeOne->GetDistanceFrom(modelENB19);
		double distance20 = modelNodeOne->GetDistanceFrom(modelENB20);
		double distance21 = modelNodeOne->GetDistanceFrom(modelENB21);
		double distance22 = modelNodeOne->GetDistanceFrom(modelENB22);
		double distance23 = modelNodeOne->GetDistanceFrom(modelENB23);
		double distance24 = modelNodeOne->GetDistanceFrom(modelENB24);
		double distance25 = modelNodeOne->GetDistanceFrom(modelENB25);
		double distance26 = modelNodeOne->GetDistanceFrom(modelENB26);
		double distance27 = modelNodeOne->GetDistanceFrom(modelENB27);
		double distance28 = modelNodeOne->GetDistanceFrom(modelENB28);
		double distance29 = modelNodeOne->GetDistanceFrom(modelENB29);
		double distance30 = modelNodeOne->GetDistanceFrom(modelENB30);
		
		double distancearray1[] = {distance1, distance2, distance3, distance4, distance5, distance6,
		distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14,
		distance15};
		double distancearray2[] = {distance16, distance17, distance18, distance19, distance20, distance21, distance22,
		distance23, distance24, distance25, distance26, distance27, distance28, distance29, distance30};

		double dist1 = distancearray1[0];
		double dist2 = distancearray2[0]; 

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray1[i] < dist1 ){
            		dist1 = distancearray1[i];
			}
		}

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray2[i] < dist2 ){
            		dist2 = distancearray2[i];
			}
		}
		
		std::cout <<"dist1: "<<dist1<<", y dist2: "<<dist2<<std::endl;
		if (dist2 < dist1)
		{
			if(dist2<150)
			{
				lteHelper->AttachToClosestEnb (ueDevsTwo.Get(v), enbDevs2);
			std::cout << "Indoor closest and in range"<<std::endl;
			}
			else
			{
				lteHelper->AttachToClosestEnb (ueDevsTwo.Get(v), enbDevs);
			std::cout << "Outdoor, closer to Indoor but not enough"<<std::endl;
			}			
		}
      		else
      		{
      		lteHelper->AttachToClosestEnb (ueDevsTwo.Get(v), enbDevs);
		std::cout << "Outdoor closest"<<std::endl;
      		}

	}

	for (uint32_t w = 0; w < ueNodesThree.GetN (); ++w) 	 
	{
		Ptr<MobilityModel> modelNodeOne = ueNodesThree.Get(w)->GetObject<MobilityModel>();
		double distance1 = modelNodeOne->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeOne->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeOne->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeOne->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeOne->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeOne->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeOne->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeOne->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeOne->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeOne->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeOne->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeOne->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeOne->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeOne->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeOne->GetDistanceFrom(modelENB15);
		double distance16 = modelNodeOne->GetDistanceFrom(modelENB16);
		double distance17 = modelNodeOne->GetDistanceFrom(modelENB17);
		double distance18 = modelNodeOne->GetDistanceFrom(modelENB18);
		double distance19 = modelNodeOne->GetDistanceFrom(modelENB19);
		double distance20 = modelNodeOne->GetDistanceFrom(modelENB20);
		double distance21 = modelNodeOne->GetDistanceFrom(modelENB21);
		double distance22 = modelNodeOne->GetDistanceFrom(modelENB22);
		double distance23 = modelNodeOne->GetDistanceFrom(modelENB23);
		double distance24 = modelNodeOne->GetDistanceFrom(modelENB24);
		double distance25 = modelNodeOne->GetDistanceFrom(modelENB25);
		double distance26 = modelNodeOne->GetDistanceFrom(modelENB26);
		double distance27 = modelNodeOne->GetDistanceFrom(modelENB27);
		double distance28 = modelNodeOne->GetDistanceFrom(modelENB28);
		double distance29 = modelNodeOne->GetDistanceFrom(modelENB29);
		double distance30 = modelNodeOne->GetDistanceFrom(modelENB30);
		
		double distancearray1[] = {distance1, distance2, distance3, distance4, distance5, distance6,
		distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14,
		distance15};
		double distancearray2[] = {distance16, distance17, distance18, distance19, distance20, distance21, distance22,
		distance23, distance24, distance25, distance26, distance27, distance28, distance29, distance30};

		double dist1 = distancearray1[0];
		double dist2 = distancearray2[0]; 

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray1[i] < dist1 ){
            		dist1 = distancearray1[i];
			}
		}

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray2[i] < dist2 ){
            		dist2 = distancearray2[i];
			}
		}
		
		std::cout <<"dist1: "<<dist1<<", y dist2: "<<dist2<<std::endl;
		if (dist2 < dist1)
		{
			if(dist2<150)
			{
				lteHelper->AttachToClosestEnb (ueDevsThree.Get(w), enbDevs2);
			std::cout << "Indoor closest and in range"<<std::endl;
			}
			else
			{
				lteHelper->AttachToClosestEnb (ueDevsThree.Get(w), enbDevs);
			std::cout << "Outdoor, closer to Indoor but not enough"<<std::endl;
			}			
		}
      		else
      		{
      		lteHelper->AttachToClosestEnb (ueDevsThree.Get(w), enbDevs);
		std::cout << "Outdoor closest"<<std::endl;
      		}

	}


	// randomize a bit start times to avoid simulation artifacts
	// (e.g., buffer overflows due to packet transmissions happening
  	// exactly at the same time) 
  	Ptr<UniformRandomVariable> startTimeSecondsOne = CreateObject<UniformRandomVariable> ();
  	startTimeSecondsOne->SetAttribute ("Min", DoubleValue (0));
  	startTimeSecondsOne->SetAttribute ("Max", DoubleValue (interPacketIntervalOne/1000.0));
  	//startTimeSecondsOne->SetAttribute ("Max", DoubleValue (simTime/2000));

  	Ptr<UniformRandomVariable> startTimeSecondsTwo = CreateObject<UniformRandomVariable> ();
  	startTimeSecondsTwo->SetAttribute ("Min", DoubleValue (0));
  	startTimeSecondsTwo->SetAttribute ("Max", DoubleValue (interPacketIntervalOne/1000.0));
  	//startTimeSecondsTwo->SetAttribute ("Max", DoubleValue (simTime/3000));

  	Ptr<UniformRandomVariable> startTimeSecondsThree = CreateObject<UniformRandomVariable> ();
  	startTimeSecondsThree->SetAttribute ("Min", DoubleValue (0));
  	startTimeSecondsThree->SetAttribute ("Max", DoubleValue (interPacketIntervalOne/1000.0));
  	//startTimeSecondsThree->SetAttribute ("Max", DoubleValue (simTime/4000));


  	// Install and start applications on UEs and remote host
  	
	uint16_t dlPort = 1234;
  	uint16_t ulPort = 2000;
	ApplicationContainer clientApps;
   	ApplicationContainer serverApps;
	
	for (uint32_t u = 0; u < ueNodesOne.GetN (); ++u) 	 
	{
      		++ulPort;		
		PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
     	 	PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
		serverApps.Add (dlPacketSinkHelper.Install (ueNodesOne.Get(u)));
      		serverApps.Add (ulPacketSinkHelper.Install (remoteHost));


	
		UdpClientHelper dlClientOne (ueIpIfaceOne.GetAddress (u), dlPort);
      		//dlClientOne.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalOne)));
      		dlClientOne.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalOne)));
      		dlClientOne.SetAttribute ("MaxPackets", UintegerValue(1000000));
      		dlClientOne.SetAttribute ("PacketSize", UintegerValue(pacchetto));
      		      		
		UdpClientHelper ulClientOne (remoteHostAddr, ulPort);
      		//ulClientOne.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalOne)));
      		ulClientOne.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalOne)));
      		ulClientOne.SetAttribute ("MaxPackets", UintegerValue(1000000));
		//ulClientOne.SetAttribute ("PacketSize", UintegerValue(pacchetto));

		
		Ptr<MobilityModel> modelNodeOne = ueNodesOne.Get(u)->GetObject<MobilityModel>();
		double distance1 = modelNodeOne->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeOne->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeOne->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeOne->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeOne->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeOne->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeOne->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeOne->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeOne->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeOne->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeOne->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeOne->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeOne->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeOne->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeOne->GetDistanceFrom(modelENB15);
		
		double distancearray[] = {distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14, distance15};
		

    		double distance = 30000;
                int imsi=0;

    		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] < distance )
            		distance = distancearray[i];
			
			}
		imsi =ueDevsOne.Get(u)->GetObject<LteUeNetDevice>()->GetImsi();
			std::cout<<imsi<<", ";

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] == distance )
			std::cout<<i+1<<", ";
			}
		std::cout<<"A, ";
		std::cout<<distance<<", ";
		/*The if statement below is used to select devices that are allowed to retransmit. This can be used to selectively enable retransmissions only for devices that experience a low efficiency in normal conditions; the set of such devices can be determined by first running a run with the setting now active (no retransmissions)
and then repeating it with the list of imsi IDs of devices that experienced low efficiency (see commented block for an example)*/
		if(imsi==-1
		   /*				  	
			if(imsi==511||
imsi==523||
imsi==930||
imsi==1224||
imsi==2181||
imsi==1811||
imsi==1965||
imsi==955||
imsi==957||
imsi==2249||
imsi==1924||
imsi==612||
imsi==2471||
imsi==2355||
imsi==2426||
imsi==2261||
imsi==45||
imsi==54||
imsi==134||
imsi==241||
imsi==778||
imsi==433||
imsi==223||
imsi==347||
imsi==2460||
imsi==64||
imsi==184||
imsi==195||
imsi==224||
imsi==1445||
imsi==1010||
imsi==2179||
imsi==2390||
imsi==2409||
imsi==1645||
imsi==2040||
imsi==2184||
imsi==1742||
imsi==56||
imsi==58||
imsi==1390||
imsi==2407||
imsi==1436||
imsi==1187||
imsi==764||
imsi==2310||
imsi==431||
imsi==635||
imsi==1559||
imsi==2028||
imsi==2442||
imsi==1194||
imsi==812||
imsi==2284||
imsi==438||
imsi==1095||
imsi==1792||
imsi==2419||
imsi==2491||
imsi==2333||
imsi==2301||
imsi==2329||
imsi==2480||
imsi==542||
imsi==2477||
imsi==515||
imsi==2385||
imsi==2263||
imsi==2283||
imsi==2254||
imsi==2396||
imsi==2465||
imsi==2314||
imsi==2293||
imsi==2381||
imsi==2258||
imsi==2302||
imsi==2257||
imsi==2470||
imsi==2445||
imsi==2441||
imsi==2282||
imsi==2397||
imsi==2453||
imsi==2286||
imsi==2415||
imsi==2348||
imsi==2379||
imsi==2383||
imsi==2436||
imsi==2335||
imsi==2264||
imsi==2253||
imsi==2297||
imsi==352||
imsi==1779||
imsi==1778||
imsi==2016||
imsi==629||
imsi==1545||
imsi==602||
imsi==642||
imsi==700||
imsi==158||
imsi==246||
imsi==1916||
imsi==337||
imsi==12||
imsi==74||
imsi==159||
imsi==33||
imsi==1481||
imsi==2009||
imsi==902||
imsi==1028||
imsi==1427||
imsi==913||
imsi==2193||
imsi==269||
imsi==1094||
imsi==557||
imsi==2289||
imsi==399||
imsi==1646||
imsi==2476||
imsi==474||
imsi==2066||
imsi==1977||
imsi==2144||
imsi==559||
imsi==312||
imsi==1089||
imsi==1304||
imsi==1741||
imsi==1883||
imsi==1190||
imsi==1888||
imsi==1790||
imsi==2005)
		   */
		{
		  ulClientOne.SetAttribute ("PacketSize", UintegerValue(pacchetto*32));//The mupltiplying factor here determines the number of retransmissions (32 in this case)
		std::cout<<"YESrepeat"<<std::endl;
		}
      		else
      		{
      		ulClientOne.SetAttribute ("PacketSize", UintegerValue(pacchetto));
		std::cout<<"NOrepeat"<<std::endl;
      		}
		
		
				
      		clientApps.Add (dlClientOne.Install (remoteHost));
      		clientApps.Add (ulClientOne.Install (ueNodesOne.Get(u)));

      		serverApps.Start (Seconds ((startTimeSecondsOne->GetValue ())));
      		clientApps.Start (Seconds ((startTimeSecondsOne->GetValue ())));

    	}

	for (uint32_t v = 0; v < ueNodesTwo.GetN (); ++v) 	 
	{
      		++ulPort;		
		PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
     	 	PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
		serverApps.Add (dlPacketSinkHelper.Install (ueNodesTwo.Get(v)));
      		serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

		UdpClientHelper dlClientTwo (ueIpIfaceTwo.GetAddress (v), dlPort);
      		//dlClientTwo.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalTwo)));
      		dlClientTwo.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalTwo)));
      		dlClientTwo.SetAttribute ("MaxPackets", UintegerValue(1000000));
      		dlClientTwo.SetAttribute ("PacketSize", UintegerValue(200));
      		
		UdpClientHelper ulClientTwo (remoteHostAddr, ulPort);
      		//ulClientTwo.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalTwo)));
      		ulClientTwo.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalTwo)));
      		ulClientTwo.SetAttribute ("MaxPackets", UintegerValue(1000000));
		
		Ptr<MobilityModel> modelNodeTwo = ueNodesTwo.Get(v)->GetObject<MobilityModel>();
		double distance1 = modelNodeTwo->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeTwo->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeTwo->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeTwo->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeTwo->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeTwo->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeTwo->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeTwo->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeTwo->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeTwo->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeTwo->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeTwo->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeTwo->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeTwo->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeTwo->GetDistanceFrom(modelENB15);
		
		
		double distancearray[] = {distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14, distance15};
		
    		double distance = 30000;
int imsi=0;
    		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] < distance )
            		distance = distancearray[i];
			
			}
		imsi =ueDevsTwo.Get(v)->GetObject<LteUeNetDevice>()->GetImsi();
			std::cout<<imsi<<", ";

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] == distance )
			std::cout<<i+1<<", ";
			}
		std::cout<<"B, ";
		std::cout<<distance<<", ";
		if(imsi==511||
imsi==523||
imsi==930||
imsi==1224||
imsi==2181||
imsi==1811||
imsi==1965||
imsi==955||
imsi==957||
imsi==2249||
imsi==1924||
imsi==612||
imsi==2471||
imsi==2355||
imsi==2426||
imsi==2261||
imsi==45||
imsi==54||
imsi==134||
imsi==241||
imsi==778||
imsi==433||
imsi==223||
imsi==347||
imsi==2460||
imsi==64||
imsi==184||
imsi==195||
imsi==224||
imsi==1445||
imsi==1010||
imsi==2179||
imsi==2390||
imsi==2409||
imsi==1645||
imsi==2040||
imsi==2184||
imsi==1742||
imsi==56||
imsi==58||
imsi==1390||
imsi==2407||
imsi==1436||
imsi==1187||
imsi==764||
imsi==2310||
imsi==431||
imsi==635||
imsi==1559||
imsi==2028||
imsi==2442||
imsi==1194||
imsi==812||
imsi==2284||
imsi==438||
imsi==1095||
imsi==1792||
imsi==2419||
imsi==2491||
imsi==2333||
imsi==2301||
imsi==2329||
imsi==2480||
imsi==542||
imsi==2477||
imsi==515||
imsi==2385||
imsi==2263||
imsi==2283||
imsi==2254||
imsi==2396||
imsi==2465||
imsi==2314||
imsi==2293||
imsi==2381||
imsi==2258||
imsi==2302||
imsi==2257||
imsi==2470||
imsi==2445||
imsi==2441||
imsi==2282||
imsi==2397||
imsi==2453||
imsi==2286||
imsi==2415||
imsi==2348||
imsi==2379||
imsi==2383||
imsi==2436||
imsi==2335||
imsi==2264||
imsi==2253||
imsi==2297||
imsi==352||
imsi==1779||
imsi==1778||
imsi==2016||
imsi==629||
imsi==1545||
imsi==602||
imsi==642||
imsi==700||
imsi==158||
imsi==246||
imsi==1916||
imsi==337||
imsi==12||
imsi==74||
imsi==159||
imsi==33||
imsi==1481||
imsi==2009||
imsi==902||
imsi==1028||
imsi==1427||
imsi==913||
imsi==2193||
imsi==269||
imsi==1094||
imsi==557||
imsi==2289||
imsi==399||
imsi==1646||
imsi==2476||
imsi==474||
imsi==2066||
imsi==1977||
imsi==2144||
imsi==559||
imsi==312||
imsi==1089||
imsi==1304||
imsi==1741||
imsi==1883||
imsi==1190||
imsi==1888||
imsi==1790||
imsi==2005)
		{
		ulClientTwo.SetAttribute ("PacketSize", UintegerValue(pacchetto*32));
		std::cout<<"YESrepeat"<<std::endl;
		}
      		else
      		{
      		ulClientTwo.SetAttribute ("PacketSize", UintegerValue(pacchetto));
		std::cout<<"NOrepeat"<<std::endl;
      		}

      		clientApps.Add (dlClientTwo.Install (remoteHost));
      		clientApps.Add (ulClientTwo.Install (ueNodesTwo.Get(v)));

      		serverApps.Start (Seconds ((startTimeSecondsTwo->GetValue ())));
      		clientApps.Start (Seconds ((startTimeSecondsTwo->GetValue ())));

    	}

	for (uint32_t w = 0; w < ueNodesThree.GetN (); ++w) 	 
	{
      		++ulPort;		
		PacketSinkHelper dlPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), dlPort));
     	 	PacketSinkHelper ulPacketSinkHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), ulPort));
		serverApps.Add (dlPacketSinkHelper.Install (ueNodesThree.Get(w)));
      		serverApps.Add (ulPacketSinkHelper.Install (remoteHost));

		UdpClientHelper dlClientThree (ueIpIfaceThree.GetAddress (w), dlPort);
      		//dlClientThree.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalThree)));
      		dlClientThree.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalThree)));
      		dlClientThree.SetAttribute ("MaxPackets", UintegerValue(1000000));
      		dlClientThree.SetAttribute ("PacketSize", UintegerValue(200));
      		
		UdpClientHelper ulClientThree (remoteHostAddr, ulPort);
      		//ulClientThree.SetAttribute ("Interval", TimeValue (Seconds(interPacketIntervalThree)));
      		ulClientThree.SetAttribute ("Interval", TimeValue (MilliSeconds(interPacketIntervalThree)));
      		ulClientThree.SetAttribute ("MaxPackets", UintegerValue(1000000));
		
		
		Ptr<MobilityModel> modelNodeThree = ueNodesThree.Get(w)->GetObject<MobilityModel>();
		double distance1 = modelNodeThree->GetDistanceFrom(modelENB1);
		double distance2 = modelNodeThree->GetDistanceFrom(modelENB2);
		double distance3 = modelNodeThree->GetDistanceFrom(modelENB3);
		double distance4 = modelNodeThree->GetDistanceFrom(modelENB4);
		double distance5 = modelNodeThree->GetDistanceFrom(modelENB5);
		double distance6 = modelNodeThree->GetDistanceFrom(modelENB6);
		double distance7 = modelNodeThree->GetDistanceFrom(modelENB7);
		double distance8 = modelNodeThree->GetDistanceFrom(modelENB8);
		double distance9 = modelNodeThree->GetDistanceFrom(modelENB9);
		double distance10 = modelNodeThree->GetDistanceFrom(modelENB10);
		double distance11 = modelNodeThree->GetDistanceFrom(modelENB11);
		double distance12 = modelNodeThree->GetDistanceFrom(modelENB12);
		double distance13 = modelNodeThree->GetDistanceFrom(modelENB13);
		double distance14 = modelNodeThree->GetDistanceFrom(modelENB14);
		double distance15 = modelNodeThree->GetDistanceFrom(modelENB15);
		
		
		double distancearray[] = {distance1, distance2, distance3, distance4, distance5, distance6, distance7, distance8, distance9, distance10, distance11, distance12, distance13, distance14, distance15};
		int imsi=0;
    		double distance = 30000;

    		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] < distance )
            		distance = distancearray[i];
			
			}
		imsi =ueDevsThree.Get(w)->GetObject<LteUeNetDevice>()->GetImsi();
			std::cout<<imsi<<", ";

		for ( int i = 0; i <= 14; i++ )
        		{
        		if ( distancearray[i] == distance )
			std::cout<<i+1<<", ";
			}
		std::cout<<"C, ";
		std::cout<<distance<<", ";
		if(imsi==511||
imsi==523||
imsi==930||
imsi==1224||
imsi==2181||
imsi==1811||
imsi==1965||
imsi==955||
imsi==957||
imsi==2249||
imsi==1924||
imsi==612||
imsi==2471||
imsi==2355||
imsi==2426||
imsi==2261||
imsi==45||
imsi==54||
imsi==134||
imsi==241||
imsi==778||
imsi==433||
imsi==223||
imsi==347||
imsi==2460||
imsi==64||
imsi==184||
imsi==195||
imsi==224||
imsi==1445||
imsi==1010||
imsi==2179||
imsi==2390||
imsi==2409||
imsi==1645||
imsi==2040||
imsi==2184||
imsi==1742||
imsi==56||
imsi==58||
imsi==1390||
imsi==2407||
imsi==1436||
imsi==1187||
imsi==764||
imsi==2310||
imsi==431||
imsi==635||
imsi==1559||
imsi==2028||
imsi==2442||
imsi==1194||
imsi==812||
imsi==2284||
imsi==438||
imsi==1095||
imsi==1792||
imsi==2419||
imsi==2491||
imsi==2333||
imsi==2301||
imsi==2329||
imsi==2480||
imsi==542||
imsi==2477||
imsi==515||
imsi==2385||
imsi==2263||
imsi==2283||
imsi==2254||
imsi==2396||
imsi==2465||
imsi==2314||
imsi==2293||
imsi==2381||
imsi==2258||
imsi==2302||
imsi==2257||
imsi==2470||
imsi==2445||
imsi==2441||
imsi==2282||
imsi==2397||
imsi==2453||
imsi==2286||
imsi==2415||
imsi==2348||
imsi==2379||
imsi==2383||
imsi==2436||
imsi==2335||
imsi==2264||
imsi==2253||
imsi==2297||
imsi==352||
imsi==1779||
imsi==1778||
imsi==2016||
imsi==629||
imsi==1545||
imsi==602||
imsi==642||
imsi==700||
imsi==158||
imsi==246||
imsi==1916||
imsi==337||
imsi==12||
imsi==74||
imsi==159||
imsi==33||
imsi==1481||
imsi==2009||
imsi==902||
imsi==1028||
imsi==1427||
imsi==913||
imsi==2193||
imsi==269||
imsi==1094||
imsi==557||
imsi==2289||
imsi==399||
imsi==1646||
imsi==2476||
imsi==474||
imsi==2066||
imsi==1977||
imsi==2144||
imsi==559||
imsi==312||
imsi==1089||
imsi==1304||
imsi==1741||
imsi==1883||
imsi==1190||
imsi==1888||
imsi==1790||
imsi==2005)
		{
		ulClientThree.SetAttribute ("PacketSize", UintegerValue(pacchetto*32));
		std::cout<<"YESrepeat"<<std::endl;
		}
      		else
      		{
      		ulClientThree.SetAttribute ("PacketSize", UintegerValue(pacchetto));
		std::cout<<"NOrepeat"<<std::endl;
      		}


      		clientApps.Add (dlClientThree.Install (remoteHost));
      		clientApps.Add (ulClientThree.Install (ueNodesThree.Get(w)));

      		serverApps.Start (Seconds ((startTimeSecondsThree->GetValue ())));
      		clientApps.Start (Seconds ((startTimeSecondsThree->GetValue ())));

    	}
/*
	//Insert RLC Performance Calculator
	Ptr<RadioEnvironmentMapHelper> remHelper;
  	
//      Use this commented code to get SNR rem.out file to show in gnuplot
      	remHelper = CreateObject<RadioEnvironmentMapHelper> ();
  	remHelper->SetAttribute ("ChannelPath", StringValue ("/ChannelList/1"));
  	remHelper->SetAttribute ("OutputFile", StringValue ("rem.out"));
  	remHelper->SetAttribute ("XMin", DoubleValue (-1200.0));
  	remHelper->SetAttribute ("XMax", DoubleValue (800.0));
	remHelper->SetAttribute ("XRes", UintegerValue (400));
  	remHelper->SetAttribute ("YMin", DoubleValue (-900.0));
  	remHelper->SetAttribute ("YMax", DoubleValue (800.0));
	remHelper->SetAttribute ("YRes", UintegerValue (400));
  	remHelper->SetAttribute ("Z", DoubleValue (0.1));
  	remHelper->Install ();

*/




        std::string dlOutFname = "DlRlcStats";
	dlOutFname.append (tag.str ());
        std::string ulOutFname = "UlRlcStats";
	ulOutFname.append (tag.str ());

	lteHelper->EnableTraces ();

	//FlowMonitorHelper fmHelper;
	//Ptr<FlowMonitor> allMon = fmHelper.InstallAll();
	//Simulator::Schedule(Seconds(simTime+simTime*0.2),&ThroughputMonitor,&fmHelper, allMon);

	Simulator::Stop (Seconds (simTime));
  	Simulator::Run ();

  	//ThroughputMonitor(&fmHelper, allMon);

	Simulator::Destroy();
	return 0;
}

void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon)
	{
		std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
		Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
		for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats)
		{
			Ipv4FlowClassifier::FiveTuple fiveTuple = classing->FindFlow (stats->first);
			std::cout<<"Flow ID			: " << stats->first <<" ; "<< fiveTuple.sourceAddress <<" -----> "<<fiveTuple.destinationAddress<<std::endl;
			std::cout<<"Tx Bytes = " << stats->second.txBytes<<std::endl;
			std::cout<<"Rx Bytes = " << stats->second.rxBytes<<std::endl;
			std::cout<<"Duration		: "<<stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds()<<std::endl;
			std::cout<<"Last Received Packet	: "<< stats->second.timeLastRxPacket.GetSeconds()<<" Seconds"<<std::endl;
			std::cout<<"Throughput: " << stats->second.rxBytes * 8.0 / (stats->second.timeLastRxPacket.GetSeconds()-stats->second.timeFirstTxPacket.GetSeconds())/1024/1024  << " 				Mbps"<<std::endl;
			std::cout<<"---------------------------------------------------------------------------"<<std::endl;
		}
			Simulator::Schedule(Seconds(1),&ThroughputMonitor, fmhelper, flowMon);

	}
