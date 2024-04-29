#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/he-configuration.h"

#include "ns3/ap-wifi-mac.h"
#include "ns3/boolean.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/ctrl-headers.h"
#include "ns3/double.h"
#include "ns3/he-configuration.h"
#include "ns3/he-phy.h"
#include "ns3/he-ppdu.h"
#include "ns3/interference-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/multi-model-spectrum-channel.h"
#include "ns3/nist-error-rate-model.h"
#include "ns3/node.h"
#include "ns3/non-communicating-net-device.h"
#include "ns3/pointer.h"
#include "ns3/rng-seed-manager.h"
#include "ns3/simulator.h"
#include "ns3/spectrum-wifi-helper.h"
#include "ns3/spectrum-wifi-phy.h"
#include "ns3/sta-wifi-mac.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/threshold-preamble-detection-model.h"
#include "ns3/waveform-generator.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-phy-listener.h"
#include "ns3/wifi-psdu.h"
#include "ns3/wifi-spectrum-phy-interface.h"
#include "ns3/wifi-spectrum-signal-parameters.h"
#include "ns3/wifi-spectrum-value-helper.h"
#include "ns3/wifi-utils.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <numeric>

// ./ns3 run ofdm-wifi2.cc

# define PKT_NUM 1500
# define PKT_INTERVAL 0.001
# define PKT_SIZE 1024

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiOfdmaExample");


std::map<uint32_t, Time> packetSentTime;
std::map<uint32_t, Time> packetRecord;
std::map<uint32_t, double> packetRecord3;
std::vector<double> fctValues;
double succCounter = 0;
double sendCounter = 0;

void PacketSentRecord(std::string context, Ptr<const Packet> packet) {
  uint32_t id = packet->GetUid();
  packetRecord3[id] = 0;
}

void PacketSentTrace(std::string context, Ptr<const Packet> packet) {
  uint32_t id = packet->GetUid();
  packetSentTime[id] = Simulator::Now();
  sendCounter++;

}

void PacketReceivedTrace(std::string context, Ptr<const Packet> packet) {
  uint32_t id = packet->GetUid();
  if ((packetSentTime.find(id) != packetSentTime.end()) && (packetRecord.find(id) == packetRecord.end())) {
    succCounter++;
    Time fct = Simulator::Now() - packetSentTime[id];
    double fctSeconds = fct.GetSeconds();
    fctValues.push_back(fctSeconds);
    std::cout << "Packet " << id << " FCT: " << fctSeconds << "s" << std::endl;
    packetSentTime.erase(id);
    packetRecord[id] = Simulator::Now();

    if (packetRecord3.find(id) != packetRecord3.end()) {
      packetRecord3[id] = fctSeconds;
    } 
  }

}

void CalculateAverageFct() {
  if (fctValues.empty()) {
    std::cout << "No FCT values recorded." << std::endl;
    return;
  }

  double sum = std::accumulate(fctValues.begin(), fctValues.end(), 0.0);
  double average = sum / fctValues.size();

  std::cout << "Average FCT: " << average << "s" << std::endl;
  std::cout << "succCounter: " << succCounter << std::endl;
  std::cout << "succRatio: " << succCounter / (PKT_NUM * 6.0) << std::endl;
  std::cout << "packetRecord len: " << packetRecord.size() << std::endl;
  std::cout << "sendCounter: " << sendCounter << std::endl;
}

void writeResult() {

  std::ofstream outputFile("/home/wyc/tarballs/ns-allinone-3.36/ns-3.36/scratch/output.txt", std::ios::trunc);

  // 检查文件是否成功打开
  if (outputFile.is_open()) {

    // for (size_t i = 0; i < fctValues.size(); ++i) {
    //     outputFile << fctValues[i];
    //     if (i != fctValues.size() - 1) {
    //         outputFile << ",";
    //     }
    // }

    for (const auto& pair : packetRecord3) {
        if (pair.second != 0) { 
            outputFile << pair.second;
            outputFile << ",";
        }
    }

    outputFile.close();  // 关闭文件
    std::cout << "done: output.txt" << std::endl;
  } else {
      std::cout << "error: output.txt" << std::endl;
  }   

  return;
}


int main(int argc, char *argv[]) {
  bool verbose = true;
  uint32_t nWifi = 6; // Number of STA nodes
  
  CommandLine cmd;
  cmd.AddValue("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue("verbose", "Tell echo applications to log if true", verbose);
  cmd.Parse(argc, argv);
  
  if (verbose) {
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);
  }

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create(nWifi);
  NodeContainer wifiApNode;
  wifiApNode.Create(1);

  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel>();

  Ptr<RangePropagationLossModel> lossModel = CreateObject<RangePropagationLossModel>();
  lossModel->SetAttribute("MaxRange", DoubleValue(10000.0)); 
  spectrumChannel->AddPropagationLossModel(lossModel);

  Ptr<ConstantSpeedPropagationDelayModel> delayModel =
      CreateObject<ConstantSpeedPropagationDelayModel>();
  delayModel->SetAttribute("Speed", DoubleValue(299792458.0));
  spectrumChannel->SetPropagationDelayModel(delayModel);


  SpectrumWifiPhyHelper phy;
  phy.SetChannel(spectrumChannel); 
  phy.SetErrorRateModel("ns3::NistErrorRateModel");
  phy.Set("ChannelSettings", StringValue("{0, 0, BAND_5GHZ, 0}"));

  phy.Set("ChannelWidth", UintegerValue(20));
  phy.Set("MaxSupportedTxSpatialStreams", UintegerValue(1));
  phy.Set("MaxSupportedRxSpatialStreams", UintegerValue(1));

  Ssid ssid = Ssid("ns-3-ssid");

  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MuBeAifsn", UintegerValue(8));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MuBkAifsn", UintegerValue(15));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MuViAifsn", UintegerValue(5));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/MuVoAifsn", UintegerValue(5));

  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/BeMuEdcaTimer", TimeValue(MicroSeconds(8192)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/BkMuEdcaTimer", TimeValue(MicroSeconds(16384)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/ViMuEdcaTimer", TimeValue(MicroSeconds(4096)));
  Config::Set("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/HeConfiguration/VoMuEdcaTimer", TimeValue(MicroSeconds(2048)));


  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ax); // Use 802.11ax (Wi-Fi 6)

  // wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
  //                               "DataMode",
  //                               StringValue("HeMcs0"),
  //                               "ControlMode",
  //                               StringValue("HeMcs0"));
  
  wifi.SetRemoteStationManager("ns3::MinstrelHtWifiManager");


  WifiMacHelper mac;
  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDevices;
  staDevices = wifi.Install(phy, mac, wifiStaNodes); 
  
  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid), "BeaconInterval", TimeValue(MicroSeconds(102400)), "EnableBeaconJitter", BooleanValue(true));
  mac.SetMultiUserScheduler("ns3::RrMultiUserScheduler");
  NetDeviceContainer apDevices;
  apDevices = wifi.Install(phy, mac, wifiApNode);
  
  phy.EnablePcap("ofdma-monitor", apDevices);

  // Set mobility
  MobilityHelper mobility;
  mobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0), "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(5.0), "DeltaY", DoubleValue(5.0), "GridWidth", UintegerValue(3), "LayoutType", StringValue("RowFirst"));
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(wifiStaNodes);
  mobility.Install(wifiApNode);
  
  // Install Internet stack
  InternetStackHelper stack;
  stack.Install(wifiApNode);
  stack.Install(wifiStaNodes);
  
  // Assign IP addresses
  Ipv4AddressHelper address;
  address.SetBase("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staInterfaces;
  staInterfaces = address.Assign(staDevices);
  Ipv4InterfaceContainer apInterface;
  apInterface = address.Assign(apDevices);
  
  // Install applications
  UdpEchoServerHelper echoServer(9);
  ApplicationContainer serverApps = echoServer.Install(wifiApNode.Get(0));
  serverApps.Start(Seconds(1.0));
  serverApps.Stop(Seconds(150.0));
  
  UdpEchoClientHelper echoClient(apInterface.GetAddress(0), 9);
  echoClient.SetAttribute("MaxPackets", UintegerValue(PKT_NUM));
  echoClient.SetAttribute("Interval", TimeValue(Seconds(PKT_INTERVAL)));
  echoClient.SetAttribute("PacketSize", UintegerValue(PKT_SIZE));
  ApplicationContainer clientApps = echoClient.Install(wifiStaNodes);
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(150.0));
  
  Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoClient/Tx", MakeCallback(&PacketSentTrace));
  Config::Connect("/NodeList/*/ApplicationList/*/$ns3::UdpEchoServer/Rx", MakeCallback(&PacketReceivedTrace)); 

  Config::Connect("/NodeList/3/ApplicationList/*/$ns3::UdpEchoClient/Tx", MakeCallback(&PacketSentRecord)); 


  Simulator::Stop(Seconds(200.0));
  Simulator::Run();

  CalculateAverageFct();
  writeResult();


  Simulator::Destroy();
  return 0;
}

