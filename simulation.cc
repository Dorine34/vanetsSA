/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
 *
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
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/mobility-model.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/tag.h"
#include "ns3/packet.h"
#include "ns3/uinteger.h"
#include "ns3/simple-tag.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"
#include "ns3/wave-helper.h"
#include "ns3/yans-wifi-helper.h"

// for NS2 files
#include "ns3/ns2-mobility-helper.h"
#include "ns3/ocb-wifi-mac.h"
#include "ns3/wifi-80211p-helper.h"
#include "ns3/wave-mac-helper.h"

// NetAnim
#include "ns3/netanim-module.h"

// C file for mkdir
#include <sys/stat.h>
#include <sys/types.h>


#include <math.h>

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("PTGLsimulator");

/** Physical mode for 802.11p */
#define DEFAULT_PHYSICALMODE "OfdmRate6MbpsBW10MHz"

// **************************************** Configuration parameters

/** Scenario file name with the absolute directory if needed */
std::string m_scenarioFileName = "eloignement.tcl" ;

/** Scenario file name with the absolute directory */
std::string m_trafficFileName = "trafic.txt" ;

/** Clear to send and request to send parameter (default = 2200) */
int m_valRtsCtsThreshold = 2200 ;
	
/** Write per-device PCAP traces if true */
bool m_pcap = false ;
	
/** In cas of speed, calculate the LineOfSightDoppler */
double m_lineOfSightDoppler ;

/** Output directory */
std::string m_outputdir = "output" ;
uint32_t m_testNumber = 0 ;

/** Packet size */
#define PACKET_SIZE_DEFAULT 256
uint32_t m_packetSize = PACKET_SIZE_DEFAULT ;

/** Number of nodes */
uint32_t m_nNodes = 3 ;

/** Packet time interval */
double m_packetInterval = 1 ;

/** Messages sent */
/** Start of sending packet */
int m_startSendTime = 0 ;

/** Stop of sending packet and end of simulation */
int m_stopSendTime = 180 ;

/** Propagation model
 */
int m_lossModel = 1 ;

// ***************************************** End of configuration parameters

/* Configuration string */
std::stringstream m_strPropagationLossConfig ;

/** */
double m_heightAboveZ = 1.5;


/** In case of Grid => define the speed */
double m_speed = 13.33;



//RAT 

double NETA=0.5;

// ---------------- To read scenario files
Ns2MobilityHelper *m_ns2Helper ;
	
// ---------------- name network
/**
 */
NodeContainer m_nodeContainer ;
NodeContainer m_nodeContainer80211p ;
NodeContainer m_nodeContainer80211b ;
NodeContainer m_nodeContainerWave ;

/**
 */
NetDeviceContainer m_netDeviceContainer;
NetDeviceContainer m_netDeviceContainer80211p;
NetDeviceContainer m_netDeviceContainer80211b;
NetDeviceContainer m_netDeviceContainerWave;
	

/** Number of nodes which send a message
 */
uint32_t m_nNodesEmitter ;

/**
 */
Ipv4InterfaceContainer m_ipv4InterfaceContainer;
Ipv4InterfaceContainer m_ipv4InterfaceContainer80211p;
Ipv4InterfaceContainer m_ipv4InterfaceContainer80211b;
Ipv4InterfaceContainer m_ipv4InterfaceContainerWave;

std::string m_lossModelName ;
int m_80211mode = 1 ; // 1->802.11p, 2->802.11b, 3->wave 
int m_fading = 0 ;
int m_verbose = 0 ;
std::string m_phyMode = "OfdmRate6MbpsBW10MHz" ;
int m_txPower = 0 ;
int m_asciiTrace = 1 ;
std::string m_trName = "trFile" ;



/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                          Class to build nodes link                                     */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */


/**
 * Class to store information about links 
 *       between nodes with emitter / receiver / packet size
 *       and messages sent and received
 */

int compteurMessagesRecus ;
int compteurMessagesEmis ;

class TechnoUsed {

public:
        TechnoUsed() = default;
	TechnoUsed (uint32_t number, double throughput, uint32_t nbUsed) {
		this->number = number ;
		this->throughput = throughput ;
		this->nbUsed = nbUsed ;
		if (nbUsed<=1)
		{
		        this->shareThroughput = throughput ;
		}
		else
		{
		        this->shareThroughput = throughput/nbUsed ;
		}
		this->hysteresis=2;
	}
        
	uint32_t GetNumber() {
		return this->number ;
	}
	
	double GetHysteresis() {
		return this->hysteresis ;
	}
	double GetThroughput() {
		return this->throughput ;
	}

	void SetThroughput(double throughput) {
		this->throughput = throughput ;
	}

	uint32_t GetnbUsed() {
		return this->nbUsed ;
	}

	double GetshareThroughput() {
		return this->shareThroughput ;
	}

	void SetnbUsed(uint32_t nbUsed) {
		this->nbUsed = nbUsed ;
		if (nbUsed<=1)
		{
		        this->shareThroughput = throughput ;
		}
		else
		{
		        this->shareThroughput = throughput/nbUsed ;
		}
	}

private:

	uint32_t number ;//indice of techno
	
	double throughput ;//global throughput
 
	double shareThroughput ;//shared throughput
	
	double hysteresis ;//shared throughput
	
	uint32_t nbUsed ;//nb Nodes which used this techno
	
} ;


class EmitterReceiver {

public:

  EmitterReceiver(int emitter, int receiver) {

    this->emitter = emitter ;
    this->receiver = receiver ;
    this->packetSize = PACKET_SIZE_DEFAULT ;
    this->msgSent = 0 ;
    this->msgReceived = 0 ;
  }

  EmitterReceiver(int emitter, int receiver, int packetSize) {

    this->emitter = emitter ;
    this->receiver = receiver ;
    this->packetSize = packetSize ;
    this->msgSent = 0 ;
    this->msgReceived = 0 ;
    this->technoUsed = TechnoUsed(1,6,0) ;
  }

  EmitterReceiver(int emitter, 
                  int receiver, 
                  int packetSize, 
                  TechnoUsed technoUsed, 
                  bool isTechnoUsed) {

    this->emitter = emitter ;
    this->receiver = receiver ;
    this->packetSize = packetSize ;
    this->msgSent = 0 ;
    this->msgReceived = 0 ;
    this->technoUsed = technoUsed ;
    this->isTechnoUsed = isTechnoUsed ;
  }

  void MessageSent() {

    this->msgSent ++ ;
  }

  void MessageReceived() {

    this->msgReceived ++ ;
  }

  uint32_t GetEmitter() {
    return this->emitter ;
  }

  uint32_t GetReceiver() {
    return this->receiver ;
  }

  uint32_t GetPacketSize() {
    return this->packetSize ;
  }

  uint32_t GetMessageSent() {
    return this->msgSent ;
  }

  uint32_t GetMessageReceived() {
    return this->msgReceived ;
  }


  TechnoUsed GetTechnoUsed() {
    return this->technoUsed ;
  }

   bool getIsTechnoUsed() {
    return this->isTechnoUsed ;
  }

  void setTechnoUsed(TechnoUsed technoUsed) {
     this->technoUsed= technoUsed;
  }

   void setIsTechnoUsed(bool isTechnoUsed) {
    this->isTechnoUsed=isTechnoUsed ;
  }

   void setIsNodeElectedS1(bool isElected) {
    this->isNodeElecteds1=isElected ;
  }
  
   void setIsNodeElectedS2(bool isElected) {
    this->isNodeElecteds2=isElected ;
  }
  
   bool getIsNodeElectedS1() {
    return this->isNodeElecteds1 ;
  }
  
   bool getIsNodeElectedS2() {
    return this->isNodeElecteds2 ;
  }
  
   double getwS1() {
    return this->w1 ;
  }
  
   double getwS2() {
    return this->w2 ;
  }
  
  
   void setwS1(double w1) {
    this->w1=w1 ;
  }
  
   void setwS2(double w1) {
    this->w2=w2 ;
  }
  
  
private:
  /** Emitter
   */
  uint32_t emitter ;
  /** Receiver
   */
  uint32_t receiver ;
  /** Packet size
   */
  uint32_t packetSize ;
  /** Messages sent
   */

  uint32_t msgSent ;
  /** Messages received
   */
  uint32_t msgReceived ;

  /** technology Used
   */
  TechnoUsed technoUsed ;

  /** true if technology Used, false otherwise
   */
  bool isTechnoUsed ;
  
  bool isNodeElecteds1 ;
  bool isNodeElecteds2 ;
  double w1 ;
  double w2 ;
  
} ;

// Vector of links
std::vector<EmitterReceiver> m_vlinks ;

    vector<vector<double> > vecQ1; 
    vector<vector<double> > vecY1; 
    
    vector<vector<double> > vecQ2; 
    vector<vector<double> > vecY2; 
// Hash map for EmitterReceiver results
std::map<std::pair<uint32_t, uint32_t>, EmitterReceiver> s_emitterReceiverResult ;

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                       All packet information to be stored                              */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */
/**
 * All packet information to be stored
 */
class PacketInformation {

public:
	PacketInformation(double sentPacketTime) {
		m_sentPacketTime = sentPacketTime ;
		m_receivedPacketTime = -1 ;
		m_ttlValue = -1 ;
	}

	double GetSentPacketTime() {
		return m_sentPacketTime ;
	}

	void SetReceivedPacketTime(double receivedPacketTime) {
		m_receivedPacketTime = receivedPacketTime ;
	}

	double GetReceivedPacketTime() {
		return m_receivedPacketTime ;
	}

	void SetTtlValue(int ttlValue) {

		m_ttlValue = ttlValue ;
	}

	int GetTtlValue() {

		return (m_ttlValue) ;
	}

private:
	/** When the packet was sent
	 */
	double m_sentPacketTime ;
	/** When the packet was received
	 */
	double m_receivedPacketTime ;
	/** TTL value at end
	 */
	int m_ttlValue ;
} ;


// Map to store information about sent time in application layer
std::map<uint64_t, PacketInformation> s_appPacketInformation ;

/* -------------------------------------------------------------------------------------- */
/*                                                                                        */
/*                          Class to build nodes link                                     */
/*                                                                                        */
/* -------------------------------------------------------------------------------------- */


bool testC1(uint32_t nEmitter, uint32_t potentialTechno, uint32_t actualTechno)
{
        double ActualSharedThroughput=0;
        
        uint32_t PotentialNbUsers=0;
        double PotentialGlobalThroughput=0;
        
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    
    if ((it->GetTechnoUsed ().GetNumber()==actualTechno) && (it->getIsTechnoUsed ()) )
        {
               cout <<it->GetEmitter ()<<"->"<<it->GetReceiver () << " : "<<it->GetTechnoUsed ().GetNumber()<<endl;
              ActualSharedThroughput=it->GetTechnoUsed ().GetshareThroughput();
        }
        
    if (it->GetTechnoUsed ().GetNumber()==potentialTechno) 
        {
              PotentialGlobalThroughput=it->GetTechnoUsed ().GetThroughput();
              PotentialNbUsers=it->GetTechnoUsed ().GetnbUsed();
        }
        
    }
        double p1 =PotentialGlobalThroughput/(PotentialNbUsers+1);
        double p2 =ActualSharedThroughput;
 
        
        return (p1/p2)>NETA;
}


void switchTechno(uint32_t nEmitter, uint32_t lastTechno, uint32_t newTechno)
{      
        uint32_t actualNbUsers=0;   
        uint32_t newNbUsers=0;
        
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    
    if ((it->GetTechnoUsed ().GetNumber()==lastTechno) && (it->getIsTechnoUsed ()) )
        {
              actualNbUsers=it->GetTechnoUsed ().GetnbUsed();
        }
        
    if (it->GetTechnoUsed ().GetNumber()==newTechno) 
        {
              newNbUsers=it->GetTechnoUsed ().GetnbUsed();
        }
    }
    
    actualNbUsers--;
    newNbUsers++;
    
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    
    if (it->GetTechnoUsed().GetNumber()==lastTechno)  
        {
                it->GetTechnoUsed().SetnbUsed(actualNbUsers); 
        }
        
    if (it->GetTechnoUsed ().GetNumber()==newTechno) 
        {
                it->GetTechnoUsed().SetnbUsed(newNbUsers); 
        }    
    
    if (it->GetEmitter()==nEmitter)
    {
        if (it->GetTechnoUsed ().GetNumber()==newTechno)
        {
                it->setIsTechnoUsed(true);
        }
        else
        {
                it->setIsTechnoUsed(false);
        
        }
    }
    }
}


bool testC2(uint32_t nEmitter, uint32_t potentialTechno)
{
        
        double PotentialshareThroughput=0;
        double hysteresis=0;
        
        
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    
    if  ((it->GetTechnoUsed ().GetNumber()==nEmitter) && (it->getIsTechnoUsed ()))
        {
              hysteresis=it->GetTechnoUsed ().GetHysteresis();
        }
        
    if (it->GetTechnoUsed ().GetNumber()==potentialTechno) 
        { 
              PotentialshareThroughput=it->GetTechnoUsed ().GetshareThroughput();
        }
        
    }
        return PotentialshareThroughput>hysteresis;
}


bool testC3()
{
        double p=0.5;
        double random = 0;
        random = rand()/(float)RAND_MAX;
        cout<< "random "<< random<<endl; 
        return random<p;
}


void updateW()
{
        uint32_t emitter =0;
        bool twin =false;
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    if  (it->getIsNodeElectedS1())
    {
        emitter = it->GetEmitter() ;
        for (std::vector<EmitterReceiver>::iterator jt = m_vlinks.begin() ; jt != m_vlinks.end() ; jt ++) {
                if  (jt->getIsNodeElectedS2())
                {
                        twin =true ;
                }
        }
        for (std::vector<EmitterReceiver>::iterator jt = m_vlinks.begin() ; jt != m_vlinks.end() ; jt ++) {
                if  ((jt->GetReceiver()==emitter)&& (it->getIsNodeElectedS1()))
                {
                        if (twin)
                        {
                                it->setwS1(jt->getwS1()/2);
                        }
                        else
                        {
                        
                                it->setwS1(jt->getwS1());
                        }
                }
        }
    }
  }
  twin = false;
    for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    if  (it->getIsNodeElectedS2())
    {
        emitter = it->GetEmitter() ;
        for (std::vector<EmitterReceiver>::iterator jt = m_vlinks.begin() ; jt != m_vlinks.end() ; jt ++) {
                if  (jt->getIsNodeElectedS1())
                {
                        twin =true ;
                }
        }
        for (std::vector<EmitterReceiver>::iterator jt = m_vlinks.begin() ; jt != m_vlinks.end() ; jt ++) {
                if  ((jt->GetReceiver()==emitter)&& (it->getIsNodeElectedS2()))
                {
                        if (twin)
                        {
                                it->setwS2(jt->getwS2()/2);
                        }
                        else
                        {
                        
                                it->setwS2(jt->getwS2());
                        }
                }
        }
    }
  }
}




void updateY(uint32_t receiver)
{
        double wsv1=0;
        double y1=0;
        double q1=0;
        
        double wsv2=0;
        double y2=0;
        double q2=0;
        
        
        double GAMMA=0.5 ;
        
        double TAU=1 ;
        uint32_t emitter =0;
        int j=0;
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
        // Vector to store column elements 
        
        emitter = i->GetEmitter();
        if ((receiver ==i->GetEmitter()) )
         {
                if (i->getIsNodeElectedS1())
                {
                        wsv1=i->getwS1();
                        wsv2=i->getwS2();
                        
                }
                //y=vecY1[i][(int)emitter];
                y2=vecY2[j][(int)emitter];
                q2=vecQ2[j][(int)emitter];
         }
         vecY1[j][(int)emitter]=y1+GAMMA*( wsv1- TAU *y1)/q1;
         vecY2[j][(int)emitter]=y2+GAMMA*( wsv2- TAU *y2)/q2;
        } 
         
          
}



void updateQ(uint32_t receiver)
{
        uint32_t emitter =0;
        int j=0;
        
        double tabY[m_nNodes]={0};
        
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
     
        emitter = i->GetEmitter();
        if (receiver == i->GetReceiver())
        {
                tabY[emitter]= vecY1[j][(int)emitter];
        }
     }
        
    double sumExpoY=0;
    
    
    for (int i=0; i<(int)m_nNodes ; i++)
    {
        sumExpoY+= exp (tabY[i]);
    } 
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
     
        if (receiver == i->GetReceiver())
        {
                for (int k = 0; k < (int) m_nNodes; k++)
                {
                        
                        vecQ1[j][k]= exp(vecY1[j][k])/sumExpoY;
                }
     }
     }  
         emitter =0;
         j=0;
          sumExpoY=0;
          
          
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
     
        emitter = i->GetEmitter();
        if (receiver == i->GetReceiver())
        {
                tabY[emitter]= vecY2[j][(int)emitter];
        }
     }
        
    
    
    for (int i=0; i<(int)m_nNodes ; i++)
    {
        sumExpoY+= exp (tabY[i]);
    } 
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
     
        if (receiver == i->GetReceiver())
        {
                for (int k = 0; k < (int) m_nNodes; k++)
                {
                        
                        vecQ2[j][k]= exp(vecY2[j][k])/sumExpoY;
                }
        }
     }       
}




void neighbourChoice(uint32_t nReceiver, uint32_t nEmitter, uint32_t techno)
{

        uint32_t emitter =0;
        double random = rand()/(float)RAND_MAX;
        double tabY[m_nNodes]={0};
        int j=0;
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
        if (emitter == i->GetEmitter())
        {
                tabY[emitter]= vecQ1[j][(int)emitter];
        }
     }
     
     double test=0;
     int i=0;
     
     while((random>test)&&(m_nNodes))
     {
        test+=tabY[i];
        i++;
     }
     
     uint32_t nElected = (uint32_t)i-1;
     
     
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
        if ((emitter == i->GetEmitter()) && (i->getIsTechnoUsed())&&(i->getIsNodeElectedS1()))
        {
                if (nElected == i->GetReceiver())
                {       
                        i->setIsNodeElectedS1(true);
                       cout<< "CHANGEMENT !!!!!!"<<endl<<endl;
                }
                       
        }
        else
        {
           if (emitter == i->GetEmitter())
           {
                       i->setIsNodeElectedS1(false);
           
           }
        }     
     }
     
     
       emitter =0;
       random = rand()/(float)RAND_MAX;
       tabY[m_nNodes]={0};
       j=0;
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
        if (emitter == i->GetEmitter())
        {
                tabY[emitter]= vecQ2[j][(int)emitter];
        }
     }
     
     test=0;
     i=0;
     
     while ((random>test)&&(i< (int)m_nNodes))
     {
        test+=tabY[i];
        i++;
        
     }
     
     nElected = (uint32_t)i-1;
     
     
     for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++, j++) {
        if ((emitter == i->GetEmitter()) && (i->getIsTechnoUsed())&&(i->getIsNodeElectedS2()))
        {
                if (nElected == i->GetReceiver())
                {       
                        i->setIsNodeElectedS2(true);
                       cout<< "CHANGEMENT !!!!!!"<<endl;
                }
                       
        }
        else
        {
           if (emitter == i->GetEmitter())
           {
                       i->setIsNodeElectedS2(false);
           
           }
      
        }
     }
     
}


/**
 * Generate the traffic avec choix de la techno
 */
static void GenerateTraffic (uint32_t nReceiver, 
                             uint32_t nEmitter, 
                             uint32_t techno, 
                             Ptr<Socket> socket, 
                             uint32_t pktSize, 
                             double pktInterval ) {
 
  const uint8_t buffer[pktSize+3] = {1,2,3} ;
  
  
  
  updateW();
  updateY(nReceiver);
  updateQ(nReceiver);
  neighbourChoice (nReceiver, nEmitter, techno);
  
  // -------------------------------------------------------
  // Packet to be sent
  Ptr<Packet> packet = Create<Packet> (buffer,pktSize);
  
  PacketInformation packetInfo = PacketInformation(Simulator::Now().GetSeconds()) ;
  s_appPacketInformation.insert(std::pair<uint64_t, 
                                PacketInformation>(packet->GetUid(), packetInfo)) ;
  
  std::pair<uint32_t, uint32_t> pairER = std::make_pair(nEmitter, nReceiver) ;
  std::map<std::pair<uint32_t, uint32_t>, EmitterReceiver>::iterator itRes = 
                                        s_emitterReceiverResult.find(pairER) ;
  
  if (itRes != s_emitterReceiverResult.end()) {
    itRes->second.MessageSent() ;
  }
  /*cas 80211p*/
        if (techno==1){
                Ptr<MobilityModel> mob80211p = m_nodeContainer80211p.Get(nReceiver)->GetObject<MobilityModel>();
                Vector pos80211p = mob80211p->GetPosition ();
                //std::cout << "Node " << nReceiver << ": POS: x=" << pos.x << ", y=" << pos.y << std::endl;

                Ptr<MobilityModel> mob180211p = m_nodeContainer80211p.Get(nEmitter)->GetObject<MobilityModel>();
                Vector pos180211p = mob180211p->GetPosition ();
                //std::cout << "Node " << nEmitter << ": POS: x=" << pos1.x << ", y=" << pos1.y << std::endl;
                double distance80211p = sqrt((pos80211p.x - pos180211p.x)*(pos80211p.x - pos180211p.x)+
                        (pos80211p.y - pos180211p.y)*(pos80211p.y - pos180211p.y)+
                        (pos80211p.z - pos180211p.z)*(pos80211p.z - pos180211p.z));

  // ----------------------------------------------------------------------------
  // Display information about sending => does not work for tag to be checked
                double speed80211p = mob180211p->GetRelativeSpeed (mob80211p) ;
  
                NS_LOG_UNCOND ("+AtApp " << Simulator::Now().GetSeconds()
                 << " s packet " << packet->GetUid()
                 << " send from node " << nEmitter << " to " << nReceiver
                 << " dist: " << distance80211p
                 << " relSpeed: " << speed80211p
                 << " txSizeNet: " << pktSize
                 << " techno : " << techno
                 );
        }
        if (techno==3)
        {       
                  Ptr<MobilityModel> mobWave = m_nodeContainerWave.Get(nReceiver)->GetObject<MobilityModel>();
                Vector posWave = mobWave->GetPosition ();
                //std::cout << "Node " << nReceiver << ": POS: x=" << pos.x << ", y=" << pos.y << std::endl;
                 Ptr<MobilityModel> mob1Wave = m_nodeContainerWave.Get(nEmitter)->GetObject<MobilityModel>();
                Vector pos1Wave = mob1Wave->GetPosition ();
                //std::cout << "Node " << nEmitter << ": POS: x=" << pos1.x << ", y=" << pos1.y << std::endl;
                double distanceWave = sqrt((posWave.x - pos1Wave.x)*(posWave.x - pos1Wave.x)+
                        (posWave.y - pos1Wave.y)*(posWave.y - pos1Wave.y)+
                        (posWave.z - pos1Wave.z)*(posWave.z - pos1Wave.z));

  // ----------------------------------------------------------------------------
  // Display information about sending => does not work for tag to be checked
                double speedWave = mob1Wave->GetRelativeSpeed (mobWave) ;
                NS_LOG_UNCOND ("+AtApp " << Simulator::Now().GetSeconds()
                 << " s packet " << packet->GetUid()
                 << " send from node " << nEmitter << " to " << nReceiver
                 << " dist: " << distanceWave
                 << " relSpeed: " << speedWave
                 << " txSizeNet: " << pktSize
                 << " techno : " << techno
                 );
               
        }

                 compteurMessagesEmis++;
  // -----------------------------------------------------------------------------
  // Adding tag
  MyTag tag;
  tag.SetSNode(nEmitter) ;
  tag.SetRNode(nReceiver) ;
  packet->AddPacketTag (tag);

  // -------------------------------------------------------
  // Send the packet ICI a modifier pour qui envoie avec quelle techno
  socket->Send (packet);

  // -------------------------------------------------------
  // Schedule next packet
  // Adding new random variable
  //double offset = myRandom->GetValue(pktInterval/100 - pktInterval, pktInterval - pktInterval/100) ;
  
  
  
  //RAT algo
  if (testC1(nEmitter, techno, 3) && testC2(nEmitter,  3) && testC3() )
  {
        switchTechno(nEmitter, techno, 3) ;
  
  }
  
  if (Simulator::Now().GetSeconds() < m_stopSendTime)  {
    double offset = 0 ;
    Simulator::Schedule (Seconds(pktInterval + offset), 
           &GenerateTraffic, nReceiver, nEmitter, techno, socket, pktSize, pktInterval);
  }
}





/** Socket application
 * \author Benoit Hilt
 */
void ReceivePacket (Ptr<Socket> socket) {

  // For the NODE id
  // Ptr<Node> node = socket->GetNode() ;

  Ptr<Packet> packet;

  NS_LOG_DEBUG ("-AtApp " << Simulator::Now().GetSeconds() <<" receive 1 Packet ") ;
  
  while ((packet = socket->Recv ())) {

    MyTag tagCopy1;
    if (packet->PeekPacketTag(tagCopy1)) {
      uint32_t sNode=tagCopy1.GetSNode();
      uint32_t rNode=tagCopy1.GetRNode();

      NS_LOG_UNCOND ("-AtApp " << Simulator::Now().GetSeconds()
                     << " s packet " << packet->GetUid()
                     << " send from node " << sNode << " to " << rNode
                     << "), txSizeNet: " << packet->GetSize()
                     );
    }
    else {
      NS_LOG_UNCOND ("-AtApp " << Simulator::Now().GetSeconds()
                     << " s packet " << packet->GetUid()
                     << " txSizeNet: " << packet->GetSize()
                     );
    }
    
    compteurMessagesRecus++;
  }
  
}


/** Load the scenario
 * \return true if scenario correctly loaded
 * \see ns3::Ns2MobilityHelper Class
 */
bool CreateNS2Nodes() {

  m_ns2Helper = new Ns2MobilityHelper(m_scenarioFileName) ;
  m_nodeContainer80211p.Create (m_nNodes) ;
  m_nodeContainerWave.Create (m_nNodes) ;
  m_nodeContainer80211b.Create (m_nNodes) ;
  m_ns2Helper->Install() ; //m_nodeConta1iner.Begin(), m_nodeContainer().End) ;

  return true ;
}

/**
 * Calculate LineOfSightDoppler
 */
double calculateLineOfSightDoppler(double speed) {

  double lineOfSightDoppler = (speed*5.9e9)/(0.3e9*10e6);

  return lineOfSightDoppler ;
}

/**
 * Yans Wifi installation
 * see https://svn.ensisa.uha.fr/redmine/projects/vanet-ns3/wiki/Configurations_Specifiques
 */
void InstallYansWifi80211b() {

     if (m_lossModel == 1)
     {
       m_lossModelName = "ns3::FriisPropagationLossModel";
     }
   else if (m_lossModel == 2)
     {
       m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
     }
   else if (m_lossModel == 3)
     {
       m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
     }
   else if (m_lossModel == 4)
     {
       m_lossModelName = "ns3::LogDistancePropagationLossModel";
     }
   else
     {
       // Unsupported propagation loss model.
       // Treating as ERROR
       NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
     }
 
   // frequency
      //  802.11b 2.4 GHz
   double  freq = 2.4e9;
   
 
   // Setup propagation models
   YansWifiChannelHelper wifiChannel;
   wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   if (m_lossModel == 3)
     {
       // two-ray requires antenna height (else defaults to Friss)
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
     }
   else
     {
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
     }
 
   // Propagation loss models are additive.
   if (m_fading != 0)
     {
       // if no obstacle model, then use Nakagami fading if requested
       wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
     }
 
   // the channel
   Ptr<YansWifiChannel> channel = wifiChannel.Create ();
 
   // The below set of helpers will help us to put together the wifi NICs we want
   YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
   wifiPhy.SetChannel (channel);
   // ns-3 supports generate a pcap trace
   wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
   wavePhy.SetChannel (channel);
   wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   // Setup WAVE PHY and MAC
   NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
   WaveHelper waveHelper = WaveHelper::Default ();
   Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
   if (m_verbose)
     {
       wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
       // likewise, turn on WAVE PHY logging
       waveHelper.EnableLogComponents ();
     }
 
   WifiHelper wifi;
 
   // Setup 802.11b stuff
   wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
 
   wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                 "DataMode",StringValue (m_phyMode),
                                 "ControlMode",StringValue (m_phyMode));
 
   // Setup 802.11p stuff
   wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Setup WAVE-PHY stuff
   waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Set Tx Power
   wifiPhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
 
   // Add an upper mac and disable rate control
   WifiMacHelper wifiMac;
   wifiMac.SetType ("ns3::AdhocWifiMac");
   QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
 
   // Setup net devices
 
       m_netDeviceContainer80211b = wifi.Install (wifiPhy, wifiMac, m_nodeContainer80211b) ;
     
 
   if (m_asciiTrace != 0) {
       std::ostringstream logFileName ;
       logFileName << m_outputdir.c_str() << "trFile" << ".tr" ;
       AsciiTraceHelper ascii;
       Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (logFileName.str().c_str()) );
       wifiPhy.EnableAsciiAll (osw);
       wavePhy.EnableAsciiAll (osw);
     }

   if (m_pcap != 0) {
     std::ostringstream fileName ;
     fileName << m_outputdir << "vanet-routing-compare-pcap" ;
     std::cout << fileName.str() << std::endl ;
     wifiPhy.EnablePcapAll (fileName.str().c_str()) ;
     wavePhy.EnablePcapAll (fileName.str().c_str()) ;
   }
}

/*creation de yansWifi autorisant plusieurs technologies de communication*/

void InstallYansWifi80211p() {

     if (m_lossModel == 1)
     {
       m_lossModelName = "ns3::FriisPropagationLossModel";
     }
   else if (m_lossModel == 2)
     {
       m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
     }
   else if (m_lossModel == 3)
     {
       m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
     }
   else if (m_lossModel == 4)
     {
       m_lossModelName = "ns3::LogDistancePropagationLossModel";
     }
   else
     {
       // Unsupported propagation loss model.
       // Treating as ERROR
       NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
     }
 
   // frequency
   double freq = 5.9e9;
 
   // Setup propagation models
   YansWifiChannelHelper wifiChannel;
   wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   if (m_lossModel == 3)
     {
       // two-ray requires antenna height (else defaults to Friss)
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
     }
   else
     {
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
     }
 
   // Propagation loss models are additive.
   if (m_fading != 0)
     {
       // if no obstacle model, then use Nakagami fading if requested
       wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
     }
 
   // the channel
   Ptr<YansWifiChannel> channel = wifiChannel.Create ();
 
   // The below set of helpers will help us to put together the wifi NICs we want
   YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
   wifiPhy.SetChannel (channel);
   // ns-3 supports generate a pcap trace
   wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
   wavePhy.SetChannel (channel);
   wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   // Setup WAVE PHY and MAC
   NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
   WaveHelper waveHelper = WaveHelper::Default ();
   Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
   if (m_verbose)
     {
       wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
       // likewise, turn on WAVE PHY logging
       waveHelper.EnableLogComponents ();
     }
 
   WifiHelper wifi;
 
   // Setup 802.11b stuff
   wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
 
   wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                 "DataMode",StringValue (m_phyMode),
                                 "ControlMode",StringValue (m_phyMode));
 
   // Setup 802.11p stuff
   wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Setup WAVE-PHY stuff
   waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Set Tx Power
   wifiPhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
 
   // Add an upper mac and disable rate control
   WifiMacHelper wifiMac;
   wifiMac.SetType ("ns3::AdhocWifiMac");
   QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
 
   // Setup net devices
 
 //  if (m_80211mode == 3) {
       //m_netDeviceContainerWave = waveHelper.Install (wavePhy, waveMac, m_nodeContainerWave) ;
 //  }
 //  else if (m_80211mode == 1) {
       m_netDeviceContainer80211p = wifi80211p.Install (wifiPhy, wifi80211pMac, m_nodeContainer80211p) ;
  //   }
  // else {
   //    m_netDeviceContainer = wifi.Install (wifiPhy, wifiMac, m_nodeContainer) ;
 //    }
 
   if (m_asciiTrace != 0) {
       std::ostringstream logFileName ;
       logFileName << m_outputdir.c_str() << "trFile" << ".tr" ;
       AsciiTraceHelper ascii;
       Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (logFileName.str().c_str()) );
       wifiPhy.EnableAsciiAll (osw);
       wavePhy.EnableAsciiAll (osw);
     }

   if (m_pcap != 0) {
     std::ostringstream fileName ;
     fileName << m_outputdir << "vanet-routing-compare-pcap" ;
     std::cout << fileName.str() << std::endl ;
     wifiPhy.EnablePcapAll (fileName.str().c_str()) ;
     wavePhy.EnablePcapAll (fileName.str().c_str()) ;
   }
        cout<< "D* : Protocoles wave et 80211p installes"<< endl;

}


void InstallYansWifiWave() {

     if (m_lossModel == 1)
     {
       m_lossModelName = "ns3::FriisPropagationLossModel";
     }
   else if (m_lossModel == 2)
     {
       m_lossModelName = "ns3::ItuR1411LosPropagationLossModel";
     }
   else if (m_lossModel == 3)
     {
       m_lossModelName = "ns3::TwoRayGroundPropagationLossModel";
     }
   else if (m_lossModel == 4)
     {
       m_lossModelName = "ns3::LogDistancePropagationLossModel";
     }
   else
     {
       // Unsupported propagation loss model.
       // Treating as ERROR
       NS_LOG_ERROR ("Invalid propagation loss model specified.  Values must be [1-4], where 1=Friis;2=ItuR1411Los;3=TwoRayGround;4=LogDistance");
     }
 
   // frequency
   double freq = 5.9e9;
 
   // Setup propagation models
   YansWifiChannelHelper wifiChannel;
   wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
   if (m_lossModel == 3)
     {
       // two-ray requires antenna height (else defaults to Friss)
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq), "HeightAboveZ", DoubleValue (1.5));
     }
   else
     {
       wifiChannel.AddPropagationLoss (m_lossModelName, "Frequency", DoubleValue (freq));
     }
 
   // Propagation loss models are additive.
   if (m_fading != 0)
     {
       // if no obstacle model, then use Nakagami fading if requested
       wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel");
     }
 
   // the channel
   Ptr<YansWifiChannel> channel = wifiChannel.Create ();
 
   // The below set of helpers will help us to put together the wifi NICs we want
   YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
   wifiPhy.SetChannel (channel);
   // ns-3 supports generate a pcap trace
   wifiPhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   YansWavePhyHelper wavePhy =  YansWavePhyHelper::Default ();
   wavePhy.SetChannel (channel);
   wavePhy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11);
 
   // Setup WAVE PHY and MAC
   NqosWaveMacHelper wifi80211pMac = NqosWaveMacHelper::Default ();
   WaveHelper waveHelper = WaveHelper::Default ();
   Wifi80211pHelper wifi80211p = Wifi80211pHelper::Default ();
   if (m_verbose)
     {
       wifi80211p.EnableLogComponents ();      // Turn on all Wifi 802.11p logging
       // likewise, turn on WAVE PHY logging
       waveHelper.EnableLogComponents ();
     }  
 
   WifiHelper wifi;
 
   // Setup 802.11b stuff
   wifi.SetStandard (WIFI_PHY_STANDARD_80211b);
 
   wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                 "DataMode",StringValue (m_phyMode),
                                 "ControlMode",StringValue (m_phyMode));
 
   // Setup 802.11p stuff
   wifi80211p.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Setup WAVE-PHY stuff
   waveHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                       "DataMode",StringValue (m_phyMode),
                                       "ControlMode",StringValue (m_phyMode));
 
   // Set Tx Power
   wifiPhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wifiPhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerStart",DoubleValue (m_txPower));
   wavePhy.Set ("TxPowerEnd", DoubleValue (m_txPower));
 
   // Add an upper mac and disable rate control
   WifiMacHelper wifiMac;
   wifiMac.SetType ("ns3::AdhocWifiMac");
   QosWaveMacHelper waveMac = QosWaveMacHelper::Default ();
 
   // Setup net devices
 
 //  if (m_80211mode == 3) {
       m_netDeviceContainerWave = waveHelper.Install (wavePhy, waveMac, m_nodeContainerWave) ;
 //  }
 //  else if (m_80211mode == 1) {
       //m_netDeviceContainer80211p = wifi80211p.Install (wifiPhy, wifi80211pMac, m_nodeContainer80211p) ;
  //   }
  // else {
   //    m_netDeviceContainer = wifi.Install (wifiPhy, wifiMac, m_nodeContainer) ;
 //    }
 
   if (m_asciiTrace != 0) {
       std::ostringstream logFileName ;
       logFileName << m_outputdir.c_str() << "trFile" << ".tr" ;
       AsciiTraceHelper ascii;
       Ptr<OutputStreamWrapper> osw = ascii.CreateFileStream ( (logFileName.str().c_str()) );
       wifiPhy.EnableAsciiAll (osw);
       wavePhy.EnableAsciiAll (osw);
     }

   if (m_pcap != 0) {
     std::ostringstream fileName ;
     fileName << m_outputdir << "vanet-routing-compare-pcap" ;
     std::cout << fileName.str() << std::endl ;
     wifiPhy.EnablePcapAll (fileName.str().c_str()) ;
     wavePhy.EnablePcapAll (fileName.str().c_str()) ;
   }
        cout<< "D* : Protocoles wave et 80211p installes"<< endl;

}

/**
 * No protocol installed for internet stack
 * \see MobilityIntegrationTest::InstallInternetStack
 */
void InstallNoProtocol () {

  
  InternetStackHelper stackWave;
  stackWave.Install (m_nodeContainerWave);
  
  Ipv4AddressHelper addressWave;
  addressWave.SetBase ("11.0.0.0", "255.0.0.0");
  m_ipv4InterfaceContainerWave = addressWave.Assign (m_netDeviceContainerWave);

  InternetStackHelper stack80211p;
  stack80211p.Install (m_nodeContainer80211p);
  
  Ipv4AddressHelper address80211p;
  address80211p.SetBase ("10.0.0.0", "255.0.0.0");
  m_ipv4InterfaceContainer80211p = address80211p.Assign (m_netDeviceContainer80211p);
  
  
  InternetStackHelper stack80211b;
  stack80211b.Install (m_nodeContainer80211b);
  
  Ipv4AddressHelper address80211b;
  address80211b.SetBase ("10.0.0.0", "255.0.0.0");
  m_ipv4InterfaceContainer80211b = address80211b.Assign (m_netDeviceContainer80211b);
  
}

/**
 * Retrieve the IPV4 address from a node
 * \return IP address
 * \param index index of the node in NodeContainer
 */
Ipv4InterfaceAddress GetIpv4Address80211p(uint32_t index) {

  for (uint32_t indexDevice = 0 ; indexDevice < m_netDeviceContainer80211p.GetN() ; indexDevice ++) {

    Ptr<NetDevice> netDevice=m_netDeviceContainer80211p.Get(indexDevice);
    Ptr<Node> node=netDevice->GetNode ();
    if (node != NULL) {
      Ptr<Ipv4> ipv4=node->GetObject<Ipv4> ();

      int32_t interface = ipv4->GetInterfaceForDevice (netDevice);
      uint32_t nb = ipv4->GetNAddresses(interface);

      for(uint32_t indexInterface = 0 ;  indexInterface < nb ; indexInterface ++) {
        Ipv4InterfaceAddress addr = ipv4->GetAddress(interface, indexInterface);

        if (index == node->GetId() ) {
          //std::cerr << "FD::Found the correct node " << node->GetId() << " has address " << addr.GetLocal() << " Mask " << addr.GetMask() << " Broadcast "<< addr.GetBroadcast() << std::endl;
          return addr ;
        }
      }
    }
    else {
      std::cerr << "FD:Node is null" << std::endl ;
    }
  }

  return Ipv4InterfaceAddress() ;
}

Ipv4InterfaceAddress GetIpv4Address80211b(uint32_t index) {

  for (uint32_t indexDevice = 0 ; indexDevice < m_netDeviceContainer80211b.GetN() ; indexDevice ++) {

    Ptr<NetDevice> netDevice=m_netDeviceContainer80211b.Get(indexDevice);
    Ptr<Node> node=netDevice->GetNode ();
    if (node != NULL) {
      Ptr<Ipv4> ipv4=node->GetObject<Ipv4> ();

      int32_t interface = ipv4->GetInterfaceForDevice (netDevice);
      uint32_t nb = ipv4->GetNAddresses(interface);

      for(uint32_t indexInterface = 0 ;  indexInterface < nb ; indexInterface ++) {
        Ipv4InterfaceAddress addr = ipv4->GetAddress(interface, indexInterface);

        if (index == node->GetId() ) {
          //std::cerr << "FD::Found the correct node " << node->GetId() << " has address " << addr.GetLocal() << " Mask " << addr.GetMask() << " Broadcast "<< addr.GetBroadcast() << std::endl;
          return addr ;
        }
      }
    }
    else {
      std::cerr << "FD:Node is null" << std::endl ;
    }
  }

  return Ipv4InterfaceAddress() ;
}


Ipv4InterfaceAddress GetIpv4AddressWave(uint32_t index) {

  for (uint32_t indexDevice = 0 ; indexDevice < m_netDeviceContainerWave.GetN() ; indexDevice ++) {

    Ptr<NetDevice> netDevice=m_netDeviceContainerWave.Get(indexDevice);
    Ptr<Node> node=netDevice->GetNode ();
    if (node != NULL) {
      Ptr<Ipv4> ipv4=node->GetObject<Ipv4> ();

      int32_t interface = ipv4->GetInterfaceForDevice (netDevice);
      uint32_t nb = ipv4->GetNAddresses(interface);

      for(uint32_t indexInterface = 0 ;  indexInterface < nb ; indexInterface ++) {
        Ipv4InterfaceAddress addr = ipv4->GetAddress(interface, indexInterface);

cout<< "coucou"<<endl;

        if (index == node->GetId() ) {
          std::cerr << "FD::Found the correct node " << node->GetId() << " has address " << addr.GetLocal() << " Mask " << addr.GetMask() << " Broadcast "<< addr.GetBroadcast() << std::endl;
          return addr ;
        }
      }
    }
    else {
      std::cerr << "FD:Node is null" << std::endl ;
    }
  }

  return Ipv4InterfaceAddress() ;
}

/**
 * Install socket for node communication
 */
void InstallSocketApplications() {

  uint32_t port = 4000 ;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
    
  if (m_vlinks.size() == 0) {
    NS_FATAL_ERROR ("No traffic defined, please used or a file or random (see parameters: NodesEmitter or TrafficFileName)");
  }

  // Configuration of the applications
  // UdpClient -> UdpServer
  int i = 1 ;
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++, i++) {
    if (it->getIsTechnoUsed ())
    {
    NS_LOG_UNCOND("Link " << i << ": node " << it->GetEmitter() << " send information to " << it->GetReceiver()<< " with techno "<< it->GetTechnoUsed().GetNumber()) ;

    uint32_t nNodeServer = it->GetReceiver() ;
    uint32_t nNodeClient = it->GetEmitter() ;
    uint32_t packetSize = it->GetPacketSize() ;
    // Configure the server

    Ipv4InterfaceAddress destAddress ;
    if (it->GetTechnoUsed().GetNumber()==1)
    {
        destAddress = GetIpv4Address80211p(nNodeServer) ;
    }
    if (it->GetTechnoUsed().GetNumber()==2)
    {
        destAddress = GetIpv4Address80211b(nNodeServer) ;
    }
    if (it->GetTechnoUsed().GetNumber()==3)
    {
        destAddress = GetIpv4AddressWave(nNodeServer) ;
    }

    Ptr<Socket> recvSink ;

    if (it->GetTechnoUsed().GetNumber()==1)
    {
        recvSink = Socket::CreateSocket (m_nodeContainer80211p.Get (nNodeServer), tid);
    }
    if (it->GetTechnoUsed().GetNumber()==2)
    {
        recvSink = Socket::CreateSocket (m_nodeContainer80211b.Get (nNodeServer), tid);
    }
    if (it->GetTechnoUsed().GetNumber()==3)
    {
        recvSink = Socket::CreateSocket (m_nodeContainerWave.Get (nNodeServer), tid);
    } 

    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny(), port);
    recvSink->Bind (local);
    recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));
        
    // Configure the client
    Ptr<Socket> source ;
    if (it->GetTechnoUsed().GetNumber()==1)
    {
        source = Socket::CreateSocket ( m_nodeContainer80211p.Get(nNodeClient), tid);
    }
    if (it->GetTechnoUsed().GetNumber()==2)
    {
        source = Socket::CreateSocket ( m_nodeContainer80211b.Get(nNodeClient), tid);
    }
    if (it->GetTechnoUsed().GetNumber()==3)
    {
        source = Socket::CreateSocket ( m_nodeContainerWave.Get(nNodeClient), tid);
    }
    InetSocketAddress remote = InetSocketAddress (destAddress.GetLocal(), port);
    source->SetAllowBroadcast (true);
    source->Bind();
    source->Connect (remote);  // Used to set the destination address of the outgoing packets


    // Display
    m_nNodesEmitter = m_vlinks.size() ;
    double offset = 0.001*(i-1) ; //* nNodeClient ;
    

    Ipv4InterfaceAddress srcAddress ;
    if (it->GetTechnoUsed().GetNumber()==1)
    {
        srcAddress = GetIpv4Address80211p(nNodeClient) ;
    }
    if (it->GetTechnoUsed().GetNumber()==2)
    {
        srcAddress = GetIpv4Address80211b(nNodeClient) ;
    }
    if (it->GetTechnoUsed().GetNumber()==3)
    {
        srcAddress = GetIpv4AddressWave(nNodeClient) ;
    }
    //attention probleme de memoire
        NS_LOG_UNCOND("\tClient " << srcAddress.GetLocal() << " (" << nNodeClient 
        << ") sends information to " << destAddress.GetLocal() << " (" << nNodeServer << ")" 
        << " (start time = " << Seconds(m_startSendTime + offset).GetSeconds() 
        << " s) with techno "<<it->GetTechnoUsed().GetNumber() ) ;

        Simulator::Schedule (Seconds(m_startSendTime + offset), 
        &GenerateTraffic, nNodeServer, nNodeClient,it->GetTechnoUsed().GetNumber(), source, packetSize, m_packetInterval);
    }
  }
}




/**
 * Les applications => depend of the traffic file name => DEFAULT -> RANDOM applications, NOT DEFAULT => take the filename and parse it
 */
void InstallApplications () {

    



//Cretation des technologies de communication


	//TechnoUsed (int number, double throughput, int nbUsed) 
        TechnoUsed techno1= TechnoUsed(1, 6, 0);
        TechnoUsed techno2= TechnoUsed(3, 6, 0);
        TechnoUsed techno3= TechnoUsed(2, 6, 0);
        uint32_t nbTechno1=0;
        uint32_t nbTechno2=0;
        uint32_t nbTechno3=0;
        

  std::ifstream trafficFile ;
  trafficFile.open(m_trafficFileName.c_str()) ;
  if (!trafficFile.is_open()) {
    NS_FATAL_ERROR("Cannot read traffic file " << m_trafficFileName << " please check the file name") ;
  }
  else {
    int iNodeEmit, iNodeRec, packetSize ;
    std::string line ;
    while (std::getline(trafficFile,line)) {
      iNodeEmit = -1 ; iNodeRec = -1 ; packetSize = -1 ;
      if (line.find("#") == std::string::npos && !line.empty()) {
        std::istringstream iss(line);
        iss >> iNodeEmit >> iNodeRec >> packetSize ;

        if ((iNodeEmit < 0) || (iNodeRec < 0)) {
          trafficFile.close() ;
          NS_FATAL_ERROR("Incorrect traffic file, should be node number and found \"" << line << "\", please review traffic file") ;
        }
        else {
          if (((uint32_t)iNodeEmit >= m_nNodes) || ((uint32_t)iNodeRec >= m_nNodes)) {
            trafficFile.close() ;
            NS_FATAL_ERROR("Incorrect traffic file, node number should be number of nodes, found emitter = " << iNodeEmit << " and receiver = " << iNodeRec << " regarding total node number of " << m_nNodes) ;
          }
          else {
            if (packetSize <= 0) packetSize = m_packetSize ;
            m_vlinks.push_back(EmitterReceiver(iNodeEmit, iNodeRec, packetSize,techno1, true)) ;
            nbTechno1++;
            
                //Installation de la 2nde technologie de communication
            m_vlinks.push_back(EmitterReceiver(iNodeEmit, iNodeRec, packetSize,techno2, false)) ;
                        //nbTechno2++;

            m_vlinks.push_back(EmitterReceiver(iNodeEmit, iNodeRec, packetSize,techno3, false)) ;
            //nbTechno3++;
          }
        }
      }
    }

    trafficFile.close();
  }

  // Add the emitter receiver from m_vlinks into map
  for (std::vector<EmitterReceiver>::iterator it = m_vlinks.begin() ; it != m_vlinks.end() ; it ++) {
    std::pair<uint32_t, uint32_t> pairER = std::make_pair(it->GetEmitter(), it->GetReceiver()) ;
      it->setIsNodeElectedS1(false) ;
      it->setIsNodeElectedS2(false) ;
      it->setwS1(0) ;
      it->setwS2(0) ;
      if (it->GetTechnoUsed().GetNumber()==1)
      {
        it->GetTechnoUsed().SetnbUsed(nbTechno1);
        
      } 
      if (it->GetTechnoUsed().GetNumber()==2)
      {
        it->GetTechnoUsed().SetnbUsed(nbTechno2);
        
      } 
      if (it->GetTechnoUsed().GetNumber()==3)
      {
        it->GetTechnoUsed().SetnbUsed(nbTechno3);
      } 
    s_emitterReceiverResult.insert(std::make_pair(pairER, *it)) ;
  }
  
  
  /**Creation des tables**/

        /*Initialisation TabQ*/
  for (std::vector<EmitterReceiver>::iterator i = m_vlinks.begin() ; i != m_vlinks.end() ; i ++) {
        // Vector to store column elements 
        vector<double> vq1; 
        vector<double> vy1;
        vector<double> vq2; 
        vector<double> vy2; 
        cout << " m_nNodes-1 : "<< m_nNodes-1<<endl;
        for (int j = 0; j < (int)m_nNodes; j++) {
         
        if ((int)i->GetEmitter()==j)
        {
                vq1.push_back(-1);
                vy1.push_back(-1);
                vq2.push_back(-1);
                vy2.push_back(-1);
        }
        else
        {
                vy1.push_back(1);
                vy2.push_back(1);
        if (m_nNodes<2)
           {
                vq1.push_back(1);
                vq2.push_back(1); 
           }
        else
           {
                vq1.push_back(1/((int)m_nNodes-1));
                vq2.push_back(1/((int)m_nNodes-1)); 
           }
        }
         
  
        // Pushing back above 1D vector 
        // to create the 2D vector 
        vecQ1.push_back(vq1);         
        vecY1.push_back(vy1);
         
         
        vecQ2.push_back(vq2);         
        vecY2.push_back(vy2); 
         
    } 
  /****/

  /**Creation des tables**/

  
        // Pushing back above 1D vector 
        // to create the 2D vector 
    } 
  InstallSocketApplications() ;
  
  
}


/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* ********************************************************************************************* */
/* ********************************************************************************************* */
int main (int argc, char *argv[]) {
        compteurMessagesRecus=0;
        compteurMessagesEmis=0;
        srand(time(NULL));

  CommandLine cmd;
  cmd.AddValue ("scenario", "Scenario file name",  m_scenarioFileName) ;
  cmd.AddValue ("traffic", "Traffic file name", m_trafficFileName) ;
  cmd.AddValue ("packetSize", "Packet Size", m_packetSize) ;
  cmd.AddValue ("nodes", "Number of nodes", m_nNodes) ;
  cmd.AddValue ("pcap", "PCAP files enable or disable", m_pcap) ;
  cmd.AddValue ("test", "Numéro du test", m_testNumber) ;
  cmd.Parse (argc, argv);

  std::cout << "Scenario File Name = " << m_scenarioFileName << std::endl ;
  std::cout << "Traffic File Name = " << m_trafficFileName << std::endl ;
  std::cout << "Packet size = " << m_packetSize << std::endl ;
  std::cout << "Number of nodes = " << m_nNodes << std::endl ;
  std::cout << "PCAP files = " << m_pcap << std::endl ;
  //std::cout << "Test number = " << m_testNumber << std::endl ;

  std::ostringstream outputdir ;
  outputdir << m_outputdir << "/test" << m_testNumber << "/" ;
  m_outputdir = outputdir.str() ;
  std::cout << "Output dir " << m_outputdir << std::endl ;

  if (mkdir(m_outputdir.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRGRP | S_IXGRP) != 0) {
    //if (error == EEXIST)
    NS_LOG_UNCOND("Directory " << m_outputdir << " exists, all files will be replaced") ;
        //else
        //NS_FATAL_ERROR ("Unable to create dir " << m_outputdir << " error " << error << " / " << EEXIST) ;
    }
  
  CreateNS2Nodes();
  InstallYansWifi80211p();
  //InstallYansWifiWave();
  //InstallYansWifi80211b();
        cout<< "D* : Suite des evenements"<< endl;

  InstallNoProtocol() ;
        cout<< "D* : Installation des protocoles"<< endl;
        
        
        
        
  InstallApplications() ;
  
        
    int sizeVector =m_vlinks.size();
    // Inserting elements into vector 
    cout << " sizeVector : "<< sizeVector<<endl;
        
    // Displaying the 2D vector 
    for (int i = 0; i < (int)vecQ1.size(); i++) { 
    cout<<"ER "<< i << " : ";
        for (int j = 0; j < (int) m_nNodes; j++)
        {
                cout<< " : "<< vecQ1[i][j] << " ("<<j<<") | ";
        } 
             
        cout << endl; 
    } 

cout<<"FIN"<<endl;

        cout<< "D* : Installation de la couche application"<< endl;
  
  // -------------------------------------------------------------------------------------           
  std::ostringstream logFileNameAnim ;
  logFileNameAnim << m_outputdir.c_str() << "animation" << ".xml" ;
  //std::cout << "Opening file " << logFileNameAnim.str() << std::endl ;                             
  AnimationInterface anim (logFileNameAnim.str().c_str()); // Mandatory    
  
 
  Simulator::Run ();

        cout<< "D* : Fonctionnement ok"<< endl;
  
  Simulator::Destroy ();


cout<<" Messages recus "<<compteurMessagesRecus<<endl;
cout<<" Messages émis "<<compteurMessagesEmis<<endl; 

cout<<" Ratio "<<(double) compteurMessagesRecus/compteurMessagesEmis<<endl; 




  return 0;
}
