#include <iostream>
#include <iomanip>
#include "karto_sdk/Mapper.h"
int main(int argc,char**argv){
 if(argc!=2)return 2;
 karto::Mapper mapper;
 mapper.LoadFromFile(argv[1]);
 auto scans=mapper.GetAllProcessedScans();
 std::cout<<std::setprecision(17)<<"[";
 bool first=true;
 for(auto s:scans){
  auto p=s->GetCorrectedPose();
  if(!first)std::cout<<",";first=false;
  std::cout<<"{\"replay_stamp\":"<<s->GetTime()<<",\"xy_yaw\":["<<p.GetX()<<","<<p.GetY()<<","<<p.GetHeading()<<"]}";
 }
 std::cout<<"]\n";
}
