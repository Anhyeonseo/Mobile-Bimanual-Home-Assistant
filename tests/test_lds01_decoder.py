"""Compile the deployed C++ decoder without ROS, using realistic sector faults."""
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def test_sector_loss_never_consumes_the_next_rotation(tmp_path):
    source = tmp_path/'decoder.cpp'
    source.write_text(r'''
#include "hls_lfcd_lds_driver/scan_decoder.hpp"
#include "hls_lfcd_lds_driver/scan_timing.hpp"
#include <cassert>
#include <cmath>
using namespace hls_lfcd_lds;
std::array<uint8_t,42> packet(int id) {
  std::array<uint8_t,42> p{};p[0]=250;p[1]=160+id;p[2]=0xb8;p[3]=0x0b;
  for(int i=0;i<6;++i){p[4+6*i]=100;p[6+6*i]=0xe8;p[7+6*i]=3;}
  uint8_t sum=255;for(int i=0;i<40;++i)sum-=p[i];p[40]=sum;p[41]=sum;return p;
}
void send(ScanDecoder & d,int id,double t,std::vector<DecodedScan>& scans,bool corrupt=false) {
  auto p=packet(id);if(corrupt)p[15]^=1;
  for(auto b:p){auto scan=d.feed(b,t);if(scan)scans.push_back(*scan);}
}
int main() {
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<59;++i)send(d,i,i/300.,s);
   assert(s.empty());send(d,0,.2,s);assert(s.size()==1 && s[0].sectors==59);
   for(int i=0;i<6;++i)assert(std::isnan(s[0].ranges[i]));
   for(int i=6;i<360;++i)assert(s[0].ranges[i]==1.0f);
   for(int i=1;i<60;++i)send(d,i,.2+i/300.,s);
   assert(s.size()==2 && s[1].sectors==60 && d.partial_scans==1 && d.complete_scans==1);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<59;++i)send(d,i,10.+i/300.,s);
   send(d,0,10.2,s);assert(s.size()==1);
   auto t=scan_timing(s[0],10.2);assert(t && t->time_increment_s<0);
   assert(s[0].last_sector==58 && s[0].last_received<10.2);
   assert(std::abs(t->index_zero_steady_s-(10.+59./300.-420./230400.))<1e-10);
   assert(t->index_zero_steady_s+359*t->time_increment_s < s[0].first_received);
   assert(!scan_timing(s[0],11.));assert(!scan_timing(s[0],9.));
   for(int i=1;i<60;++i)send(d,i,10.2+i/300.,s);
   auto full=scan_timing(s[1],10.4);assert(full);
   assert(std::abs(full->index_zero_steady_s-t->index_zero_steady_s-.2)<1e-10);
   assert(full->index_zero_steady_s<10.4);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<60;++i)send(d,i,i/300.,s,i==15);
   assert(s.size()==1 && s[0].sectors==59 && d.checksum_errors>=1);
   for(int i=264;i<270;++i)assert(std::isnan(s[0].ranges[i]));}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<57;++i)send(d,i,i/300.,s);
   for(int i=0;i<60;++i)send(d,i,.2+i/300.,s);
   assert(s.size()==1 && s[0].sectors==60 && d.rejected_scans==1);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<59;++i)send(d,i,i/300.,s);
   send(d,59,1.,s);assert(s.empty() && d.rejected_scans==1);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=0;i<60;++i){send(d,i,i/300.,s);if(i==20)send(d,i,i/300.,s);}
   assert(s.size()==1 && s[0].sectors==60 && d.duplicate_sectors==1);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   for(int i=30;i<60;++i){send(d,i,i/300.,s);}
   assert(s.empty());
   for(int i=0;i<60;++i){send(d,i,.2+i/300.,s);}
   assert(s.size()==1);
   d.reset();for(int i=10;i<60;++i)send(d,i,1.+i/300.,s);assert(s.size()==1);}
  {ScanDecoder d;std::vector<DecodedScan> s;
   send(d,0,1.,s);send(d,1,.5,s);assert(s.empty() && d.rejected_scans==1);}
}
''')
    exe = tmp_path/'decoder'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror','-I',
        str(ROOT/'ros2_ws/src/hls_lfcd_lds_driver/include'),str(source),'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True,timeout=10)
