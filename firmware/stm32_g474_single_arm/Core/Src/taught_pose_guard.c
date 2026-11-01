#include "taught_pose_guard.h"
#include TAUGHT_POSE_PROFILE_HEADER
#include <stdlib.h>
#include <string.h>
#define CORRIDOR 12000
#define MAX_STEP 1100
static int vertex, destination;
static int64_t progress;
static int32_t previous[6];
static bool has_previous,failed,aligned;
void TaughtPoseGuard_Reset(void){vertex=0;destination=-1;progress=0;has_previous=false;failed=false;aligned=false;}
static int64_t mag(int64_t x){return x<0?-x:x;}
static bool near(const int32_t q[12],int v){
 for(unsigned j=0;j<6;j++)if(mag((int64_t)q[j]-taught_targets[v][j])>CORRIDOR)return false;
 return true;
}
static bool on_edge(const int32_t q[12],int a,int b,int64_t *out){
 unsigned k=0;int64_t d[6];
 for(unsigned j=0;j<6;j++){d[j]=(int64_t)taught_targets[b][j]-taught_targets[a][j];if(mag(d[j])>mag(d[k]))k=j;}
 int64_t width=mag(d[k]);if(width==0)return false;
 int64_t s=((int64_t)q[k]-taught_targets[a][k])*(d[k]<0?-1:1);
 if(s < -CORRIDOR || s > width+CORRIDOR)return false;
 for(unsigned j=0;j<6;j++)if(mag(((int64_t)q[j]-taught_targets[a][j])*width-d[j]*s)>CORRIDOR*width)return false;
 *out=s;return true;
}
bool TaughtPoseGuard_Check(const int32_t q[12],bool terminal){
 if(failed||q==NULL)return false;
 for(unsigned j=6;j<12;j++)if(q[j]!=0)goto reject;
 for(unsigned j=0;j<6;j++)if(has_previous&&mag((int64_t)q[j]-previous[j])>MAX_STEP)goto reject;
 /* First leg returns the measured pose to Home from anywhere within the
  * installed bounds. Speed and monotonic direction remain enforced. */
 if(!aligned){
  for(unsigned j=0;j<6;j++){
   int64_t error=(int64_t)q[j]-taught_targets[0][j];
   if(q[j]<taught_minimum_urad[j] || q[j]>taught_maximum_urad[j])goto reject;
   if(has_previous){
    int64_t old=(int64_t)previous[j]-taught_targets[0][j];
    if(mag(error)>mag(old)+4 || (old>4&&error < -4) || (old < -4&&error>4))goto reject;
   }
  }
  if(terminal){if(!near(q,0))goto reject;aligned=true;}
  goto accept;
 }
 if(destination<0){
  if(near(q,vertex))goto accept;
  int64_t s=0;int candidate=-1;
  for(int v=0;v<5;v++)if(abs(v-vertex)==1&&on_edge(q,vertex,v,&s)){candidate=v;break;}
  if(candidate<0)goto reject;
  destination=candidate;progress=s;
 }
 {int64_t s=0;if(!on_edge(q,vertex,destination,&s)||s<progress-4)goto reject;progress=s;}
 if(terminal){if(!near(q,destination))goto reject;vertex=destination;destination=-1;progress=0;}
accept:
 memcpy(previous,q,sizeof(previous));has_previous=true;return true;
reject:
 failed=true;return false;
}
