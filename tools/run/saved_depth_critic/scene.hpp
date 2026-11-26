#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace cabinet {
struct XY { double x, y; };
struct Section {
  std::vector<XY> normals;
  std::vector<double> bounds;
  std::vector<XY> vertices;
};
struct Band {
  std::vector<Section> parts;
  std::vector<XY> occupied;
};

class SavedScene {
public:
  explicit SavedScene(const std::string &path) {
    std::ifstream in(path);
    std::string version;
    int count;
    if (!(in >> version >> side_ >> margin_ >> count) || version!="cabinet-depth-v1" ||
        side_ != .05 || !(margin_ > 0 && margin_ < .15) || count < 1 || count > 100)
      throw std::runtime_error("invalid saved cabinet depth scene");
    for (int i=0;i<count;++i) {
      int level, parts, cells;
      if (!(in >> level >> parts >> cells) || parts < 1 || parts > 100 || cells < 1 || cells > 1000000)
        throw std::runtime_error("invalid cabinet height band");
      Band band;
      for (int j=0;j<parts;++j) {
        int n;
        if (!(in >> n) || n<3 || n>10000) throw std::runtime_error("invalid cabinet section");
        Section poly;
        for (int k=0;k<n;++k) {
          XY p;
          if (!(in >> p.x >> p.y) || !std::isfinite(p.x) || !std::isfinite(p.y))
            throw std::runtime_error("invalid cabinet section vertex");
          poly.vertices.push_back(p);
          radius_=std::max(radius_,std::hypot(p.x,p.y));
        }
        for (int k=0;k<n;++k) {
          auto a=poly.vertices[k], b=poly.vertices[(k+1)%n];
          double dx=b.x-a.x, dy=b.y-a.y, length=std::hypot(dx,dy);
          if (length<1e-12) throw std::runtime_error("degenerate cabinet section edge");
          XY normal{dy/length,-dx/length};
          poly.normals.push_back(normal);
          poly.bounds.push_back(normal.x*a.x+normal.y*a.y);
        }
        band.parts.push_back(std::move(poly));
      }
      for (int k=0;k<cells;++k) {
        int x,y;
        if (!(in >> x >> y)) throw std::runtime_error("truncated cabinet voxel list");
        band.occupied.push_back({(x+.5)*side_,(y+.5)*side_});
      }
      bands_.push_back(std::move(band));
    }
    std::string trailing;
    if (in >> trailing) throw std::runtime_error("unexpected cabinet scene data");
  }

  // Select nearby original map voxels once per DWB cycle. If a trajectory
  // leaves this window, refresh around that pose before evaluating it.
  void prepare(double x,double y) {
    origin_={x,y};
    near_.assign(bands_.size(),{});
    for (size_t i=0;i<bands_.size();++i)
      for (const auto &p:bands_[i].occupied)
        if (std::hypot(p.x-x,p.y-y)<=window_+radius_+margin_+side_)
          near_[i].push_back(p);
  }

  bool blocked(double x,double y,double yaw) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(yaw)) return true;
    if (near_.empty() || std::hypot(x-origin_.x,y-origin_.y)>window_) prepare(x,y);
    const double c=std::cos(yaw), s=std::sin(yaw), half=side_/2;
    for (size_t i=0;i<bands_.size();++i) {
      if (near_[i].empty()) continue;
      for (const auto &part:bands_[i].parts) {
        double xmin=1e30,xmax=-1e30,ymin=1e30,ymax=-1e30;
        for (const auto &v:part.vertices) {
          double px=c*v.x-s*v.y,py=s*v.x+c*v.y;
          xmin=std::min(xmin,px); xmax=std::max(xmax,px);
          ymin=std::min(ymin,py); ymax=std::max(ymax,py);
        }
        for (const auto &cell:near_[i]) {
          double dx=cell.x-x,dy=cell.y-y, pad=half+margin_;
          if (dx<xmin-pad || dx>xmax+pad || dy<ymin-pad || dy>ymax+pad) continue;
          bool separated=false;
          for (size_t k=0;k<part.normals.size();++k) {
            const auto &n=part.normals[k];
            double nx=c*n.x-s*n.y,ny=s*n.x+c*n.y;
            if (nx*dx+ny*dy>part.bounds[k]+margin_+half*(std::abs(nx)+std::abs(ny))) {
              separated=true;
              break;
            }
          }
          if (!separated) return true;
        }
      }
    }
    return false;
  }
private:
  double side_{},margin_{},radius_{};
  static constexpr double window_=.18;
  XY origin_{};
  std::vector<Band> bands_;
  std::vector<std::vector<XY>> near_;
};
}
