#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/common/Console.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Angle.hh>

#include <vector>
#include <algorithm>
#include <cmath>

using namespace gazebo;

class WaypointFollower : public ModelPlugin
{
public:
  struct WP {
    double t;                    // seconds
    ignition::math::Pose3d pose; // world pose
  };

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override
  {
    this->model = _model;
    this->world = _model->GetWorld();

    // Params
    this->loop = _sdf->HasElement("loop")   ? _sdf->Get<bool>("loop")   : true;
    this->zFix = _sdf->HasElement("z_fix")  ? _sdf->Get<double>("z_fix"): 0.85;
    this->speed = _sdf->HasElement("speed") ? _sdf->Get<double>("speed"): 0.6; // m/s (for <waypoints>)

    // Try parse <trajectory> (time + pose)
    bool ok = ParseTrajectory(_sdf);

    // If not found, try parse <waypoints> (positions only, alias: <waypoint> or <wp>)
    if (!ok) ok = ParseWaypoints(_sdf);

    if (this->wps.size() < 2u) {
      gzerr << "[WaypointFollower] Need at least 2 waypoints.\n";
      return;
    }

    // Sort by time
    std::sort(this->wps.begin(), this->wps.end(),
              [](const WP& a, const WP& b){ return a.t < b.t; });

    // Kinematic, low load
    for (auto &lnk : this->model->GetLinks()) {
      lnk->SetGravityMode(false);
      lnk->SetSelfCollide(false);
      lnk->SetKinematic(true);
    }

    this->startTime = world->SimTime().Double();

    updateConn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&WaypointFollower::OnUpdate, this, std::placeholders::_1));

    // Debug print
    gzdbg << "[WaypointFollower] Loaded on [" << this->model->GetName()
          << "] with " << this->wps.size() << " waypoints, loop="
          << (this->loop ? "true" : "false")
          << ", z_fix=" << this->zFix
          << ", speed=" << this->speed << " m/s" << std::endl;

    for (size_t i = 0; i < wps.size(); ++i) {
      const auto &p = wps[i].pose;
      gzdbg << "  [" << i << "] t=" << wps[i].t
            << " pos=(" << p.Pos().X() << "," << p.Pos().Y() << "," << p.Pos().Z()
            << ") yaw=" << p.Rot().Yaw() << std::endl;
    }
  }

  void OnUpdate(const common::UpdateInfo &info)
  {
    if (this->wps.size() < 2u) return;

    const double sim = info.simTime.Double();
    const double T = this->wps.back().t;
    double t = sim - this->startTime;

    if (this->loop && T > 0.0)       t = fmod(t, T);
    else if (t >= T)                 t = T;

    // find active segment [i, i+1]
    size_t i = 0;
    while (i + 1 < this->wps.size() && t > this->wps[i+1].t) ++i;
    if (i + 1 >= this->wps.size()) i = this->wps.size() - 2;

    const WP& a = this->wps[i];
    const WP& b = this->wps[i+1];
    const double denom = std::max(1e-6, (b.t - a.t));
    const double tau = (t - a.t) / denom;

    // interp position
    ignition::math::Vector3d pos =
      a.pose.Pos() * (1.0 - tau) + b.pose.Pos() * tau;

    // shortest yaw interp
    double yawA = a.pose.Rot().Yaw();
    double yawB = b.pose.Rot().Yaw();
    ignition::math::Angle dYaw(yawB - yawA);
    dYaw.Normalize();
    double yaw = yawA + dYaw.Radian() * tau;

    ignition::math::Quaterniond rot(0, 0, yaw);
    ignition::math::Pose3d target(pos, rot);

    this->model->SetWorldPose(target);
  }

private:
  // -------- Parsers --------
  bool ParseTrajectory(sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("trajectory")) return false;
    auto traj = _sdf->GetElement("trajectory");
    auto wp = traj->GetElement("waypoint");
    std::vector<WP> tmp;

    while (wp) {
      double t = wp->HasElement("time") ? wp->Get<double>("time") : 0.0;
      ignition::math::Pose3d p = wp->HasElement("pose")
          ? wp->Get<ignition::math::Pose3d>("pose")
          : ignition::math::Pose3d::Zero;

      // fix Z to center height
      p.Pos().Z(this->zFix);
      tmp.push_back({t, p});

      wp = wp->GetNextElement("waypoint");
    }
    if (tmp.size() < 2u) return false;
    this->wps = std::move(tmp);
    return true;
  }

  bool ParseWaypoints(sdf::ElementPtr _sdf)
  {
    if (!_sdf->HasElement("waypoints")) return false;
    auto wpsElem = _sdf->GetElement("waypoints");

    std::vector<ignition::math::Vector3d> pts;

    // <waypoint>
    for (auto e = wpsElem->GetElement("waypoint"); e; e = e->GetNextElement("waypoint")) {
      pts.push_back(e->Get<ignition::math::Vector3d>());
    }
    // alias: <wp>
    for (auto e = wpsElem->GetElement("wp"); e; e = e->GetNextElement("wp")) {
      pts.push_back(e->Get<ignition::math::Vector3d>());
    }
    if (pts.size() < 2u) return false;

    // Build timeline with constant speed
    std::vector<double> tacc;
    tacc.reserve(pts.size());
    double acc = 0.0;
    tacc.push_back(0.0);
    for (size_t i = 1; i < pts.size(); ++i) {
      double d = (pts[i] - pts[i-1]).Length();
      double dt = (this->speed > 1e-6) ? d / this->speed : 1.0;
      acc += dt;
      tacc.push_back(acc);
    }

    // Build poses with yaw facing next point
    std::vector<WP> out; out.reserve(pts.size());
    for (size_t i = 0; i < pts.size(); ++i) {
      ignition::math::Vector3d pos = pts[i];
      pos.Z(this->zFix);

      double yaw = 0.0;
      if (i + 1 < pts.size()) {
        auto dir = pts[i+1] - pts[i];
        yaw = std::atan2(dir.Y(), dir.X());
      } else if (i > 0) {
        auto dir = pts[i] - pts[i-1];
        yaw = std::atan2(dir.Y(), dir.X());
      }
      ignition::math::Quaterniond rot(0, 0, yaw);
      out.push_back({tacc[i], ignition::math::Pose3d(pos, rot)});
    }

    this->wps = std::move(out);
    return true;
  }

private:
  physics::ModelPtr model;
  physics::WorldPtr world;
  event::ConnectionPtr updateConn;

  std::vector<WP> wps;
  double startTime{0.0};
  bool loop{true};
  double zFix{0.85};
  double speed{0.6}; // for <waypoints> mode
};

GZ_REGISTER_MODEL_PLUGIN(WaypointFollower)

