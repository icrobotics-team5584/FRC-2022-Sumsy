#pragma once

#include <photonlib/Packet.h>
#include <photonlib/PhotonTrackedTarget.h>

namespace ICPhotonFixes {

int GetFiducialId(photonlib::PhotonTrackedTarget& target) {
  photonlib::Packet packet;
  double yaw, pitch, area, skew;
  int fiducialId;
  packet << target;
  packet >> yaw >> pitch >> area >> skew >> fiducialId;

  return fiducialId;
}

}  // namespace ICPhotonFixes