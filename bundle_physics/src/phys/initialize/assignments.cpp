//
// Created by bontius on 29/03/16.
//

#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/common/impl/assignments.hpp"
#include "tracking/common/assignments.h"
#include "tracking/common/io/impl/assignmentsIo.hpp"
#include "tracking/common/io/assignmentsIo.h"

namespace tracking {
template
class TracksToTarget<CuboidId, -1>;

namespace io {
template int readAssignments(tracking::TracksToTarget<CuboidId, -1>& assignments, std::string const& assignmentsPath,
                             GroupedTracks2d const& tracks2d, std::string const& tagName);
} //...ns io
} //...ns trackign