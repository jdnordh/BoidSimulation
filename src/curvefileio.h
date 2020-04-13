#pragma once

#include <string>

#include "curve.h"

namespace geometry {

void saveCurveToFile(geometry::Curve const &curve, std::string const &filePath);

geometry::Curve loadCurveFromFile(std::string const &filePath);

geometry::Curve loadCurveFrom_OBJ_File(std::string const &filePath);

} // namespace geometry
