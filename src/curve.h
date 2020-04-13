#pragma once

#include <iosfwd>
#include <vector>

#include <glm/glm.hpp>

namespace geometry {

using Points = std::vector<glm::vec3>;

// helper function, put in another header file?
glm::vec3 midpoint(glm::vec3 const &a, glm::vec3 const &b);

class Curve {
public:
  Curve();
  Curve(Points points);

  glm::vec3 operator[](int idx) const;
  glm::vec3 &operator[](int idx);
  glm::vec3 front() const;
  glm::vec3 back() const;

  // intended interface of curve: scalar parameter
  glm::vec3 operator()(float t) const;

  size_t pointCount() const;
  glm::vec3 const *data() const;
  Points const &points() const;

private:
  Points m_points;
};

// Free functions
float length(Curve const &curve);

Points midpointSubdivide(Points const &points);
Points repeatedAveragingStep(Points points);
Points repeatedAveraging(Points points, int numberOfAveragingSteps);

Points repeatedAveraging(Curve const &curve, int numberOfAveragingSteps);
Curve cubicSubdivideCurve(Curve const &curve, int numberOfSubdivisionSteps);

} // namespace geometry
