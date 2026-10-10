// Frame placement: the projection against hand-computed values and PX4's own inverse, the WGS84
// tangent-plane inverse, the full ellipsoid-correct placement against hand-computed values, the
// meta rule (anchor / ekf_local_ned / refused), bounds, and the execution artifact RPP will load.
#include "dyx3_mission/frame_placement.hpp"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cmath>
#include <filesystem>
#include <string>
#include <vector>

using namespace dyx3_mission;  // NOLINT

namespace {

constexpr double kR = 6371000.0;  // PX4 CONSTANTS_RADIUS_OF_EARTH
constexpr double kDeg = 3.14159265358979323846 / 180.0;
// WGS84 radii of curvature at 13 deg N, written out independently of the implementation:
// e2 = f (2 - f), f = 1 / 298.257223563; M = a (1 - e2) / (1 - e2 sin^2)^1.5; N = a / sqrt(...).
constexpr double kM13 = 6338659.938909;  // m
constexpr double kN13 = 6379217.589221;  // m

PathArtifact make_source(const std::string& meta, const std::vector<ArtifactPoint>& pts) {
  const std::string bytes = serialize_artifact("app_v1", meta, pts);
  EXPECT_FALSE(bytes.empty()) << meta;
  const auto r = parse_artifact(bytes);
  EXPECT_TRUE(r.ok) << r.error;
  return r.artifact;
}

std::string anchored_meta(double lat, double lon) {
  return "{\"anchor\":{\"alt\":null,\"lat\":" + python_repr(lat) + ",\"lon\":" + python_repr(lon) +
         "},\"densified_steps\":0,\"frame\":\"local_ned\",\"max_boundary_snap_m\":0.0}";
}
const std::string kEkfMeta =
    "{\"anchor\":null,\"densified_steps\":0,\"frame\":\"ekf_local_ned\","
    "\"max_boundary_snap_m\":0.0}";

EkfReference ref(double lat, double lon) {
  EkfReference r;
  r.global_reference_valid = true;
  r.lat_deg = lat;
  r.lon_deg = lon;
  return r;
}

EkfReference ref_of(const GeoPoint& g) { return ref(g.lat_deg, g.lon_deg); }

// PX4 MapProjection::reproject (src/lib/geo/geo.cpp), the inverse of project, written out here
// independently of the implementation under test.
GeoPoint px4_reproject(const GeoPoint& ref, double x, double y) {
  const double rlat = ref.lat_deg * kDeg, rlon = ref.lon_deg * kDeg;
  const double xr = x / kR, yr = y / kR;
  const double c = std::sqrt(xr * xr + yr * yr);
  if (c == 0.0) return ref;
  const double lat =
      std::asin(std::cos(c) * std::sin(rlat) + xr * std::sin(c) * std::cos(rlat) / c);
  const double lon = rlon + std::atan2(yr * std::sin(c), c * std::cos(rlat) * std::cos(c) -
                                                             xr * std::sin(rlat) * std::sin(c));
  return {lat / kDeg, lon / kDeg};
}

}  // namespace

// ---- projection
TEST(Projection, HandComputedCases) {
  // On the projection's own axes the azimuthal-equidistant distance is the arc: R * angle.
  const NePoint east = project_to_ekf({0.0, 0.0}, {0.0, 0.001});
  EXPECT_NEAR(east.north_m, 0.0, 1e-9);
  EXPECT_NEAR(east.east_m, kR * 0.001 * kDeg, 1e-6);  // 111.194927 m
  EXPECT_NEAR(east.east_m, 111.194927, 1e-6);
  const NePoint north = project_to_ekf({45.0, 7.0}, {45.001, 7.0});
  EXPECT_NEAR(north.north_m, 111.194927, 1e-6);
  EXPECT_NEAR(north.east_m, 0.0, 1e-9);
  const NePoint same = project_to_ekf({52.1, 4.3}, {52.1, 4.3});
  EXPECT_EQ(same.north_m, 0.0);
  EXPECT_EQ(same.east_m, 0.0);
  // A general case against the small-offset equirectangular hand formula
  // (N = R dlat, E = R cos(mid lat) dlon): they agree to well under a millimetre at ~85 m.
  const GeoPoint r{12.9716, 77.5946}, a{12.9721, 77.5952};
  const NePoint p = project_to_ekf(r, a);
  EXPECT_NEAR(p.north_m, kR * 0.0005 * kDeg, 1e-3);                             // 55.5975 m
  EXPECT_NEAR(p.east_m, kR * std::cos(12.97185 * kDeg) * 0.0006 * kDeg, 1e-3);  // 65.0143 m
}

TEST(Projection, InverseOfPx4Reproject) {
  for (const GeoPoint r :
       {GeoPoint{12.9716, 77.5946}, GeoPoint{-33.86, 151.21}, GeoPoint{59.9, 10.7}}) {
    for (const auto& ne :
         {NePoint{0.0, 0.0}, NePoint{900.0, -400.0}, NePoint{-3.25, 0.5}, NePoint{-700.0, 700.0}}) {
      const GeoPoint g = px4_reproject(r, ne.north_m, ne.east_m);
      const NePoint back = project_to_ekf(r, g);
      EXPECT_NEAR(back.north_m, ne.north_m, 1e-6);
      EXPECT_NEAR(back.east_m, ne.east_m, 1e-6);
    }
  }
}

// ---- WGS84 tangent plane (the tablet's model) and the one placement path
TEST(Wgs84, RadiiOfCurvature) {
  const Wgs84Radii r13 = wgs84_radii(13.0);
  EXPECT_NEAR(r13.meridian_m, kM13, 1e-5);
  EXPECT_NEAR(r13.prime_vertical_m, kN13, 1e-5);
  const Wgs84Radii eq = wgs84_radii(0.0);  // equator: N = a, M = a (1 - e2)
  EXPECT_NEAR(eq.prime_vertical_m, 6378137.0, 1e-6);
  EXPECT_NEAR(eq.meridian_m, 6335439.327, 1e-3);
  const Wgs84Radii pole = wgs84_radii(90.0);  // pole: M = N = a / sqrt(1 - e2)
  EXPECT_NEAR(pole.meridian_m, pole.prime_vertical_m, 1e-6);
  EXPECT_NEAR(pole.meridian_m, 6399593.626, 1e-3);
}

TEST(Wgs84, TangentPlaneInverse) {
  const GeoPoint anchor{13.0, 77.5};
  const GeoPoint g = tangent_plane_to_geo(anchor, {100.0, -250.0});
  EXPECT_NEAR((g.lat_deg - 13.0) * kDeg * kM13, 100.0, 1e-6);
  EXPECT_NEAR((g.lon_deg - 77.5) * kDeg * kN13 * std::cos(13.0 * kDeg), -250.0, 1e-6);
  const GeoPoint same = tangent_plane_to_geo(anchor, {0.0, 0.0});
  EXPECT_EQ(same.lat_deg, 13.0);
  EXPECT_EQ(same.lon_deg, 77.5);
  EXPECT_FALSE(std::isfinite(tangent_plane_to_geo({90.0, 0.0}, {0.0, 1.0}).lon_deg));
}

// Owner case: anchor == EKF reference at 13 deg N. 100 m of ground north is 100 R / M EKF metres
// (+0.510203 %), 100 m of ground east is 100 R / N EKF metres (-0.128818 %): a design translated
// without this would be painted 51.0 cm short north-south and 12.9 cm long east-west per 100 m.
TEST(Placement, EllipsoidScaleAtThirteenDegreesNorth) {
  const GeoPoint ref{13.0, 77.5};
  const NePoint north = place_point(ref, ref, {100.0, 0.0});
  EXPECT_NEAR(north.north_m, 100.0 * kR / kM13, 1e-4);  // 100.510203 m
  EXPECT_NEAR(north.north_m, 100.510203441, 1e-6);
  EXPECT_NEAR(north.east_m, 0.0, 1e-9);
  const NePoint east = place_point(ref, ref, {0.0, 100.0});
  EXPECT_NEAR(east.east_m, 100.0 * kR / kN13, 1e-4);  // 99.871182 m
  EXPECT_NEAR(east.east_m, 99.871181863, 1e-6);
  // The parallel through the reference is not the azimuthal projection's east axis: it lies
  // R sin(phi0) cos(phi0) (1 - cos dlambda) north of it (0.18 mm at 100 m, hand formula).
  const double dlon = 100.0 / (kN13 * std::cos(13.0 * kDeg));
  EXPECT_NEAR(east.north_m,
              kR * std::sin(13.0 * kDeg) * std::cos(13.0 * kDeg) * (1.0 - std::cos(dlon)), 1e-9);
  EXPECT_NEAR(east.north_m, 0.000180720, 1e-9);
  // Both at once: the same scales plus the second-order terms of the projection (0.18 mm north,
  // -0.36 mm east at 141 m, meridian convergence); independent reference implementation values.
  const NePoint both = place_point(ref, ref, {100.0, 100.0});
  EXPECT_NEAR(both.north_m, 100.510384164, 1e-6);
  EXPECT_NEAR(both.east_m, 99.870818102, 1e-6);
  EXPECT_NEAR(both.north_m, 100.0 * kR / kM13, 0.5e-3);
  EXPECT_NEAR(both.east_m, 100.0 * kR / kN13, 0.5e-3);
  // (0, 0) at the reference is the origin, exactly.
  const NePoint zero = place_point(ref, ref, {0.0, 0.0});
  EXPECT_EQ(zero.north_m, 0.0);
  EXPECT_EQ(zero.east_m, 0.0);
}

// Anchor 50 m from the EKF reference (29.870 m north, 40.139 m east on the ground at 13 deg N).
// Hand check of the anchor on the sphere (k = c / sin c = 1 + 1e-11 here): north = R dlat plus
// the parallel's offset R sin(lat_r) cos(lat_r) dlon^2 / 2 (29 micrometres), east =
// R cos(lat_anchor) sin(dlon). The points: independent reference implementation (WGS84 inverse at
// the anchor, then PX4 MapProjection::project in double), to 1 micrometre.
TEST(Placement, AnchorAwayFromTheReferenceHandComputed) {
  const GeoPoint ref{13.0, 77.5};
  const GeoPoint anchor{13.00027, 77.50037};
  const NePoint a = place_point(ref, anchor, {0.0, 0.0});
  const double dlon = 0.00037 * kDeg;
  EXPECT_NEAR(
      a.north_m,
      kR * 0.00027 * kDeg + kR * std::sin(13.0 * kDeg) * std::cos(13.0 * kDeg) * dlon * dlon / 2.0,
      1e-6);
  EXPECT_NEAR(a.north_m, 30.022659312, 1e-6);
  EXPECT_NEAR(a.east_m, kR * std::cos(13.00027 * kDeg) * std::sin(dlon), 1e-6);
  EXPECT_NEAR(a.east_m, 40.087609302, 1e-6);
  const NePoint p = place_point(ref, anchor, {10.0, -20.0});
  EXPECT_NEAR(p.north_m, 40.073657660, 1e-6);
  EXPECT_NEAR(p.east_m, 20.113365742, 1e-6);
  const NePoint q = place_point(ref, anchor, {-50.0, 30.0});
  EXPECT_NEAR(q.north_m, -20.232381578, 1e-6);
  EXPECT_NEAR(q.east_m, 70.049091221, 1e-6);
  // The same through place_artifact (the node's path) and in the stored bytes.
  const auto src = make_source(anchored_meta(13.00027, 77.50037),
                               {{0.0, 0.0, 0}, {10.0, -20.0, 1}, {-50.0, 30.0, 3}});
  const Placement pl = place_artifact(src, ref_of(ref), 1000.0);
  ASSERT_TRUE(pl.ok) << pl.detail;
  const auto back = parse_artifact(pl.bytes, pl.execution.sha256);
  ASSERT_TRUE(back.ok) << back.error;
  ASSERT_EQ(back.artifact.points.size(), 3U);
  EXPECT_NEAR(back.artifact.points[1].north_m, 40.073657660, 1e-6);
  EXPECT_NEAR(back.artifact.points[1].east_m, 20.113365742, 1e-6);
  EXPECT_NEAR(back.artifact.points[2].north_m, -20.232381578, 1e-6);
  EXPECT_NEAR(back.artifact.points[2].east_m, 70.049091221, 1e-6);
  EXPECT_NE(back.artifact.meta_json.find("wgs84_tangent_plane_to_px4_map_projection"),
            std::string::npos);
}

// ---- meta rule
TEST(FrameSpec, AnchoredLocalNed) {
  const auto f = read_frame_spec(anchored_meta(12.5, 77.25));
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_EQ(f.kind, FrameKind::kAnchored);
  EXPECT_EQ(f.anchor.lat_deg, 12.5);
  EXPECT_EQ(f.anchor.lon_deg, 77.25);
  // integer degrees and an altitude are fine
  EXPECT_TRUE(read_frame_spec("{\"anchor\":{\"alt\":912.5,\"lat\":12,\"lon\":77},\"frame\":"
                              "\"local_ned\"}")
                  .ok);
}

TEST(FrameSpec, EkfLocalNedWithNullOrNoAnchor) {
  auto f = read_frame_spec(kEkfMeta);
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_EQ(f.kind, FrameKind::kEkfLocal);
  f = read_frame_spec("{\"frame\":\"ekf_local_ned\"}");
  ASSERT_TRUE(f.ok) << f.error;
  EXPECT_EQ(f.kind, FrameKind::kEkfLocal);
}

TEST(FrameSpec, EverythingElseIsRefused) {
  for (const std::string meta : {
           std::string("{\"num_waypoints\":3,\"origin_ne_m\":[0.0,0.0]}"),  // old artifact
           std::string("{\"planning\":{\"frame\":\"ekf_local_ned\"}}"),     // nested is not THE key
           std::string("{\"anchor\":null,\"frame\":\"local_ned\"}"),
           std::string("{\"frame\":\"local_ned\"}"),
           std::string("{\"anchor\":{\"lat\":1.0,\"lon\":2.0},\"frame\":\"ekf_local_ned\"}"),
           std::string("{\"anchor\":{\"lat\":1.0},\"frame\":\"local_ned\"}"),
           std::string("{\"anchor\":{\"lat\":\"1\",\"lon\":2.0},\"frame\":\"local_ned\"}"),
           std::string("{\"anchor\":{\"lat\":91.0,\"lon\":2.0},\"frame\":\"local_ned\"}"),
           std::string("{\"anchor\":{\"lat\":1.0,\"lon\":180.5},\"frame\":\"local_ned\"}"),
           std::string("{\"anchor\":[1.0,2.0],\"frame\":\"local_ned\"}"),
           std::string("{\"frame\":\"ned\"}"),
           std::string("{\"frame\":\"ekf_execution\"}"),  // a placed execution is never a source
           std::string("{\"frame\":1}"),
       }) {
    EXPECT_FALSE(read_frame_spec(meta).ok) << meta;
  }
}

// ---- placement
TEST(Placement, AnchoredTrajectoryIsPlacedThroughTheEllipsoid) {
  // EKF origin at (45, 7); anchor 0.001 deg north of it: 111.194927 m north, 0 m east. The point
  // (10, -5) ground metres is NOT anchor + (10, -5): at 45 deg R/M = 1.000568, R/N = 0.997206.
  const auto src = make_source(anchored_meta(45.001, 7.0), {{0.0, 0.0, 2}, {10.0, -5.0, 3}});
  const Placement p = place_artifact(src, ref(45.0, 7.0), 1000.0);
  ASSERT_TRUE(p.ok) << p.detail;
  EXPECT_TRUE(p.transformed);
  EXPECT_NEAR(p.anchor_ekf.north_m, 111.194927, 1e-6);
  EXPECT_NEAR(p.anchor_ekf.east_m, 0.0, 1e-9);
  ASSERT_EQ(p.execution.points.size(), 2U);
  EXPECT_NEAR(p.execution.points[0].north_m, 111.194927, 1e-6);
  EXPECT_NEAR(p.execution.points[0].east_m, 0.0, 1e-9);
  EXPECT_NEAR(p.execution.points[1].north_m, 121.200609211, 1e-6);
  EXPECT_NEAR(p.execution.points[1].east_m, -4.986031366, 1e-6);
  EXPECT_EQ(p.execution.points[0].flags, 2);  // flags travel unchanged
  EXPECT_EQ(p.execution.points[1].flags, 3);
  // The bytes are a canonical DYX3PATH 1 that the reader (RPP's) accepts under the new sha.
  const auto back = parse_artifact(p.bytes, p.execution.sha256);
  ASSERT_TRUE(back.ok) << back.error;
  EXPECT_NE(p.execution.sha256, src.sha256);
  EXPECT_EQ(back.artifact.points.size(), 2U);
  EXPECT_NE(back.artifact.meta_json.find(src.sha256), std::string::npos);  // provenance
  // An execution artifact is never accepted as a source (it would be placed twice).
  EXPECT_FALSE(read_frame_spec(back.artifact.meta_json).ok);
}

TEST(Placement, IsDeterministicAndDependsOnTheReference) {
  const auto src = make_source(anchored_meta(12.9721, 77.5952), {{0.0, 0.0, 0}, {1.0, 1.0, 1}});
  const Placement a = place_artifact(src, ref(12.9716, 77.5946), 1000.0);
  const Placement b = place_artifact(src, ref(12.9716, 77.5946), 1000.0);
  const Placement c = place_artifact(src, ref(12.9717, 77.5946), 1000.0);
  ASSERT_TRUE(a.ok && b.ok && c.ok);
  EXPECT_EQ(a.bytes, b.bytes);
  EXPECT_EQ(a.execution.sha256, b.execution.sha256);
  EXPECT_NE(a.execution.sha256, c.execution.sha256);
  EXPECT_NEAR(a.execution.points[0].north_m, 55.597540, 1e-6);
  EXPECT_NEAR(a.execution.points[0].east_m, 65.014305, 1e-6);
}

TEST(Placement, EkfLocalNedIsAPassthrough) {
  const auto src = make_source(kEkfMeta, {{1.0, 2.0, 1}, {3.0, 4.0, 3}});
  const Placement p = place_artifact(src, EkfReference{}, 1000.0);  // no reference needed
  ASSERT_TRUE(p.ok) << p.detail;
  EXPECT_FALSE(p.transformed);
  EXPECT_EQ(p.execution.sha256, src.sha256);
  EXPECT_TRUE(p.bytes.empty());
  ASSERT_EQ(p.execution.points.size(), 2U);
  EXPECT_EQ(p.execution.points[1].north_m, 3.0);
}

TEST(Placement, InvalidReferenceIsRefused) {
  const auto src = make_source(anchored_meta(45.001, 7.0), {{0.0, 0.0, 0}});
  EXPECT_EQ(place_artifact(src, EkfReference{}, 1000.0).error, PlacementError::kReferenceInvalid);
  EkfReference bad = ref(std::nan(""), 7.0);
  EXPECT_EQ(place_artifact(src, bad, 1000.0).error, PlacementError::kReferenceInvalid);
  bad = ref(45.0, 200.0);
  EXPECT_EQ(place_artifact(src, bad, 1000.0).error, PlacementError::kReferenceInvalid);
}

TEST(Placement, OutOfBoundsIsRefused) {
  // Anchor ~1.1 km from the EKF origin.
  auto src = make_source(anchored_meta(45.01, 7.0), {{0.0, 0.0, 0}});
  EXPECT_EQ(place_artifact(src, ref(45.0, 7.0), 1000.0).error, PlacementError::kOutOfBounds);
  // Anchor near, a placed point far.
  src = make_source(anchored_meta(45.0, 7.0), {{0.0, 0.0, 0}, {999.0, 100.0, 0}});
  EXPECT_EQ(place_artifact(src, ref(45.0, 7.0), 1000.0).error, PlacementError::kOutOfBounds);
  EXPECT_TRUE(place_artifact(src, ref(45.0, 7.0), 1000.0 + 6.0).ok);
  // The bound is on the PLACED (EKF) coordinates: 998 m of ground north at 13 deg N is 1003.09
  // EKF metres, beyond 1000 m (a plain translation would have let it through).
  src = make_source(anchored_meta(13.0, 77.5), {{0.0, 0.0, 0}, {998.0, 0.0, 0}});
  const Placement far = place_artifact(src, ref(13.0, 77.5), 1000.0);
  EXPECT_EQ(far.error, PlacementError::kOutOfBounds);
  EXPECT_NE(far.detail.find("placed point 1"), std::string::npos);
  EXPECT_TRUE(place_artifact(src, ref(13.0, 77.5), 1003.1).ok);
  // An anchor at a pole has no east direction: refused, never NaN coordinates.
  src = make_source(anchored_meta(90.0, 0.0), {{0.0, 0.0, 0}, {0.0, 1.0, 0}});
  EXPECT_EQ(place_artifact(src, ref(89.9999, 0.0), 1000.0).error, PlacementError::kOutOfBounds);
  // An ekf_local_ned path far from the origin.
  src = make_source(kEkfMeta, {{0.0, 0.0, 0}, {0.0, 1200.0, 0}});
  EXPECT_EQ(place_artifact(src, EkfReference{}, 1000.0).error, PlacementError::kOutOfBounds);
}

TEST(Placement, NoFrameIsRefusedWithItsOwnError) {
  const auto src = make_source("{\"num_waypoints\":1}", {{0.0, 0.0, 0}});
  const Placement p = place_artifact(src, ref(45.0, 7.0), 1000.0);
  EXPECT_FALSE(p.ok);
  EXPECT_EQ(p.error, PlacementError::kNoPlacementFrame);
  EXPECT_FALSE(p.detail.empty());
}

TEST(Placement, StoredExecutionArtifactLoadsByItsSha) {
  namespace fs = std::filesystem;
  const fs::path dir =
      fs::temp_directory_path() / ("dyx3_placement_test_" + std::to_string(getpid()));
  fs::create_directories(dir);
  const auto src = make_source(anchored_meta(45.0005, 7.0005), {{0.0, 0.0, 2}, {2.0, 0.0, 3}});
  const Placement p = place_artifact(src, ref(45.0, 7.0), 1000.0);
  ASSERT_TRUE(p.ok);
  std::string err;
  ASSERT_TRUE(store_execution_artifact(dir.string(), p, &err)) << err;
  ASSERT_TRUE(store_execution_artifact(dir.string(), p, &err)) << err;  // idempotent
  const auto loaded = load_artifact(dir.string(), p.execution.sha256);
  ASSERT_TRUE(loaded.ok) << loaded.error;
  EXPECT_EQ(loaded.artifact.points.size(), 2U);
  std::size_t files = 0;
  for (const auto& e : fs::directory_iterator(dir)) {
    (void)e;
    ++files;
  }
  EXPECT_EQ(files, 1U);  // no temporary left behind
  // A passthrough writes nothing.
  const Placement pass = place_artifact(make_source(kEkfMeta, {{0.0, 0.0, 0}}), {}, 1000.0);
  ASSERT_TRUE(store_execution_artifact((dir / "absent").string(), pass, &err));
  EXPECT_FALSE(fs::exists(dir / "absent"));
  fs::remove_all(dir);
}
