// Frame placement: the projection against hand-computed values and PX4's own inverse, the meta
// rule (anchor / ekf_local_ned / refused), bounds, and the execution artifact RPP will load.
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
TEST(Placement, AnchoredTrajectoryIsTranslatedByTheProjectedAnchor) {
  // EKF origin at (45, 7); anchor 0.001 deg north of it: 111.194927 m north, 0 m east.
  const auto src = make_source(anchored_meta(45.001, 7.0), {{0.0, 0.0, 2}, {10.0, -5.0, 3}});
  const Placement p = place_artifact(src, ref(45.0, 7.0), 1000.0);
  ASSERT_TRUE(p.ok) << p.detail;
  EXPECT_TRUE(p.transformed);
  EXPECT_NEAR(p.anchor_ekf.north_m, 111.194927, 1e-6);
  EXPECT_NEAR(p.anchor_ekf.east_m, 0.0, 1e-9);
  ASSERT_EQ(p.execution.points.size(), 2U);
  EXPECT_NEAR(p.execution.points[0].north_m, 111.194927, 1e-6);
  EXPECT_NEAR(p.execution.points[0].east_m, 0.0, 1e-9);
  EXPECT_NEAR(p.execution.points[1].north_m, 121.194927, 1e-6);
  EXPECT_NEAR(p.execution.points[1].east_m, -5.0, 1e-9);
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
