#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <gp_Pnt.hxx>
#include <iomanip>
#include <iostream>
#include <limits>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>
#include <vector>
namespace {
using Json = nlohmann::json;
using Point = std::array<double, 3>;
using Face = std::array<size_t, 3>;
constexpr double linear_deflection = .25, angular_deflection = .15;
struct Mesh {
  std::vector<Point> vertices;
  std::vector<Face> faces;
};
Point transform(const std::string& part, Point p) {
  if (part == "armor_am02") {
    p[0] -= -17.40697118333;
    p[1] -= 63.89617963739;
    p[2] -= -45.00439550208;
    // Exact row-vector matrix used by the original converter: [x,-z,y].
    return {p[0] * .001, -p[2] * .001, p[1] * .001};
  }
  if (part == "armor_frame_a") {
    const double s = std::sin(std::acos(-1.) / 12), c = std::cos(std::acos(-1.) / 12);
    p[1] -= 32.;
    return {(-s * p[1] + c * p[2]) * .001, p[0] * .001, (c * p[1] + s * p[2]) * .001};
  }
  throw std::invalid_argument("Unknown official CAD part: " + part);
}
Mesh tessellate(const std::filesystem::path& file, const std::string& part) {
  STEPControl_Reader reader;
  if (!Interface_Static::SetCVal("xstep.cascade.unit", "MM"))
    throw std::runtime_error("Cannot select STEP millimetre units");
  if (reader.ReadFile(file.c_str()) != IFSelect_RetDone)
    throw std::runtime_error("Cannot read STEP: " + file.string());
  for (int i = 1; i <= reader.NbRootsForTransfer(); ++i)
    if (!reader.TransferRoot(i)) throw std::runtime_error("STEP root transfer failed");
  if (reader.NbShapes() < 1) throw std::runtime_error("STEP has no transferred shapes");
  // CadQuery importStep(...).val() selects the first transferred shape.
  TopoDS_Shape shape = reader.Shape(1);
  if (!BRepTools::Triangulation(shape, linear_deflection)) {
    BRepMesh_IncrementalMesh mesher(shape, linear_deflection, true, angular_deflection);
    if (!mesher.IsDone()) throw std::runtime_error("OCCT triangulation failed");
  }
  TopTools_IndexedMapOfShape faces;
  TopExp::MapShapes(shape, TopAbs_FACE, faces);
  Mesh result;
  for (int fi = 1; fi <= faces.Extent(); ++fi) {
    auto face = TopoDS::Face(faces(fi));
    TopLoc_Location location;
    auto poly = BRep_Tool::Triangulation(face, location);
    if (poly.IsNull()) continue;
    const size_t offset = result.vertices.size();
    auto trsf = location.Transformation();
    for (int i = 1; i <= poly->NbNodes(); ++i) {
#if OCC_VERSION_HEX < 0x070600
      auto p = poly->Nodes().Value(i).Transformed(trsf);
#else
      auto p = poly->Node(i).Transformed(trsf);
#endif
      auto v = transform(part, {p.X(), p.Y(), p.Z()});
      for (double a : v)
        if (!std::isfinite(a)) throw std::runtime_error("Nonfinite tessellated vertex");
      result.vertices.push_back(v);
    }
    for (int i = 1; i <= poly->NbTriangles(); ++i) {
#if OCC_VERSION_HEX < 0x070600
      auto triangle = poly->Triangles().Value(i);
#else
      auto triangle = poly->Triangle(i);
#endif
      int a, b, c;
      triangle.Get(a, b, c);
      if (a < 1 || b < 1 || c < 1 || a > poly->NbNodes() || b > poly->NbNodes() ||
          c > poly->NbNodes())
        throw std::runtime_error("OCCT returned invalid triangle indices");
      if (face.Orientation() == TopAbs_REVERSED) std::swap(b, c);
      result.faces.push_back(
          {offset + size_t(a - 1), offset + size_t(b - 1), offset + size_t(c - 1)});
    }
  }
  if (result.vertices.empty() || result.faces.empty())
    throw std::runtime_error("STEP tessellation is empty");
  return result;
}
void write_obj(const std::filesystem::path& file, const Mesh& mesh) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("Cannot create OBJ: " + file.string());
  out << std::fixed << std::setprecision(9);
  for (auto v : mesh.vertices) out << "v " << v[0] << ' ' << v[1] << ' ' << v[2] << '\n';
  for (auto f : mesh.faces) out << "f " << f[0] + 1 << ' ' << f[1] + 1 << ' ' << f[2] + 1 << '\n';
  out.flush();
  if (!out) throw std::runtime_error("OBJ write failed: " + file.string());
}
Json metadata(const Mesh& mesh) {
  Point lo = mesh.vertices.front(), hi = lo;
  for (auto v : mesh.vertices)
    for (int a = 0; a < 3; ++a) {
      lo[a] = std::min(lo[a], v[a]);
      hi[a] = std::max(hi[a], v[a]);
    }
  return {{"min_m", lo}, {"max_m", hi}, {"triangles", mesh.faces.size()}};
}
void write_json(const std::filesystem::path& file, const Json& data) {
  std::ofstream out(file);
  if (!out) throw std::runtime_error("Cannot create " + file.string());
  out << data.dump(2) << '\n';
  out.flush();
  if (!out) throw std::runtime_error("Write failed: " + file.string());
}
void self_test() {
  auto near = [](Point a, Point b) {
    for (int i = 0; i < 3; ++i)
      if (std::abs(a[i] - b[i]) > 1e-12) throw std::runtime_error("Coordinate transform mismatch");
  };
  near(transform("armor_am02", {-17.40697118333, 63.89617963739, -45.00439550208}), {0, 0, 0});
  near(transform("armor_am02", {-16.40697118333, 65.89617963739, -42.00439550208}),
       {.001, -.003, .002});
  near(transform("armor_frame_a", {0, 32, 0}), {0, 0, 0});
  near(transform("armor_frame_a", {1, 32, 0}), {0, .001, 0});
  const double s = std::sin(std::acos(-1.) / 12), c = std::cos(std::acos(-1.) / 12);
  near(transform("armor_frame_a", {0, 33, 0}), {-.001 * s, 0, .001 * c});
  near(transform("armor_frame_a", {0, 32, 1}), {.001 * c, 0, .001 * s});
  std::cout << "Official-part coordinate transforms passed\n";
}
}  // namespace
int main(int argc, char** argv) {
  try {
    std::filesystem::path root = RM_CAD_SOURCE_ROOT, input, output, reference;
    bool strict = false;
    for (int i = 1; i < argc; ++i) {
      std::string a = argv[i];
      auto value = [&]() {
        if (++i >= argc) throw std::invalid_argument("Missing argument for " + a);
        return std::filesystem::path(argv[i]);
      };
      if (a == "--root")
        root = value();
      else if (a == "--input-dir")
        input = value();
      else if (a == "--output")
        output = value();
      else if (a == "--compare")
        reference = value();
      else if (a == "--strict-triangles")
        strict = true;
      else if (a == "--self-test") {
        self_test();
        return 0;
      } else if (a == "--help") {
        std::cout << "STEP 转换器用法：\n"
                     "rm_convert_cad [--root 仓库路径] [--input-dir 输入目录] [--output 输出目录] "
                     "[--compare metadata.json] [--strict-triangles]\n"
                     "导出两份官方 STEP 零件。默认输出目录为 output/native-cad。\n";
        return 0;
      } else
        throw std::invalid_argument("Unknown option: " + a);
    }
    root = std::filesystem::absolute(root);
    if (input.empty()) input = root / "assets/official";
    if (output.empty()) output = root / "output/native-cad";
    Json baseline;
    if (!reference.empty()) {
      std::ifstream in(reference);
      if (!in) throw std::runtime_error("Cannot read reference metadata");
      in >> baseline;
    }
    // Complete both imports before writing: invalid source must not yield partial exports.
    std::array<std::string, 2> names = {"armor_am02", "armor_frame_a"};
    std::array<Mesh, 2> meshes;
    Json meta, comparison;
    for (int i = 0; i < 2; ++i) {
      meshes[i] = tessellate(input / (names[i] + ".step"), names[i]);
      meta[names[i]] = metadata(meshes[i]);
    }
    std::filesystem::create_directories(output);
    for (int i = 0; i < 2; ++i) write_obj(output / (names[i] + ".obj"), meshes[i]);
    write_json(output / "metadata.json", meta);
    write_json(output / "conversion.json",
               {{"occt_version", OCC_VERSION_COMPLETE},
                {"step_unit", "MM"},
                {"output_unit", "m"},
                {"linear_deflection", linear_deflection},
                {"relative_deflection", true},
                {"angular_deflection_rad", angular_deflection},
                {"face_winding", "reverse for TopAbs_REVERSED"},
                {"source_shape", "first transferred STEP shape, matching CadQuery .val()"}});
    bool matches = true;
    if (!reference.empty()) {
      for (auto& name : names) {
        double error = 0;
        for (auto bound : {"min_m", "max_m"})
          for (int axis = 0; axis < 3; ++axis)
            error = std::max(error, std::abs(meta.at(name).at(bound).at(axis).get<double>() -
                                             baseline.at(name).at(bound).at(axis).get<double>()));
        bool triangles = meta.at(name).at("triangles") == baseline.at(name).at("triangles");
        comparison[name] = {{"max_bound_error_m", error},
                            {"bounds_match_1um", error <= 1e-6},
                            {"triangles", meta[name]["triangles"]},
                            {"baseline_triangles", baseline[name]["triangles"]},
                            {"triangle_count_matches", triangles}};
        matches = matches && error <= 1e-6 && (!strict || triangles);
      }
      write_json(output / "comparison.json", comparison);
    }
    std::cout << meta.dump(2) << '\n';
    if (!reference.empty()) std::cout << comparison.dump(2) << '\n';
    return matches ? 0 : 2;
  } catch (const Standard_Failure& e) {
    std::cerr << "OCCT: " << e.GetMessageString() << '\n';
    return 1;
  } catch (const std::exception& e) {
    std::cerr << "rm_convert_cad: " << e.what() << '\n';
    return 1;
  }
}
