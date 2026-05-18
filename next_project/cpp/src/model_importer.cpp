#include "model_importer.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sim {

// ============================================================
// 辅助：字符串分割 & 空白裁剪
// ============================================================

namespace {

std::vector<std::string> split(const std::string& s, char delim = ' ') {
    std::vector<std::string> out;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        if (!item.empty()) out.push_back(item);
    }
    return out;
}

std::string trim(const std::string& s) {
    std::size_t start = 0;
    std::size_t end = s.size();
    while (start < end && (s[start] == ' ' || s[start] == '\t' || s[start] == '\r')) ++start;
    while (end > start && (s[end - 1] == ' ' || s[end - 1] == '\t' || s[end - 1] == '\r')) --end;
    return s.substr(start, end - start);
}

std::array<int, 3> vec_cell(double x, double y, double z, double voxel) {
    return {
        static_cast<int>(std::floor(x / voxel)),
        static_cast<int>(std::floor(y / voxel)),
        static_cast<int>(std::floor(z / voxel)),
    };
}

std::string lower(const std::string& s) {
    std::string out = s;
    for (auto& c : out) c = static_cast<char>(std::tolower(c));
    return out;
}

}  // namespace

// ============================================================
// OBJ 解析
// ============================================================

std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_obj(const std::string& text) {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;

    std::stringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) {
        line = trim(line);
        if (line.empty()) continue;

        auto parts = split(line);
        if (parts.empty()) continue;

        if (parts[0] == "v" && parts.size() >= 4) {
            vertices.push_back({std::stod(parts[1]), std::stod(parts[2]), std::stod(parts[3])});
        } else if (parts[0] == "f" && parts.size() >= 4) {
            std::vector<int> refs;
            for (std::size_t i = 1; i < parts.size(); ++i) {
                auto slash_pos = parts[i].find('/');
                std::string idx_str = (slash_pos != std::string::npos) ? parts[i].substr(0, slash_pos) : parts[i];
                if (idx_str.empty()) continue;
                int idx = std::stoi(idx_str);
                refs.push_back(idx > 0 ? idx - 1 : static_cast<int>(vertices.size()) + idx);
            }
            for (std::size_t i = 1; i + 1 < refs.size(); ++i) {
                triangles.push_back({refs[0], refs[i], refs[i + 1]});
            }
        }
    }
    return {vertices, triangles};
}

// ============================================================
// STL 解析（二进制 + ASCII 回退）
// ============================================================

std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_stl(const std::vector<uint8_t>& data) {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;

    if (data.size() >= 84) {
        uint32_t tri_count = 0;
        std::memcpy(&tri_count, data.data() + 80, sizeof(tri_count));
        if (84 + tri_count * 50 == data.size()) {
            for (uint32_t t = 0; t < tri_count; ++t) {
                std::size_t off = 84 + t * 50;
                int base = static_cast<int>(vertices.size());
                for (int j = 0; j < 3; ++j) {
                    float fx, fy, fz;
                    std::memcpy(&fx, data.data() + off + 12 + j * 12, 4);
                    std::memcpy(&fy, data.data() + off + 12 + j * 12 + 4, 4);
                    std::memcpy(&fz, data.data() + off + 12 + j * 12 + 8, 4);
                    vertices.push_back({static_cast<double>(fx), static_cast<double>(fy), static_cast<double>(fz)});
                }
                triangles.push_back({base, base + 1, base + 2});
            }
            return {vertices, triangles};
        }
    }

    // ASCII 回退
    std::string text(data.begin(), data.end());
    std::stringstream ss(text);
    std::string line;
    std::vector<int> current;
    while (std::getline(ss, line)) {
        auto parts = split(trim(line));
        if (parts.size() == 4 && lower(parts[0]) == "vertex") {
            current.push_back(static_cast<int>(vertices.size()));
            vertices.push_back({std::stod(parts[1]), std::stod(parts[2]), std::stod(parts[3])});
            if (current.size() == 3) {
                triangles.push_back({current[0], current[1], current[2]});
                current.clear();
            }
        }
    }
    return {vertices, triangles};
}

// ============================================================
// PLY 解析（ASCII + binary_little_endian）
// ============================================================

std::pair<std::vector<Vertex>, std::vector<Triangle>> parse_ply(const std::vector<uint8_t>& data) {
    std::string marker = "end_header";
    auto it = std::search(data.begin(), data.end(), marker.begin(), marker.end());
    if (it == data.end()) throw std::runtime_error("PLY header missing end_header");

    auto header_stop = std::find(it + marker.size(), data.end(), static_cast<uint8_t>('\n'));
    std::size_t header_end_idx = (header_stop != data.end())
        ? static_cast<std::size_t>(header_stop - data.begin())
        : static_cast<std::size_t>(it + marker.size() - data.begin());

    std::string header(data.begin(), data.begin() + static_cast<long>(header_end_idx));
    std::stringstream hss(header);
    std::string hline;
    std::getline(hss, hline);
    if (trim(hline) != "ply") throw std::runtime_error("Invalid PLY file");

    int vertex_count = 0;
    int face_count = 0;
    std::string format = "ascii";
    std::vector<std::pair<std::string, std::string>> vertex_props;
    bool in_vertex = false;
    bool in_face = false;
    bool has_face_list = false;
    std::string face_count_type = "uchar";
    std::string face_index_type = "int";

    while (std::getline(hss, hline)) {
        auto parts = split(trim(hline));
        if (parts.size() >= 3 && parts[0] == "format") {
            format = parts[1];
        }
        if (parts.size() == 3 && parts[0] == "element" && parts[1] == "vertex") {
            vertex_count = std::stoi(parts[2]);
            in_vertex = true;
            in_face = false;
        } else if (parts.size() == 3 && parts[0] == "element" && parts[1] == "face") {
            face_count = std::stoi(parts[2]);
            in_vertex = false;
            in_face = true;
        } else if (parts.size() == 3 && parts[0] == "property" && in_vertex) {
            vertex_props.emplace_back(parts[2], parts[1]);
        } else if (parts.size() == 5 && parts[0] == "property" && parts[1] == "list" && in_face) {
            face_count_type = parts[2];
            face_index_type = parts[3];
            has_face_list = true;
        } else if (!parts.empty() && parts[0] == "element") {
            in_vertex = false;
            in_face = false;
        }
    }

    std::size_t body_start = header_end_idx + 1;

    // ---- ASCII PLY ----
    if (format == "ascii") {
        std::string body(data.begin() + static_cast<long>(body_start), data.end());
        std::stringstream bss(body);
        std::string bline;
        std::vector<Vertex> vertices;
        for (int i = 0; i < vertex_count; ++i) {
            if (!std::getline(bss, bline)) break;
            auto parts = split(trim(bline));
            if (parts.size() >= 3) {
                vertices.push_back({std::stod(parts[0]), std::stod(parts[1]), std::stod(parts[2])});
            }
        }
        std::vector<Triangle> triangles;
        for (int i = 0; i < face_count; ++i) {
            if (!std::getline(bss, bline)) break;
            auto parts = split(trim(bline));
            if (parts.empty()) continue;
            int count = std::stoi(parts[0]);
            std::vector<int> refs;
            for (int j = 0; j < count && j + 1 < static_cast<int>(parts.size()); ++j) {
                refs.push_back(std::stoi(parts[j + 1]));
            }
            for (std::size_t j = 1; j + 1 < refs.size(); ++j) {
                triangles.push_back({refs[0], refs[j], refs[j + 1]});
            }
        }
        return {vertices, triangles};
    }

    // ---- Binary PLY ----
    if (format != "binary_little_endian") {
        throw std::runtime_error("Unsupported PLY format: " + format);
    }

    if (vertex_props.empty()) {
        vertex_props = {{"x", "float"}, {"y", "float"}, {"z", "float"}};
    }

    // 阶段1: 构建解析计划（header 结束后只执行一次，仅 62 次字符串比较）
    {
    {
        enum class PlyField { X, Y, Z, Unknown };
        auto ply_type_size = [](const std::string& t) -> int {
            if (t == "char" || t == "int8" || t == "uchar" || t == "uint8") return 1;
            if (t == "short" || t == "int16" || t == "ushort" || t == "uint16") return 2;
            if (t == "int" || t == "int32" || t == "uint" || t == "uint32" || t == "float" || t == "float32") return 4;
            if (t == "double" || t == "float64") return 8;
            return 0;
        };
        auto ply_is_float = [](const std::string& t) -> bool {
            return t == "float" || t == "float32" || t == "double" || t == "float64";
        };
        auto ply_is_signed = [](const std::string& t) -> bool {
            return t == "char" || t == "int8" || t == "short" || t == "int16" ||
                   t == "int" || t == "int32";
        };

        struct FieldPlan {
            PlyField field;
            int byte_size;
            bool is_float;
            bool is_signed;
        };
        std::vector<FieldPlan> plan;
        int vertex_stride = 0;
        int x_off = -1, y_off = -1, z_off = -1;
        int x_sz = 0, y_sz = 0, z_sz = 0;
        bool x_fl = false, y_fl = false, z_fl = false;
        bool x_signed = false, y_signed = false, z_signed = false;

        for (const auto& [name, typ] : vertex_props) {
            int sz = ply_type_size(typ);
            if (sz == 0) throw std::runtime_error("Unsupported PLY vertex property type: " + typ);
            bool fl = ply_is_float(typ);
            bool sg = ply_is_signed(typ);

            PlyField f = PlyField::Unknown;
            if (name == "x")      { f = PlyField::X; x_off = vertex_stride; x_sz = sz; x_fl = fl; x_signed = sg; }
            else if (name == "y") { f = PlyField::Y; y_off = vertex_stride; y_sz = sz; y_fl = fl; y_signed = sg; }
            else if (name == "z") { f = PlyField::Z; z_off = vertex_stride; z_sz = sz; z_fl = fl; z_signed = sg; }

            plan.push_back({f, sz, fl, sg});
            vertex_stride += sz;
        }

        // 阶段2: 一次性读取全部顶点数据
        std::size_t vertex_data_size = static_cast<std::size_t>(vertex_count) * vertex_stride;
        std::size_t vertex_data_size = static_cast<std::size_t>(vertex_count) * vertex_stride;
        if (body_start + vertex_data_size > data.size())
            throw std::runtime_error("Unexpected end of binary PLY vertex data");
        const uint8_t* vbuf = data.data() + body_start;

        // 阶段3: 按 offset 直接读取 x/y/z（无字符串比较、无属性循环）
        auto read_float = [&](const uint8_t* base, int off, int sz, bool fl, bool sg) -> double {
            const uint8_t* p = base + off;
            if (fl && sz == 4) {
                float f; std::memcpy(&f, p, 4); return static_cast<double>(f);
            }
            if (fl && sz == 8) {
                double d; std::memcpy(&d, p, 8); return d;
            }
            if (sz == 1 && sg) { int8_t s; std::memcpy(&s, p, 1); return static_cast<double>(s); }
            if (sz == 1) { uint8_t u; std::memcpy(&u, p, 1); return static_cast<double>(u); }
            if (sz == 2 && sg) { int16_t s; std::memcpy(&s, p, 2); return static_cast<double>(s); }
            if (sz == 2) { uint16_t u; std::memcpy(&u, p, 2); return static_cast<double>(u); }
            if (sz == 4 && sg) { int32_t i32; std::memcpy(&i32, p, 4); return static_cast<double>(i32); }
            if (sz == 4) { uint32_t u32; std::memcpy(&u32, p, 4); return static_cast<double>(u32); }
            return 0.0;
        };

        std::vector<Vertex> vertices;
        vertices.reserve(vertex_count);
        for (int i = 0; i < vertex_count; ++i) {
            const uint8_t* base = vbuf + static_cast<std::size_t>(i) * vertex_stride;
            double vx = (x_off >= 0) ? read_float(base, x_off, x_sz, x_fl, x_signed) : 0.0;
            double vy = (y_off >= 0) ? read_float(base, y_off, y_sz, y_fl, y_signed) : 0.0;
            double vz = (z_off >= 0) ? read_float(base, z_off, z_sz, z_fl, z_signed) : 0.0;
            vertices.push_back({vx, vy, vz});
        }

        // 阶段4: 解析面数据（维持原逻辑）
        std::size_t offset = body_start + vertex_data_size;
        std::size_t offset = body_start + vertex_data_size;
        int face_count_size = ply_type_size(face_count_type);
        int face_index_size = ply_type_size(face_index_type);
        if (face_count > 0 && (!has_face_list || face_count_size == 0 || face_index_size == 0)) {
            throw std::runtime_error("Unsupported binary PLY face list property");
        }
        auto read_uint = [&](std::size_t& off, const std::string& typ) -> uint32_t {
            int sz = ply_type_size(typ);
            if (off + static_cast<std::size_t>(sz) > data.size()) {
                throw std::runtime_error("Unexpected end of binary PLY face data");
            }
            const uint8_t* p = data.data() + off;
            off += static_cast<std::size_t>(sz);
            if (sz == 1) { uint8_t v; std::memcpy(&v, p, 1); return v; }
            if (sz == 2 && ply_is_signed(typ)) { int16_t v; std::memcpy(&v, p, 2); return static_cast<uint32_t>(v); }
            if (sz == 2) { uint16_t v; std::memcpy(&v, p, 2); return v; }
            if (sz == 4 && ply_is_signed(typ)) { int32_t v; std::memcpy(&v, p, 4); return static_cast<uint32_t>(v); }
            uint32_t v; std::memcpy(&v, p, 4); return v;
        };
        std::vector<Triangle> triangles;
        for (int i = 0; i < face_count; ++i) {
            if (offset >= data.size()) break;
            uint32_t count = read_uint(offset, face_count_type);
            std::vector<int> refs;
            for (uint32_t j = 0; j < count; ++j) {
                refs.push_back(static_cast<int>(read_uint(offset, face_index_type)));
            }
            for (std::size_t j = 1; j + 1 < refs.size(); ++j) {
                triangles.push_back({refs[0], refs[j], refs[j + 1]});
            }
        }
        return {vertices, triangles};
    }
}

// ============================================================
// 三角面采样
// ============================================================

std::vector<Vertex> sample_triangle(const Vertex& a, const Vertex& b, const Vertex& c, double voxel) {
    double edge = std::max({
        norm(b - a), norm(c - a), norm(c - b),
    });
    int steps = std::max(1, std::min(48, static_cast<int>(edge / std::max(voxel * 0.5, 1e-9)) + 1));
    std::vector<Vertex> out;
    for (int i = 0; i <= steps; ++i) {
        for (int j = 0; j <= steps - i; ++j) {
            double u = static_cast<double>(i) / steps;
            double v = static_cast<double>(j) / steps;
            double w = 1.0 - u - v;
            out.push_back({
                a.x * w + b.x * u + c.x * v,
                a.y * w + b.y * u + c.y * v,
                a.z * w + b.z * u + c.z * v,
            });
        }
    }
    return out;
}

// ============================================================
// 模型 → 障碍物场
// ============================================================

namespace {

std::pair<ObstacleField, std::array<Vec3, 2>> model_to_field(
    const std::vector<Vertex>& vertices,
    const std::vector<Triangle>& triangles,
    double voxel_size,
    double scale,
    double padding,
    int max_obstacles
) {
    if (vertices.empty()) throw std::runtime_error("Model contains no vertices");
    if (voxel_size <= 0 || scale <= 0) throw std::runtime_error("voxel_size and scale must be positive");

    // 缩放
    std::vector<Vertex> scaled;
    scaled.reserve(vertices.size());
    double min_x = 1e18, min_y = 1e18, min_z = 1e18;
    double max_x = -1e18, max_y = -1e18, max_z = -1e18;
    for (const auto& v : vertices) {
        double sx = v.x * scale, sy = v.y * scale, sz = v.z * scale;
        scaled.push_back({sx, sy, sz});
        min_x = std::min(min_x, sx); min_y = std::min(min_y, sy); min_z = std::min(min_z, sz);
        max_x = std::max(max_x, sx); max_y = std::max(max_y, sy); max_z = std::max(max_z, sz);
    }

    // 平移至正象限
    std::vector<Vertex> shifted;
    shifted.reserve(scaled.size());
    std::set<std::array<int, 3>> occupied;
    for (const auto& v : scaled) {
        shifted.push_back({v.x - min_x + padding, v.y - min_y + padding, v.z - min_z});
    }

    // 体素化顶点
    std::set<std::array<int, 3>> occupied;
    for (const auto& v : shifted) {
        occupied.insert(vec_cell(v.x, v.y, v.z, voxel_size));
    }

    // 体素化三角面
    for (const auto& tri : triangles) {
        if (tri[0] >= static_cast<int>(shifted.size()) ||
            tri[1] >= static_cast<int>(shifted.size()) ||
            tri[2] >= static_cast<int>(shifted.size())) continue;
        for (const auto& pt : sample_triangle(shifted[tri[0]], shifted[tri[1]], shifted[tri[2]], voxel_size)) {
            occupied.insert(vec_cell(pt.x, pt.y, pt.z, voxel_size));
        }
    }

    // run-length 编码 (按 y,z 分组)
    std::map<std::pair<int, int>, std::set<int>> by_yz;
    for (const auto& [ix, iy, iz] : occupied) {
        by_yz[{iy, iz}].insert(ix);
    }

    struct Run { int x0, x1, iy, iz; };
    std::vector<Run> runs;
    for (auto& [yz, xs] : by_yz) {
        auto it = xs.begin();
        int start = *it, prev = *it;
        for (++it; it != xs.end(); ++it) {
            if (*it == prev + 1) {
                prev = *it;
            } else {
                runs.push_back({start, prev, yz.first, yz.second});
                start = prev = *it;
            }
        }
        runs.push_back({start, prev, yz.first, yz.second});
    }

    if (static_cast<int>(runs.size()) > max_obstacles) {
        throw std::runtime_error(
            "Converted model produced " + std::to_string(runs.size()) +
            " obstacles. Increase voxel_size or max_obstacles."
        );
    }

    ObstacleField field;
    for (const auto& r : runs) {
        field.add_aabb(
            Vec3{r.x0 * voxel_size, r.iy * voxel_size, r.iz * voxel_size},
            Vec3{(r.x1 + 1) * voxel_size, (r.iy + 1) * voxel_size, (r.iz + 1) * voxel_size}
        );
    }

    double bound_x = max_x - min_x + padding * 2;
    double bound_y = max_y - min_y + padding * 2;
    double bound_z = max_z - min_z + std::max(voxel_size, padding * 0.25);
    std::array<Vec3, 2> bounds{Vec3{0, 0, 0}, Vec3{bound_x, bound_y, bound_z}};

    return {field, bounds};
}

}  // namespace

// ============================================================
// 公共接口
// ============================================================

std::pair<ObstacleField, std::array<Vec3, 2>> import_model(
    const std::string& filepath,
    double voxel_size,
    double scale,
    double padding,
    int max_obstacles
) {
    std::ifstream f(filepath, std::ios::binary);
    if (!f.is_open()) throw std::runtime_error("Cannot open model file: " + filepath);
    std::vector<uint8_t> data((std::istreambuf_iterator<char>(f)),
                               std::istreambuf_iterator<char>());
    f.close();

    // 根据扩展名分发
    auto ext_pos = filepath.rfind('.');
    auto ext_pos = filepath.rfind('.');
    std::string ext = (ext_pos != std::string::npos) ? filepath.substr(ext_pos) : "";
    for (auto& c : ext) c = static_cast<char>(std::tolower(c));

    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;

    if (ext == ".obj") {
        std::string text(data.begin(), data.end());
        std::tie(vertices, triangles) = parse_obj(text);
    } else if (ext == ".stl") {
        std::tie(vertices, triangles) = parse_stl(data);
    } else if (ext == ".ply") {
        std::tie(vertices, triangles) = parse_ply(data);
    } else {
        throw std::runtime_error("Unsupported model format: " + ext + ". Supported: .obj, .stl, .ply");
    }

    return model_to_field(vertices, triangles, voxel_size, scale, padding, max_obstacles);
}

}  // namespace sim
