#include "map_loader.hpp"

#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace sim {

namespace {

// Minimal JSON parser for our specific map format
void skip_ws(const std::string& s, std::size_t& i) {
    while (i < s.size() && (s[i] == ' ' || s[i] == '\n' || s[i] == '\r' || s[i] == '\t')) ++i;
}

double parse_number(const std::string& s, std::size_t& i) {
    skip_ws(s, i);
    std::size_t start = i;
    bool has_dot = false;
    if (s[i] == '-') ++i;
    while (i < s.size() && (std::isdigit(s[i]) || (s[i] == '.' && !has_dot))) {
        if (s[i] == '.') has_dot = true;
        ++i;
    }
    return std::stod(s.substr(start, i - start));
}

Vec3 parse_vec3(const std::string& s, std::size_t& i) {
    skip_ws(s, i);
    if (s[i] == '[') ++i;
    double x = parse_number(s, i);
    skip_ws(s, i); if (s[i] == ',') ++i;
    double y = parse_number(s, i);
    double z = 0.0;
    skip_ws(s, i);
    if (s[i] == ',') {
        ++i;
        z = parse_number(s, i);
    }
    skip_ws(s, i); if (s[i] == ']') ++i;
    return Vec3{x, y, z};
}

std::string parse_string(const std::string& s, std::size_t& i) {
    skip_ws(s, i);
    if (s[i] == '"') ++i;
    std::size_t start = i;
    while (i < s.size() && s[i] != '"') ++i;
    std::string val = s.substr(start, i - start);
    if (s[i] == '"') ++i;
    return val;
}

}  // namespace

std::pair<ObstacleField, std::array<Vec3, 2>> load_from_json(const std::string& filepath) {
    std::ifstream f(filepath);
    if (!f.is_open()) throw std::runtime_error("Cannot open map file: " + filepath);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string s = ss.str();

    ObstacleField field;
    std::array<Vec3, 2> bounds{};

    std::size_t i = 0;
    while (i < s.size()) {
        skip_ws(s, i);
        if (i >= s.size()) break;

        if (s[i] == '{' || s[i] == '}' || s[i] == '[' || s[i] == ']' || s[i] == ',' || s[i] == ':') {
            if (s[i] == '[' && s.find("\"bounds\"", i > 10 ? i - 10 : 0) == std::string::npos) {
                // obstacle array start
            }
            ++i;
            continue;
        }

        // Check for keys
        if (s[i] == '"') {
            std::string key = parse_string(s, i);
            skip_ws(s, i);
            if (s[i] == ':') ++i;

            if (key == "bounds") {
                skip_ws(s, i); if (s[i] == '[') ++i;
                bounds[0] = parse_vec3(s, i);
                skip_ws(s, i); if (s[i] == ',') ++i;
                bounds[1] = parse_vec3(s, i);
                skip_ws(s, i); if (s[i] == ']') ++i;
            } else if (key == "type") {
                std::string type = parse_string(s, i);
                skip_ws(s, i); if (s[i] == ',') ++i;

                if (type == "aabb") {
                    // skip "min": [...]
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    Vec3 mn = parse_vec3(s, i);
                    skip_ws(s, i); if (s[i] == ',') ++i;
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    Vec3 mx = parse_vec3(s, i);
                    field.add_aabb(mn, mx);
                } else if (type == "sphere") {
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    Vec3 c = parse_vec3(s, i);
                    skip_ws(s, i); if (s[i] == ',') ++i;
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    double r = parse_number(s, i);
                    field.add_sphere(c, r);
                } else if (type == "cylinder") {
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    Vec3 c = parse_vec3(s, i);
                    skip_ws(s, i); if (s[i] == ',') ++i;
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    double r = parse_number(s, i);
                    skip_ws(s, i); if (s[i] == ',') ++i;
                    skip_ws(s, i); parse_string(s, i); skip_ws(s, i); if (s[i] == ':') ++i;
                    skip_ws(s, i); if (s[i] == '[') ++i;
                    double zmin = parse_number(s, i);
                    skip_ws(s, i); if (s[i] == ',') ++i;
                    double zmax = parse_number(s, i);
                    skip_ws(s, i); if (s[i] == ']') ++i;
                    field.add_cylinder(c, r, zmin, zmax);
                }
            }
        } else {
            ++i;
        }
    }

    field.regenerate_ids("obs_");
    return {field, bounds};
}

}  // namespace sim
