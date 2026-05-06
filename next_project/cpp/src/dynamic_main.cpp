#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dynamic_scenario.hpp"

namespace {

struct JsonValue {
    enum class Type { Null, Bool, Number, String, Array, Object };
    Type type = Type::Null;
    bool boolean = false;
    double number = 0.0;
    std::string text;
    std::vector<JsonValue> array;
    std::map<std::string, JsonValue> object;

    [[nodiscard]] bool has(const std::string& key) const {
        return type == Type::Object && object.find(key) != object.end();
    }
    [[nodiscard]] const JsonValue& at(const std::string& key) const {
        static JsonValue null_value;
        if (type != Type::Object) return null_value;
        auto it = object.find(key);
        return it == object.end() ? null_value : it->second;
    }
};

class JsonParser {
public:
    explicit JsonParser(std::string source) : source_(std::move(source)) {}

    JsonValue parse() {
        if (source_.size() >= 3
            && static_cast<unsigned char>(source_[0]) == 0xEF
            && static_cast<unsigned char>(source_[1]) == 0xBB
            && static_cast<unsigned char>(source_[2]) == 0xBF) {
            pos_ = 3;
        }
        JsonValue value = parse_value();
        skip_ws();
        return value;
    }

private:
    JsonValue parse_value() {
        skip_ws();
        if (pos_ >= source_.size()) return {};
        char c = source_[pos_];
        if (c == '{') return parse_object();
        if (c == '[') return parse_array();
        if (c == '"') return parse_string_value();
        if (c == 't' || c == 'f') return parse_bool();
        if (c == 'n') {
            pos_ += 4;
            return {};
        }
        if (c != '-' && c != '+' && c != '.' && !std::isdigit(static_cast<unsigned char>(c))) {
            throw std::runtime_error(std::string("Unexpected JSON token: ") + c);
        }
        return parse_number();
    }

    JsonValue parse_object() {
        JsonValue value;
        value.type = JsonValue::Type::Object;
        ++pos_;
        skip_ws();
        while (pos_ < source_.size() && source_[pos_] != '}') {
            std::string key = parse_string();
            skip_ws();
            if (pos_ < source_.size() && source_[pos_] == ':') ++pos_;
            value.object[key] = parse_value();
            skip_ws();
            if (pos_ < source_.size() && source_[pos_] == ',') {
                ++pos_;
                skip_ws();
            }
        }
        if (pos_ < source_.size() && source_[pos_] == '}') ++pos_;
        return value;
    }

    JsonValue parse_array() {
        JsonValue value;
        value.type = JsonValue::Type::Array;
        ++pos_;
        skip_ws();
        while (pos_ < source_.size() && source_[pos_] != ']') {
            value.array.push_back(parse_value());
            skip_ws();
            if (pos_ < source_.size() && source_[pos_] == ',') {
                ++pos_;
                skip_ws();
            }
        }
        if (pos_ < source_.size() && source_[pos_] == ']') ++pos_;
        return value;
    }

    JsonValue parse_string_value() {
        JsonValue value;
        value.type = JsonValue::Type::String;
        value.text = parse_string();
        return value;
    }

    JsonValue parse_bool() {
        JsonValue value;
        value.type = JsonValue::Type::Bool;
        if (source_.compare(pos_, 4, "true") == 0) {
            value.boolean = true;
            pos_ += 4;
        } else {
            value.boolean = false;
            pos_ += 5;
        }
        return value;
    }

    JsonValue parse_number() {
        JsonValue value;
        value.type = JsonValue::Type::Number;
        std::size_t start = pos_;
        if (source_[pos_] == '-') ++pos_;
        while (pos_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[pos_]))) ++pos_;
        if (pos_ < source_.size() && source_[pos_] == '.') {
            ++pos_;
            while (pos_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[pos_]))) ++pos_;
        }
        if (pos_ < source_.size() && (source_[pos_] == 'e' || source_[pos_] == 'E')) {
            ++pos_;
            if (source_[pos_] == '+' || source_[pos_] == '-') ++pos_;
            while (pos_ < source_.size() && std::isdigit(static_cast<unsigned char>(source_[pos_]))) ++pos_;
        }
        value.number = std::stod(source_.substr(start, pos_ - start));
        return value;
    }

    std::string parse_string() {
        std::string out;
        if (source_[pos_] == '"') ++pos_;
        while (pos_ < source_.size() && source_[pos_] != '"') {
            char c = source_[pos_++];
            if (c == '\\' && pos_ < source_.size()) {
                char esc = source_[pos_++];
                if (esc == 'n') out.push_back('\n');
                else if (esc == 'r') out.push_back('\r');
                else if (esc == 't') out.push_back('\t');
                else out.push_back(esc);
            } else {
                out.push_back(c);
            }
        }
        if (pos_ < source_.size() && source_[pos_] == '"') ++pos_;
        return out;
    }

    void skip_ws() {
        while (pos_ < source_.size()
               && std::isspace(static_cast<unsigned char>(source_[pos_]))) ++pos_;
    }

    std::string source_;
    std::size_t pos_ = 0;
};

std::string read_file(const std::string& path) {
    std::ifstream in(path);
    if (!in.is_open()) throw std::runtime_error("Cannot open input: " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string escape_json(const std::string& value) {
    std::string out;
    for (char c : value) {
        if (c == '"' || c == '\\') {
            out.push_back('\\');
            out.push_back(c);
        } else if (c == '\n') {
            out += "\\n";
        } else if (c == '\r') {
            out += "\\r";
        } else if (c == '\t') {
            out += "\\t";
        } else {
            out.push_back(c);
        }
    }
    return out;
}

double number_or(const JsonValue& value, double fallback) {
    return value.type == JsonValue::Type::Number ? value.number : fallback;
}

int int_or(const JsonValue& value, int fallback) {
    return value.type == JsonValue::Type::Number ? static_cast<int>(value.number) : fallback;
}

bool bool_or(const JsonValue& value, bool fallback) {
    return value.type == JsonValue::Type::Bool ? value.boolean : fallback;
}

std::string string_or(const JsonValue& value, const std::string& fallback) {
    return value.type == JsonValue::Type::String ? value.text : fallback;
}

sim::Vec3 vec3_or(const JsonValue& value, sim::Vec3 fallback = {}) {
    if (value.type != JsonValue::Type::Array || value.array.size() < 2) return fallback;
    return sim::Vec3{
        number_or(value.array[0], fallback.x),
        number_or(value.array[1], fallback.y),
        value.array.size() >= 3 ? number_or(value.array[2], fallback.z) : fallback.z,
    };
}

std::vector<sim::Vec3> waypoints_or(const JsonValue& value, std::vector<sim::Vec3> fallback) {
    if (value.type != JsonValue::Type::Array) return fallback;
    std::vector<sim::Vec3> out;
    for (const auto& item : value.array) out.push_back(vec3_or(item));
    return out.empty() ? fallback : out;
}

void dump_string_array(std::ostream& out, const std::vector<std::string>& values) {
    out << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i) out << ",";
        out << "\"" << escape_json(values[i]) << "\"";
    }
    out << "]";
}

void apply_z_bounds(sim::ObstacleConfig& cfg, const JsonValue& value) {
    if (value.type != JsonValue::Type::Array || value.array.size() < 2) return;
    cfg.planner_z_min = number_or(value.array[0], cfg.planner_z_min);
    cfg.planner_z_max = number_or(value.array[1], cfg.planner_z_max);
    cfg.planner_has_z_bounds = true;
}

bool file_exists(const std::string& path) {
    std::ifstream in(path);
    return in.good();
}

std::string dirname_of(const std::string& path) {
    const std::size_t pos = path.find_last_of("/\\");
    if (pos == std::string::npos) return ".";
    return path.substr(0, pos);
}

bool is_absolute_path(const std::string& path) {
    if (path.size() >= 3 && std::isalpha(static_cast<unsigned char>(path[0])) && path[1] == ':'
        && (path[2] == '\\' || path[2] == '/')) {
        return true;
    }
    return path.size() >= 2 && (path[0] == '\\' || path[0] == '/');
}

std::string join_path(const std::string& left, const std::string& right) {
    if (left.empty() || left == ".") return right;
    const char back = left.back();
    if (back == '\\' || back == '/') return left + right;
    return left + "\\" + right;
}

std::string resolve_map_file(const std::string& raw, const std::string& input_dir) {
    if (raw.empty()) return raw;
    if (is_absolute_path(raw) && file_exists(raw)) return raw;
    if (file_exists(raw)) return raw;
    const std::string near_input = join_path(input_dir, raw);
    if (file_exists(near_input)) return near_input;
    const std::string near_input_json = near_input.find('.') == std::string::npos ? near_input + ".json" : near_input;
    if (file_exists(near_input_json)) return near_input_json;
    const std::string maps_path = join_path("maps", raw);
    if (file_exists(maps_path)) return maps_path;
    const std::string maps_json = raw.find('.') == std::string::npos ? join_path("maps", raw + ".json") : maps_path;
    if (file_exists(maps_json)) return maps_json;
    return raw;
}

void apply_config_overrides(sim::ObstacleConfig& cfg, const JsonValue& root) {
    const JsonValue& source = root.has("base_config") ? root.at("base_config") : root;
    cfg.max_sim_time = number_or(source.at("max_sim_time"), cfg.max_sim_time);
    cfg.dt = number_or(source.at("dt"), cfg.dt);
    cfg.num_followers = int_or(source.at("num_followers"), cfg.num_followers);
    cfg.formation_spacing = number_or(source.at("formation_spacing"), cfg.formation_spacing);
    cfg.leader_max_vel = number_or(source.at("leader_max_vel"), cfg.leader_max_vel);
    cfg.leader_max_acc = number_or(source.at("leader_max_acc"), cfg.leader_max_acc);
    cfg.wp_radius = number_or(source.at("wp_radius"), cfg.wp_radius);
    cfg.wp_radius_final = number_or(source.at("wp_radius_final"), cfg.wp_radius_final);
    cfg.planner_resolution = number_or(source.at("planner_resolution"), cfg.planner_resolution);
    cfg.safety_margin = number_or(source.at("safety_margin"), cfg.safety_margin);
    cfg.detect_margin_scale = number_or(source.at("detect_margin_scale"), cfg.detect_margin_scale);
    cfg.plan_clearance_extra = number_or(source.at("plan_clearance_extra"), cfg.plan_clearance_extra);
    cfg.planner_kind = string_or(source.at("planner_kind"), cfg.planner_kind);
    cfg.planner_mode = string_or(source.at("planner_mode"), cfg.planner_mode);
    cfg.planner_sdf_aware = bool_or(source.at("planner_sdf_aware"), cfg.planner_sdf_aware);
    cfg.planner_esdf_aware = bool_or(source.at("planner_esdf_aware"), cfg.planner_esdf_aware);
    cfg.planner_use_formation_envelope = bool_or(
        source.at("planner_use_formation_envelope"), cfg.planner_use_formation_envelope);
    apply_z_bounds(cfg, source.at("planner_z_bounds"));
    cfg.planner_replan_interval = number_or(source.at("planner_replan_interval"), cfg.planner_replan_interval);
    cfg.planner_horizon = number_or(source.at("planner_horizon"), cfg.planner_horizon);
    cfg.sensor_max_range = number_or(source.at("sensor_max_range"), cfg.sensor_max_range);
    cfg.sensor_enabled = bool_or(source.at("sensor_enabled"), cfg.sensor_enabled);
    cfg.apf_paper1_profile = string_or(source.at("apf_paper1_profile"), cfg.apf_paper1_profile);
    cfg.apf_comm_range = number_or(source.at("apf_comm_range"), cfg.apf_comm_range);
    cfg.apf_centroid_alpha = number_or(source.at("apf_centroid_alpha"), cfg.apf_centroid_alpha);
    cfg.apf_centroid_beta = number_or(source.at("apf_centroid_beta"), cfg.apf_centroid_beta);
    cfg.apf_dev_override = bool_or(source.at("apf_dev_override"), cfg.apf_dev_override);
    cfg.apf_adaptive_n_decay = bool_or(source.at("apf_adaptive_n_decay"), cfg.apf_adaptive_n_decay);
    cfg.apf_formation_centroid = bool_or(source.at("apf_formation_centroid"), cfg.apf_formation_centroid);
    cfg.apf_comm_constraint = bool_or(source.at("apf_comm_constraint"), cfg.apf_comm_constraint);
    cfg.apf_rotational_escape = bool_or(source.at("apf_rotational_escape"), cfg.apf_rotational_escape);
    cfg.danger_mode_enabled = bool_or(source.at("danger_mode_enabled"), cfg.danger_mode_enabled);
    cfg.waypoints = waypoints_or(source.at("waypoints"), cfg.waypoints);
}

std::vector<std::string> parse_planners(const JsonValue& root) {
    std::vector<std::string> planners{"dstar_lite", "astar"};
    const JsonValue& value = root.at("compare_planners");
    if (value.type != JsonValue::Type::Array) return planners;
    planners.clear();
    for (const auto& item : value.array) {
        std::string planner = string_or(item, "");
        if (!planner.empty()) planners.push_back(planner);
    }
    return planners.empty() ? std::vector<std::string>{"dstar_lite", "astar"} : planners;
}

sim::ObstacleDesc parse_obstacle(const JsonValue& value) {
    sim::ObstacleDesc obs;
    obs.id = string_or(value.at("id"), "");
    obs.type = string_or(value.at("type"), "sphere");
    obs.center_or_min = vec3_or(value.at("center_or_min"));
    if (value.has("center")) obs.center_or_min = vec3_or(value.at("center"));
    if (value.has("min")) obs.center_or_min = vec3_or(value.at("min"));
    obs.size_or_max = vec3_or(value.at("size_or_max"));
    if (value.has("radius")) obs.size_or_max.x = number_or(value.at("radius"), obs.size_or_max.x);
    if (value.has("max")) obs.size_or_max = vec3_or(value.at("max"));
    if (value.has("z_range") && value.at("z_range").type == JsonValue::Type::Array
        && value.at("z_range").array.size() >= 2) {
        obs.size_or_max.y = number_or(value.at("z_range").array[0], obs.size_or_max.y);
        obs.size_or_max.z = number_or(value.at("z_range").array[1], obs.size_or_max.z);
    }
    return obs;
}

std::vector<sim::DynamicEvent> parse_events(const JsonValue& root) {
    const JsonValue& events_value = root.has("events") ? root.at("events") : root.at("dynamic_events");
    std::vector<sim::DynamicEvent> events;
    if (events_value.type != JsonValue::Type::Array) return events;
    for (const auto& item : events_value.array) {
        sim::DynamicEvent event;
        event.t = number_or(item.at("t"), 0.0);
        event.action = string_or(item.at("action"), "add");
        event.target_id = string_or(item.at("target_id"), string_or(item.at("obstacle_id"), ""));
        if (item.has("obstacle")) event.obstacle = parse_obstacle(item.at("obstacle"));
        events.push_back(event);
    }
    return events;
}

sim::DynamicSimInput parse_input(const JsonValue& root, const std::string& input_dir) {
    std::string preset = string_or(root.at("preset"), "warehouse");
    if (root.has("base_config")) preset = string_or(root.at("base_config").at("preset"), preset);
    sim::DynamicSimInput input;
    input.base_config = sim::get_config(preset);
    apply_config_overrides(input.base_config, root);
    input.map_file = string_or(root.at("map_file"), input.base_config.map_file);
    if (root.has("base_config")) {
        input.map_file = string_or(root.at("base_config").at("map_file"), input.map_file);
    }
    input.map_file = resolve_map_file(input.map_file, input_dir);
    input.base_config.map_file = input.map_file;
    input.events = parse_events(root);
    input.compare_planners = parse_planners(root);
    input.repeat_count = int_or(root.at("repeat_count"), 1);
    if (root.has("base_config")) {
        input.repeat_count = int_or(root.at("base_config").at("repeat_count"), input.repeat_count);
    }
    input.repeat_count = std::max(1, input.repeat_count);
    return input;
}

void dump_vec3(std::ostream& out, const sim::Vec3& value) {
    out << "[" << value.x << "," << value.y << "," << value.z << "]";
}

void dump_string(std::ostream& out, const std::string& value) {
    out << "\"" << escape_json(value) << "\"";
}

void dump_obstacle(std::ostream& out, const sim::ObstacleDesc& obs) {
    out << "{\"id\":";
    dump_string(out, obs.id);
    out << ",\"type\":";
    dump_string(out, obs.type);
    out << ",\"center_or_min\":";
    dump_vec3(out, obs.center_or_min);
    out << ",\"size_or_max\":";
    dump_vec3(out, obs.size_or_max);
    out << "}";
}

void dump_event(std::ostream& out, const sim::DynamicEvent& event) {
    out << "{\"t\":" << event.t << ",\"action\":";
    dump_string(out, event.action);
    if (event.action == "remove") {
        out << ",\"target_id\":";
        dump_string(out, event.target_id);
    } else if (event.action == "move") {
        out << ",\"target_id\":";
        dump_string(out, event.target_id);
        out << ",\"obstacle\":";
        dump_obstacle(out, event.obstacle);
    } else {
        out << ",\"obstacle\":";
        dump_obstacle(out, event.obstacle);
    }
    out << "}";
}

void dump_planner_result(std::ostream& out, const sim::PlannerResult& result) {
    out << "{\"path\":[";
    for (std::size_t i = 0; i < result.path.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, result.path[i]);
    }
    out << "],\"phase\":";
    dump_string(out, result.phase);
    out << ",\"replan_triggered\":" << (result.replan_triggered ? "true" : "false")
        << ",\"compute_time_ms\":" << result.compute_time_ms
        << ",\"success\":" << (result.success ? "true" : "false") << "}";
}

void dump_summary(std::ostream& out, const sim::SummaryMetrics& summary) {
    out << "{\"total_replans\":" << summary.total_replans
        << ",\"avg_compute_ms\":" << summary.avg_compute_ms
        << ",\"max_compute_ms\":" << summary.max_compute_ms
        << ",\"p95_compute_ms\":" << summary.p95_compute_ms
        << ",\"collisions\":" << summary.collisions
        << ",\"path_length_m\":" << summary.path_length_m
        << ",\"final_dist_to_goal\":" << summary.final_dist_to_goal
        << ",\"phase_distribution\":{";
    bool first = true;
    for (const auto& kv : summary.phase_distribution) {
        if (!first) out << ",";
        first = false;
        dump_string(out, kv.first);
        out << ":" << kv.second;
    }
    out << "}}";
}

void dump_replan_event(std::ostream& out, const sim::ReplayReplanEvent& event) {
    out << "{\"t\":" << event.t << ",\"planner\":";
    dump_string(out, event.planner);
    out << ",\"phase\":";
    dump_string(out, event.phase);
    out << ",\"success\":" << (event.success ? "true" : "false") << "}";
}

void dump_collision_event(std::ostream& out, const sim::ReplayCollisionEvent& event) {
    out << "{\"t\":" << event.t << ",\"actor\":";
    dump_string(out, event.actor);
    out << ",\"pos\":";
    dump_vec3(out, event.pos);
    out << "}";
}

void dump_fault_event(std::ostream& out, const sim::ReplayFaultEvent& event) {
    out << "{\"t\":" << event.t << ",\"type\":";
    dump_string(out, event.type);
    out << ",\"detail\":";
    dump_string(out, event.detail);
    out << "}";
}

void dump_replay(std::ostream& out, const sim::ReplayOutput& replay) {
    out << "{\"metadata\":{\"map_file\":";
    dump_string(out, replay.metadata.map_file);
    out << ",\"scenario\":";
    dump_string(out, replay.metadata.scenario);
    out << ",\"execution_scope\":";
    dump_string(out, replay.metadata.execution_scope);
    out << ",\"follower_pose_mode\":";
    dump_string(out, replay.metadata.follower_pose_mode);
    out << ",\"summary_mode\":";
    dump_string(out, replay.metadata.summary_mode);
    out << ",\"apf_profile_scope\":";
    dump_string(out, replay.metadata.apf_profile_scope);
    out << ",\"apf_runtime_fields\":";
    dump_string_array(out, replay.metadata.apf_runtime_fields);
    out << ",\"apf_python_only_fields\":";
    dump_string_array(out, replay.metadata.apf_python_only_fields);
    out << ",\"apf_cpp_pending_fields\":";
    dump_string_array(out, replay.metadata.apf_cpp_pending_fields);
    out << ",\"step_count\":" << replay.metadata.step_count
        << ",\"dt\":" << replay.metadata.dt
        << ",\"total_time\":" << replay.metadata.total_time
        << ",\"repeat_index\":" << replay.metadata.repeat_index
        << ",\"seed\":" << replay.metadata.seed << "},";
    out << "\"static_map\":{\"bounds\":[";
    dump_vec3(out, replay.bounds[0]);
    out << ",";
    dump_vec3(out, replay.bounds[1]);
    out << "],\"obstacles\":[";
    for (std::size_t i = 0; i < replay.static_obstacles.size(); ++i) {
        if (i) out << ",";
        dump_obstacle(out, replay.static_obstacles[i]);
    }
    out << "]},\"dynamic_events\":[";
    for (std::size_t i = 0; i < replay.dynamic_events.size(); ++i) {
        if (i) out << ",";
        dump_event(out, replay.dynamic_events[i]);
    }
    out << "],\"waypoints\":[";
    for (std::size_t i = 0; i < replay.waypoints.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, replay.waypoints[i]);
    }
    out << "],\"task_waypoints\":[";
    for (std::size_t i = 0; i < replay.task_waypoints.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, replay.task_waypoints[i]);
    }
    out << "],\"replanned_waypoints\":[";
    for (std::size_t i = 0; i < replay.replanned_waypoints.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, replay.replanned_waypoints[i]);
    }
    out << "],\"executed_path\":[";
    for (std::size_t i = 0; i < replay.executed_path.size(); ++i) {
        if (i) out << ",";
        dump_vec3(out, replay.executed_path[i]);
    }
    out << "],\"replan_events\":[";
    for (std::size_t i = 0; i < replay.replan_events.size(); ++i) {
        if (i) out << ",";
        dump_replan_event(out, replay.replan_events[i]);
    }
    out << "],\"sensor_logs\":[";
    for (std::size_t i = 0; i < replay.sensor_logs.size(); ++i) {
        if (i) out << ",";
        out << "[";
        for (std::size_t j = 0; j < replay.sensor_logs[i].size(); ++j) {
            if (j) out << ",";
            out << replay.sensor_logs[i][j];
        }
        out << "]";
    }
    out << "],\"collision_log\":[";
    for (std::size_t i = 0; i < replay.collision_log.size(); ++i) {
        if (i) out << ",";
        dump_collision_event(out, replay.collision_log[i]);
    }
    out << "],\"fault_log\":[";
    for (std::size_t i = 0; i < replay.fault_log.size(); ++i) {
        if (i) out << ",";
        dump_fault_event(out, replay.fault_log[i]);
    }
    out << "],\"frames\":[";
    for (std::size_t i = 0; i < replay.frames.size(); ++i) {
        if (i) out << ",";
        const auto& frame = replay.frames[i];
        out << "{\"t\":" << frame.t << ",\"leader_pose\":";
        dump_vec3(out, frame.leader_pose);
        out << ",\"followers\":[";
        for (std::size_t j = 0; j < frame.follower_poses.size(); ++j) {
            if (j) out << ",";
            dump_vec3(out, frame.follower_poses[j]);
        }
        out << "],\"obstacles\":[";
        for (std::size_t j = 0; j < frame.obstacles.size(); ++j) {
            if (j) out << ",";
            dump_obstacle(out, frame.obstacles[j]);
        }
        out << "],\"sensor_readings\":[";
        for (std::size_t j = 0; j < frame.sensor_readings.size(); ++j) {
            if (j) out << ",";
            out << frame.sensor_readings[j];
        }
        out << "]";
        for (const auto& kv : frame.planner_results) {
            out << ",";
            dump_string(out, kv.first);
            out << ":";
            dump_planner_result(out, kv.second);
        }
        out << ",\"collision\":" << (frame.collision ? "true" : "false")
            << ",\"clearance_blocked\":" << (frame.clearance_blocked ? "true" : "false")
            << ",\"obstacle_changed\":" << (frame.obstacle_changed ? "true" : "false") << "}";
    }
    out << "],\"summary\":{";
    bool first = true;
    for (const auto& kv : replay.summaries) {
        if (!first) out << ",";
        first = false;
        dump_string(out, kv.first);
        out << ":";
        dump_summary(out, kv.second);
    }
    out << "}}";
}

void dump_outputs(std::ostream& out, const std::vector<sim::ReplayOutput>& outputs) {
    out << std::setprecision(8) << "[";
    for (std::size_t i = 0; i < outputs.size(); ++i) {
        if (i) out << ",";
        dump_replay(out, outputs[i]);
    }
    out << "]";
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        if (argc < 2) {
            std::cerr << "Usage: sim_dynamic_replay <input.json> [-o <output.json>]\n";
            return 2;
        }
        std::string input_path = argv[1];
        std::string output_path = join_path(dirname_of(input_path), "replay_output.json");
        if (argc >= 4 && std::string(argv[2]) == "-o") output_path = argv[3];

        JsonParser parser(read_file(input_path));
        JsonValue root = parser.parse();
        sim::DynamicSimInput input = parse_input(root, dirname_of(input_path));

        sim::DynamicScenarioRunner runner(input, 42);
        std::vector<sim::ReplayOutput> outputs = runner.run_batch();

        std::ofstream out(output_path);
        if (!out.is_open()) throw std::runtime_error("Cannot open output: " + output_path);
        dump_outputs(out, outputs);
        std::cout << "Done. " << outputs.size() << " runs -> " << output_path << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << ex.what() << "\n";
        return 1;
    }
}
