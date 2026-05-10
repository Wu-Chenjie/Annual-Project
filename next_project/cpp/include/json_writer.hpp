#pragma once
/// json_writer.hpp — 轻量 schema-compliant JSON 序列化工具。
///
/// 用法：
///   JsonWriter w(out);
///   w.begin_object();
///   w.key("schema_version").value("1.0.0");
///   w.key("metrics").begin_object();
///   w.key("mean").array_double(metrics.mean);
///   ...
///   w.end_object();  // metrics
///   w.end_object();  // root
///
/// 设计原则：
/// - 零外部依赖，std::ostream + std::chrono 构建；
/// - 与 schemas/*.schema.json 字段名严格对齐；
/// - begin_*/end_* 配对。

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "math_utils.hpp"

namespace sim {

/// 把 seconds-since-epoch 转为 ISO-8601 UTC 时间戳字符串（秒级）。
inline std::string iso8601_utc(const std::chrono::system_clock::time_point& tp) {
    std::time_t t = std::chrono::system_clock::to_time_t(tp);
    std::tm* gm = std::gmtime(&t);
    if (gm == nullptr) {
        return "1970-01-01T00:00:00Z";
    }
    std::ostringstream oss;
    oss << std::put_time(gm, "%Y-%m-%dT%H:%M:%SZ");
    return oss.str();
}

inline std::string engine_version() {
#ifdef UAV_ENGINE_VERSION
    return UAV_ENGINE_VERSION;
#else
    const char* env = std::getenv("UAV_ENGINE_VERSION");
    return env != nullptr ? env : "0.1.0";
#endif
}

/// 轻量 JSON 写入器，保证缩进和逗号正确。
class JsonWriter {
public:
    explicit JsonWriter(std::ostream& os) : os_(os) {}

    // ---- value types ----

    JsonWriter& value(double v) {
        value_prefix();
        if (std::isnan(v)) {
            os_ << "null";
        } else {
            os_ << std::fixed << std::setprecision(8) << v;
        }
        finish_value();
        return *this;
    }

    JsonWriter& value(int v) {
        value_prefix();
        os_ << v;
        finish_value();
        return *this;
    }

    JsonWriter& value(bool v) {
        value_prefix();
        os_ << (v ? "true" : "false");
        finish_value();
        return *this;
    }

    JsonWriter& value(const char* v) {
        value_prefix();
        dump_string(v);
        finish_value();
        return *this;
    }

    JsonWriter& value(const std::string& v) {
        return value(v.c_str());
    }

    // ---- key (for objects) ----

    JsonWriter& key(const char* k) {
        comma();
        dump_string(k);
        os_ << ":";
        after_key_ = true;
        return *this;
    }

    JsonWriter& key(const std::string& k) {
        return key(k.c_str());
    }

    // ---- array helpers ----

    template <typename Fn>
    JsonWriter& array_int(int count, Fn fn) {
        value_prefix();
        os_ << "[";
        for (int i = 0; i < count; ++i) {
            if (i > 0) os_ << ",";
            fn(os_, i);
        }
        os_ << "]";
        finish_value();
        return *this;
    }

    template <typename Fn>
    JsonWriter& array(std::size_t count, Fn fn) {
        value_prefix();
        os_ << "[";
        for (std::size_t i = 0; i < count; ++i) {
            if (i > 0) os_ << ",";
            fn(os_, i);
        }
        os_ << "]";
        finish_value();
        return *this;
    }

    JsonWriter& array_double(const std::vector<double>& values) {
        value_prefix();
        os_ << "[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) os_ << ",";
            if (std::isnan(values[i])) {
                os_ << "null";
            } else {
                os_ << std::fixed << std::setprecision(8) << values[i];
            }
        }
        os_ << "]";
        finish_value();
        return *this;
    }

    JsonWriter& array_vec3(const std::vector<Vec3>& values) {
        value_prefix();
        os_ << "[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) os_ << ",";
            os_ << "[" << std::fixed << std::setprecision(8)
                << values[i].x << "," << values[i].y << "," << values[i].z << "]";
        }
        os_ << "]";
        finish_value();
        return *this;
    }

    JsonWriter& array_string(const std::vector<std::string>& values) {
        value_prefix();
        os_ << "[";
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i > 0) os_ << ",";
            dump_string(values[i]);
        }
        os_ << "]";
        finish_value();
        return *this;
    }

    // ---- container open/close ----

    JsonWriter& begin_object() {
        value_prefix();
        os_ << "{";
        push_scope();
        return *this;
    }

    JsonWriter& end_object() {
        pop_scope("}");
        return *this;
    }

    JsonWriter& begin_array() {
        value_prefix();
        os_ << "[";
        push_scope();
        return *this;
    }

    JsonWriter& end_array() {
        pop_scope("]");
        return *this;
    }

    // ---- convenience: key + object pair ----

    JsonWriter& key_object(const char* k) { key(k); return begin_object(); }
    JsonWriter& key_array(const char* k) { key(k); return begin_array(); }

    void flush() { os_.flush(); }

private:
    void dump_string(const std::string& v) {
        os_ << "\"";
        for (char c : v) {
            if (c == '"' || c == '\\') os_ << "\\";
            os_ << c;
        }
        os_ << "\"";
    }

    void comma() {
        if (need_comma_) {
            os_ << ",";
        }
        need_comma_ = false;
        if (pending_newline_) {
            os_ << "\n";
            for (int i = 0; i < indent_ * 2; ++i) os_ << " ";
            pending_newline_ = false;
        }
    }

    void value_prefix() {
        if (after_key_) {
            after_key_ = false;
            return;
        }
        comma();
    }

    void finish_value() {
        need_comma_ = true;
        pending_newline_ = false;
    }

    void push_scope() {
        indent_++;
        need_comma_ = false;
        pending_newline_ = true;
    }

    void pop_scope(const char* closer) {
        indent_--;
        os_ << "\n";
        for (int i = 0; i < indent_ * 2; ++i) os_ << " ";
        os_ << closer;
        need_comma_ = true;
        pending_newline_ = false;
    }

    std::ostream& os_;
    int indent_ = 0;
    bool need_comma_ = false;
    bool pending_newline_ = false;
    bool after_key_ = false;
};

}  // namespace sim
