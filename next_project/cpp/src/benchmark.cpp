#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#ifdef _WIN32
#include <windows.h>
#endif

#include "formation_simulation.hpp"
#include "json_writer.hpp"

namespace {

struct BenchmarkRecord {
    int run_index = 0;
    double runtime_s = 0.0;
    std::vector<double> mean_error;
    std::vector<double> max_error;
    std::vector<double> final_error;
    int completed_waypoint_count = 0;
    int seed = 0;
};

double mean(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }
    double sum = 0.0;
    for (double v : data) {
        sum += v;
    }
    return sum / static_cast<double>(data.size());
}

double stddev(const std::vector<double>& data) {
    if (data.empty()) {
        return 0.0;
    }
    const double m = mean(data);
    double s = 0.0;
    for (double v : data) {
        const double d = v - m;
        s += d * d;
    }
    return std::sqrt(s / static_cast<double>(data.size()));
}

std::string timestamp_dir_name() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm* local = std::localtime(&t);
    std::ostringstream oss;
    if (local != nullptr) {
        oss << std::put_time(local, "%Y%m%d-%H%M%S");
    } else {
        oss << "unknown";
    }
    return oss.str();
}

void write_benchmark_json(
    const std::filesystem::path& output_path,
    const std::string& preset,
    const std::string& runtime_engine,
    int runs,
    const std::vector<BenchmarkRecord>& records,
    const std::vector<double>& mean_error_mean,
    const std::vector<double>& mean_error_std,
    const std::vector<double>& max_error_mean,
    const std::vector<double>& max_error_std,
    double runtime_mean,
    double runtime_std,
    double worst_case_max_error,
    bool all_runs_pass_0_3m
) {
    namespace fs = std::filesystem;
    fs::create_directories(output_path.parent_path());

    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        std::cerr << "无法写入结果文件: " << output_path.string() << "\n";
        return;
    }

    sim::JsonWriter w(out);

    w.begin_object();
    w.key("schema_version").value("1.0.0");
    w.key("preset").value(preset);
    w.key("runtime_engine").value(runtime_engine);
    w.key("engine_version").value(sim::engine_version());
    w.key("generated_at").value(sim::iso8601_utc(std::chrono::system_clock::now()));

    // config_template (最小快照)
    w.key("config_template").begin_object();
    w.key("max_sim_time").value(30.0);
    w.key("use_smc").value(true);
    w.end_object();  // config_template

    // summary
    w.key("summary").begin_object();
    w.key("runs").value(runs);
    w.key("runtime_mean_s").value(runtime_mean);
    w.key("runtime_std_s").value(runtime_std);
    w.key("mean_error_mean").array_double(mean_error_mean);
    w.key("mean_error_std").array_double(mean_error_std);
    w.key("max_error_mean").array_double(max_error_mean);
    w.key("max_error_std").array_double(max_error_std);
    w.key("worst_case_max_error").value(worst_case_max_error);
    w.key("all_runs_pass_0_3m").value(all_runs_pass_0_3m);
    w.end_object();  // summary

    // records
    w.key("records").begin_array();
    for (std::size_t i = 0; i < records.size(); ++i) {
        const auto& r = records[i];
        w.begin_object();
        w.key("run_index").value(r.run_index);
        w.key("runtime_s").value(r.runtime_s);
        w.key("mean_error").array_double(r.mean_error);
        w.key("max_error").array_double(r.max_error);
        w.key("final_error").array_double(r.final_error);
        w.key("completed_waypoint_count").value(r.completed_waypoint_count);
        w.key("seed").value(r.seed);
        w.end_object();
    }
    w.end_array();  // records

    w.end_object();  // root
    out << "\n";
}

}  // namespace

int main() {
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    using sim::FormationSimulation;
    using sim::SimulationConfig;

    const std::string preset = "benchmark_default";
    const std::string runtime_engine = "cpp";
    constexpr int runs = 5;
    std::vector<BenchmarkRecord> records;
    records.reserve(static_cast<std::size_t>(runs));

    for (int i = 0; i < runs; ++i) {
        SimulationConfig cfg;
        cfg.max_sim_time = 30.0;
        cfg.use_smc = true;
        cfg.leader_wind_seed = static_cast<unsigned int>(42 + i);
        cfg.follower_wind_seed_start = static_cast<unsigned int>(100 + 10 * i);

        const auto start = std::chrono::high_resolution_clock::now();
        const sim::SimulationResult result = FormationSimulation(cfg).run();
        const auto end = std::chrono::high_resolution_clock::now();

        BenchmarkRecord record;
        record.run_index = i;
        record.runtime_s = std::chrono::duration<double>(end - start).count();
        record.mean_error = result.metrics.mean;
        record.max_error = result.metrics.max;
        record.final_error = result.metrics.final;
        record.completed_waypoint_count = result.completed_waypoint_count;
        record.seed = 42 + i;
        records.push_back(record);
    }

    const std::size_t follower_count = records.empty() ? 0U : records.front().mean_error.size();

    std::vector<double> runtime_values;
    runtime_values.reserve(records.size());

    std::vector<std::vector<double>> mean_matrix(follower_count);
    std::vector<std::vector<double>> max_matrix(follower_count);

    double worst_case_max_error = 0.0;
    bool all_runs_pass_0_3m = true;

    for (const BenchmarkRecord& r : records) {
        runtime_values.push_back(r.runtime_s);

        for (std::size_t j = 0; j < follower_count; ++j) {
            mean_matrix[j].push_back(r.mean_error[j]);
            max_matrix[j].push_back(r.max_error[j]);
            worst_case_max_error = std::max(worst_case_max_error, r.max_error[j]);
            if (r.max_error[j] >= 0.3) {
                all_runs_pass_0_3m = false;
            }
        }
    }

    std::vector<double> mean_error_mean(follower_count, 0.0);
    std::vector<double> mean_error_std(follower_count, 0.0);
    std::vector<double> max_error_mean(follower_count, 0.0);
    std::vector<double> max_error_std(follower_count, 0.0);

    for (std::size_t j = 0; j < follower_count; ++j) {
        mean_error_mean[j] = mean(mean_matrix[j]);
        mean_error_std[j] = stddev(mean_matrix[j]);
        max_error_mean[j] = mean(max_matrix[j]);
        max_error_std[j] = stddev(max_matrix[j]);
    }

    const double runtime_mean_val = mean(runtime_values);
    const double runtime_std_val = stddev(runtime_values);

    const std::filesystem::path output_path =
        std::filesystem::path("outputs") / preset / timestamp_dir_name() / "benchmark_results.json";

    write_benchmark_json(
        output_path,
        preset,
        runtime_engine,
        runs,
        records,
        mean_error_mean,
        mean_error_std,
        max_error_mean,
        max_error_std,
        runtime_mean_val,
        runtime_std_val,
        worst_case_max_error,
        all_runs_pass_0_3m
    );

    std::cout << "批量评测完成\n";
    std::cout << "结果文件: " << output_path.string() << "\n";
    std::cout << "平均运行时间: " << std::fixed << std::setprecision(6) << runtime_mean_val << " s\n";
    std::cout << "最差工况最大误差: " << worst_case_max_error << "\n";
    std::cout << "0.3m 阈值全通过: " << (all_runs_pass_0_3m ? "true" : "false") << "\n";

    return 0;
}
