#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "formation_simulation.hpp"

namespace {

struct BenchmarkRecord {
    int run_index = 0;
    double runtime_s = 0.0;
    std::vector<double> mean_error;
    std::vector<double> max_error;
    std::vector<double> final_error;
    int completed_waypoint_count = 0;
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

std::string array_to_json(const std::vector<double>& values, int precision = 6) {
    std::ostringstream oss;
    oss << "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << std::fixed << std::setprecision(precision) << values[i];
    }
    oss << "]";
    return oss.str();
}

}  // namespace

int main() {
    using sim::FormationSimulation;
    using sim::SimulationConfig;

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

    const std::filesystem::path output_path = std::filesystem::path("outputs") / "benchmark_results.json";
    std::filesystem::create_directories(output_path.parent_path());

    std::ofstream out(output_path, std::ios::binary);
    if (!out) {
        std::cerr << "无法写入结果文件: " << output_path.string() << "\n";
        return 1;
    }

    out << "{\n";
    out << "  \"summary\": {\n";
    out << "    \"runs\": " << runs << ",\n";
    out << "    \"runtime_mean_s\": " << std::fixed << std::setprecision(6) << mean(runtime_values) << ",\n";
    out << "    \"runtime_std_s\": " << stddev(runtime_values) << ",\n";
    out << "    \"mean_error_mean\": " << array_to_json(mean_error_mean) << ",\n";
    out << "    \"mean_error_std\": " << array_to_json(mean_error_std) << ",\n";
    out << "    \"max_error_mean\": " << array_to_json(max_error_mean) << ",\n";
    out << "    \"max_error_std\": " << array_to_json(max_error_std) << ",\n";
    out << "    \"worst_case_max_error\": " << worst_case_max_error << ",\n";
    out << "    \"all_runs_pass_0_3m\": " << (all_runs_pass_0_3m ? "true" : "false") << "\n";
    out << "  },\n";
    out << "  \"records\": [\n";

    for (std::size_t i = 0; i < records.size(); ++i) {
        const BenchmarkRecord& r = records[i];
        out << "    {\n";
        out << "      \"run_index\": " << r.run_index << ",\n";
        out << "      \"runtime_s\": " << r.runtime_s << ",\n";
        out << "      \"mean_error\": " << array_to_json(r.mean_error) << ",\n";
        out << "      \"max_error\": " << array_to_json(r.max_error) << ",\n";
        out << "      \"final_error\": " << array_to_json(r.final_error) << ",\n";
        out << "      \"completed_waypoint_count\": " << r.completed_waypoint_count << "\n";
        out << "    }" << (i + 1 < records.size() ? "," : "") << "\n";
    }

    out << "  ]\n";
    out << "}\n";
    out.close();

    std::cout << "批量评测完成\n";
    std::cout << "结果文件: " << output_path.string() << "\n";
    std::cout << "平均运行时间: " << std::fixed << std::setprecision(6) << mean(runtime_values) << " s\n";
    std::cout << "最差工况最大误差: " << worst_case_max_error << "\n";
    std::cout << "0.3m 阈值全通过: " << (all_runs_pass_0_3m ? "true" : "false") << "\n";

    return 0;
}
