#include "visualization.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace sim {

namespace {

struct Bounds2D {
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
};

void update_bounds(Bounds2D* b, double x, double y) {
    b->min_x = std::min(b->min_x, x);
    b->max_x = std::max(b->max_x, x);
    b->min_y = std::min(b->min_y, y);
    b->max_y = std::max(b->max_y, y);
}

Bounds2D normalize_bounds(Bounds2D b) {
    if (!std::isfinite(b.min_x) || !std::isfinite(b.max_x)) {
        b.min_x = -1.0;
        b.max_x = 1.0;
    }
    if (!std::isfinite(b.min_y) || !std::isfinite(b.max_y)) {
        b.min_y = -1.0;
        b.max_y = 1.0;
    }
    if (std::abs(b.max_x - b.min_x) < 1e-9) {
        b.min_x -= 1.0;
        b.max_x += 1.0;
    }
    if (std::abs(b.max_y - b.min_y) < 1e-9) {
        b.min_y -= 1.0;
        b.max_y += 1.0;
    }
    return b;
}

struct Color {
    std::uint8_t r = 0;
    std::uint8_t g = 0;
    std::uint8_t b = 0;
    std::uint8_t a = 255;
};

struct RasterImage {
    int width;
    int height;
    std::vector<std::uint8_t> pixels;

    RasterImage(int w, int h, Color background)
        : width(w), height(h), pixels(static_cast<std::size_t>(w * h * 4), 255U) {
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                const std::size_t idx = static_cast<std::size_t>((y * width + x) * 4);
                pixels[idx + 0] = background.r;
                pixels[idx + 1] = background.g;
                pixels[idx + 2] = background.b;
                pixels[idx + 3] = background.a;
            }
        }
    }
};

int hex_nibble(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return 10 + (c - 'a');
    }
    if (c >= 'A' && c <= 'F') {
        return 10 + (c - 'A');
    }
    return 0;
}

Color color_from_hex(const std::string& hex) {
    if (hex.size() < 7 || hex[0] != '#') {
        return Color{0, 0, 0, 255};
    }

    const auto parse_byte = [&](std::size_t index) -> std::uint8_t {
        const int hi = hex_nibble(hex[index]);
        const int lo = hex_nibble(hex[index + 1]);
        return static_cast<std::uint8_t>((hi << 4) | lo);
    };

    return Color{
        parse_byte(1),
        parse_byte(3),
        parse_byte(5),
        255,
    };
}

void set_pixel(RasterImage* image, int x, int y, Color c) {
    if (x < 0 || y < 0 || x >= image->width || y >= image->height) {
        return;
    }
    const std::size_t idx = static_cast<std::size_t>((y * image->width + x) * 4);
    image->pixels[idx + 0] = c.r;
    image->pixels[idx + 1] = c.g;
    image->pixels[idx + 2] = c.b;
    image->pixels[idx + 3] = c.a;
}

void draw_disk(RasterImage* image, int cx, int cy, int radius, Color c) {
    const int r = std::max(radius, 0);
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            if (dx * dx + dy * dy <= r * r) {
                set_pixel(image, cx + dx, cy + dy, c);
            }
        }
    }
}

void draw_line(RasterImage* image, double x0, double y0, double x1, double y1,
               Color c, int thickness = 1, bool dashed = false) {
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(dx), std::abs(dy)))));
    const int radius = std::max(0, thickness / 2);

    for (int i = 0; i <= steps; ++i) {
        if (dashed) {
            const int cycle = 11;
            if ((i % cycle) >= 6) {
                continue;
            }
        }
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const int x = static_cast<int>(std::lround(x0 + dx * t));
        const int y = static_cast<int>(std::lround(y0 + dy * t));
        draw_disk(image, x, y, radius, c);
    }
}

void draw_polyline(RasterImage* image, const std::vector<std::pair<double, double>>& points,
                   Color c, int thickness = 1, bool dashed = false) {
    if (points.size() < 2) {
        return;
    }
    for (std::size_t i = 1; i < points.size(); ++i) {
        draw_line(image,
                  points[i - 1].first,
                  points[i - 1].second,
                  points[i].first,
                  points[i].second,
                  c,
                  thickness,
                  dashed);
    }
}

void draw_rect_outline(RasterImage* image, int x, int y, int w, int h, Color c, int thickness = 1) {
    for (int t = 0; t < thickness; ++t) {
        const int lx = x + t;
        const int rx = x + w - 1 - t;
        const int ty = y + t;
        const int by = y + h - 1 - t;

        draw_line(image, static_cast<double>(lx), static_cast<double>(ty), static_cast<double>(rx), static_cast<double>(ty), c, 1, false);
        draw_line(image, static_cast<double>(lx), static_cast<double>(by), static_cast<double>(rx), static_cast<double>(by), c, 1, false);
        draw_line(image, static_cast<double>(lx), static_cast<double>(ty), static_cast<double>(lx), static_cast<double>(by), c, 1, false);
        draw_line(image, static_cast<double>(rx), static_cast<double>(ty), static_cast<double>(rx), static_cast<double>(by), c, 1, false);
    }
}

void draw_rect_filled(RasterImage* image, int x, int y, int w, int h, Color c) {
    if (w <= 0 || h <= 0) {
        return;
    }
    for (int yy = y; yy < y + h; ++yy) {
        for (int xx = x; xx < x + w; ++xx) {
            set_pixel(image, xx, yy, c);
        }
    }
}

std::pair<double, double> to_canvas(double x, double y, const Bounds2D& b,
                                    double left, double top, double width, double height) {
    const double nx = (x - b.min_x) / (b.max_x - b.min_x);
    const double ny = (y - b.min_y) / (b.max_y - b.min_y);
    const double cx = left + nx * width;
    const double cy = top + (1.0 - ny) * height;
    return {cx, cy};
}

std::string format_double(double v, int precision = 2) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << v;
    return oss.str();
}

void write_text_file(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to write file: " + path);
    }
    out << content;
}

void write_binary_file(const std::string& path, const std::vector<std::uint8_t>& content) {
    std::ofstream out(path, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Failed to write file: " + path);
    }
    out.write(reinterpret_cast<const char*>(content.data()), static_cast<std::streamsize>(content.size()));
}

std::string color_for_index(std::size_t i) {
    static const std::vector<std::string> colors{
        "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#17becf",
        "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#1f77b4"
    };
    return colors[i % colors.size()];
}

std::pair<double, double> project_3d(const Vec3& p) {
    return {p.x - 0.5 * p.y, -p.z + 0.35 * p.y};
}

void append_u32_be(std::vector<std::uint8_t>* out, std::uint32_t v) {
    out->push_back(static_cast<std::uint8_t>((v >> 24) & 0xFFU));
    out->push_back(static_cast<std::uint8_t>((v >> 16) & 0xFFU));
    out->push_back(static_cast<std::uint8_t>((v >> 8) & 0xFFU));
    out->push_back(static_cast<std::uint8_t>(v & 0xFFU));
}

std::uint32_t crc32_bytes(const std::uint8_t* data, std::size_t size) {
    std::uint32_t crc = 0xFFFFFFFFU;
    for (std::size_t i = 0; i < size; ++i) {
        crc ^= static_cast<std::uint32_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit) {
            const std::uint32_t mask = static_cast<std::uint32_t>(-(static_cast<std::int32_t>(crc & 1U)));
            crc = (crc >> 1U) ^ (0xEDB88320U & mask);
        }
    }
    return ~crc;
}

std::uint32_t adler32_bytes(const std::uint8_t* data, std::size_t size) {
    constexpr std::uint32_t mod = 65521U;
    std::uint32_t a = 1U;
    std::uint32_t b = 0U;
    for (std::size_t i = 0; i < size; ++i) {
        a = (a + data[i]) % mod;
        b = (b + a) % mod;
    }
    return (b << 16U) | a;
}

void append_chunk(std::vector<std::uint8_t>* out, const char type[4], const std::vector<std::uint8_t>& data) {
    append_u32_be(out, static_cast<std::uint32_t>(data.size()));

    const std::size_t type_pos = out->size();
    out->push_back(static_cast<std::uint8_t>(type[0]));
    out->push_back(static_cast<std::uint8_t>(type[1]));
    out->push_back(static_cast<std::uint8_t>(type[2]));
    out->push_back(static_cast<std::uint8_t>(type[3]));
    out->insert(out->end(), data.begin(), data.end());

    const std::uint32_t crc = crc32_bytes(out->data() + type_pos, 4U + data.size());
    append_u32_be(out, crc);
}

std::vector<std::uint8_t> zlib_store_compress(const std::vector<std::uint8_t>& raw) {
    std::vector<std::uint8_t> out;
    out.reserve(raw.size() + raw.size() / 65535U + 16U);

    out.push_back(0x78U);
    out.push_back(0x01U);

    std::size_t pos = 0;
    while (pos < raw.size()) {
        const std::size_t remaining = raw.size() - pos;
        const std::uint16_t block_len = static_cast<std::uint16_t>(std::min<std::size_t>(65535U, remaining));
        const bool final_block = (pos + block_len == raw.size());

        out.push_back(final_block ? 0x01U : 0x00U);
        out.push_back(static_cast<std::uint8_t>(block_len & 0xFFU));
        out.push_back(static_cast<std::uint8_t>((block_len >> 8) & 0xFFU));

        const std::uint16_t nlen = static_cast<std::uint16_t>(~block_len);
        out.push_back(static_cast<std::uint8_t>(nlen & 0xFFU));
        out.push_back(static_cast<std::uint8_t>((nlen >> 8) & 0xFFU));

        out.insert(out.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos), raw.begin() + static_cast<std::ptrdiff_t>(pos + block_len));
        pos += block_len;
    }

    const std::uint32_t adler = adler32_bytes(raw.data(), raw.size());
    append_u32_be(&out, adler);
    return out;
}

void write_png_rgba(const std::string& path, const RasterImage& image) {
    if (image.width <= 0 || image.height <= 0) {
        throw std::runtime_error("Invalid image size for PNG output");
    }

    std::vector<std::uint8_t> raw;
    raw.reserve(static_cast<std::size_t>(image.height) * static_cast<std::size_t>(image.width * 4 + 1));

    for (int y = 0; y < image.height; ++y) {
        raw.push_back(0U);
        const std::size_t row_start = static_cast<std::size_t>(y * image.width * 4);
        raw.insert(raw.end(), image.pixels.begin() + static_cast<std::ptrdiff_t>(row_start), image.pixels.begin() + static_cast<std::ptrdiff_t>(row_start + image.width * 4));
    }

    const std::vector<std::uint8_t> compressed = zlib_store_compress(raw);

    std::vector<std::uint8_t> png;
    png.reserve(compressed.size() + 256U);

    const std::array<std::uint8_t, 8> signature{137U, 80U, 78U, 71U, 13U, 10U, 26U, 10U};
    png.insert(png.end(), signature.begin(), signature.end());

    std::vector<std::uint8_t> ihdr;
    ihdr.reserve(13U);
    append_u32_be(&ihdr, static_cast<std::uint32_t>(image.width));
    append_u32_be(&ihdr, static_cast<std::uint32_t>(image.height));
    ihdr.push_back(8U);
    ihdr.push_back(6U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);
    ihdr.push_back(0U);

    append_chunk(&png, "IHDR", ihdr);
    append_chunk(&png, "IDAT", compressed);
    append_chunk(&png, "IEND", std::vector<std::uint8_t>{});

    write_binary_file(path, png);
}

void render_trajectory_png(const SimulationResult& result,
                           const Bounds2D& bounds,
                           double width,
                           double height,
                           double left,
                           double top,
                           double plot_w,
                           double plot_h,
                           const std::string& png_path) {
    RasterImage image(static_cast<int>(std::lround(width)), static_cast<int>(std::lround(height)), Color{255, 255, 255, 255});

    draw_rect_outline(
        &image,
        static_cast<int>(std::lround(left)),
        static_cast<int>(std::lround(top)),
        static_cast<int>(std::lround(plot_w)),
        static_cast<int>(std::lround(plot_h)),
        color_from_hex("#999999"),
        1
    );

    for (const Vec3& wp : result.waypoints) {
        const auto pr = project_3d(wp);
        const auto c = to_canvas(pr.first, pr.second, bounds, left, top, plot_w, plot_h);
        draw_disk(&image, static_cast<int>(std::lround(c.first)), static_cast<int>(std::lround(c.second)), 4, Color{0, 0, 0, 255});
    }

    auto draw_path = [&](const std::vector<Vec3>& points, const std::string& color, int thickness, bool dashed) {
        std::vector<std::pair<double, double>> poly;
        poly.reserve(points.size());
        for (const Vec3& p : points) {
            const auto pr = project_3d(p);
            poly.push_back(to_canvas(pr.first, pr.second, bounds, left, top, plot_w, plot_h));
        }
        draw_polyline(&image, poly, color_from_hex(color), thickness, dashed);
    };

    draw_path(result.leader, "#1f77b4", 3, false);
    for (std::size_t i = 0; i < result.followers.size(); ++i) {
        draw_path(result.followers[i], color_for_index(i), 2, true);
    }

    write_png_rgba(png_path, image);
}

void render_error_components_png(const SimulationResult& result,
                                 const std::array<Bounds2D, 3>& comp_bounds,
                                 double width,
                                 double height,
                                 double left,
                                 double top,
                                 double plot_w,
                                 double subplot_h,
                                 double gap,
                                 const std::string& png_path) {
    RasterImage image(static_cast<int>(std::lround(width)), static_cast<int>(std::lround(height)), Color{255, 255, 255, 255});

    auto component_value = [](const Vec3& v, int comp) {
        if (comp == 0) {
            return v.x;
        }
        if (comp == 1) {
            return v.y;
        }
        return v.z;
    };

    for (int comp = 0; comp < 3; ++comp) {
        const double y0 = top + comp * (subplot_h + gap);

        draw_rect_outline(
            &image,
            static_cast<int>(std::lround(left)),
            static_cast<int>(std::lround(y0)),
            static_cast<int>(std::lround(plot_w)),
            static_cast<int>(std::lround(subplot_h)),
            color_from_hex("#999999"),
            1
        );

        for (std::size_t i = 0; i < result.error_vectors.size(); ++i) {
            std::vector<std::pair<double, double>> poly;
            poly.reserve(result.error_vectors[i].size());
            for (std::size_t k = 0; k < result.error_vectors[i].size(); ++k) {
                const double x = result.time[k];
                const double y = component_value(result.error_vectors[i][k], comp);
                poly.push_back(to_canvas(x, y, comp_bounds[comp], left, y0, plot_w, subplot_h));
            }
            draw_polyline(&image, poly, color_from_hex(color_for_index(i)), 2, false);
        }
    }

    write_png_rgba(png_path, image);
}

void render_error_stats_png(const SimulationResult& result,
                            double width,
                            double height,
                            double left,
                            double top,
                            double plot_w,
                            double plot_h,
                            double y_max,
                            const std::string& png_path) {
    RasterImage image(static_cast<int>(std::lround(width)), static_cast<int>(std::lround(height)), Color{255, 255, 255, 255});

    draw_rect_outline(
        &image,
        static_cast<int>(std::lround(left)),
        static_cast<int>(std::lround(top)),
        static_cast<int>(std::lround(plot_w)),
        static_cast<int>(std::lround(plot_h)),
        color_from_hex("#999999"),
        1
    );

    const std::size_t n = result.metrics.mean.size();
    const double slot = (n > 0) ? (plot_w / static_cast<double>(n)) : plot_w;
    const double bar_w = slot * 0.28;

    auto y_to_canvas = [&](double v) {
        const double ratio = (y_max > 1e-9) ? (v / y_max) : 0.0;
        return top + plot_h * (1.0 - ratio);
    };

    for (std::size_t i = 0; i < n; ++i) {
        const double cx = left + slot * (static_cast<double>(i) + 0.5);
        const double mean_v = result.metrics.mean[i];
        const double max_v = result.metrics.max[i];

        const int mean_x = static_cast<int>(std::lround(cx - bar_w - 3.0));
        const int max_x = static_cast<int>(std::lround(cx + 3.0));
        const int mean_y = static_cast<int>(std::lround(y_to_canvas(mean_v)));
        const int max_y = static_cast<int>(std::lround(y_to_canvas(max_v)));
        const int mean_h = std::max(1, static_cast<int>(std::lround(top + plot_h - y_to_canvas(mean_v))));
        const int max_h = std::max(1, static_cast<int>(std::lround(top + plot_h - y_to_canvas(max_v))));
        const int bar_width = std::max(1, static_cast<int>(std::lround(bar_w)));

        draw_rect_filled(&image, mean_x, mean_y, bar_width, mean_h, color_from_hex("#4e79a7"));
        draw_rect_filled(&image, max_x, max_y, bar_width, max_h, color_from_hex("#e15759"));
    }

    const double threshold = 0.3;
    const int th_y = static_cast<int>(std::lround(y_to_canvas(threshold)));
    draw_line(&image, left, static_cast<double>(th_y), left + plot_w, static_cast<double>(th_y), Color{0, 0, 0, 255}, 1, true);

    write_png_rgba(png_path, image);
}

}  // namespace

SimulationVisualizer::SimulationVisualizer(std::string output_dir)
    : output_dir_(std::move(output_dir)) {
    std::filesystem::create_directories(output_dir_);
}

std::map<std::string, std::string> SimulationVisualizer::plot_all(const SimulationResult& result) const {
    return {
        {"trajectory", plot_trajectory_svg(result)},
        {"error_3d", plot_error_components_svg(result)},
        {"error_stats", plot_error_stats_svg(result)},
    };
}

std::string SimulationVisualizer::plot_trajectory_svg(const SimulationResult& result) const {
    const double width = 1100.0;
    const double height = 820.0;
    const double left = 85.0;
    const double top = 80.0;
    const double plot_w = 820.0;
    const double plot_h = 660.0;

    Bounds2D bounds{};
    for (const Vec3& p : result.leader) {
        const auto pr = project_3d(p);
        update_bounds(&bounds, pr.first, pr.second);
    }
    for (const auto& follower : result.followers) {
        for (const Vec3& p : follower) {
            const auto pr = project_3d(p);
            update_bounds(&bounds, pr.first, pr.second);
        }
    }
    for (const Vec3& wp : result.waypoints) {
        const auto pr = project_3d(wp);
        update_bounds(&bounds, pr.first, pr.second);
    }
    bounds = normalize_bounds(bounds);

    std::ostringstream svg;
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height << "'>\n";
    svg << "<rect x='0' y='0' width='" << width << "' height='" << height << "' fill='white'/>\n";
    svg << "<rect x='" << left << "' y='" << top << "' width='" << plot_w << "' height='" << plot_h
        << "' fill='none' stroke='#999' stroke-width='1.2'/>\n";
    svg << "<text x='" << (left + plot_w / 2.0) << "' y='42' text-anchor='middle' font-size='22'"
        << " font-family='Segoe UI'>UAV Formation 3D Trajectory (Isometric Projection)</text>\n";

    for (const Vec3& wp : result.waypoints) {
        const auto pr = project_3d(wp);
        const auto c = to_canvas(pr.first, pr.second, bounds, left, top, plot_w, plot_h);
        svg << "<circle cx='" << format_double(c.first) << "' cy='" << format_double(c.second)
            << "' r='4' fill='black'/>\n";
    }

    auto draw_polyline = [&](const std::vector<Vec3>& points, const std::string& color, double stroke, bool dashed) {
        if (points.empty()) {
            return;
        }
        svg << "<polyline fill='none' stroke='" << color << "' stroke-width='" << stroke << "'";
        if (dashed) {
            svg << " stroke-dasharray='6 5'";
        }
        svg << " points='";
        for (const Vec3& p : points) {
            const auto pr = project_3d(p);
            const auto c = to_canvas(pr.first, pr.second, bounds, left, top, plot_w, plot_h);
            svg << format_double(c.first) << "," << format_double(c.second) << " ";
        }
        svg << "'/>\n";
    };

    draw_polyline(result.leader, "#1f77b4", 2.6, false);
    for (std::size_t i = 0; i < result.followers.size(); ++i) {
        draw_polyline(result.followers[i], color_for_index(i), 1.7, true);
    }

    const double legend_x = left + plot_w + 25.0;
    double legend_y = top + 30.0;
    svg << "<text x='" << legend_x << "' y='" << legend_y - 10.0
        << "' font-size='14' font-family='Segoe UI'>Legend</text>\n";

    svg << "<line x1='" << legend_x << "' y1='" << legend_y << "' x2='" << (legend_x + 26)
        << "' y2='" << legend_y << "' stroke='#1f77b4' stroke-width='2.6'/>\n";
    svg << "<text x='" << (legend_x + 34) << "' y='" << (legend_y + 5)
        << "' font-size='13' font-family='Segoe UI'>Leader</text>\n";
    legend_y += 24.0;

    for (std::size_t i = 0; i < result.followers.size(); ++i) {
        svg << "<line x1='" << legend_x << "' y1='" << legend_y << "' x2='" << (legend_x + 26)
            << "' y2='" << legend_y << "' stroke='" << color_for_index(i)
            << "' stroke-width='1.8' stroke-dasharray='6 5'/>\n";
        svg << "<text x='" << (legend_x + 34) << "' y='" << (legend_y + 5)
            << "' font-size='13' font-family='Segoe UI'>Follower " << (i + 1) << "</text>\n";
        legend_y += 20.0;
    }

    svg << "<text x='" << (left + plot_w / 2.0) << "' y='" << (top + plot_h + 36.0)
        << "' text-anchor='middle' font-size='13' font-family='Segoe UI'>Projected horizontal axis</text>\n";
    svg << "<text x='28' y='" << (top + plot_h / 2.0)
        << "' text-anchor='middle' transform='rotate(-90 28 " << (top + plot_h / 2.0)
        << ")' font-size='13' font-family='Segoe UI'>Projected vertical axis</text>\n";

    svg << "</svg>\n";

    const std::string svg_path = output_dir_ + "/trajectory_3d.svg";
    write_text_file(svg_path, svg.str());

    const std::string png_path = output_dir_ + "/trajectory_3d.png";
    render_trajectory_png(result, bounds, width, height, left, top, plot_w, plot_h, png_path);
    return png_path;
}

std::string SimulationVisualizer::plot_error_components_svg(const SimulationResult& result) const {
    const double width = 1200.0;
    const double height = 940.0;
    const double left = 95.0;
    const double right = 36.0;
    const double top = 76.0;
    const double bottom = 72.0;
    const double gap = 34.0;

    const double plot_w = width - left - right;
    const double subplot_h = (height - top - bottom - 2.0 * gap) / 3.0;

    auto component_value = [](const Vec3& v, int comp) {
        if (comp == 0) {
            return v.x;
        }
        if (comp == 1) {
            return v.y;
        }
        return v.z;
    };

    std::array<Bounds2D, 3> comp_bounds{};
    for (int comp = 0; comp < 3; ++comp) {
        for (std::size_t i = 0; i < result.error_vectors.size(); ++i) {
            for (std::size_t k = 0; k < result.error_vectors[i].size(); ++k) {
                update_bounds(&comp_bounds[comp], result.time[k], component_value(result.error_vectors[i][k], comp));
            }
        }
        comp_bounds[comp] = normalize_bounds(comp_bounds[comp]);
    }

    std::ostringstream svg;
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height << "'>\n";
    svg << "<rect x='0' y='0' width='" << width << "' height='" << height << "' fill='white'/>\n";
    svg << "<text x='" << (width / 2.0) << "' y='42' text-anchor='middle' font-size='23' font-family='Segoe UI'"
        << ">Real-time Error Components vs Time</text>\n";

    const std::array<std::string, 3> labels{"e_x (m)", "e_y (m)", "e_z (m)"};

    for (int comp = 0; comp < 3; ++comp) {
        const double y0 = top + comp * (subplot_h + gap);

        svg << "<rect x='" << left << "' y='" << y0 << "' width='" << plot_w << "' height='" << subplot_h
            << "' fill='none' stroke='#999' stroke-width='1.1'/>\n";

        for (std::size_t i = 0; i < result.error_vectors.size(); ++i) {
            svg << "<polyline fill='none' stroke='" << color_for_index(i)
                << "' stroke-width='1.5' points='";
            for (std::size_t k = 0; k < result.error_vectors[i].size(); ++k) {
                const double x = result.time[k];
                const double y = component_value(result.error_vectors[i][k], comp);
                const auto c = to_canvas(x, y, comp_bounds[comp], left, y0, plot_w, subplot_h);
                svg << format_double(c.first) << "," << format_double(c.second) << " ";
            }
            svg << "'/>\n";
        }

        svg << "<text x='" << 25.0 << "' y='" << (y0 + subplot_h / 2.0)
            << "' text-anchor='middle' transform='rotate(-90 25 " << (y0 + subplot_h / 2.0)
            << ")' font-size='13' font-family='Segoe UI'>" << labels[comp] << "</text>\n";

        svg << "<text x='" << (left + 2.0) << "' y='" << (y0 + 16.0)
            << "' font-size='11' fill='#555' font-family='Segoe UI'>"
            << "min=" << format_double(comp_bounds[comp].min_y, 3)
            << ", max=" << format_double(comp_bounds[comp].max_y, 3)
            << "</text>\n";
    }

    svg << "<text x='" << (left + plot_w / 2.0) << "' y='" << (height - 22.0)
        << "' text-anchor='middle' font-size='14' font-family='Segoe UI'>Time (s)</text>\n";

    const double legend_x = left + plot_w - 185.0;
    double legend_y = top + 16.0;
    for (std::size_t i = 0; i < result.error_vectors.size(); ++i) {
        svg << "<line x1='" << legend_x << "' y1='" << legend_y << "' x2='" << (legend_x + 24)
            << "' y2='" << legend_y << "' stroke='" << color_for_index(i) << "' stroke-width='2'/>\n";
        svg << "<text x='" << (legend_x + 32) << "' y='" << (legend_y + 4)
            << "' font-size='12' font-family='Segoe UI'>Follower " << (i + 1) << "</text>\n";
        legend_y += 18.0;
    }

    svg << "</svg>\n";

    const std::string svg_path = output_dir_ + "/error_realtime_3d.svg";
    write_text_file(svg_path, svg.str());

    const std::string png_path = output_dir_ + "/error_realtime_3d.png";
    render_error_components_png(result, comp_bounds, width, height, left, top, plot_w, subplot_h, gap, png_path);
    return png_path;
}

std::string SimulationVisualizer::plot_error_stats_svg(const SimulationResult& result) const {
    const double width = 1000.0;
    const double height = 620.0;
    const double left = 90.0;
    const double top = 70.0;
    const double right = 42.0;
    const double bottom = 90.0;

    const double plot_w = width - left - right;
    const double plot_h = height - top - bottom;

    double y_max = 0.5;
    for (double v : result.metrics.max) {
        y_max = std::max(y_max, v * 1.15);
    }

    std::ostringstream svg;
    svg << "<svg xmlns='http://www.w3.org/2000/svg' width='" << width << "' height='" << height << "'>\n";
    svg << "<rect x='0' y='0' width='" << width << "' height='" << height << "' fill='white'/>\n";
    svg << "<text x='" << (width / 2.0) << "' y='42' text-anchor='middle' font-size='23' font-family='Segoe UI'"
        << ">Formation Error Statistics</text>\n";
    svg << "<rect x='" << left << "' y='" << top << "' width='" << plot_w << "' height='" << plot_h
        << "' fill='none' stroke='#999' stroke-width='1.1'/>\n";

    const std::size_t n = result.metrics.mean.size();
    const double slot = (n > 0) ? (plot_w / static_cast<double>(n)) : plot_w;
    const double bar_w = slot * 0.28;

    auto y_to_canvas = [&](double v) {
        const double ratio = (y_max > 1e-9) ? (v / y_max) : 0.0;
        return top + plot_h * (1.0 - ratio);
    };

    for (std::size_t i = 0; i < n; ++i) {
        const double cx = left + slot * (static_cast<double>(i) + 0.5);

        const double mean_v = result.metrics.mean[i];
        const double max_v = result.metrics.max[i];

        const double mean_y = y_to_canvas(mean_v);
        const double max_y = y_to_canvas(max_v);

        svg << "<rect x='" << (cx - bar_w - 3.0) << "' y='" << mean_y << "' width='" << bar_w
            << "' height='" << (top + plot_h - mean_y) << "' fill='#4e79a7'/>\n";
        svg << "<rect x='" << (cx + 3.0) << "' y='" << max_y << "' width='" << bar_w
            << "' height='" << (top + plot_h - max_y) << "' fill='#e15759'/>\n";

        svg << "<text x='" << cx << "' y='" << (top + plot_h + 22.0)
            << "' text-anchor='middle' font-size='12' font-family='Segoe UI'>F" << (i + 1) << "</text>\n";
    }

    const double threshold = 0.3;
    const double th_y = y_to_canvas(threshold);
    svg << "<line x1='" << left << "' y1='" << th_y << "' x2='" << (left + plot_w)
        << "' y2='" << th_y << "' stroke='black' stroke-width='1.4' stroke-dasharray='8 6'/>\n";

    svg << "<text x='" << (left + 6.0) << "' y='" << (th_y - 6.0)
        << "' font-size='11' font-family='Segoe UI'>0.3m threshold</text>\n";

    svg << "<text x='" << (left + plot_w / 2.0) << "' y='" << (height - 24.0)
        << "' text-anchor='middle' font-size='14' font-family='Segoe UI'>Follower Index</text>\n";
    svg << "<text x='28' y='" << (top + plot_h / 2.0)
        << "' text-anchor='middle' transform='rotate(-90 28 " << (top + plot_h / 2.0)
        << ")' font-size='14' font-family='Segoe UI'>Error (m)</text>\n";

    const double legend_x = left + plot_w - 180.0;
    const double legend_y = top + 24.0;
    svg << "<rect x='" << legend_x << "' y='" << (legend_y - 10.0)
        << "' width='14' height='14' fill='#4e79a7'/>\n";
    svg << "<text x='" << (legend_x + 22.0) << "' y='" << legend_y
        << "' font-size='12' font-family='Segoe UI'>Mean Error</text>\n";
    svg << "<rect x='" << legend_x << "' y='" << (legend_y + 14.0)
        << "' width='14' height='14' fill='#e15759'/>\n";
    svg << "<text x='" << (legend_x + 22.0) << "' y='" << (legend_y + 24.0)
        << "' font-size='12' font-family='Segoe UI'>Max Error</text>\n";

    svg << "</svg>\n";

    const std::string svg_path = output_dir_ + "/error_statistics.svg";
    write_text_file(svg_path, svg.str());

    const std::string png_path = output_dir_ + "/error_statistics.png";
    render_error_stats_png(result, width, height, left, top, plot_w, plot_h, y_max, png_path);
    return png_path;
}

}  // namespace sim
