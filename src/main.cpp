#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <csignal>
#include <ctime>
#include <cstdint>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <sstream>
#include <mutex>
#include <queue>
#include <string>
#include <string.h>
#include <thread>
#include <unordered_map>
#include <vector>

#include <libpq-fe.h>
#include <png.h>
#include <zmq.hpp>

#include <GL/glew.h>
#include <SDL2/SDL.h>

#include "backends/imgui_impl_opengl3.h"
#include "backends/imgui_impl_sdl2.h"
#include "imgui.h"
#include "implot.h"
#include "nlohmann/json.hpp"

using nlohmann::json;

using namespace std;
using namespace zmq;

namespace {

constexpr double kMaxLatitude = 85.05112878;
constexpr double kPi = 3.14159265358979323846;
constexpr int kMinZoom = 1;
constexpr int kMaxZoom = 19;
constexpr int kTileWorkerCount = 4;
const chrono::seconds kTileRetryDelay(20);

struct LocationSnapshot {
    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    float accuracy = 0.0f;
    long long int time_j = 0;
    string allcellinfo;
    int signalDbm = -999;
    long long rx = 0;
    long long tx = 0;
    vector<float> signal_x;
    vector<float> signal_y;
    unsigned long long cnt = 0;
};

struct ServerConfig {
    string bind_endpoint = "tcp://*:5656";
    filesystem::path log_file = "locations1.jsonl";
    string db_conninfo;
};

struct ProgramOptions {
    ServerConfig server;
    bool help = false;
    string import_json_to_db_file;
};

struct MapViewState {
    double center_latitude = 55.0292;
    double center_longitude = 82.9221;
    double view_span_x = 0.2;
    double view_span_y = 0.2;
    int current_zoom = 15;
    int last_tile_zoom = -1;
    bool follow_location = true;
    bool center_initialized = false;
};

struct DecodedImage {
    int width = 0;
    int height = 0;
    vector<unsigned char> rgba;
};

struct TileTexture {
    GLuint texture_id = 0;
    int width = 0;
    int height = 0;
    bool is_loading = false;
    vector<unsigned char> rgba_blob;
    filesystem::path file_path;
    string last_error;
    chrono::steady_clock::time_point next_retry_at{};
};

struct TileJob {
    string id;
    int zoom = 0;
    int x = 0;
    int y = 0;
};

enum class HeatmapCriterion {
    RSRP = 0,
    RSRQ = 1,
    RSSI = 2,
    Altitude = 3,
};

static const char* kHeatmapCriterionNames[] = {
    "RSRP",
    "RSRQ",
    "RSSI",
    "Altitude",
};

struct HeatmapPoint {
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    double accuracy_m = 0.0;
    double rsrp = numeric_limits<double>::quiet_NaN();
    double rsrq = numeric_limits<double>::quiet_NaN();
    double rssi = numeric_limits<double>::quiet_NaN();
    int earfcn = -1;
};

struct HeatmapOptions {
    HeatmapCriterion criterion = HeatmapCriterion::RSRP;
    bool per_tile = false;
    bool overlay = true;
    bool use_earfcn_groups = true;
    int selected_earfcn = -1;
    float radius_m = 30.0f;
    float display_radius_m = 80.0f;
    float power = 2.0f;
    float opacity = 0.75f;
    int output_image_size = 1024;
};

struct HeatmapTileJob {
    string id;
    string signature;
    filesystem::path file_path;
    HeatmapOptions options;
    int zoom = 0;
    int x = 0;
    int y = 0;
};

struct HeatmapBounds {
    double min_lon = 0.0;
    double max_lon = 0.0;
    double min_merc_y = 0.0;
    double max_merc_y = 0.0;
    bool valid = false;
};

struct HeatmapSample {
    double x_m = 0.0;
    double y_m = 0.0;
    double value = 0.0;
};

atomic_bool g_keep_running{true};

double clamp_latitude(double latitude) {
    return clamp(latitude, -kMaxLatitude, kMaxLatitude);
}

double wrap_longitude(double longitude) {
    double wrapped = fmod(longitude + 180.0, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped - 180.0;
}

int tile_count_for_zoom(int zoom) {
    return 1 << zoom;
}

double latitude_to_mercator_y(double latitude) {
    const double lat_rad = clamp_latitude(latitude) * kPi / 180.0;
    return log(tan(kPi / 4.0 + lat_rad / 2.0)) * 180.0 / kPi;
}

double mercator_x_to_tile_x(double mercator_x, int zoom) {
    return (0.5 + mercator_x / 360.0) * tile_count_for_zoom(zoom);
}

double mercator_y_to_tile_y(double mercator_y, int zoom) {
    return (0.5 - mercator_y / 360.0) * tile_count_for_zoom(zoom);
}

double tile_x_to_mercator_x(double tile_x, int zoom) {
    return (tile_x / static_cast<double>(tile_count_for_zoom(zoom)) - 0.5) * 360.0;
}

double tile_y_to_mercator_y(double tile_y, int zoom) {
    return (0.5 - tile_y / static_cast<double>(tile_count_for_zoom(zoom))) * 360.0;
}

int zoom_from_longitude_span(double span) {
    span = max(0.000001, min(360.0, span));
    int zoom = 4;
    double threshold = 90.0;
    while (span <= threshold && zoom < kMaxZoom) {
        ++zoom;
        threshold *= 0.5;
    }
    return clamp(zoom, kMinZoom, kMaxZoom);
}

filesystem::path resolve_cache_root() {
    error_code ec;
    const filesystem::path exe_path = filesystem::read_symlink("/proc/self/exe", ec);
    if (!ec) {
        const filesystem::path exe_dir = exe_path.parent_path();
        if (exe_dir.filename() == "build") {
            return exe_dir;
        }
    }

    const filesystem::path cwd = filesystem::current_path(ec);
    if (!ec) {
        if (cwd.filename() == "build") {
            return cwd;
        }
        return cwd / "build";
    }

    return "build";
}

filesystem::path resolve_osm_cache_root() {
    return resolve_cache_root();
}

string shell_escape(const string& value) {
    string escaped;
    escaped.reserve(value.size() + 2);
    escaped.push_back('"');
    for (char ch : value) {
        if (ch == '"' || ch == '\\' || ch == '$' || ch == '`') {
            escaped.push_back('\\');
        }
        escaped.push_back(ch);
    }
    escaped.push_back('"');
    return escaped;
}

bool decode_png_rgba(const filesystem::path& file_path, DecodedImage* decoded, string* error_message) {
    FILE* file = fopen(file_path.string().c_str(), "rb");
    if (file == nullptr) {
        if (error_message != nullptr) {
            *error_message = "failed to open " + file_path.string();
        }
        return false;
    }

    unsigned char header[8] = {};
    if (fread(header, 1, sizeof(header), file) != sizeof(header) || png_sig_cmp(header, 0, sizeof(header)) != 0) {
        fclose(file);
        if (error_message != nullptr) {
            *error_message = "invalid PNG header: " + file_path.string();
        }
        return false;
    }

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (png == nullptr) {
        fclose(file);
        if (error_message != nullptr) {
            *error_message = "png_create_read_struct failed";
        }
        return false;
    }

    png_infop info = png_create_info_struct(png);
    if (info == nullptr) {
        png_destroy_read_struct(&png, nullptr, nullptr);
        fclose(file);
        if (error_message != nullptr) {
            *error_message = "png_create_info_struct failed";
        }
        return false;
    }

    if (setjmp(png_jmpbuf(png)) != 0) {
        png_destroy_read_struct(&png, &info, nullptr);
        fclose(file);
        if (error_message != nullptr) {
            *error_message = "libpng decode failed: " + file_path.string();
        }
        return false;
    }

    png_init_io(png, file);
    png_set_sig_bytes(png, sizeof(header));
    png_read_info(png, info);

    const png_uint_32 width = png_get_image_width(png, info);
    const png_uint_32 height = png_get_image_height(png, info);
    const png_byte color_type = png_get_color_type(png, info);
    const png_byte bit_depth = png_get_bit_depth(png, info);

    if (bit_depth == 16) {
        png_set_strip_16(png);
    }
    if (color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_palette_to_rgb(png);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8) {
        png_set_expand_gray_1_2_4_to_8(png);
    }
    if (png_get_valid(png, info, PNG_INFO_tRNS) != 0) {
        png_set_tRNS_to_alpha(png);
    }
    if (color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_GRAY_ALPHA) {
        png_set_gray_to_rgb(png);
    }
    if (color_type == PNG_COLOR_TYPE_RGB || color_type == PNG_COLOR_TYPE_GRAY || color_type == PNG_COLOR_TYPE_PALETTE) {
        png_set_add_alpha(png, 0xFF, PNG_FILLER_AFTER);
    }

    png_read_update_info(png, info);

    decoded->width = static_cast<int>(width);
    decoded->height = static_cast<int>(height);
    decoded->rgba.assign(static_cast<size_t>(width) * static_cast<size_t>(height) * 4U, 0);

    vector<png_bytep> row_pointers(height);
    for (png_uint_32 row = 0; row < height; ++row) {
        row_pointers[row] = decoded->rgba.data() + static_cast<size_t>(row) * static_cast<size_t>(width) * 4U;
    }

    png_read_image(png, row_pointers.data());
    png_read_end(png, nullptr);

    png_destroy_read_struct(&png, &info, nullptr);
    fclose(file);
    return true;
}

GLuint upload_rgba_texture(int width, int height, const vector<unsigned char>& rgba) {
    GLuint texture_id = 0;
    glGenTextures(1, &texture_id);
    glBindTexture(GL_TEXTURE_2D, texture_id);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_RGBA,
                 width,
                 height,
                 0,
                 GL_RGBA,
                 GL_UNSIGNED_BYTE,
                 rgba.data());
    glBindTexture(GL_TEXTURE_2D, 0);
    return texture_id;
}

GLuint upload_rgba_texture(const DecodedImage& decoded) {
    return upload_rgba_texture(decoded.width, decoded.height, decoded.rgba);
}

bool load_png_texture(const filesystem::path& path,
                      GLuint* texture_id,
                      int* width,
                      int* height,
                      string* error_message) {
    if (texture_id == nullptr || width == nullptr || height == nullptr) {
        return false;
    }
    DecodedImage decoded;
    if (!decode_png_rgba(path, &decoded, error_message)) {
        return false;
    }
    if (*texture_id != 0) {
        glDeleteTextures(1, texture_id);
        *texture_id = 0;
    }
    *texture_id = upload_rgba_texture(decoded);
    *width = decoded.width;
    *height = decoded.height;
    return true;
}

class TileManager {
public:
    TileManager()
        : cache_root_(resolve_osm_cache_root()) {
        error_code ec;
        filesystem::create_directories(cache_root_, ec);
        workers_.reserve(kTileWorkerCount);
        for (int i = 0; i < kTileWorkerCount; ++i) {
            workers_.emplace_back(&TileManager::fetch_worker, this, i);
        }
    }

    ~TileManager() {
        shutdown_workers();
        clear();
    }

    void shutdown_workers() {
        {
            lock_guard<mutex> lock(job_mutex_);
            if (stopping_) {
                return;
            }
            stopping_ = true;
            for (auto& queue : job_queues_) {
                std::queue<TileJob> empty;
                swap(queue, empty);
            }
        }
        job_cv_.notify_all();
        for (thread& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void clear() {
        lock_guard<mutex> lock(cache_mutex_);
        for (auto& [_, tile] : tiles_) {
            if (tile.texture_id != 0) {
                glDeleteTextures(1, &tile.texture_id);
                tile.texture_id = 0;
            }
        }
        tiles_.clear();
    }

    const filesystem::path& cache_root() const {
        return cache_root_;
    }

    string last_status() const {
        lock_guard<mutex> lock(cache_mutex_);
        return last_status_;
    }

    void reset_pending_jobs() {
        {
            lock_guard<mutex> lock(job_mutex_);
            for (auto& queue : job_queues_) {
                std::queue<TileJob> empty;
                swap(queue, empty);
            }
        }

        lock_guard<mutex> lock(cache_mutex_);
        for (auto& [_, tile] : tiles_) {
            if (tile.texture_id == 0 && tile.rgba_blob.empty()) {
                tile.is_loading = false;
            }
        }
        last_status_ = "Tile queue reset for new zoom";
    }

    GLuint get_or_enqueue_tile(int zoom, int tile_x, int tile_y, int worker_index) {
        if (tile_x < 0 || tile_y < 0 ||
            tile_x >= tile_count_for_zoom(zoom) ||
            tile_y >= tile_count_for_zoom(zoom)) {
            return 0;
        }
        worker_index = clamp(worker_index, 0, kTileWorkerCount - 1);

        const string key = tile_key(zoom, tile_x, tile_y);
        vector<unsigned char> rgba_blob;
        int width = 0;
        int height = 0;
        bool should_enqueue = false;

        {
            lock_guard<mutex> lock(cache_mutex_);
            TileTexture& tile = tiles_[key];
            if (tile.file_path.empty()) {
                tile.file_path = tile_file_path(zoom, tile_x, tile_y);
            }

            if (tile.texture_id != 0) {
                return tile.texture_id;
            }

            const auto now = chrono::steady_clock::now();
            if (!tile.last_error.empty() && now < tile.next_retry_at) {
                return 0;
            }

            if (!tile.rgba_blob.empty()) {
                width = tile.width;
                height = tile.height;
                rgba_blob.swap(tile.rgba_blob);
            } else if (!tile.is_loading) {
                tile.is_loading = true;
                should_enqueue = true;
            } else {
                return 0;
            }
        }

        if (should_enqueue) {
            enqueue_job({key, zoom, tile_x, tile_y}, worker_index);
            return 0;
        }

        const GLuint texture_id = upload_rgba_texture(width, height, rgba_blob);

        {
            lock_guard<mutex> lock(cache_mutex_);
            TileTexture& tile = tiles_[key];
            tile.texture_id = texture_id;
            tile.is_loading = false;
        }

        return texture_id;
    }

private:
    static string tile_key(int zoom, int x, int y) {
        return to_string(zoom) + "/" + to_string(x) + "/" + to_string(y);
    }

    filesystem::path tile_file_path(int zoom, int x, int y) const {
        return cache_root_ / to_string(zoom) / to_string(x) / (to_string(y) + ".png");
    }

    void enqueue_job(const TileJob& job, int worker_index) {
        {
            lock_guard<mutex> lock(job_mutex_);
            if (stopping_) {
                return;
            }
            job_queues_[worker_index].push(job);
        }
        job_cv_.notify_all();
    }

    void fetch_worker(int worker_index) {
        while (true) {
            TileJob job;
            {
                unique_lock<mutex> lock(job_mutex_);
                job_cv_.wait(lock, [this, worker_index]() {
                    return stopping_ || !job_queues_[worker_index].empty();
                });

                if (stopping_ && job_queues_[worker_index].empty()) {
                    return;
                }

                job = job_queues_[worker_index].front();
                job_queues_[worker_index].pop();
            }

            fetch_tile(job);
        }
    }

    void fetch_tile(const TileJob& job) {
        const filesystem::path path = tile_file_path(job.zoom, job.x, job.y);
        string error_message;

        if (!filesystem::exists(path) &&
            !download_tile(path, job.zoom, job.x, job.y, &error_message)) {
            mark_tile_failed(job.id, error_message);
            return;
        }

        DecodedImage decoded;
        if (!decode_png_rgba(path, &decoded, &error_message)) {
            error_code ec;
            filesystem::remove(path, ec);
            if (!download_tile(path, job.zoom, job.x, job.y, &error_message) ||
                !decode_png_rgba(path, &decoded, &error_message)) {
                mark_tile_failed(job.id, error_message);
                return;
            }
        }

        {
            lock_guard<mutex> lock(cache_mutex_);
            TileTexture& tile = tiles_[job.id];
            tile.file_path = path;
            tile.width = decoded.width;
            tile.height = decoded.height;
            tile.rgba_blob = std::move(decoded.rgba);
            tile.is_loading = false;
            tile.last_error.clear();
            tile.next_retry_at = {};
            last_status_ = "Tile decoded: " + job.id;
        }
    }

    void mark_tile_failed(const string& id, const string& error_message) {
        lock_guard<mutex> lock(cache_mutex_);
        TileTexture& tile = tiles_[id];
        tile.is_loading = false;
        tile.last_error = error_message;
        tile.next_retry_at = chrono::steady_clock::now() + kTileRetryDelay;
        last_status_ = error_message;
    }

    bool download_tile(const filesystem::path& destination,
                       int zoom,
                       int x,
                       int y,
                       string* error_message) const {
        error_code ec;
        filesystem::create_directories(destination.parent_path(), ec);
        if (ec) {
            if (error_message != nullptr) {
                *error_message = "failed to create cache directory: " + destination.parent_path().string();
            }
            return false;
        }

        const filesystem::path temp_file = destination.string() + ".part";
        filesystem::remove(temp_file, ec);

        const string url = "https://tile.openstreetmap.org/" +
                           to_string(zoom) + "/" +
                           to_string(x) + "/" +
                           to_string(y) + ".png";

        const string command =
            "curl -L --fail --silent --connect-timeout 8 --max-time 20 --retry 1 "
            "-A \"backend-server-android-osm-imgui/1.0\" -o " +
            shell_escape(temp_file.string()) + " " +
            shell_escape(url);

        const int result = system(command.c_str());
        if (result != 0 || !filesystem::exists(temp_file)) {
            filesystem::remove(temp_file, ec);
            if (error_message != nullptr) {
                *error_message = "curl failed for " + url;
            }
            return false;
        }

        filesystem::rename(temp_file, destination, ec);
        if (ec) {
            filesystem::remove(temp_file, ec);
            if (error_message != nullptr) {
                *error_message = "failed to move tile into cache: " + destination.string();
            }
            return false;
        }

        return true;
    }

    filesystem::path cache_root_;
    unordered_map<string, TileTexture> tiles_;
    array<queue<TileJob>, kTileWorkerCount> job_queues_;
    vector<thread> workers_;
    mutable mutex cache_mutex_;
    mutex job_mutex_;
    condition_variable job_cv_;
    string last_status_;
    bool stopping_ = false;
};

struct location {
    float latitude = 55.0292f;
    float longitude = 82.9221f;
    float altitude = 0.0f;
    float accuracy = 0.0f;
    long long int time_j = 0;
    string allcellinfo;

    int signalDbm = -999;
    long long rx = 0;
    long long tx = 0;

    vector<float> signal_x;
    vector<float> signal_y;

    atomic<long long int> cnt = 0;
    mutex m;
};

LocationSnapshot snapshot_location(location* loc) {
    LocationSnapshot snapshot;
    {
        lock_guard<mutex> lk(loc->m);
        snapshot.latitude = loc->latitude;
        snapshot.longitude = loc->longitude;
        snapshot.altitude = loc->altitude;
        snapshot.accuracy = loc->accuracy;
        snapshot.time_j = loc->time_j;
        snapshot.allcellinfo = loc->allcellinfo;
        snapshot.signalDbm = loc->signalDbm;
        snapshot.rx = loc->rx;
        snapshot.tx = loc->tx;
        snapshot.signal_x = loc->signal_x;
        snapshot.signal_y = loc->signal_y;
    }
    snapshot.cnt = static_cast<unsigned long long>(loc->cnt.load());
    return snapshot;
}

void request_stop(int) {
    g_keep_running.store(false);
}

bool parse_number(const json& value, double* out) {
    if (value.is_number()) {
        *out = value.get<double>();
        return true;
    }

    if (value.is_string()) {
        try {
            size_t consumed = 0;
            const string raw = value.get<string>();
            const double parsed = stod(raw, &consumed);
            if (consumed == raw.size()) {
                *out = parsed;
                return true;
            }
        } catch (const exception&) {
            return false;
        }
    }

    return false;
}

bool read_number(const json& object, initializer_list<const char*> keys, double* out) {
    for (const char* key : keys) {
        const auto it = object.find(key);
        if (it != object.end() && !it->is_null() && parse_number(*it, out)) {
            return true;
        }
    }
    return false;
}

bool read_valid_signal_number(const json& object, initializer_list<const char*> keys, double* out) {
    double value = numeric_limits<double>::quiet_NaN();
    if (!read_number(object, keys, &value) || !isfinite(value) || abs(value) >= 100000.0) {
        return false;
    }
    *out = value;
    return true;
}

bool read_int64(const json& object, initializer_list<const char*> keys, long long* out) {
    double value = 0.0;
    if (!read_number(object, keys, &value)) {
        return false;
    }
    *out = static_cast<long long>(value);
    return true;
}

string read_string(const json& object, initializer_list<const char*> keys, const string& fallback = "") {
    for (const char* key : keys) {
        const auto it = object.find(key);
        if (it == object.end() || it->is_null()) {
            continue;
        }
        if (it->is_string()) {
            return it->get<string>();
        }
        return it->dump();
    }
    return fallback;
}

string lower_ascii(string value) {
    transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(tolower(ch));
    });
    return value;
}

bool is_token_boundary(char ch) {
    return !isalnum(static_cast<unsigned char>(ch)) && ch != '_';
}

bool parse_cellinfo_number(const string& cell_info, const string& key, double* out) {
    const string haystack = lower_ascii(cell_info);
    const string needle = lower_ascii(key);
    size_t pos = 0;

    while ((pos = haystack.find(needle, pos)) != string::npos) {
        const size_t before = pos;
        size_t after = pos + needle.size();
        if (before > 0 && !is_token_boundary(haystack[before - 1])) {
            pos = after;
            continue;
        }
        while (after < haystack.size() && isspace(static_cast<unsigned char>(haystack[after]))) {
            ++after;
        }
        if (after >= haystack.size() || haystack[after] != '=') {
            pos = after;
            continue;
        }
        ++after;
        while (after < haystack.size() && isspace(static_cast<unsigned char>(haystack[after]))) {
            ++after;
        }

        char* end = nullptr;
        const double value = strtod(cell_info.c_str() + after, &end);
        if (cell_info.c_str() + after != end) {
            *out = value;
            return true;
        }
        pos = after;
    }

    return false;
}

bool parse_cellinfo_int(const string& cell_info, const string& key, int* out) {
    double value = 0.0;
    if (!parse_cellinfo_number(cell_info, key, &value)) {
        return false;
    }
    *out = static_cast<int>(value);
    return true;
}

bool parse_first_cellinfo_number(const string& cell_info,
                                 initializer_list<const char*> keys,
                                 double* out) {
    for (const char* key : keys) {
        double value = numeric_limits<double>::quiet_NaN();
        if (parse_cellinfo_number(cell_info, key, &value) &&
            isfinite(value) &&
            abs(value) < 100000.0) {
            *out = value;
            return true;
        }
    }
    return false;
}

bool parse_first_cellinfo_int(const string& cell_info,
                              initializer_list<const char*> keys,
                              int* out) {
    for (const char* key : keys) {
        if (parse_cellinfo_int(cell_info, key, out)) {
            return true;
        }
    }
    return false;
}

bool read_cells_array(const json& object, json* out_cells) {
    if (out_cells == nullptr || !object.is_object()) {
        return false;
    }

    const auto cells_it = object.find("cells");
    if (cells_it != object.end() && cells_it->is_array()) {
        *out_cells = *cells_it;
        return true;
    }

    const string cell_info = read_string(object, {"Cellallinfo", "cell_info", "cellInfo", "allcellinfo"}, "");
    size_t first = 0;
    while (first < cell_info.size() && isspace(static_cast<unsigned char>(cell_info[first]))) {
        ++first;
    }
    if (first >= cell_info.size() || (cell_info[first] != '[' && cell_info[first] != '{')) {
        return false;
    }

    try {
        const json parsed = json::parse(cell_info);
        if (parsed.is_array()) {
            *out_cells = parsed;
            return true;
        }
        if (parsed.is_object()) {
            const auto nested_cells = parsed.find("cells");
            if (nested_cells != parsed.end() && nested_cells->is_array()) {
                *out_cells = *nested_cells;
                return true;
            }
            *out_cells = json::array({parsed});
            return true;
        }
    } catch (const exception&) {
        return false;
    }

    return false;
}

void normalize_coordinate_pair(double* latitude, double* longitude) {
    if (latitude == nullptr || longitude == nullptr) {
        return;
    }

    if (abs(*latitude) > kMaxLatitude && abs(*longitude) <= kMaxLatitude) {
        swap(*latitude, *longitude);
        return;
    }

    if (*latitude >= 70.0 && *latitude <= kMaxLatitude &&
        *longitude >= 45.0 && *longitude <= 65.0) {
        swap(*latitude, *longitude);
    }
}

double metric_value(const HeatmapPoint& point, HeatmapCriterion criterion) {
    switch (criterion) {
        case HeatmapCriterion::RSRP: return point.rsrp;
        case HeatmapCriterion::RSRQ: return point.rsrq;
        case HeatmapCriterion::RSSI: return point.rssi;
        case HeatmapCriterion::Altitude: return point.altitude;
    }
    return numeric_limits<double>::quiet_NaN();
}

bool is_valid_metric_value(double value, HeatmapCriterion criterion) {
    if (!isfinite(value) || abs(value) >= 100000.0) {
        return false;
    }

    switch (criterion) {
        case HeatmapCriterion::RSRP:
            return value >= -160.0 && value <= 0.0;
        case HeatmapCriterion::RSRQ:
            return value >= -50.0 && value <= 0.0;
        case HeatmapCriterion::RSSI:
            return value >= -160.0 && value <= 0.0;
        case HeatmapCriterion::Altitude:
            return value > -500.0 && value < 10000.0;
    }
    return false;
}

static double clamp01(double value) {
    if (value < 0.0) {
        return 0.0;
    }
    if (value > 1.0) {
        return 1.0;
    }
    return value;
}

static double smoothstep(double edge0, double edge1, double x) {
    const double t = clamp01((x - edge0) / max(1e-9, edge1 - edge0));
    return t * t * (3.0 - 2.0 * t);
}

static double dbm_to_milliwatts(double dbm) {
    return pow(10.0, dbm / 10.0);
}

static double milliwatts_to_dbm(double milliwatts) {
    return 10.0 * log10(max(1e-12, milliwatts));
}

static double db_to_linear(double db) {
    return pow(10.0, db / 10.0);
}

static double linear_to_db(double linear) {
    return 10.0 * log10(max(1e-12, linear));
}

struct HeatmapMetricConfig {
    double min_value = 0.0;
    double max_value = 1.0;
    double transparent_below = numeric_limits<double>::lowest();
    bool use_dynamic_range = false;
};

static HeatmapMetricConfig heatmap_metric_config(HeatmapCriterion criterion) {
    switch (criterion) {
        case HeatmapCriterion::RSRP:
            return {-110.0, -80.0, -110.0, false};
        case HeatmapCriterion::RSRQ:
            return {-20.0, -3.0, -20.0, false};
        case HeatmapCriterion::RSSI:
            return {-110.0, -65.0, -110.0, false};
        case HeatmapCriterion::Altitude:
            return {0.0, 1.0, numeric_limits<double>::lowest(), true};
    }
    return {-110.0, -80.0, -110.0, false};
}

static bool is_logarithmic_heatmap_metric(HeatmapCriterion criterion) {
    return criterion == HeatmapCriterion::RSRP ||
           criterion == HeatmapCriterion::RSRQ ||
           criterion == HeatmapCriterion::RSSI;
}

static void colorize_heatmap(double ratio,
                             float opacity,
                             unsigned char* out_r,
                             unsigned char* out_g,
                             unsigned char* out_b,
                             unsigned char* out_a) {
    struct ColorStop {
        double ratio;
        double r;
        double g;
        double b;
    };

    static const ColorStop stops[] = {
        {0.0, 10.0, 20.0, 120.0},
        {0.35, 0.0, 120.0, 255.0},
        {0.7, 255.0, 200.0, 0.0},
        {1.0, 255.0, 0.0, 0.0},
    };

    const double normalized_ratio = clamp01(ratio);
    const ColorStop* left = &stops[0];
    const ColorStop* right = &stops[3];
    for (int i = 1; i < 4; ++i) {
        if (normalized_ratio <= stops[i].ratio) {
            left = &stops[i - 1];
            right = &stops[i];
            break;
        }
    }

    const double segment = max(1e-9, right->ratio - left->ratio);
    const double local_ratio = (normalized_ratio - left->ratio) / segment;
    *out_r = static_cast<unsigned char>(clamp(left->r + (right->r - left->r) * local_ratio, 0.0, 255.0));
    *out_g = static_cast<unsigned char>(clamp(left->g + (right->g - left->g) * local_ratio, 0.0, 255.0));
    *out_b = static_cast<unsigned char>(clamp(left->b + (right->b - left->b) * local_ratio, 0.0, 255.0));
    *out_a = static_cast<unsigned char>(clamp(static_cast<double>(opacity) * 255.0, 0.0, 255.0));
}

static vector<int> available_heatmap_earfcns(const vector<HeatmapPoint>& points, HeatmapCriterion criterion) {
    vector<int> result;
    for (const auto& point : points) {
        if (point.earfcn <= 0 || !is_valid_metric_value(metric_value(point, criterion), criterion)) {
            continue;
        }
        if (find(result.begin(), result.end(), point.earfcn) == result.end()) {
            result.push_back(point.earfcn);
        }
    }
    sort(result.begin(), result.end());
    return result;
}

static void ensure_selected_heatmap_earfcn(HeatmapOptions* options, const vector<HeatmapPoint>& points) {
    if (options == nullptr) {
        return;
    }

    const vector<int> earfcns = available_heatmap_earfcns(points, options->criterion);
    if (earfcns.empty() || options->selected_earfcn <= 0) {
        options->selected_earfcn = -1;
        return;
    }

    if (find(earfcns.begin(), earfcns.end(), options->selected_earfcn) == earfcns.end()) {
        options->selected_earfcn = -1;
    }
}

bool load_heatmap_point(const json& object, HeatmapPoint* point) {
    if (!object.is_object()) {
        return false;
    }
    double latitude = 0.0;
    double longitude = 0.0;
    if (!read_number(object, {"lat", "latitude"}, &latitude) ||
        !read_number(object, {"lon", "lng", "longitude"}, &longitude)) {
        return false;
    }
    normalize_coordinate_pair(&latitude, &longitude);

    double altitude = 0.0;
    double accuracy = 0.0;
    read_number(object, {"alt", "altitude"}, &altitude);
    read_number(object, {"accuracy"}, &accuracy);

    double rsrp = numeric_limits<double>::quiet_NaN();
    double rsrq = numeric_limits<double>::quiet_NaN();
    double rssi = numeric_limits<double>::quiet_NaN();
    double signal_dbm = numeric_limits<double>::quiet_NaN();
    int earfcn = -1;

    read_valid_signal_number(object, {"signalDbm", "signal_dbm", "dbm"}, &signal_dbm);
    read_valid_signal_number(object, {"rsrp", "RSRP", "ssRsrp", "csiRsrp"}, &rsrp);
    read_valid_signal_number(object, {"rsrq", "RSRQ", "ssRsrq", "csiRsrq"}, &rsrq);
    read_valid_signal_number(object, {"rssi", "RSSI"}, &rssi);

    const string cell_allinfo = read_string(object, {"Cellallinfo", "cell_info", "cellInfo", "allcellinfo"}, "");
    parse_first_cellinfo_number(cell_allinfo, {"rsrp", "ssRsrp", "csiRsrp"}, &rsrp);
    parse_first_cellinfo_number(cell_allinfo, {"rsrq", "ssRsrq", "csiRsrq"}, &rsrq);
    if (!parse_first_cellinfo_number(cell_allinfo, {"rssi", "ssRssi"}, &rssi)) {
        rssi = signal_dbm;
    }
    double earfcn_value = numeric_limits<double>::quiet_NaN();
    if (read_valid_signal_number(object, {"earfcn", "EARFCN", "mEarfcn", "mNrArfcn", "nrArfcn"}, &earfcn_value)) {
        earfcn = static_cast<int>(earfcn_value);
    }
    parse_first_cellinfo_int(cell_allinfo, {"mEarfcn", "earfcn", "mNrArfcn", "nrArfcn"}, &earfcn);

    point->latitude = latitude;
    point->longitude = longitude;
    point->altitude = altitude;
    point->accuracy_m = accuracy;
    point->rsrp = rsrp;
    point->rsrq = rsrq;
    point->rssi = rssi;
    point->earfcn = earfcn;
    return true;
}

bool apply_cell_object_to_heatmap_point(const json& cell, HeatmapPoint* point) {
    if (!cell.is_object() || point == nullptr) {
        return false;
    }

    bool has_signal = false;
    double value = numeric_limits<double>::quiet_NaN();
    if (read_valid_signal_number(cell, {"rsrp", "RSRP", "ssRsrp", "csiRsrp"}, &value)) {
        point->rsrp = value;
        has_signal = true;
    }
    if (read_valid_signal_number(cell, {"rsrq", "RSRQ", "ssRsrq", "csiRsrq"}, &value)) {
        point->rsrq = value;
        has_signal = true;
    }
    if (read_valid_signal_number(cell, {"rssi", "RSSI", "ssRssi"}, &value)) {
        point->rssi = value;
        has_signal = true;
    }
    if (read_valid_signal_number(cell, {"signalDbm", "signal_dbm", "dbm"}, &value) &&
        !is_valid_metric_value(point->rssi, HeatmapCriterion::RSSI)) {
        point->rssi = value;
        has_signal = true;
    }

    double earfcn = numeric_limits<double>::quiet_NaN();
    if (read_valid_signal_number(cell, {"earfcn", "EARFCN", "mEarfcn", "mNrArfcn", "nrArfcn"}, &earfcn) &&
        earfcn > 0.0) {
        point->earfcn = static_cast<int>(earfcn);
    }

    return has_signal;
}

bool heatmap_point_has_any_metric(const HeatmapPoint& point) {
    return is_valid_metric_value(point.rsrp, HeatmapCriterion::RSRP) ||
           is_valid_metric_value(point.rsrq, HeatmapCriterion::RSRQ) ||
           is_valid_metric_value(point.rssi, HeatmapCriterion::RSSI) ||
           is_valid_metric_value(point.altitude, HeatmapCriterion::Altitude);
}

bool append_heatmap_points_from_json(const json& object, vector<HeatmapPoint>* out_points) {
    if (out_points == nullptr) {
        return false;
    }

    HeatmapPoint base;
    if (!load_heatmap_point(object, &base)) {
        return false;
    }

    json cells;
    if (!read_cells_array(object, &cells) || cells.empty()) {
        if (heatmap_point_has_any_metric(base)) {
            out_points->push_back(base);
            return true;
        }
        return false;
    }

    const size_t before = out_points->size();
    for (const auto& cell : cells) {
        HeatmapPoint point = base;
        apply_cell_object_to_heatmap_point(cell, &point);
        if (heatmap_point_has_any_metric(point)) {
            out_points->push_back(point);
        }
    }
    return out_points->size() > before;
}

bool load_heatmap_points(const filesystem::path& path, vector<HeatmapPoint>* out_points, string* error_message) {
    if (out_points == nullptr) {
        return false;
    }
    out_points->clear();

    ifstream input(path);
    if (!input.is_open()) {
        if (error_message) {
            *error_message = "Failed to open heatmap data file: " + path.string();
        }
        return false;
    }

    string content;
    content.assign(istreambuf_iterator<char>(input), istreambuf_iterator<char>());
    input.close();
    if (content.empty()) {
        if (error_message) {
            *error_message = "Heatmap data file is empty: " + path.string();
        }
        return false;
    }

    try {
        size_t index = 0;
        while (index < content.size() && isspace((unsigned char)content[index])) {
            ++index;
        }
        if (index >= content.size()) {
            if (error_message) {
                *error_message = "Heatmap data file contains no JSON data";
            }
            return false;
        }

        if (content[index] == '[') {
            const json root = json::parse(content);
            if (!root.is_array()) {
                if (error_message) {
                    *error_message = "Heatmap data JSON root must be an array";
                }
                return false;
            }
            for (const auto& item : root) {
                append_heatmap_points_from_json(item, out_points);
            }
        } else {
            istringstream stream(content);
            string line;
            while (getline(stream, line)) {
                if (line.empty()) {
                    continue;
                }
                const json item = json::parse(line);
                append_heatmap_points_from_json(item, out_points);
            }
        }
    } catch (const exception& e) {
        if (error_message) {
            *error_message = string("Failed to parse heatmap JSON: ") + e.what();
        }
        return false;
    }

    return !out_points->empty();
}

static double degrees_to_radians(double degrees) {
    return degrees * (kPi / 180.0);
}

static double haversine_distance_m(double lat1, double lon1, double lat2, double lon2) {
    const double r = 6371000.0;
    const double dlat = degrees_to_radians(lat2 - lat1);
    const double dlon = degrees_to_radians(lon2 - lon1);
    const double a = sin(dlat / 2.0) * sin(dlat / 2.0) +
                     cos(degrees_to_radians(lat1)) * cos(degrees_to_radians(lat2)) *
                     sin(dlon / 2.0) * sin(dlon / 2.0);
    const double c = 2.0 * atan2(sqrt(a), sqrt(max(0.0, 1.0 - a)));
    return r * c;
}

static double mercator_y_to_lat(double mercator_y) {
    const double y_rad = mercator_y * kPi / 180.0;
    const double lat_rad = 2.0 * atan(exp(y_rad)) - kPi / 2.0;
    return lat_rad * 180.0 / kPi;
}

static double longitude_to_local_x_m(double longitude, double cos_reference_latitude) {
    return degrees_to_radians(longitude) * 6371000.0 * cos_reference_latitude;
}

static double latitude_to_local_y_m(double latitude) {
    return degrees_to_radians(latitude) * 6371000.0;
}

static uint64_t heatmap_grid_key(int cell_x, int cell_y) {
    const uint64_t x = static_cast<uint32_t>(cell_x);
    const uint64_t y = static_cast<uint32_t>(cell_y);
    return (x << 32U) | y;
}

static ImVec4 heatmap_color_for_value(double value, HeatmapCriterion criterion, double min_value, double max_value) {
    if (criterion == HeatmapCriterion::RSRP) {
        if (value < -110.0) {
            return ImVec4(0, 0, 0, 0);
        }
        if (value >= -80.0) {
            return ImVec4(1.0f, 0.0f, 0.0f, 1.0f);
        }
        if (value >= -90.0) {
            const float t = static_cast<float>((value + 90.0) / 10.0);
            return ImVec4(1.0f, 0.4f + 0.6f * t, 0.0f, 1.0f);
        }
        if (value >= -100.0) {
            const float t = static_cast<float>((value + 100.0) / 10.0);
            return ImVec4(0.0f + 0.5f * t, 0.4f + 0.6f * t, 1.0f - 0.5f * t, 1.0f);
        }
        const float t = static_cast<float>((value + 110.0) / 10.0);
        return ImVec4(0.0f, 0.0f, 0.4f + 0.5f * t, 1.0f);
    }

    const double v = max(min(value, max_value), min_value);
    const double normalized = (v - min_value) / max(1e-6, max_value - min_value);
    const float t = static_cast<float>(normalized);
    return ImVec4(t, 0.25f + 0.75f * t, 1.0f - t, 1.0f);
}

static bool write_png(const filesystem::path& file_path,
                      int width,
                      int height,
                      const vector<unsigned char>& rgba,
                      string* error_message) {
    FILE* file = fopen(file_path.string().c_str(), "wb");
    if (!file) {
        if (error_message) {
            *error_message = "Failed to open PNG for writing: " + file_path.string();
        }
        return false;
    }

    png_structp png = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png) {
        fclose(file);
        if (error_message) {
            *error_message = "png_create_write_struct failed";
        }
        return false;
    }

    png_infop info = png_create_info_struct(png);
    if (!info) {
        png_destroy_write_struct(&png, nullptr);
        fclose(file);
        if (error_message) {
            *error_message = "png_create_info_struct failed";
        }
        return false;
    }

    if (setjmp(png_jmpbuf(png)) != 0) {
        png_destroy_write_struct(&png, &info);
        fclose(file);
        if (error_message) {
            *error_message = "libpng write failed";
        }
        return false;
    }

    png_init_io(png, file);
    png_set_IHDR(png, info,
                 width,
                 height,
                 8,
                 PNG_COLOR_TYPE_RGBA,
                 PNG_INTERLACE_NONE,
                 PNG_COMPRESSION_TYPE_DEFAULT,
                 PNG_FILTER_TYPE_DEFAULT);
    png_write_info(png, info);

    vector<png_bytep> rows(height);
    for (int y = 0; y < height; ++y) {
        rows[y] = const_cast<png_bytep>(rgba.data() + static_cast<size_t>(y) * static_cast<size_t>(width) * 4);
    }
    png_write_image(png, rows.data());
    png_write_end(png, nullptr);

    png_destroy_write_struct(&png, &info);
    fclose(file);
    return true;
}

static bool build_heatmap_image(const vector<HeatmapPoint>& points,
                                HeatmapCriterion criterion,
                                double radius_m,
                                int width,
                                int height,
                                double min_lon,
                                double max_lon,
                                double min_merc_y,
                                double max_merc_y,
                                vector<unsigned char>* out_rgba,
                                double display_radius_m = 30.0,
                                double power = 2.0,
                                double opacity = 0.75,
                                int selected_earfcn = -1) {
    if (points.empty() || out_rgba == nullptr || width <= 0 || height <= 0) {
        return false;
    }
    out_rgba->assign(static_cast<size_t>(width) * static_cast<size_t>(height) * 4, 0);

    const double min_lat = mercator_y_to_lat(min_merc_y);
    const double max_lat = mercator_y_to_lat(max_merc_y);
    const double center_lat = (min_lat + max_lat) * 0.5;
    const double idw_radius = clamp(radius_m, 10.0, 40.0);
    const double draw_radius = max(1.0, display_radius_m);
    double max_sample_radius = max(idw_radius, draw_radius);
    const double cos_reference_latitude = max(0.1, cos(center_lat * kPi / 180.0));

    for (const auto& point : points) {
        if (selected_earfcn > 0 && point.earfcn != selected_earfcn) {
            continue;
        }
        if (isfinite(point.accuracy_m) && point.accuracy_m > 0.0) {
            max_sample_radius = max(max_sample_radius, min(point.accuracy_m, draw_radius));
        }
    }

    const double lat_margin = max_sample_radius / 111320.0;
    const double lon_margin = max_sample_radius / (111320.0 * cos_reference_latitude);

    vector<const HeatmapPoint*> candidates;
    candidates.reserve(points.size());

    HeatmapMetricConfig config = heatmap_metric_config(criterion);
    double dynamic_min = numeric_limits<double>::infinity();
    double dynamic_max = -numeric_limits<double>::infinity();
    for (const auto& point : points) {
        if (selected_earfcn > 0 && point.earfcn != selected_earfcn) {
            continue;
        }
        if (point.latitude < min_lat - lat_margin || point.latitude > max_lat + lat_margin ||
            point.longitude < min_lon - lon_margin || point.longitude > max_lon + lon_margin) {
            continue;
        }

        const double value = metric_value(point, criterion);
        if (!is_valid_metric_value(value, criterion)) {
            continue;
        }

        candidates.push_back(&point);
        if (config.use_dynamic_range) {
            dynamic_min = min(dynamic_min, value);
            dynamic_max = max(dynamic_max, value);
        }
    }

    if (candidates.empty()) {
        return false;
    }
    if (config.use_dynamic_range) {
        if (!isfinite(dynamic_min) || !isfinite(dynamic_max)) {
            return false;
        }
        if (abs(dynamic_max - dynamic_min) < 1e-6) {
            dynamic_min -= 1.0;
            dynamic_max += 1.0;
        }
        config.min_value = dynamic_min;
        config.max_value = dynamic_max;
    }

    const bool logarithmic = is_logarithmic_heatmap_metric(criterion);
    const double lon_range = max(1e-6, max_lon - min_lon);
    const double merc_range = max(1e-6, max_merc_y - min_merc_y);
    const double safe_power = max(1.0, power * 0.7);
    struct IndexedCandidate {
        const HeatmapPoint* point = nullptr;
        double x_m = 0.0;
        double y_m = 0.0;
        double radius_m = 1.0;
    };

    vector<IndexedCandidate> indexed_candidates;
    indexed_candidates.reserve(candidates.size());
    unordered_map<uint64_t, vector<size_t>> grid;
    const double grid_cell_size = max(10.0, max_sample_radius);
    for (const HeatmapPoint* point : candidates) {
        const double accuracy_radius = (isfinite(point->accuracy_m) && point->accuracy_m > 0.0)
            ? min(point->accuracy_m, draw_radius)
            : draw_radius;
        const double sample_radius = max(1.0, max(draw_radius, accuracy_radius));
        const double sample_x_m = longitude_to_local_x_m(point->longitude, cos_reference_latitude);
        const double sample_y_m = latitude_to_local_y_m(point->latitude);
        const int cell_x = static_cast<int>(floor(sample_x_m / grid_cell_size));
        const int cell_y = static_cast<int>(floor(sample_y_m / grid_cell_size));
        const size_t sample_index = indexed_candidates.size();
        indexed_candidates.push_back({point, sample_x_m, sample_y_m, sample_radius});
        grid[heatmap_grid_key(cell_x, cell_y)].push_back(sample_index);
    }
    const int grid_search_radius = max(1, static_cast<int>(ceil(max_sample_radius / grid_cell_size)));

    bool has_visible_pixels = false;
    for (int y = 0; y < height; ++y) {
        const double merc_y = max_merc_y - (static_cast<double>(y) + 0.5) / height * merc_range;
        const double lat = mercator_y_to_lat(merc_y);
        for (int x = 0; x < width; ++x) {
            const double lon = min_lon + (static_cast<double>(x) + 0.5) / width * lon_range;
            const double pixel_x_m = longitude_to_local_x_m(lon, cos_reference_latitude);
            const double pixel_y_m = latitude_to_local_y_m(lat);
            const int pixel_cell_x = static_cast<int>(floor(pixel_x_m / grid_cell_size));
            const int pixel_cell_y = static_cast<int>(floor(pixel_y_m / grid_cell_size));
            double weighted_sum = 0.0;
            double total_weight = 0.0;
            double influence_sum = 0.0;
            double nearest_edge_ratio = 1.0;
            bool direct_hit = false;
            bool has_sample_in_display_radius = false;
            double interpolated_value = 0.0;

            for (int grid_y = pixel_cell_y - grid_search_radius; grid_y <= pixel_cell_y + grid_search_radius && !direct_hit; ++grid_y) {
                for (int grid_x = pixel_cell_x - grid_search_radius; grid_x <= pixel_cell_x + grid_search_radius && !direct_hit; ++grid_x) {
                    const auto bucket = grid.find(heatmap_grid_key(grid_x, grid_y));
                    if (bucket == grid.end()) {
                        continue;
                    }

                    for (size_t sample_index : bucket->second) {
                        const IndexedCandidate& sample = indexed_candidates[sample_index];
                        const double dx = pixel_x_m - sample.x_m;
                        const double dy = pixel_y_m - sample.y_m;
                        const double distance = sqrt(dx * dx + dy * dy);
                        const double sample_radius = sample.radius_m;
                        if (distance > sample_radius) {
                            continue;
                        }

                        nearest_edge_ratio = min(nearest_edge_ratio, distance / sample_radius);
                        has_sample_in_display_radius = true;
                        influence_sum += max(0.0, 1.0 - distance / sample_radius);

                        const double value = metric_value(*sample.point, criterion);
                        if (distance < 0.5) {
                            interpolated_value = value;
                            direct_hit = true;
                            break;
                        }

                        const double softened_distance = max(1.0, distance);
                        const double radial_weight = max(0.05, 1.0 - distance / sample_radius);
                        const double idw_edge_ratio = min(distance, idw_radius) / max(1.0, idw_radius);
                        const double idw_fade = max(0.05, 1.0 - idw_edge_ratio);
                        const double weight = radial_weight * idw_fade / pow(softened_distance, safe_power);
                        if (logarithmic) {
                            weighted_sum += (criterion == HeatmapCriterion::RSRQ
                                ? db_to_linear(value)
                                : dbm_to_milliwatts(value)) * weight;
                        } else {
                            weighted_sum += value * weight;
                        }
                        total_weight += weight;
                    }
                }
            }

            if (!direct_hit) {
                if (total_weight <= 0.0) {
                    continue;
                }
                if (logarithmic) {
                    const double avg_linear = weighted_sum / total_weight;
                    interpolated_value = criterion == HeatmapCriterion::RSRQ
                        ? linear_to_db(avg_linear)
                        : milliwatts_to_dbm(avg_linear);
                } else {
                    interpolated_value = weighted_sum / total_weight;
                }
            }

            if (!isfinite(interpolated_value)) {
                continue;
            }
            if (!config.use_dynamic_range && interpolated_value <= config.transparent_below) {
                continue;
            }
            if (!has_sample_in_display_radius) {
                continue;
            }

            const double value_range = config.max_value - config.min_value;
            if (value_range <= 0.0) {
                continue;
            }

            const double ratio = clamp01((interpolated_value - config.min_value) / value_range);
            const double edge_fade = 1.0 - smoothstep(0.35, 1.0, nearest_edge_ratio);
            const double influence_fade = clamp01(influence_sum / 2.0);
            unsigned char r = 0;
            unsigned char g = 0;
            unsigned char b = 0;
            unsigned char a = 0;
            colorize_heatmap(ratio, static_cast<float>(opacity * edge_fade * influence_fade), &r, &g, &b, &a);
            if (a == 0) {
                continue;
            }

            const size_t idx = (static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x)) * 4;
            (*out_rgba)[idx + 0] = r;
            (*out_rgba)[idx + 1] = g;
            (*out_rgba)[idx + 2] = b;
            (*out_rgba)[idx + 3] = a;
            has_visible_pixels = true;
        }
    }
    return has_visible_pixels;
}

static filesystem::path locate_heatmap_data_file() {
    const filesystem::path candidates[] = {
        "src/data.json",
        "data.json",
        "../src/data.json",
        "../data.json",
        "locations1.jsonl",
        "../locations1.jsonl",
    };
    for (const auto& candidate : candidates) {
        if (filesystem::exists(candidate)) {
            return candidate;
        }
    }
    return {};
}

HeatmapBounds compute_heatmap_bounds(const vector<HeatmapPoint>& points) {
    HeatmapBounds bounds;
    double min_lon = numeric_limits<double>::infinity();
    double max_lon = -numeric_limits<double>::infinity();
    double min_lat = numeric_limits<double>::infinity();
    double max_lat = -numeric_limits<double>::infinity();

    for (const auto& pt : points) {
        if (!isfinite(pt.latitude) || !isfinite(pt.longitude)) {
            continue;
        }
        min_lon = min(min_lon, pt.longitude);
        max_lon = max(max_lon, pt.longitude);
        min_lat = min(min_lat, pt.latitude);
        max_lat = max(max_lat, pt.latitude);
    }

    if (!isfinite(min_lon) || !isfinite(max_lon) || !isfinite(min_lat) || !isfinite(max_lat)) {
        return bounds;
    }

    const double padding_lon = max(0.0001, (max_lon - min_lon) * 0.05);
    const double padding_lat = max(0.0001, (max_lat - min_lat) * 0.05);
    min_lon -= padding_lon;
    max_lon += padding_lon;
    min_lat = clamp_latitude(min_lat - padding_lat);
    max_lat = clamp_latitude(max_lat + padding_lat);

    bounds.min_lon = min_lon;
    bounds.max_lon = max_lon;
    bounds.min_merc_y = latitude_to_mercator_y(min_lat);
    bounds.max_merc_y = latitude_to_mercator_y(max_lat);
    bounds.valid = true;
    return bounds;
}

HeatmapBounds bounds_for_saved_heatmap(const filesystem::path& path,
                                       const vector<HeatmapPoint>& points) {
    vector<HeatmapPoint> filtered = points;
    const string filename = path.filename().string();
    const string marker = "_earfcn_";
    const size_t marker_pos = filename.find(marker);
    if (marker_pos != string::npos) {
        const char* start = filename.c_str() + marker_pos + marker.size();
        char* end = nullptr;
        const long earfcn = strtol(start, &end, 10);
        if (start != end) {
            filtered.clear();
            for (const auto& point : points) {
                if (point.earfcn == static_cast<int>(earfcn)) {
                    filtered.push_back(point);
                }
            }
        }
    }
    return compute_heatmap_bounds(filtered);
}

void center_map_on_heatmap_points(MapViewState* view_state, const vector<HeatmapPoint>& points) {
    if (view_state == nullptr || points.empty()) {
        return;
    }

    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        if (!isfinite(it->latitude) || !isfinite(it->longitude) ||
            !is_valid_metric_value(it->rsrp, HeatmapCriterion::RSRP) ||
            it->rsrp <= heatmap_metric_config(HeatmapCriterion::RSRP).transparent_below) {
            continue;
        }
        view_state->center_latitude = it->latitude;
        view_state->center_longitude = it->longitude;
        view_state->view_span_x = 0.01;
        view_state->view_span_y = 0.01;
        view_state->center_initialized = true;
        view_state->follow_location = false;
        return;
    }

    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        if (!isfinite(it->latitude) || !isfinite(it->longitude)) {
            continue;
        }
        view_state->center_latitude = it->latitude;
        view_state->center_longitude = it->longitude;
        view_state->view_span_x = 0.01;
        view_state->view_span_y = 0.01;
        view_state->center_initialized = true;
        view_state->follow_location = false;
        return;
    }
}

void generate_and_save_heatmap(const vector<HeatmapPoint>& points,
                               const HeatmapOptions& options,
                               const filesystem::path& build_root,
                               int zoom = -1,
                               int min_x = 0,
                               int max_x = 0,
                               int min_y = 0,
                               int max_y = 0,
                               vector<filesystem::path>* out_saved_paths = nullptr,
                               string* out_status = nullptr) {
    error_code ec;
    filesystem::create_directories(build_root, ec);
    if (ec) {
        if (out_status) {
            *out_status = "Failed to create build directory: " + build_root.string();
        }
        return;
    }
    const vector<HeatmapPoint>* groups_to_generate[] = { &points };
    vector<vector<HeatmapPoint>> earfcn_groups;
    vector<int> earfcn_values;
    if (options.use_earfcn_groups) {
        unordered_map<int, vector<HeatmapPoint>> grouped;
        for (const auto& pt : points) {
            if (pt.earfcn >= 0) {
                grouped[pt.earfcn].push_back(pt);
            }
        }
        earfcn_values.reserve(grouped.size());
        earfcn_groups.reserve(grouped.size());
        for (const auto& pair : grouped) {
            earfcn_values.push_back(pair.first);
        }
        sort(earfcn_values.begin(), earfcn_values.end());
        for (int earfcn : earfcn_values) {
            earfcn_groups.push_back(std::move(grouped[earfcn]));
        }
        if (!earfcn_groups.empty()) {
            groups_to_generate[0] = nullptr;
        }
    }

    const auto save_single = [&](const vector<HeatmapPoint>& pts,
                                 const string& suffix,
                                 const filesystem::path& output_path,
                                 int image_width,
                                 int image_height,
                                 double left_lon,
                                 double right_lon,
                                 double bottom_merc_y,
                                 double top_merc_y) {
        vector<unsigned char> rgba;
        if (!build_heatmap_image(pts,
                                 options.criterion,
                                 options.radius_m,
                                 image_width,
                                 image_height,
                                 left_lon,
                                 right_lon,
                                 bottom_merc_y,
                                 top_merc_y,
                                 &rgba,
                                 options.display_radius_m,
                                 options.power,
                                 options.opacity,
                                 options.use_earfcn_groups ? -1 : options.selected_earfcn)) {
            if (out_status) {
                *out_status = "Failed to compute heatmap image " + suffix;
            }
            return;
        }
        filesystem::create_directories(output_path.parent_path(), ec);
        if (!write_png(output_path, image_width, image_height, rgba, out_status)) {
            return;
        }
        if (out_saved_paths) {
            out_saved_paths->push_back(output_path);
        }
        if (out_status) {
            *out_status = "Saved heatmap: " + output_path.string();
        }
    };

    const auto save_global = [&](const vector<HeatmapPoint>& pts, const string& suffix) {
        const HeatmapBounds bounds = compute_heatmap_bounds(pts);
        if (!bounds.valid) {
            if (out_status) {
                *out_status = "Boundary computation failed";
            }
            return;
        }
        const filesystem::path out_path = build_root / ("heatmap_" + suffix + ".png");
        save_single(pts, suffix, out_path, options.output_image_size, options.output_image_size,
                    bounds.min_lon, bounds.max_lon, bounds.min_merc_y, bounds.max_merc_y);
    };

    const auto save_tile = [&](const vector<HeatmapPoint>& pts, const string& suffix) {
        if (zoom < 0) {
            if (out_status) {
                *out_status = "Tile generation requires a valid zoom level";
            }
            return;
        }
        for (int tile_x = min_x; tile_x <= max_x; ++tile_x) {
            for (int tile_y = min_y; tile_y <= max_y; ++tile_y) {
                const double left_lon = tile_x_to_mercator_x(tile_x, zoom);
                const double right_lon = tile_x_to_mercator_x(tile_x + 1, zoom);
                const double top_merc_y = tile_y_to_mercator_y(tile_y, zoom);
                const double bottom_merc_y = tile_y_to_mercator_y(tile_y + 1, zoom);
                filesystem::path out_path = build_root / to_string(zoom) / to_string(tile_x) / (to_string(tile_y) + "_" + suffix + ".png");
                save_single(pts, suffix, out_path, 256, 256, left_lon, right_lon, bottom_merc_y, top_merc_y);
            }
        }
    };

    if (groups_to_generate[0] != nullptr) {
        const string suffix = string(kHeatmapCriterionNames[static_cast<int>(options.criterion)]);
        if (options.per_tile) {
            save_tile(points, suffix);
        } else {
            save_global(points, suffix);
        }
        return;
    }

    for (size_t group_index = 0; group_index < earfcn_groups.size(); ++group_index) {
        const int earfcn = earfcn_values[group_index];
        const string suffix = string(kHeatmapCriterionNames[static_cast<int>(options.criterion)]) + "_earfcn_" + to_string(earfcn);
        if (options.per_tile) {
            save_tile(earfcn_groups[group_index], suffix);
        } else {
            save_global(earfcn_groups[group_index], suffix);
        }
    }
}

class HeatmapTileManager {
public:
    HeatmapTileManager()
        : cache_root_(resolve_cache_root() / "heatmap" / "v3") {
        error_code ec;
        filesystem::create_directories(cache_root_, ec);
        workers_.reserve(kTileWorkerCount);
        for (int i = 0; i < kTileWorkerCount; ++i) {
            workers_.emplace_back(&HeatmapTileManager::worker_loop, this, i);
        }
    }

    ~HeatmapTileManager() {
        shutdown_workers();
        clear();
    }

    void configure(const vector<HeatmapPoint>& points, const HeatmapOptions& options) {
        HeatmapOptions next_options = options;
        next_options.per_tile = true;
        next_options.use_earfcn_groups = false;
        next_options.radius_m = clamp(next_options.radius_m, 10.0f, 40.0f);
        next_options.display_radius_m = max(1.0f, next_options.display_radius_m);
        next_options.power = max(1.0f, next_options.power);
        next_options.opacity = clamp(next_options.opacity, 0.0f, 1.0f);
        next_options.output_image_size = 256;
        ensure_selected_heatmap_earfcn(&next_options, points);

        const string next_signature = make_signature(points, next_options);
        {
            lock_guard<mutex> lock(cache_mutex_);
            if (next_signature == signature_) {
                return;
            }
        }

        reset_pending_jobs();

        {
            lock_guard<mutex> lock(cache_mutex_);
            clear_textures_locked();
            points_ = points;
            options_ = next_options;
            signature_ = next_signature;
            last_status_ = points_.empty()
                ? "Heatmap tile overlay disabled: no points"
                : "Heatmap tile overlay ready: " + signature_;
        }
    }

    void shutdown_workers() {
        {
            lock_guard<mutex> lock(job_mutex_);
            if (stopping_) {
                return;
            }
            stopping_ = true;
            for (auto& queue : job_queues_) {
                std::queue<HeatmapTileJob> empty;
                swap(queue, empty);
            }
        }
        job_cv_.notify_all();
        for (thread& worker : workers_) {
            if (worker.joinable()) {
                worker.join();
            }
        }
    }

    void clear() {
        lock_guard<mutex> lock(cache_mutex_);
        clear_textures_locked();
    }

    const filesystem::path& cache_root() const {
        return cache_root_;
    }

    string last_status() const {
        lock_guard<mutex> lock(cache_mutex_);
        return last_status_;
    }

    GLuint get_or_enqueue_tile(int zoom, int tile_x, int tile_y, int worker_index) {
        if (tile_x < 0 || tile_y < 0 ||
            tile_x >= tile_count_for_zoom(zoom) ||
            tile_y >= tile_count_for_zoom(zoom)) {
            return 0;
        }
        worker_index = clamp(worker_index, 0, kTileWorkerCount - 1);

        string signature;
        HeatmapOptions options;
        filesystem::path file_path;
        string key;
        vector<unsigned char> rgba_blob;
        int width = 0;
        int height = 0;
        bool should_enqueue = false;

        {
            lock_guard<mutex> lock(cache_mutex_);
            if (signature_.empty() || points_.empty()) {
                return 0;
            }

            signature = signature_;
            options = options_;
            key = tile_key(signature, zoom, tile_x, tile_y);
            file_path = tile_file_path(signature, zoom, tile_x, tile_y);

            TileTexture& tile = tiles_[key];
            if (tile.file_path.empty()) {
                tile.file_path = file_path;
            }

            if (tile.texture_id != 0) {
                return tile.texture_id;
            }

            const auto now = chrono::steady_clock::now();
            if (!tile.last_error.empty() && now < tile.next_retry_at) {
                return 0;
            }

            if (!tile.rgba_blob.empty()) {
                width = tile.width;
                height = tile.height;
                rgba_blob.swap(tile.rgba_blob);
            } else if (!tile.is_loading) {
                tile.is_loading = true;
                should_enqueue = true;
            } else {
                return 0;
            }
        }

        if (should_enqueue) {
            enqueue_job({key, signature, file_path, options, zoom, tile_x, tile_y}, worker_index);
            return 0;
        }

        const GLuint texture_id = upload_rgba_texture(width, height, rgba_blob);
        {
            lock_guard<mutex> lock(cache_mutex_);
            TileTexture& tile = tiles_[key];
            tile.texture_id = texture_id;
            tile.is_loading = false;
        }
        return texture_id;
    }

private:
    static string criterion_name(HeatmapCriterion criterion) {
        return kHeatmapCriterionNames[static_cast<int>(criterion)];
    }

    static string make_signature(const vector<HeatmapPoint>& points, const HeatmapOptions& options) {
        uint64_t checksum = 1469598103934665603ULL;
        const size_t step = max<size_t>(1, points.size() / 64);
        for (size_t index = 0; index < points.size(); index += step) {
            const HeatmapPoint& point = points[index];
            if (options.selected_earfcn > 0 && point.earfcn != options.selected_earfcn) {
                continue;
            }
            const long long lat = llround(point.latitude * 10000000.0);
            const long long lon = llround(point.longitude * 10000000.0);
            const double metric = metric_value(point, options.criterion);
            const long long value = is_valid_metric_value(metric, options.criterion)
                ? llround(metric * 100.0)
                : 0;
            checksum ^= static_cast<uint64_t>(lat);
            checksum *= 1099511628211ULL;
            checksum ^= static_cast<uint64_t>(lon);
            checksum *= 1099511628211ULL;
            checksum ^= static_cast<uint64_t>(value);
            checksum *= 1099511628211ULL;
        }

        const int radius_dm = static_cast<int>(lround(clamp(options.radius_m, 10.0f, 40.0f) * 10.0f));
        const int display_radius_dm = static_cast<int>(lround(max(1.0f, options.display_radius_m) * 10.0f));
        const int power_dm = static_cast<int>(lround(max(1.0f, options.power) * 10.0f));
        const int opacity_pct = static_cast<int>(lround(clamp(options.opacity, 0.0f, 1.0f) * 100.0f));
        stringstream stream;
        stream << criterion_name(options.criterion)
               << "_e" << options.selected_earfcn
               << "_r" << radius_dm
               << "_d" << display_radius_dm
               << "_p" << power_dm
               << "_o" << opacity_pct
               << "_n" << points.size()
               << "_c" << hex << checksum;
        return stream.str();
    }

    static string tile_key(const string& signature, int zoom, int x, int y) {
        return signature + "/" + to_string(zoom) + "/" + to_string(x) + "/" + to_string(y);
    }

    filesystem::path tile_file_path(const string& signature, int zoom, int x, int y) const {
        (void)signature;
        const string metric = criterion_name(options_.criterion);
        const string earfcn = options_.selected_earfcn > 0 ? to_string(options_.selected_earfcn) : "all";
        const int radius_m = static_cast<int>(lround(clamp(options_.radius_m, 10.0f, 40.0f)));
        const int display_radius_m = static_cast<int>(lround(max(1.0f, options_.display_radius_m)));
        const string mode = "idw_" + to_string(radius_m) + "_draw_" + to_string(display_radius_m);
        return cache_root_ / metric / earfcn / mode / to_string(zoom) / to_string(x) / (to_string(y) + ".png");
    }

    void clear_textures_locked() {
        for (auto& [_, tile] : tiles_) {
            if (tile.texture_id != 0) {
                glDeleteTextures(1, &tile.texture_id);
                tile.texture_id = 0;
            }
        }
        tiles_.clear();
    }

    void reset_pending_jobs() {
        lock_guard<mutex> lock(job_mutex_);
        for (auto& queue : job_queues_) {
            std::queue<HeatmapTileJob> empty;
            swap(queue, empty);
        }
    }

    void enqueue_job(const HeatmapTileJob& job, int worker_index) {
        {
            lock_guard<mutex> lock(job_mutex_);
            if (stopping_) {
                return;
            }
            job_queues_[worker_index].push(job);
        }
        job_cv_.notify_all();
    }

    void worker_loop(int worker_index) {
        while (true) {
            HeatmapTileJob job;
            {
                unique_lock<mutex> lock(job_mutex_);
                job_cv_.wait(lock, [this, worker_index]() {
                    return stopping_ || !job_queues_[worker_index].empty();
                });

                if (stopping_ && job_queues_[worker_index].empty()) {
                    return;
                }

                job = job_queues_[worker_index].front();
                job_queues_[worker_index].pop();
            }

            build_or_load_tile(job);
        }
    }

    void build_or_load_tile(const HeatmapTileJob& job) {
        vector<HeatmapPoint> points;
        {
            lock_guard<mutex> lock(cache_mutex_);
            if (job.signature != signature_) {
                return;
            }
            points = points_;
        }

        string error_message;
        const double left_lon = tile_x_to_mercator_x(job.x, job.zoom);
        const double right_lon = tile_x_to_mercator_x(job.x + 1, job.zoom);
        const double top_merc_y = tile_y_to_mercator_y(job.y, job.zoom);
        const double bottom_merc_y = tile_y_to_mercator_y(job.y + 1, job.zoom);

        vector<unsigned char> rgba;
        if (!build_heatmap_image(points,
                                 job.options.criterion,
                                 job.options.radius_m,
                                 256,
                                 256,
                                 left_lon,
                                 right_lon,
                                 bottom_merc_y,
                                 top_merc_y,
                                 &rgba,
                                 job.options.display_radius_m,
                                 job.options.power,
                                 job.options.opacity,
                                 job.options.selected_earfcn)) {
            rgba.assign(256 * 256 * 4, 0);
        }

        error_code ec;
        filesystem::create_directories(job.file_path.parent_path(), ec);
        if (ec) {
            mark_tile_failed(job, "failed to create heatmap tile directory: " + job.file_path.parent_path().string());
            return;
        }
        if (!write_png(job.file_path, 256, 256, rgba, &error_message)) {
            mark_tile_failed(job, error_message);
            return;
        }

        DecodedImage decoded;
        if (!decode_png_rgba(job.file_path, &decoded, &error_message)) {
            mark_tile_failed(job, error_message);
            return;
        }

        {
            lock_guard<mutex> lock(cache_mutex_);
            if (job.signature != signature_) {
                return;
            }
            TileTexture& tile = tiles_[job.id];
            tile.file_path = job.file_path;
            tile.width = decoded.width;
            tile.height = decoded.height;
            tile.rgba_blob = std::move(decoded.rgba);
            tile.is_loading = false;
            tile.last_error.clear();
            tile.next_retry_at = {};
            last_status_ = "Heatmap tile ready: " + job.id;
        }
    }

    void mark_tile_failed(const HeatmapTileJob& job, const string& error_message) {
        lock_guard<mutex> lock(cache_mutex_);
        if (job.signature != signature_) {
            return;
        }
        TileTexture& tile = tiles_[job.id];
        tile.is_loading = false;
        tile.last_error = error_message;
        tile.next_retry_at = chrono::steady_clock::now() + kTileRetryDelay;
        last_status_ = "Heatmap tile error: " + error_message;
    }

    filesystem::path cache_root_;
    vector<HeatmapPoint> points_;
    HeatmapOptions options_;
    string signature_;
    unordered_map<string, TileTexture> tiles_;
    array<queue<HeatmapTileJob>, kTileWorkerCount> job_queues_;
    vector<thread> workers_;
    mutable mutex cache_mutex_;
    mutex job_mutex_;
    condition_variable job_cv_;
    string last_status_;
    bool stopping_ = false;
};

void append_signal_sample(location* loc, int signal_dbm) {
    float next_x = static_cast<float>(loc->signal_x.size());
    if (loc->cnt.load() > 0 && loc->signal_x.size() >= 200) {
        next_x = loc->signal_x.back() + 1.0f;
    }

    loc->signal_x.push_back(next_x);
    loc->signal_y.push_back(static_cast<float>(signal_dbm));

    if (loc->signal_x.size() > 200) {
        loc->signal_x.erase(loc->signal_x.begin());
        loc->signal_y.erase(loc->signal_y.begin());
    }
}

bool parse_location_payload(const json& j, location* parsed, string* error_message) {
    if (!j.is_object()) {
        *error_message = "payload must be a JSON object";
        return false;
    }

    double latitude = 0.0;
    double longitude = 0.0;
    if (!read_number(j, {"lat", "latitude"}, &latitude) ||
        !read_number(j, {"lon", "lng", "longitude"}, &longitude)) {
        *error_message = "missing numeric lat/lon";
        return false;
    }
    normalize_coordinate_pair(&latitude, &longitude);

    double altitude = 0.0;
    double accuracy = 0.0;
    long long event_time = 0;
    long long rx = 0;
    long long tx = 0;
    double signal_dbm = -999.0;
    bool has_signal_dbm = false;

    read_number(j, {"alt", "altitude"}, &altitude);
    read_number(j, {"accuracy"}, &accuracy);
    read_int64(j, {"current_time", "timestamp", "event_time", "time"}, &event_time);
    read_int64(j, {"RX", "rx"}, &rx);
    read_int64(j, {"TX", "tx"}, &tx);
    has_signal_dbm = read_valid_signal_number(j, {"signalDbm", "signal_dbm", "dbm"}, &signal_dbm);

    const auto traffic_it = j.find("traffic");
    if (traffic_it != j.end() && traffic_it->is_object()) {
        read_int64(*traffic_it, {"total_rx", "rx", "RX"}, &rx);
        read_int64(*traffic_it, {"total_tx", "tx", "TX"}, &tx);
    }

    string cell_info = read_string(j, {"Cellallinfo", "cell_info", "cellInfo", "allcellinfo"});
    json cells;
    if (read_cells_array(j, &cells)) {
        if (cell_info.empty()) {
            cell_info = cells.dump();
        }

        double best_signal = -999.0;
        bool found_best_signal = false;
        for (const auto& cell : cells) {
            double cell_signal = numeric_limits<double>::quiet_NaN();
            if (read_valid_signal_number(cell, {"rsrp", "RSRP", "ssRsrp", "csiRsrp"}, &cell_signal) ||
                read_valid_signal_number(cell, {"rssi", "RSSI", "ssRssi"}, &cell_signal)) {
                if (!found_best_signal || cell_signal > best_signal) {
                    best_signal = cell_signal;
                    found_best_signal = true;
                }
            }
        }
        if (!has_signal_dbm && found_best_signal) {
            signal_dbm = best_signal;
            has_signal_dbm = true;
        }
    }

    parsed->latitude = static_cast<float>(latitude);
    parsed->longitude = static_cast<float>(longitude);
    parsed->altitude = static_cast<float>(altitude);
    parsed->accuracy = static_cast<float>(accuracy);
    parsed->time_j = event_time;
    parsed->allcellinfo = cell_info;
    parsed->signalDbm = static_cast<int>(signal_dbm);
    parsed->rx = rx;
    parsed->tx = tx;

    return true;
}

void apply_location_update(location* target, const location& parsed) {
    {
        lock_guard<mutex> lock(target->m);
        target->latitude = parsed.latitude;
        target->longitude = parsed.longitude;
        target->altitude = parsed.altitude;
        target->accuracy = parsed.accuracy;
        target->time_j = parsed.time_j;
        target->allcellinfo = parsed.allcellinfo;
        target->signalDbm = parsed.signalDbm;
        target->rx = parsed.rx;
        target->tx = parsed.tx;
        append_signal_sample(target, parsed.signalDbm);
    }
    target->cnt++;
}

ProgramOptions parse_program_options(int argc, char** argv) {
    ProgramOptions options;
    if (const char* conninfo = getenv("PGCONNINFO")) {
        options.server.db_conninfo = conninfo;
    }

    for (int i = 1; i < argc; ++i) {
        const string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            options.help = true;
        } else if (arg == "--bind" && i + 1 < argc) {
            options.server.bind_endpoint = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            options.server.log_file = argv[++i];
        } else if (arg == "--db" && i + 1 < argc) {
            options.server.db_conninfo = argv[++i];
        } else if (arg == "--import-json-to-db" && i + 1 < argc) {
            options.import_json_to_db_file = argv[++i];
        } else {
            cerr << "Unknown or incomplete argument: " << arg << endl;
            options.help = true;
        }
    }

    return options;
}

void print_usage(const char* executable) {
    cout << "Usage: " << executable << " [--bind tcp://*:5656] [--log locations1.jsonl]\n\n"
         << "PostgreSQL is optional. Pass --db \"host=... dbname=... user=... password=...\"\n"
         << "or set PGCONNINFO. Without it the server writes only JSONL and updates the GUI.\n\n"
         << "DB import: " << executable << " --db \"...\" --import-json-to-db src/data.json\n";
}

void render_location_window(const LocationSnapshot& snapshot) {
    ImGui::Begin("Location");

    ImGui::Text("Updates: %llu", snapshot.cnt);
    ImGui::Text("lat: %.7f", snapshot.latitude);
    ImGui::Text("lon: %.7f", snapshot.longitude);
    ImGui::Text("alt: %.2f", snapshot.altitude);
    ImGui::Text("accuracy: %.2f", snapshot.accuracy);
    ImGui::Text("time: %lld", snapshot.time_j);
    ImGui::Text("signalDbm: %d", snapshot.signalDbm);
    ImGui::Text("RX/TX: %lld / %lld", snapshot.rx, snapshot.tx);

    if (!snapshot.signal_x.empty() && !snapshot.signal_y.empty()) {
        if (ImPlot::BeginPlot("Cell Signal Strength")) {
            ImPlot::SetupAxes("sample", "dBm");
            ImPlot::PlotLine("signal",
                             snapshot.signal_x.data(),
                             snapshot.signal_y.data(),
                             static_cast<int>(snapshot.signal_x.size()));
            ImPlot::EndPlot();
        }
    }

    ImGui::Text("ALL_CELL:");
    ImGui::TextUnformatted(snapshot.allcellinfo.c_str());

    ImGui::End();
}

bool start_heatmap_generation(const vector<HeatmapPoint>& heatmap_points,
                              const HeatmapOptions& heatmap_options,
                              atomic_bool* heatmap_busy,
                              string* heatmap_status,
                              vector<filesystem::path>* saved_paths,
                              mutex* heatmap_mutex,
                              thread* heatmap_thread,
                              bool have_tile_range,
                              int tile_zoom,
                              int tile_min_x,
                              int tile_max_x,
                              int tile_min_y,
                              int tile_max_y,
                              const string& start_status) {
    if (heatmap_points.empty()) {
        lock_guard<mutex> lock(*heatmap_mutex);
        *heatmap_status = "No heatmap points loaded from source file.";
        return false;
    }
    if (heatmap_options.per_tile && !have_tile_range) {
        lock_guard<mutex> lock(*heatmap_mutex);
        *heatmap_status = "Need a visible tile range to generate per-tile images.";
        return false;
    }
    if (heatmap_busy->load()) {
        return false;
    }
    if (heatmap_thread != nullptr && heatmap_thread->joinable()) {
        heatmap_thread->join();
    }

    heatmap_busy->store(true);
    {
        lock_guard<mutex> lock(*heatmap_mutex);
        saved_paths->clear();
        *heatmap_status = start_status;
    }

    const filesystem::path build_root = resolve_cache_root();
    const int zoom = heatmap_options.per_tile ? tile_zoom : -1;
    const HeatmapOptions options_copy = heatmap_options;
    const vector<HeatmapPoint> points_copy = heatmap_points;
    *heatmap_thread = thread([points_copy,
                              options_copy,
                              build_root,
                              zoom,
                              tile_min_x,
                              tile_max_x,
                              tile_min_y,
                              tile_max_y,
                              saved_paths,
                              heatmap_status,
                              heatmap_mutex,
                              heatmap_busy]() mutable {
        vector<filesystem::path> local_saved;
        string local_status;
        generate_and_save_heatmap(points_copy,
                                  options_copy,
                                  build_root,
                                  zoom,
                                  tile_min_x,
                                  tile_max_x,
                                  tile_min_y,
                                  tile_max_y,
                                  &local_saved,
                                  &local_status);
        {
            lock_guard<mutex> lock(*heatmap_mutex);
            *heatmap_status = local_status;
            saved_paths->assign(local_saved.begin(), local_saved.end());
        }
        heatmap_busy->store(false);
    });

    return true;
}

void render_heatmap_window(HeatmapOptions* heatmap_options,
                           const vector<HeatmapPoint>& heatmap_points,
                           const string& data_source_label,
                           atomic_bool* heatmap_busy,
                           string* heatmap_status,
                           vector<filesystem::path>* saved_paths,
                           mutex* heatmap_mutex,
                           thread* heatmap_thread,
                           bool have_tile_range,
                           int tile_zoom,
                           int tile_min_x,
                           int tile_max_x,
                           int tile_min_y,
                           int tile_max_y) {
    ImGui::Begin("Heatmap");

    if (!data_source_label.empty()) {
        ImGui::Text("Heatmap data: %s", data_source_label.c_str());
    } else {
        ImGui::TextColored(ImVec4(1, 0.7f, 0.2f, 1.0f), "Heatmap source not found");
    }

    int criterion_index = static_cast<int>(heatmap_options->criterion);
    if (ImGui::BeginCombo("Criterion", kHeatmapCriterionNames[criterion_index])) {
        for (int index = 0; index < IM_ARRAYSIZE(kHeatmapCriterionNames); ++index) {
            const bool selected = (criterion_index == index);
            if (ImGui::Selectable(kHeatmapCriterionNames[index], selected)) {
                criterion_index = index;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }
    heatmap_options->criterion = static_cast<HeatmapCriterion>(criterion_index);
    ensure_selected_heatmap_earfcn(heatmap_options, heatmap_points);

    const vector<int> earfcns = available_heatmap_earfcns(heatmap_points, heatmap_options->criterion);
    string earfcn_label = heatmap_options->selected_earfcn > 0
        ? to_string(heatmap_options->selected_earfcn)
        : "All / no EARFCN";
    if (ImGui::BeginCombo("EARFCN", earfcn_label.c_str())) {
        const bool all_selected = heatmap_options->selected_earfcn <= 0;
        if (ImGui::Selectable("All / no EARFCN", all_selected)) {
            heatmap_options->selected_earfcn = -1;
        }
        if (all_selected) {
            ImGui::SetItemDefaultFocus();
        }
        for (int earfcn : earfcns) {
            const bool selected = heatmap_options->selected_earfcn == earfcn;
            const string label = to_string(earfcn);
            if (ImGui::Selectable(label.c_str(), selected)) {
                heatmap_options->selected_earfcn = earfcn;
            }
            if (selected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    ImGui::Checkbox("Overlay heatmap on map", &heatmap_options->overlay);
    ImGui::Checkbox("Separate EARFCN groups", &heatmap_options->use_earfcn_groups);
    ImGui::SliderFloat("IDW radius (m)", &heatmap_options->radius_m, 10.0f, 40.0f);
    ImGui::SliderFloat("Draw radius (m)", &heatmap_options->display_radius_m, 1.0f, 200.0f);
    ImGui::SliderFloat("IDW power", &heatmap_options->power, 1.0f, 4.0f);
    ImGui::SliderFloat("Opacity", &heatmap_options->opacity, 0.05f, 1.0f);
    ImGui::Checkbox("Generate per-tile images", &heatmap_options->per_tile);
    ImGui::SliderInt("Global heatmap size", &heatmap_options->output_image_size, 256, 2048);

    if (!have_tile_range && heatmap_options->per_tile) {
        ImGui::TextColored(ImVec4(1, 0.5f, 0.2f, 1.0f), "Pan/zoom map to select a visible tile range before generating tiles.");
    }

    if (heatmap_busy->load()) {
        ImGui::Text("Heatmap generation is running...");
    } else {
        if (ImGui::Button("Start heatmap generation")) {
            start_heatmap_generation(heatmap_points,
                                     *heatmap_options,
                                     heatmap_busy,
                                     heatmap_status,
                                     saved_paths,
                                     heatmap_mutex,
                                     heatmap_thread,
                                     have_tile_range,
                                     tile_zoom,
                                     tile_min_x,
                                     tile_max_x,
                                     tile_min_y,
                                     tile_max_y,
                                     "Starting heatmap generation...");
        }
    }

    {
        lock_guard<mutex> lock(*heatmap_mutex);
        if (!heatmap_status->empty()) {
            ImGui::TextWrapped("Status: %s", heatmap_status->c_str());
        }
        if (!saved_paths->empty()) {
            ImGui::Text("Saved files:");
            for (const auto& path : *saved_paths) {
                ImGui::TextWrapped("%s", path.string().c_str());
            }
        }
    }

    ImGui::End();
}

void render_map_window(TileManager* tile_manager,
                       HeatmapTileManager* heatmap_tile_manager,
                       MapViewState* view_state,
                       const LocationSnapshot& snapshot,
                       bool* have_tile_range,
                       int* out_zoom,
                       int* out_min_x,
                       int* out_max_x,
                       int* out_min_y,
                       int* out_max_y,
                       GLuint heatmap_texture_id,
                       const HeatmapBounds& heatmap_bounds,
                       bool overlay_heatmap) {
    const bool has_live_location = snapshot.cnt > 0;
    if (have_tile_range != nullptr) {
        *have_tile_range = false;
    }
    if (!view_state->center_initialized && has_live_location) {
        view_state->center_latitude = snapshot.latitude;
        view_state->center_longitude = snapshot.longitude;
        view_state->center_initialized = true;
    }
    if (view_state->follow_location && has_live_location) {
        view_state->center_latitude = snapshot.latitude;
        view_state->center_longitude = snapshot.longitude;
        view_state->center_initialized = true;
    }

    view_state->center_latitude = clamp_latitude(view_state->center_latitude);
    view_state->center_longitude = wrap_longitude(view_state->center_longitude);

    ImGui::Begin("Map");

    ImGui::Checkbox("Follow current location", &view_state->follow_location);
    ImGui::SameLine();
    if (ImGui::Button("Center on GPS") && has_live_location) {
        view_state->center_latitude = snapshot.latitude;
        view_state->center_longitude = snapshot.longitude;
        view_state->center_initialized = true;
        view_state->follow_location = true;
    }

    ImGui::Text("Center lat/lon: %.6f / %.6f", view_state->center_latitude, view_state->center_longitude);
    ImGui::Text("Mercator zoom: %d", view_state->current_zoom);
    ImGui::TextWrapped("Tile cache directory: %s", tile_manager->cache_root().string().c_str());

    const ImVec2 plot_size = ImGui::GetContentRegionAvail();
    if (plot_size.x < 32.0f || plot_size.y < 32.0f) {
        ImGui::End();
        return;
    }

    const double center_x = wrap_longitude(view_state->center_longitude);
    const double center_y = latitude_to_mercator_y(view_state->center_latitude);
    ImPlot::SetNextAxesLimits(center_x - view_state->view_span_x * 0.5,
                              center_x + view_state->view_span_x * 0.5,
                              center_y - view_state->view_span_y * 0.5,
                              center_y + view_state->view_span_y * 0.5,
                              view_state->follow_location ? ImGuiCond_Always : ImGuiCond_Once);

    int requested_tiles = 0;
    int loaded_tiles = 0;
    int requested_heatmap_tiles = 0;
    int loaded_heatmap_tiles = 0;

    ImPlot::PushStyleVar(ImPlotStyleVar_PlotPadding, ImVec2(0.0f, 0.0f));
    ImPlot::PushStyleVar(ImPlotStyleVar_FitPadding, ImVec2(0.0f, 0.0f));
    if (ImPlot::BeginPlot("##OpenStreetMap",
                          plot_size,
                          ImPlotFlags_CanvasOnly | ImPlotFlags_NoFrame)) {
        ImPlot::SetupAxes(nullptr,
                          nullptr,
                          ImPlotAxisFlags_NoDecorations,
                          ImPlotAxisFlags_NoDecorations);

        ImPlotRect limits = ImPlot::GetPlotLimits();
        const double longitude_span = limits.X.Max - limits.X.Min;
        const int zoom = zoom_from_longitude_span(longitude_span);
        if (view_state->last_tile_zoom != -1 && view_state->last_tile_zoom != zoom) {
            tile_manager->reset_pending_jobs();
        }
        view_state->current_zoom = zoom;
        view_state->last_tile_zoom = zoom;
        if (!view_state->follow_location || has_live_location) {
            view_state->view_span_x = max(0.000001, longitude_span);
            view_state->view_span_y = max(0.000001, limits.Y.Max - limits.Y.Min);
        }

        int min_x = static_cast<int>(floor(mercator_x_to_tile_x(limits.X.Min, zoom)));
        int max_x = static_cast<int>(floor(mercator_x_to_tile_x(limits.X.Max, zoom)));
        int min_y = static_cast<int>(floor(mercator_y_to_tile_y(limits.Y.Max, zoom)));
        int max_y = static_cast<int>(floor(mercator_y_to_tile_y(limits.Y.Min, zoom)));

        const int max_tile_index = tile_count_for_zoom(zoom) - 1;
        min_x = clamp(min_x, 0, max_tile_index);
        max_x = clamp(max_x, 0, max_tile_index);
        min_y = clamp(min_y, 0, max_tile_index);
        max_y = clamp(max_y, 0, max_tile_index);

        if (have_tile_range != nullptr) {
            *have_tile_range = true;
            *out_zoom = zoom;
            *out_min_x = min_x;
            *out_max_x = max_x;
            *out_min_y = min_y;
            *out_max_y = max_y;
        }

        const auto render_tile = [&](int tile_x, int tile_y, int worker_index) {
            ++requested_tiles;
            const GLuint gpu_id = tile_manager->get_or_enqueue_tile(zoom, tile_x, tile_y, worker_index);
            if (gpu_id == 0) {
                return;
            }

            ++loaded_tiles;
            const ImPlotPoint min_point{
                tile_x_to_mercator_x(tile_x, zoom),
                tile_y_to_mercator_y(tile_y + 1, zoom)
            };
            const ImPlotPoint max_point{
                tile_x_to_mercator_x(tile_x + 1, zoom),
                tile_y_to_mercator_y(tile_y, zoom)
            };
            const string tile_id =
                "##tile_" + to_string(zoom) + "_" + to_string(tile_x) + "_" + to_string(tile_y);
            ImPlot::PlotImage(tile_id.c_str(), (ImTextureID)(intptr_t)gpu_id, min_point, max_point);
        };

        const int mid_x = (min_x + max_x) / 2;
        const int mid_y = (min_y + max_y) / 2;

        for (int tile_y = min_y; tile_y <= mid_y; ++tile_y) {
            for (int tile_x = min_x; tile_x <= mid_x; ++tile_x) {
                render_tile(tile_x, tile_y, 0);
            }
        }

        for (int tile_y = min_y; tile_y <= mid_y; ++tile_y) {
            for (int tile_x = max_x; tile_x > mid_x; --tile_x) {
                render_tile(tile_x, tile_y, 1);
            }
        }

        for (int tile_y = max_y; tile_y > mid_y; --tile_y) {
            for (int tile_x = min_x; tile_x <= mid_x; ++tile_x) {
                render_tile(tile_x, tile_y, 2);
            }
        }

        for (int tile_y = max_y; tile_y > mid_y; --tile_y) {
            for (int tile_x = max_x; tile_x > mid_x; --tile_x) {
                render_tile(tile_x, tile_y, 3);
            }
        }

        if (overlay_heatmap && heatmap_tile_manager != nullptr) {
            for (int tile_y = min_y; tile_y <= max_y; ++tile_y) {
                for (int tile_x = min_x; tile_x <= max_x; ++tile_x) {
                    ++requested_heatmap_tiles;
                    const GLuint gpu_id = heatmap_tile_manager->get_or_enqueue_tile(zoom, tile_x, tile_y, (tile_x + tile_y) % kTileWorkerCount);
                    if (gpu_id == 0) {
                        continue;
                    }

                    ++loaded_heatmap_tiles;
                    const ImPlotPoint min_point{
                        tile_x_to_mercator_x(tile_x, zoom),
                        tile_y_to_mercator_y(tile_y + 1, zoom)
                    };
                    const ImPlotPoint max_point{
                        tile_x_to_mercator_x(tile_x + 1, zoom),
                        tile_y_to_mercator_y(tile_y, zoom)
                    };
                    const string tile_id =
                        "##heatmap_tile_" + to_string(zoom) + "_" + to_string(tile_x) + "_" + to_string(tile_y);
                    ImPlot::PlotImage(tile_id.c_str(),
                                      (ImTextureID)(intptr_t)gpu_id,
                                      min_point,
                                      max_point,
                                      ImVec2(0.0f, 0.0f),
                                      ImVec2(1.0f, 1.0f),
                                      ImVec4(1.0f, 1.0f, 1.0f, 0.8f));
                }
            }
        } else if (overlay_heatmap && heatmap_texture_id != 0) {
            const ImPlotPoint min_point = heatmap_bounds.valid
                ? ImPlotPoint{heatmap_bounds.min_lon, heatmap_bounds.min_merc_y}
                : ImPlotPoint{tile_x_to_mercator_x(min_x, zoom), tile_y_to_mercator_y(max_y + 1, zoom)};
            const ImPlotPoint max_point = heatmap_bounds.valid
                ? ImPlotPoint{heatmap_bounds.max_lon, heatmap_bounds.max_merc_y}
                : ImPlotPoint{tile_x_to_mercator_x(max_x + 1, zoom), tile_y_to_mercator_y(min_y, zoom)};
            ImPlot::PlotImage("##heatmap_overlay",
                              (ImTextureID)(intptr_t)heatmap_texture_id,
                              min_point,
                              max_point,
                              ImVec2(0.0f, 0.0f),
                              ImVec2(1.0f, 1.0f),
                              ImVec4(1.0f, 1.0f, 1.0f, 0.6f));
        }

        if (has_live_location) {
            const double marker_x = wrap_longitude(snapshot.longitude);
            const double marker_y = latitude_to_mercator_y(snapshot.latitude);
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle,
                                       8.0f,
                                       ImVec4(1.0f, 0.15f, 0.10f, 1.0f),
                                       2.0f,
                                       ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
            ImPlot::PlotScatter("Current position", &marker_x, &marker_y, 1);
        }

        if (ImPlot::IsPlotHovered() &&
            (ImGui::IsMouseDragging(ImGuiMouseButton_Left) || ImGui::GetIO().MouseWheel != 0.0f)) {
            view_state->follow_location = false;
        }

        ImPlot::EndPlot();
    }
    ImPlot::PopStyleVar(2);

    ImGui::Text("Visible tiles requested: %d", requested_tiles);
    ImGui::Text("Visible tiles ready: %d", loaded_tiles);
    const string tile_status = tile_manager->last_status();
    if (!tile_status.empty()) {
        ImGui::TextWrapped("Tile status: %s", tile_status.c_str());
    }
    if (overlay_heatmap && heatmap_tile_manager != nullptr) {
        ImGui::Text("Heatmap tiles ready: %d / %d", loaded_heatmap_tiles, requested_heatmap_tiles);
        const string heatmap_tile_status = heatmap_tile_manager->last_status();
        if (!heatmap_tile_status.empty()) {
            ImGui::TextWrapped("Heatmap tile status: %s", heatmap_tile_status.c_str());
        }
    }
    ImGui::TextUnformatted("Controls: drag to pan, mouse wheel to zoom.");

    ImGui::End();
}

} // namespace

static PGconn* open_db_connection(const string& conninfo) {
    if (conninfo.empty()) {
        cout << "PostgreSQL disabled: pass --db or set PGCONNINFO to enable inserts" << endl;
        return nullptr;
    }

    PGconn* conn = PQconnectdb(conninfo.c_str());
    if (PQstatus(conn) != CONNECTION_OK) {
        cerr << "DB connection failed: " << PQerrorMessage(conn) << endl;
        PQfinish(conn);
        return nullptr;
    }

    cout << "Connected to PostgreSQL" << endl;
    return conn;
}

static bool ensure_user_equipment_table(PGconn* conn) {
    PGresult* result = PQexec(
        conn,
        "CREATE TABLE IF NOT EXISTS user_equipment ("
        "id BIGSERIAL PRIMARY KEY, "
        "latitude DOUBLE PRECISION NOT NULL, "
        "longitude DOUBLE PRECISION NOT NULL, "
        "altitude DOUBLE PRECISION, "
        "accuracy DOUBLE PRECISION, "
        "event_time BIGINT, "
        "signal_dbm INTEGER, "
        "cell_info TEXT, "
        "created_at TIMESTAMPTZ NOT NULL DEFAULT now()"
        ")"
    );

    if (PQresultStatus(result) != PGRES_COMMAND_OK) {
        cerr << "Failed to prepare PostgreSQL table: " << PQerrorMessage(conn) << endl;
        PQclear(result);
        return false;
    }

    PQclear(result);

    result = PQexec(conn, "ALTER TABLE user_equipment ADD COLUMN IF NOT EXISTS accuracy DOUBLE PRECISION");
    if (PQresultStatus(result) != PGRES_COMMAND_OK) {
        cerr << "Failed to migrate PostgreSQL table: " << PQerrorMessage(conn) << endl;
        PQclear(result);
        return false;
    }
    PQclear(result);
    return true;
}

static bool insert_user_equipment(PGconn* conn, const location& nloc) {
    const char* params[7];
    string latitude = to_string(nloc.latitude);
    string longitude = to_string(nloc.longitude);
    string altitude = to_string(nloc.altitude);
    string accuracy = to_string(nloc.accuracy);
    string event_time = to_string(nloc.time_j);
    string signal_dbm = to_string(nloc.signalDbm);

    params[0] = latitude.c_str();
    params[1] = longitude.c_str();
    params[2] = altitude.c_str();
    params[3] = accuracy.c_str();
    params[4] = event_time.c_str();
    params[5] = signal_dbm.c_str();
    params[6] = nloc.allcellinfo.c_str();

    PGresult* result = PQexecParams(
        conn,
        "INSERT INTO user_equipment "
        "(latitude, longitude, altitude, accuracy, event_time, signal_dbm, cell_info) "
        "VALUES ($1, $2, $3, $4, $5, $6, $7)",
        7,
        nullptr,
        params,
        nullptr,
        nullptr,
        0
    );

    if (PQresultStatus(result) != PGRES_COMMAND_OK) {
        cerr << "Failed to insert into PostgreSQL: " << PQerrorMessage(conn) << endl;
        PQclear(result);
        return false;
    }

    PQclear(result);
    return true;
}

static json row_to_heatmap_json(PGresult* result, int row) {
    json item;
    item["lat"] = atof(PQgetvalue(result, row, 0));
    item["lon"] = atof(PQgetvalue(result, row, 1));
    if (!PQgetisnull(result, row, 2)) {
        item["alt"] = atof(PQgetvalue(result, row, 2));
    }
    if (!PQgetisnull(result, row, 3)) {
        item["accuracy"] = atof(PQgetvalue(result, row, 3));
    }
    if (!PQgetisnull(result, row, 4)) {
        item["time"] = atoll(PQgetvalue(result, row, 4));
    }
    if (!PQgetisnull(result, row, 5)) {
        item["signalDbm"] = atoi(PQgetvalue(result, row, 5));
    }
    if (!PQgetisnull(result, row, 6)) {
        item["Cellallinfo"] = PQgetvalue(result, row, 6);
    }
    return item;
}

static bool load_heatmap_points_from_db(const string& conninfo,
                                        vector<HeatmapPoint>* out_points,
                                        string* error_message) {
    if (out_points == nullptr) {
        return false;
    }
    out_points->clear();

    PGconn* conn = open_db_connection(conninfo);
    if (conn == nullptr) {
        if (error_message) {
            *error_message = "Cannot connect to PostgreSQL for heatmap.";
        }
        return false;
    }
    if (!ensure_user_equipment_table(conn)) {
        if (error_message) {
            *error_message = "Cannot prepare user_equipment table.";
        }
        PQfinish(conn);
        return false;
    }

    PGresult* result = PQexec(
        conn,
        "SELECT latitude, longitude, altitude, accuracy, event_time, signal_dbm, cell_info "
        "FROM user_equipment ORDER BY id"
    );
    if (PQresultStatus(result) != PGRES_TUPLES_OK) {
        if (error_message) {
            *error_message = string("Failed to read heatmap points from PostgreSQL: ") + PQerrorMessage(conn);
        }
        PQclear(result);
        PQfinish(conn);
        return false;
    }

    const int rows = PQntuples(result);
    for (int row = 0; row < rows; ++row) {
        append_heatmap_points_from_json(row_to_heatmap_json(result, row), out_points);
    }

    PQclear(result);
    PQfinish(conn);

    if (out_points->empty()) {
        if (error_message) {
            *error_message = "PostgreSQL table user_equipment is empty.";
        }
        return false;
    }
    return true;
}

static bool load_latest_location_from_db(const string& conninfo,
                                         location* out_location,
                                         string* error_message) {
    PGconn* conn = open_db_connection(conninfo);
    if (conn == nullptr) {
        if (error_message) {
            *error_message = "Cannot connect to PostgreSQL for latest location.";
        }
        return false;
    }
    if (!ensure_user_equipment_table(conn)) {
        PQfinish(conn);
        return false;
    }

    PGresult* result = PQexec(
        conn,
        "SELECT latitude, longitude, altitude, accuracy, event_time, signal_dbm, cell_info "
        "FROM user_equipment ORDER BY id DESC LIMIT 1"
    );
    if (PQresultStatus(result) != PGRES_TUPLES_OK || PQntuples(result) == 0) {
        if (error_message) {
            *error_message = PQntuples(result) == 0
                ? "PostgreSQL table user_equipment has no latest location."
                : string("Failed to read latest location from PostgreSQL: ") + PQerrorMessage(conn);
        }
        PQclear(result);
        PQfinish(conn);
        return false;
    }

    const json item = row_to_heatmap_json(result, 0);
    location parsed;
    string parse_error;
    const bool ok = parse_location_payload(item, &parsed, &parse_error);
    if (ok && out_location != nullptr) {
        out_location->latitude = parsed.latitude;
        out_location->longitude = parsed.longitude;
        out_location->altitude = parsed.altitude;
        out_location->accuracy = parsed.accuracy;
        out_location->time_j = parsed.time_j;
        out_location->allcellinfo = parsed.allcellinfo;
        out_location->signalDbm = parsed.signalDbm;
        out_location->rx = parsed.rx;
        out_location->tx = parsed.tx;
    } else if (error_message) {
        *error_message = parse_error;
    }

    PQclear(result);
    PQfinish(conn);
    return ok;
}

static bool import_json_file_to_db(const filesystem::path& file_path,
                                   const string& conninfo,
                                   int* inserted_count,
                                   string* error_message) {
    if (inserted_count != nullptr) {
        *inserted_count = 0;
    }
    PGconn* conn = open_db_connection(conninfo);
    if (conn == nullptr) {
        if (error_message) {
            *error_message = "Cannot connect to PostgreSQL for import.";
        }
        return false;
    }
    if (!ensure_user_equipment_table(conn)) {
        PQfinish(conn);
        return false;
    }

    ifstream input(file_path);
    if (!input.is_open()) {
        if (error_message) {
            *error_message = "Cannot open JSON import file: " + file_path.string();
        }
        PQfinish(conn);
        return false;
    }

    string content;
    content.assign(istreambuf_iterator<char>(input), istreambuf_iterator<char>());
    vector<json> items;
    try {
        size_t index = 0;
        while (index < content.size() && isspace(static_cast<unsigned char>(content[index]))) {
            ++index;
        }
        if (index < content.size() && content[index] == '[') {
            const json root = json::parse(content);
            for (const auto& item : root) {
                items.push_back(item);
            }
        } else {
            istringstream stream(content);
            string line;
            while (getline(stream, line)) {
                if (!line.empty()) {
                    items.push_back(json::parse(line));
                }
            }
        }
    } catch (const exception& e) {
        if (error_message) {
            *error_message = string("Failed to parse import JSON: ") + e.what();
        }
        PQfinish(conn);
        return false;
    }

    int count = 0;
    for (const auto& item : items) {
        location parsed;
        string parse_error;
        if (parse_location_payload(item, &parsed, &parse_error) &&
            insert_user_equipment(conn, parsed)) {
            ++count;
        }
    }

    PQfinish(conn);
    if (inserted_count != nullptr) {
        *inserted_count = count;
    }
    return count > 0;
}

void run_gui(location* loc, const ServerConfig& config, const filesystem::path& heatmap_source_override = {}) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init failed: " << SDL_GetError() << endl;
        g_keep_running.store(false);
        return;
    }

#ifdef __APPLE__
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, SDL_GL_CONTEXT_FORWARD_COMPATIBLE_FLAG);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
#else
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
#endif
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

    SDL_Window* window = SDL_CreateWindow(
        "Android Location Server", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1024, 768, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    if (window == nullptr) {
        cerr << "SDL_CreateWindow failed: " << SDL_GetError() << endl;
        SDL_Quit();
        g_keep_running.store(false);
        return;
    }

    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    if (gl_context == nullptr) {
        cerr << "SDL_GL_CreateContext failed: " << SDL_GetError() << endl;
        SDL_DestroyWindow(window);
        SDL_Quit();
        g_keep_running.store(false);
        return;
    }
    SDL_GL_SetSwapInterval(1);

    glewInit();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
#ifdef __APPLE__
    ImGui_ImplOpenGL3_Init("#version 150");
#else
    ImGui_ImplOpenGL3_Init("#version 330");
#endif

    TileManager tile_manager;
    HeatmapTileManager heatmap_tile_manager;
    MapViewState map_view_state;
    vector<HeatmapPoint> heatmap_points;
    HeatmapOptions heatmap_options;
    atomic_bool heatmap_busy{false};
    string heatmap_status;
    vector<filesystem::path> heatmap_saved_paths;
    mutex heatmap_mutex;
    thread heatmap_thread;
    filesystem::path heatmap_data_file = heatmap_source_override.empty()
        ? locate_heatmap_data_file()
        : heatmap_source_override;
    string heatmap_data_source_label;
    bool heatmap_have_tile_range = false;
    int heatmap_tile_zoom = 0;
    int heatmap_tile_min_x = 0;
    int heatmap_tile_max_x = 0;
    int heatmap_tile_min_y = 0;
    int heatmap_tile_max_y = 0;
    GLuint heatmap_texture_id = 0;
    int heatmap_texture_width = 0;
    int heatmap_texture_height = 0;
    filesystem::path heatmap_texture_path;
    HeatmapBounds heatmap_texture_bounds;

    if (!config.db_conninfo.empty()) {
        string db_error;
        if (load_heatmap_points_from_db(config.db_conninfo, &heatmap_points, &db_error)) {
            heatmap_data_source_label = "PostgreSQL:user_equipment";
            heatmap_status = "Loaded " + to_string(heatmap_points.size()) + " heatmap points from PostgreSQL.";

            location latest;
            if (load_latest_location_from_db(config.db_conninfo, &latest, nullptr)) {
                apply_location_update(loc, latest);
            }
        } else {
            heatmap_data_source_label = "PostgreSQL:user_equipment";
            heatmap_status = db_error;
        }
    } else if (!heatmap_data_file.empty()) {
        string load_error;
        if (load_heatmap_points(heatmap_data_file, &heatmap_points, &load_error)) {
            heatmap_data_source_label = heatmap_data_file.string();
            heatmap_status = "Loaded " + to_string(heatmap_points.size()) + " heatmap points.";
        } else {
            heatmap_data_source_label = heatmap_data_file.string();
            heatmap_status = load_error;
        }
    } else {
        heatmap_status = "Heatmap source file not found.";
    }

    if (!heatmap_points.empty()) {
        ensure_selected_heatmap_earfcn(&heatmap_options, heatmap_points);
        if (loc->cnt.load() == 0) {
            center_map_on_heatmap_points(&map_view_state, heatmap_points);
        }
        heatmap_status += " Transparent heatmap overlay is generated automatically on the map.";
    }

    while (g_keep_running.load()) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                g_keep_running.store(false);
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        const LocationSnapshot snapshot = snapshot_location(loc);
        heatmap_tile_manager.configure(heatmap_points, heatmap_options);
        render_location_window(snapshot);
        render_map_window(&tile_manager,
                          &heatmap_tile_manager,
                          &map_view_state,
                          snapshot,
                          &heatmap_have_tile_range,
                          &heatmap_tile_zoom,
                          &heatmap_tile_min_x,
                          &heatmap_tile_max_x,
                          &heatmap_tile_min_y,
                          &heatmap_tile_max_y,
                          heatmap_texture_id,
                          heatmap_texture_bounds,
                          heatmap_options.overlay);

        render_heatmap_window(&heatmap_options,
                              heatmap_points,
                              heatmap_data_source_label,
                              &heatmap_busy,
                              &heatmap_status,
                              &heatmap_saved_paths,
                              &heatmap_mutex,
                              &heatmap_thread,
                              heatmap_have_tile_range,
                              heatmap_tile_zoom,
                              heatmap_tile_min_x,
                              heatmap_tile_max_x,
                              heatmap_tile_min_y,
                              heatmap_tile_max_y);

        {
            lock_guard<mutex> lock(heatmap_mutex);
            if (!heatmap_busy.load() &&
                !heatmap_saved_paths.empty() &&
                !heatmap_options.per_tile &&
                heatmap_options.overlay &&
                (heatmap_texture_id == 0 || heatmap_saved_paths[0] != heatmap_texture_path)) {
                string load_error;
                if (load_png_texture(heatmap_saved_paths[0], &heatmap_texture_id, &heatmap_texture_width, &heatmap_texture_height, &load_error)) {
                    heatmap_status = string("Loaded heatmap overlay: ") + heatmap_saved_paths[0].string();
                    heatmap_texture_path = heatmap_saved_paths[0];
                    heatmap_texture_bounds = bounds_for_saved_heatmap(heatmap_texture_path, heatmap_points);
                } else {
                    heatmap_status = string("Failed to load overlay texture: ") + load_error;
                }
            }
        }

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    if (heatmap_thread.joinable()) {
        heatmap_thread.join();
    }
    heatmap_tile_manager.shutdown_workers();
    heatmap_tile_manager.clear();
    tile_manager.shutdown_workers();
    tile_manager.clear();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext(nullptr);
    ImGui::DestroyContext(nullptr);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void run_server(location* loc, ServerConfig config) {
    context_t context(1);
    socket_t socket(context, socket_type::rep);
    socket.set(zmq::sockopt::rcvtimeo, 250);
    socket.set(zmq::sockopt::linger, 0);
    socket.bind(config.bind_endpoint);
    cout << "Server is listening on " << config.bind_endpoint << endl;

    PGconn* db_conn = open_db_connection(config.db_conninfo);
    if (db_conn) {
        ensure_user_equipment_table(db_conn);
    }

    ofstream out(config.log_file, ios::app);
    if (!out.is_open()) {
        cerr << "Failed to open file for writing: " << config.log_file << endl;
        if (db_conn) {
            PQfinish(db_conn);
        }
        return;
    }
    cout << "Writing received JSON to: " << config.log_file << endl;

    while (g_keep_running.load()) {
        message_t request;
        const auto recv_result = socket.recv(request, recv_flags::none);
        if (!recv_result) {
            continue;
        }

        string req_str(static_cast<char*>(request.data()), request.size());
        cout << "Received request: " << req_str << endl;

        string reply_str = "OK";
        try {
            const json j = json::parse(req_str);
            location nloc;
            string error_message;
            if (!parse_location_payload(j, &nloc, &error_message)) {
                reply_str = "ERROR: " + error_message;
                cerr << reply_str << endl;
            } else {
                apply_location_update(loc, nloc);

                if (db_conn && insert_user_equipment(db_conn, nloc)) {
                    cout << "Inserted row into user_equipment" << endl;
                }
            }
        } catch (const exception& e) {
            reply_str = string("ERROR: invalid json: ") + e.what();
            cerr << reply_str << endl;
        }

        out << req_str << "\n";
        out.flush();

        message_t reply(reply_str.size());
        memcpy(reply.data(), reply_str.c_str(), reply_str.size());
        socket.send(reply, send_flags::none);
    }

    if (db_conn) {
        PQfinish(db_conn);
    }
}

int main(int argc, char** argv) {
    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);

    const ProgramOptions options = parse_program_options(argc, argv);
    if (options.help) {
        print_usage(argv[0]);
        return 0;
    }

    if (!options.import_json_to_db_file.empty()) {
        int inserted = 0;
        string import_error;
        if (import_json_file_to_db(options.import_json_to_db_file,
                                   options.server.db_conninfo,
                                   &inserted,
                                   &import_error)) {
            cout << "Imported " << inserted << " points into PostgreSQL user_equipment" << endl;
            return 0;
        }
        cerr << import_error << endl;
        return 1;
    }

    static location locationInfo{};
    thread background_thread(run_server, &locationInfo, options.server);
    run_gui(&locationInfo, options.server);
    g_keep_running.store(false);

    if (background_thread.joinable()) {
        background_thread.join();
    }

    return 0;
}
