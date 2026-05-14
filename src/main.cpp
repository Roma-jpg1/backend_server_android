#include <algorithm>
#include <array>
#include <atomic>
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
    bool gui_enabled = true;
    bool help = false;
    string replay_file;
};

struct MapViewState {
    double center_latitude = 0.0;
    double center_longitude = 0.0;
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

class TileManager {
public:
    TileManager()
        : cache_root_(resolve_cache_root()) {
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
            tile.rgba_blob = move(decoded.rgba);
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

    double altitude = 0.0;
    double accuracy = 0.0;
    long long event_time = 0;
    long long rx = 0;
    long long tx = 0;
    double signal_dbm = -999.0;

    read_number(j, {"alt", "altitude"}, &altitude);
    read_number(j, {"accuracy"}, &accuracy);
    read_int64(j, {"time", "timestamp", "event_time"}, &event_time);
    read_int64(j, {"RX", "rx"}, &rx);
    read_int64(j, {"TX", "tx"}, &tx);
    read_number(j, {"signalDbm", "signal_dbm", "dbm"}, &signal_dbm);

    parsed->latitude = static_cast<float>(latitude);
    parsed->longitude = static_cast<float>(longitude);
    parsed->altitude = static_cast<float>(altitude);
    parsed->accuracy = static_cast<float>(accuracy);
    parsed->time_j = event_time;
    parsed->allcellinfo = read_string(j, {"Cellallinfo", "cell_info", "cellInfo", "allcellinfo"});
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
        } else if (arg == "--no-gui") {
            options.gui_enabled = false;
        } else if (arg == "--bind" && i + 1 < argc) {
            options.server.bind_endpoint = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            options.server.log_file = argv[++i];
        } else if (arg == "--db" && i + 1 < argc) {
            options.server.db_conninfo = argv[++i];
        } else if (arg == "--json" && i + 1 < argc) {
            options.replay_file = argv[++i];
        } else {
            cerr << "Unknown or incomplete argument: " << arg << endl;
            options.help = true;
        }
    }

    return options;
}

void print_usage(const char* executable) {
    cout << "Usage: " << executable << " [--no-gui] [--bind tcp://*:5656] [--log locations1.jsonl]\n"
         << "       " << executable << " --json src/data.json [--no-gui]\n\n"
         << "PostgreSQL is optional. Pass --db \"host=... dbname=... user=... password=...\"\n"
         << "or set PGCONNINFO. Without it the server writes only JSONL and updates the GUI.\n";
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

void render_map_window(TileManager* tile_manager, MapViewState* view_state, const LocationSnapshot& snapshot) {
    const bool has_live_location = snapshot.cnt > 0;
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
    return true;
}

static bool insert_user_equipment(PGconn* conn, const location& nloc) {
    const char* params[6];
    string latitude = to_string(nloc.latitude);
    string longitude = to_string(nloc.longitude);
    string altitude = to_string(nloc.altitude);
    string event_time = to_string(nloc.time_j);
    string signal_dbm = to_string(nloc.signalDbm);

    params[0] = latitude.c_str();
    params[1] = longitude.c_str();
    params[2] = altitude.c_str();
    params[3] = event_time.c_str();
    params[4] = signal_dbm.c_str();
    params[5] = nloc.allcellinfo.c_str();

    PGresult* result = PQexecParams(
        conn,
        "INSERT INTO user_equipment "
        "(latitude, longitude, altitude, event_time, signal_dbm, cell_info) "
        "VALUES ($1, $2, $3, $4, $5, $6)",
        6,
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

void run_gui(location* loc) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        cerr << "SDL_Init failed: " << SDL_GetError() << endl;
        g_keep_running.store(false);
        return;
    }

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
    ImGui_ImplOpenGL3_Init("#version 330");

    TileManager tile_manager;
    MapViewState map_view_state;

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
        render_location_window(snapshot);
        render_map_window(&tile_manager, &map_view_state, snapshot);

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

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

void run_json_parser(location* loc, const string& file) {
    ifstream in(file);
    if (!in.is_open()) {
        cerr << "Cannot open replay file: " << file << endl;
        g_keep_running.store(false);
        return;
    }

    json arr;
    in >> arr;

    if (!arr.is_array()) {
        arr = json::array({arr});
    }

    for (const auto& j : arr) {
        if (!g_keep_running.load()) {
            break;
        }

        location parsed;
        string error_message;
        if (parse_location_payload(j, &parsed, &error_message)) {
            apply_location_update(loc, parsed);
        }
        this_thread::sleep_for(chrono::milliseconds(50));
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

    static location locationInfo{};

    thread gui_thread;
    if (options.gui_enabled) {
        gui_thread = thread(run_gui, &locationInfo);
    }

    if (!options.replay_file.empty()) {
        thread json_thread(run_json_parser, &locationInfo, options.replay_file);
        json_thread.join();
        if (!options.gui_enabled) {
            g_keep_running.store(false);
        }
    } else {
        thread server_thread(run_server, &locationInfo, options.server);
        server_thread.join();
    }

    if (gui_thread.joinable()) {
        gui_thread.join();
    }
    return 0;
}
