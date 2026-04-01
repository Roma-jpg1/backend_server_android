#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <chrono>
#include <string.h>
#include <fstream>
#include <ctime>
#include <mutex>
#include <atomic>
#include <vector>

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

struct location {
    float latitude = 0.0f;
    float longitude = 0.0f;
    float altitude = 0.0f;
    long long int time_j = 0;
    string allcellinfo;

    int signalDbm = -999;

    vector<float> signal_x;
    vector<float> signal_y;

    atomic<long long int> cnt = 0;
    mutex m;
};

void run_gui(location* loc) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1024, 768, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    
    glewInit();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            std::cout << "Processing some event: " << event.type
                      << " timestamp: " << event.motion.timestamp << std::endl;
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        ImGui::DockSpaceOverViewport(0, nullptr, ImGuiDockNodeFlags_None);

        {
            ImGui::Begin("Location");

            float lat, lon, alt;
            long long t;
            unsigned long long c;
            string Cell;
            int signal;
            vector<float> xs;
            vector<float> ys;

            {
                std::lock_guard<std::mutex> lk(loc->m);
                lat = loc->latitude;
                lon = loc->longitude;
                alt = loc->altitude;
                t = loc->time_j;
                Cell = loc->allcellinfo;
                signal = loc->signalDbm;
                xs = loc->signal_x;
                ys = loc->signal_y;
            }
            c = loc->cnt.load();

            ImGui::Text("Updates: %lld", c);
            ImGui::Text("lat: %.7f", lat);
            ImGui::Text("lon: %.7f", lon);
            ImGui::Text("alt: %.2f", alt);
            ImGui::Text("time: %lld", t);
            ImGui::Text("signalDbm: %d", signal);

            if (!xs.empty() && !ys.empty()) {
                if (ImPlot::BeginPlot("Cell Signal Strength")) {
                    ImPlot::SetupAxes("sample", "dBm");
                    ImPlot::PlotLine("signal", xs.data(), ys.data(), static_cast<int>(xs.size()));
                    ImPlot::EndPlot();
                }
            }

            ImGui::Text("ALL_CELL:");
            ImGui::TextUnformatted(Cell.c_str());

            ImGui::End();
        }

        ImGui::Render();
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        SDL_GL_SwapWindow(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext(nullptr);
    ImGui::DestroyContext(nullptr);
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

void run_server(location* loc) {
    context_t context(1);
    socket_t socket(context, socket_type::rep);
    socket.bind("tcp://*:5656");
    cout << "Server is listening on port 5656..." << endl;

    ofstream out("locations1.jsonl", ios::app);
    if (!out.is_open()) {
        cerr << "Failed to open file for writing: " << "locations1.jsonl" << endl;
        return;
    }
    cout << "Writing received JSON to: " << "locations1.jsonl" << endl;

    while (true) {
        message_t request;
        socket.recv(request, recv_flags::none);

        string req_str(static_cast<char*>(request.data()), request.size());
        cout << "Received request: " << req_str << endl;

        try {
            auto j = json::parse(req_str);
            location nloc;
            nloc.latitude = j.at("lat").get<float>();
            nloc.longitude = j.at("lon").get<float>();
            nloc.altitude = j.at("alt").get<float>();
            nloc.time_j = stoll(j.at("time").get<string>());
            nloc.allcellinfo = j.at("Cellallinfo").get<string>();

            if (j.contains("signalDbm") && !j.at("signalDbm").is_null()) {
                nloc.signalDbm = j.at("signalDbm").get<int>();
            } else {
                nloc.signalDbm = -999;
            }

            {
                lock_guard<mutex> lll(loc->m);
                loc->latitude = nloc.latitude;
                loc->longitude = nloc.longitude;
                loc->altitude = nloc.altitude;
                loc->time_j = nloc.time_j;
                loc->allcellinfo = nloc.allcellinfo;
                loc->signalDbm = nloc.signalDbm;

                float next_x = static_cast<float>(loc->signal_x.size());
                loc->signal_x.push_back(next_x);
                loc->signal_y.push_back(static_cast<float>(nloc.signalDbm));

                if (loc->signal_x.size() > 200) {
                    loc->signal_x.erase(loc->signal_x.begin());
                    loc->signal_y.erase(loc->signal_y.begin());
                }
            }
            loc->cnt++;

        } catch (const exception& e) {
            cout << "parse errr json " << e.what() << endl;
        }

        out << req_str << "\n";
        out.flush();

        string reply_str = "OK";
        message_t reply(reply_str.size());
        memcpy(reply.data(), reply_str.c_str(), reply_str.size());
        socket.send(reply, send_flags::none);
    }
}

void run_json_parser(location* loc, const string& file) {
    ifstream in(file);
    json arr;
    in >> arr; 

    static long long start_time = -1;

for (const auto& j : arr) {
    try {
        lock_guard<mutex> lk(loc->m);

        loc->signalDbm = j.value("signalDbm", -999);
        loc->latitude  = j.value("lat", 0.0f);
        loc->longitude = j.value("lon", 0.0f);
        loc->altitude  = j.value("alt", 0.0f);
        loc->time_j    = j["time"].is_number() ? j["time"].get<long long>() : 0;

        if (start_time < 0) start_time = loc->time_j;

        float x = (float)(loc->time_j - start_time) / 1000.0f;

        loc->signal_x.push_back(x);
        loc->signal_y.push_back((float)loc->signalDbm);

        if (loc->signal_x.size() > 200) {
            loc->signal_x.erase(loc->signal_x.begin());
            loc->signal_y.erase(loc->signal_y.begin());
        }

        loc->cnt++;

    } catch (...) {
        continue;
    }

    this_thread::sleep_for(chrono::milliseconds(50));
}
}

int main(int argc, char** argv) {
    static location locationInfo{};

    thread gui_thread(run_gui, &locationInfo);

    if (argc == 3 && string(argv[1]) == "--json") {
        thread json_thread(run_json_parser, &locationInfo, string(argv[2]));
        json_thread.join();
    } else {
        thread server_thread(run_server, &locationInfo);
        server_thread.join();
    }

    gui_thread.join();
    return 0;
}