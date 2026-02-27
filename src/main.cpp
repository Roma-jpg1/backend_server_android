#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <chrono>
#include <string.h>
#include <fstream>     
#include <ctime>
#include <mutex>
#include <atomic>

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
    float latitude;
    float longitude;
    float altitude;
    long long int time_j;
    string allcellinfo;
    atomic<long long int> cnt=0;
    mutex m;

};

void run_gui(location* loc) {
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER);
    SDL_Window* window = SDL_CreateWindow(
        "Backend start", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1024, 768, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);

    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Включить Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Включить Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;         // Включить Docking

    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL3_Init("#version 330");

    bool running = true;
    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            std::cout << "Processing some event: "<< event.type << " timestamp: " << event.motion.timestamp << std::endl;
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

            {
                std::lock_guard<std::mutex> lk(loc->m);
                lat = loc->latitude;
                lon = loc->longitude;
                alt = loc->altitude;
                t   = loc->time_j;
                Cell = loc->allcellinfo;
            }
            c = loc->cnt.load();

            ImGui::Text("Updates: %lld", c);
            ImGui::Text("lat: %.7f", lat);
            ImGui::Text("lon: %.7f", lon);
            ImGui::Text("alt: %.2f", alt);
            ImGui::Text("time: %lld", t);
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

        try{
            auto j =json::parse(req_str);
            location nloc;
            nloc.latitude = j.at("lat").get<float>();
            nloc.longitude = j.at("lon").get<float>();
            nloc.altitude = j.at("alt").get<float>();
            nloc.time_j = stoll(j.at("time").get<string>());
            nloc.allcellinfo = j.at("Cellallinfo").get<string>();

            {
                lock_guard<mutex> lll (loc->m);
                loc->latitude  = nloc.latitude;
                loc->longitude = nloc.longitude;
                loc->altitude  = nloc.altitude;
                loc->time_j    = nloc.time_j;
                loc->allcellinfo = nloc.allcellinfo;
            }
            loc->cnt++;

        } catch(const exception& e){
            cout<<"parse errr json"<<e.what()<<endl;

        }
        out << req_str << "\n";
        out.flush();

        string reply_str = "OK";
        message_t reply(reply_str.size());
        memcpy(reply.data(), reply_str.c_str(), reply_str.size());
        socket.send(reply, send_flags::none);
    }
}

int main(){
    static location locationInfo{};
    thread gui_thread(run_gui, &locationInfo);
    thread server_thread(run_server, &locationInfo);
    gui_thread.join();
    server_thread.join();
    return 0;
}