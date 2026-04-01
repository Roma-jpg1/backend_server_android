#include <iostream>
#include <zmq.hpp>
#include <thread>
#include <chrono>
#include <string.h>
#include <fstream>     
#include <ctime>
#include "nlohmann/json.hpp"
using json = nlohmann::json;
using namespace std;
using namespace zmq;

struct location {
    float latitude;
    float longitude;
    float altitude;
};

void run_gui(location* loc) {
    while(true){
        cout << "GUI is running..." << endl;
        this_thread::sleep_for(chrono::seconds(5));
    }
}

static string make_filename() {
    std::time_t t = std::time(nullptr);
    return "locations_" + std::to_string((long long)t) + ".jsonl";
}

void run_server(location* loc) {
    context_t context(1);
    socket_t socket(context, socket_type::rep);
    socket.bind("tcp://*:5656");
    cout << "Server is listening on port 5656..." << endl;

    const string filename = make_filename();
    ofstream out(filename, ios::app);
    if (!out.is_open()) {
        cerr << "Failed to open file for writing: " << filename << endl;
        return;
    }
    cout << "Writing received JSON to: " << filename << endl;

    while (true) {
        message_t request;
        socket.recv(request, recv_flags::none);

        string req_str(static_cast<char*>(request.data()), request.size());
        cout << "Received request: " << req_str << endl;

        out << req_str << "\n";
        out.flush();

        string reply_str = "OK";
        message_t reply(reply_str.size());
        memcpy(reply.data(), reply_str.c_str(), reply_str.size());
        socket.send(reply, send_flags::none);
    }
}

bool load_data(const string& filename, vector<DataPoint>& out) {
    ifstream in(filename);

    if (!in.is_open()) {
        cerr << "Cannot open file: " << filename << endl;
        return false;
    }

    json j;
    in >> j;

    out.clear();

    for (const auto& item : j) {
        DataPoint p;

        p.lat       = item.value("lat", 0.0f);
        p.lon       = item.value("lon", 0.0f);
        p.alt       = item.value("alt", 0.0f);
        p.accuracy  = item.value("accuracy", 0.0f);
        p.signalDbm = item.value("signalDbm", -999);
        p.time      = item.value("time", 0LL);

        out.push_back(p);
    }

    cout << "Loaded points: " << out.size() << endl;
    return true;
}

int main(){
    load_data("data.json", dataset);
    static location locationInfo{};
    thread gui_thread(run_gui, &locationInfo);
    thread server_thread(run_server, &locationInfo);
    gui_thread.join();
    server_thread.join();
    return 0;
}
