// Test socket connection to SITL
//
// Use a copy of the ArduPilot SocketAPM class from
// ardupilot/libraries/AP_HAL/utility/Socket.h
//
// This example is based on socket_example.py which in turn is a
// simplified copy of ardupilot/libraries/SITL/examples/JSON/pybullet/robot.py
//
// Usage
// -----
// 
// 1. Build and run the example:
//
//  $ mkdir build && cd build
//  $ cmake ..
//  $ make
//  $ ./SITLSocketExample  
// 
// 2. Start SITL:
//
//  $ sim_vehicle.py -v Rover -f JSON:127.0.0.1 --aircraft=sitl_json
//
#include <Socket.h>

#include <chrono>
#include <cinttypes>
#include <exception>
#include <iostream>
#include <string>
#include <thread>

#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

const double GRAVITY_MSS = 9.80665;

// SIM_JSON.h
struct servo_packet {
    uint16_t magic;         // 18458 expected magic value
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

int main(int argc, const char* argv[])
{
    try
    {
        // configure socket 
        std::string address = "127.0.0.1";
        uint16_t port = 9002;
        SocketAPM sock(true);
        // sock.set_blocking(false);
        // sock.reuseaddress();
        sock.bind(address.c_str(), port);
        std::cout << "SITL interface set to " << address << ":" << port << "\n"; 

        // sitl input
        bool sitl_connected = false;
        uint16_t sitl_frame_rate = 50;
        uint32_t sitl_frame_count = 0;
        uint32_t sitl_last_frame = -1;
        uint16_t sitl_pwm[16];

        // start clocks
        uint16_t update_rate_hz = 50;
        auto update_duration = std::chrono::milliseconds(1000/update_rate_hz);
        auto sim_start_time = std::chrono::steady_clock::now();        
        auto last_update_time = sim_start_time; 

        while (true)
        {
            auto now = std::chrono::steady_clock::now();
            auto sim_time = now - sim_start_time;
            auto sim_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time);
            // std::cout << "sim_time_ns: " << sim_time_ns.count() << "\n";

            servo_packet pkt;
            ssize_t bytes_recv = 0;
            const char* ip;
            try
            {   
                bytes_recv = sock.recv(&pkt, sizeof(pkt), 100);
                sock.last_recv_address(ip, port);
            }
            catch(...)
            {
                using namespace std::chrono_literals;
                std::this_thread::sleep_for(1000ms);
                continue;
            }

            if (bytes_recv >= 0)
            {
                // inspect sitl packet
                std::cout << "recv " << bytes_recv << " bytes from " << ip << ":" << port << "\n";
                std::cout << "magic: " << pkt.magic << "\n";
                std::cout << "frame_rate: " << pkt.frame_rate << "\n";
                std::cout << "frame_count: " << pkt.frame_count << "\n";
                std::cout << "pwm: [";
                for (auto i=0; i<15; ++i)
                {
                    std::cout << pkt.pwm[i] << ", ";
                }
                std::cout << pkt.pwm[15] << "]\n";

                // check magic
                uint16_t magic = 18458;
                if (magic != pkt.magic)
                {
                    std::cout << "Incorrect protocol magic "
                        << pkt.magic << " should be "
                        << magic << "\n";
                    continue; 
                }

                // capture sitl packet
                sitl_frame_rate = pkt.frame_rate;
                sitl_frame_count = pkt.frame_count;
                std::copy(std::begin(pkt.pwm), std::end(pkt.pwm), std::begin(sitl_pwm));

                // check frame rate
                if (pkt.frame_rate != sitl_frame_rate)
                {
                    std::cout << "New frame rate " << pkt.frame_rate << "\n";
                    sitl_frame_rate = pkt.frame_rate;
                }

                // check frame order
                if (pkt.frame_count < sitl_last_frame)
                {
                    // controller has reset
                    std::cout << "Controller reset\n";
                }
                else if (pkt.frame_count == sitl_last_frame)
                {
                    // recv duplicate frame, skip
                    std::cout << "Duplicate input frame\n";
                    // continue;
                }
                else if (pkt.frame_count != sitl_last_frame + 1 && sitl_connected)
                {
                    // missed frame
                    std::cout << "Missed "
                        << pkt.frame_count - sitl_last_frame - 1
                        << " input frames\n";
                }
                sitl_last_frame = pkt.frame_count;

                if (!sitl_connected)
                {
                    sitl_connected = true;
                    std::cout << "Connected to " << address << "\n";
                }

                // update and send JSON
                // if (now - last_update_time > update_duration)
                {    
                    using namespace rapidjson;

                    last_update_time += update_duration;

                    // update physics for time step here

                    // require the duration since sim start in seconds 
                    double timestamp = sim_time_ns.count() * 1.0E-9;

                    // build JSON document
                    StringBuffer s;
                    Writer<StringBuffer> writer(s);            

                    writer.StartObject();

                    writer.Key("timestamp");
                    writer.Double(timestamp);

                    writer.Key("imu");
                    writer.StartObject();
                    writer.Key("gyro");
                    writer.StartArray();
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.EndArray();
                    writer.Key("accel_body");
                    writer.StartArray();
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.Double(-GRAVITY_MSS);
                    writer.EndArray();
                    writer.EndObject();

                    writer.Key("position");
                    writer.StartArray();
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.EndArray();

                    // ArduPilot quaternion convention: q[0] = 1 for identity.
                    writer.Key("quaternion");
                    writer.StartArray();
                    writer.Double(1.0);
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.EndArray();

                    writer.Key("velocity");
                    writer.StartArray();
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.Double(0.0);
                    writer.EndArray();

                    writer.Key("rng_1");
                    writer.Double(0.0);

                    writer.Key("rng_1");
                    writer.Double(0.0);

                    writer.Key("windvane");
                    writer.StartObject();
                    writer.Key("direction");
                    writer.Double(1.57079633);
                    writer.Key("speed");
                    writer.Double(5.5);
                    writer.EndObject();

                    // send JSON
                    std::string json_str = "\n" + std::string(s.GetString()) + "\n";
                    auto bytes_sent = sock.sendto(json_str.c_str(), json_str.size(), address.c_str(), port);

                    std::cout << "sent " << bytes_sent <<  " bytes to " 
                        << ip << ":" << port << "\n";
                    std::cout << json_str << "\n";

                }
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
