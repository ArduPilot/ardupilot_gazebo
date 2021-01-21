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
#include <memory>
#include <string>
#include <thread>

#include <json/json.h>

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

        // JSON output
        //
        // for SITL set indentation = "", i.e. JSON string does not contain embedded newlines.
        //
        Json::StreamWriterBuilder builder;
        builder["commentStyle"] = "None";
        builder["indentation"] = "";
        builder["enableYAMLCompatibility"] = false;
        builder["dropNullPlaceholders"] = false;
        builder["useSpecialFloats"] = false;
        builder["emitUTF8"] = true;
        builder["precision"] = 17;
        builder["precisionType"] = "significant";

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
                    last_update_time += update_duration;

                    // update physics for time step here

                    // build JSON
                    Json::Value root;

                    // require the duration since sim start in seconds 
                    Json::Value timestamp(sim_time_ns.count() * 1.0E-9);
                    root["timestamp"] = timestamp;

                    Json::Value imu;
                    Json::Value gyro(Json::ValueType::arrayValue);
                    gyro.resize(3);
                    gyro[0] = 0.0;
                    gyro[1] = 0.0;
                    gyro[2] = 0.0;
                    imu["gyro"] = gyro;
                    Json::Value accel_body(Json::ValueType::arrayValue);
                    accel_body.resize(3);
                    accel_body[0] = 0.0;
                    accel_body[1] = 0.0;
                    accel_body[2] = -GRAVITY_MSS;
                    imu["accel_body"] = accel_body;
                    root["imu"] = imu;

                    Json::Value position(Json::ValueType::arrayValue);
                    position.resize(3);
                    position[0] = 0.0;
                    position[1] = 0.0;
                    position[2] = 0.0;
                    root["position"] = position;

                    // Json::Value attitude(Json::ValueType::arrayValue);
                    // attitude.resize(3);
                    // attitude[0] = 0.0;
                    // attitude[1] = 0.0;
                    // attitude[2] = 0.0;
                    // root["attitude"] = attitude;

                    // ArduPilot quaternion convention: q[0] = 1 for identity.
                    Json::Value quaternion(Json::ValueType::arrayValue);
                    quaternion.resize(4);
                    quaternion[0] = 1.0;
                    quaternion[1] = 0.0;
                    quaternion[2] = 0.0;
                    quaternion[3] = 0.0;
                    root["quaternion"] = quaternion;

                    Json::Value velocity(Json::ValueType::arrayValue);
                    velocity.resize(3);
                    velocity[0] = 0.0;
                    velocity[1] = 0.0;
                    velocity[2] = 0.0;
                    root["velocity"] = velocity;

                    Json::Value windvane;
                    Json::Value direction(1.57079633);
                    windvane["direction"] = direction;
                    Json::Value speed(5.5);
                    windvane["speed"] = speed;
                    root["windvane"] = windvane;

                    // send JSON
                    std::string json_str = "\n" + Json::writeString(builder, root) + "\n";
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
