// Test JSON input / output using jsoncpp.

#include <cinttypes>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

#include <json/json.h>

std::string json_doc = "\n\
{\n\
    \"encoding\": \"UTF-8\",\n\
    \"plugins\": [\n\
        \"python\",\n\
        \"c++\",\n\
        \"ruby\"\n\
    ],\n\
    \"indent\": { \"length\": 3, \"use_space\": true }\n\
}\n";

// Notes on SITL JSON interface:
// 
// Run SITL with -f json:127.0.0.1
// Physics backend should listen on port 9002
// ArduPilot SITL output data in binary format:
struct pkt
{
    uint16_t magic = 18458;
    uint16_t frame_rate;
    uint32_t frame_count;
    uint16_t pwm[16];
};

// ArduPilot SITL input data is JSON format (mandatory fields)
std::string sitl_out_required = "\n\
{\n\
    \"timestamp\": 2500,\n\
    \"imu\": {\n\
        \"gyro\": [0, 0, 0],\n\
        \"accel_body\": [0, 0, 0]\n\
    },\n\
    \"position\": [0, 0, 0],\n\
    \"attitude\": [0, 0, 0],\n\
    \"velocity\": [0, 0, 0]\n\
}\n";

// ArduPilot SITL input data is JSON format (all fields)
std::string sitl_out_sensors = "\n\
{\n\
    \"timestamp\": 2500,\n\
    \"imu\": {\n\
        \"gyro\": [0, 0, 0],\n\
        \"accel_body\": [0, 0, 0]\n\
    },\n\
    \"position\": [0, 0, 0],\n\
    \"attitude\": [0, 0, 0],\n\
    \"velocity\": [0, 0, 0],\n\
    \"rng_1\": 0,\n\
    \"rng_2\": 0,\n\
    \"rng_3\": 0,\n\
    \"rng_4\": 0,\n\
    \"rng_5\": 0,\n\
    \"rng_6\": 0,\n\
    \"windvane\": {\n\
        \"direction\": 0,\n\
        \"speed\": 0\n\
    }\n\
}\n";

int main(int argc, const char* argv[])
{
    try
    {
        std::cout << "ArduPilotJson" <<  "\n";
        std::cout << json_doc <<  "\n";
        std::cout << sitl_out_required <<  "\n";
        std::cout << sitl_out_sensors <<  "\n";

        // example 1: parse JSON
        {
            std::cout << "example1: parse" << "\n\n";

            Json::CharReaderBuilder builder;
            builder["collectComments"] = false;        
            std::unique_ptr<Json::CharReader> reader(builder.newCharReader());

            Json::Value root;
            std::string errors;
            bool ok = reader->parse(
                json_doc.c_str(),
                json_doc.c_str() + json_doc.size(),
                &root,
                &errors);

            // iterate over all members
            auto members = root.getMemberNames();
            for (auto&& it = members.begin(); it != members.end(); ++it) {
                std::cout << *it << "\n";
            }

            // get with default
            auto value = root.get("encoding", "UTF-8").asString();
            std::cout << "encoding: " << value << "\n";
        }

        // example 2: create JSON
        {
            std::cout << "example2: create" << "\n\n";

            Json::Value root;

            Json::Value timestamp(2500);
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
            accel_body[2] = 0.0;
            imu["accel_body"] = accel_body;
            root["imu"] = imu;

            Json::Value position(Json::ValueType::arrayValue);
            position.resize(3);
            position[0] = 0.0;
            position[1] = 0.0;
            position[2] = 0.0;
            root["position"] = position;

            Json::Value attitude(Json::ValueType::arrayValue);
            attitude.resize(3);
            attitude[0] = 0.0;
            attitude[1] = 0.0;
            attitude[2] = 0.0;
            root["attitude"] = attitude;

            Json::Value velocity(Json::ValueType::arrayValue);
            velocity.resize(3);
            velocity[0] = 0.0;
            velocity[1] = 0.0;
            velocity[2] = 0.0;
            root["velocity"] = attitude;

            Json::Value rng_1(0.0);
            root["rng_1"] = rng_1;

            Json::Value rng_2(0.0);
            root["rng_2"] = rng_2;

            Json::Value windvane;
            Json::Value direction(0.0);
            windvane["direction"] = direction;
            Json::Value speed(0.0);
            windvane["speed"] = speed;
            root["windvane"] = windvane;

            std::cout << root << "\n";
        }

   }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
