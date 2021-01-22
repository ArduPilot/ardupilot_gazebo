// Test JSON input / output using jsoncpp.

#include <exception>
#include <iostream>
#include <string>

#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

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
        // example 1: parse JSON
        {
            using namespace rapidjson;

            std::cout << "example1: parse" << "\n";

            // Json::CharReaderBuilder builder;
            // builder["collectComments"] = false;        
            // std::unique_ptr<Json::CharReader> reader(builder.newCharReader());
            Document d;
            d.Parse(sitl_out_sensors.c_str());


            // // iterate over all members
            // auto members = root.getMemberNames();
            // for (auto&& it = members.begin(); it != members.end(); ++it) {
            //     std::cout << *it << "\n";
            // }

            // // get with default
            // auto value = root.get("encoding", "UTF-8").asString();
            // std::cout << "encoding: " << value << "\n";

            // Stringify
            StringBuffer s;
            Writer<StringBuffer> writer(s);
            d.Accept(writer);

            // JSON output
            std::string json_str = "\n" + std::string(s.GetString()) + "\n\n";
            std::cout << json_str;
        }

        // example 2: create JSON
        {
            using namespace rapidjson;

            std::cout << "example2: create" << "\n";

            StringBuffer s;
            Writer<StringBuffer> writer(s);            

            writer.StartObject();

            writer.Key("timestamp");
            writer.Double(2500.0);

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
            writer.Double(0.0);
            writer.EndArray();
            writer.EndObject();

            writer.Key("position");
            writer.StartArray();
            writer.Double(0.0);
            writer.Double(0.0);
            writer.Double(0.0);
            writer.EndArray();

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
            writer.Double(0.0);
            writer.Key("speed");
            writer.Double(0.0);
            writer.EndObject();

            // JSON output
            std::string json_str = "\n" + std::string(s.GetString()) + "\n\n";
            std::cout << json_str;
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
