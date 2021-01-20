// Test socket connection to SITL
//
// Usage
// -----
// 
// Start SITL:
//
//  $ sim_vehicle.py -v Rover -f JSON:127.0.0.1 --aircraft=sitl_json
//
// Start the example:
//
//  $ SITLSocketExample  
// 

#include <functional>
#include <fcntl.h>
#ifdef _WIN32
  #include <Winsock2.h>
  #include <Ws2def.h>
  #include <Ws2ipdef.h>
  #include <Ws2tcpip.h>
  using raw_type = char;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  using raw_type = void;
#endif

#if defined(_MSC_VER)
  #include <BaseTsd.h>
  typedef SSIZE_T ssize_t;
#endif

#include <unistd.h>

#include <chrono>
#include <exception>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <iostream>
#include <sstream>
#include <json/json.h>

/// \brief SITL JSON output packet
struct ServoPacket
{
    uint16_t magic = 18458;
    uint16_t frame_rate = 0;
    uint32_t frame_count = 0;
    uint16_t pwm[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};

// Private data class
class ArduPilotSocketPrivate
{
  /// \brief constructor
  public: ArduPilotSocketPrivate()
  {
    // initialize socket udp socket
    fd = socket(AF_INET, SOCK_DGRAM, 0);
    #ifndef _WIN32
    // Windows does not support FD_CLOEXEC
    fcntl(fd, F_SETFD, FD_CLOEXEC);
    #endif
  }

  /// \brief destructor
  public: ~ArduPilotSocketPrivate()
  {
    if (fd != -1)
    {
      ::close(fd);
      fd = -1;
    }
  }

  /// \brief Bind to an adress and port
  /// \param[in] _address Address to bind to.
  /// \param[in] _port Port to bind to.
  /// \return True on success.
  public: bool Bind(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (bind(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Connect to an adress and port
  /// \param[in] _address Address to connect to.
  /// \param[in] _port Port to connect to.
  /// \return True on success.
  public : bool Connect(const char *_address, const uint16_t _port)
  {
    struct sockaddr_in sockaddr;
    this->MakeSockAddr(_address, _port, sockaddr);

    if (connect(this->fd, (struct sockaddr *)&sockaddr, sizeof(sockaddr)) != 0)
    {
      shutdown(this->fd, 0);
      #ifdef _WIN32
      closesocket(this->fd);
      #else
      close(this->fd);
      #endif
      return false;
    }
    int one = 1;
    setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT,
        reinterpret_cast<const char *>(&one), sizeof(one));

    #ifdef _WIN32
    u_long on = 1;
    ioctlsocket(this->fd, FIONBIO,
              reinterpret_cast<u_long FAR *>(&on));
    #else
    fcntl(this->fd, F_SETFL,
        fcntl(this->fd, F_GETFL, 0) | O_NONBLOCK);
    #endif
    return true;
  }

  /// \brief Make a socket
  /// \param[in] _address Socket address.
  /// \param[in] _port Socket port
  /// \param[out] _sockaddr New socket address structure.
  public: void MakeSockAddr(const char *_address, const uint16_t _port,
    struct sockaddr_in &_sockaddr)
  {
    memset(&_sockaddr, 0, sizeof(_sockaddr));

    #ifdef HAVE_SOCK_SIN_LEN
      _sockaddr.sin_len = sizeof(_sockaddr);
    #endif

    _sockaddr.sin_port = htons(_port);
    _sockaddr.sin_family = AF_INET;
    _sockaddr.sin_addr.s_addr = inet_addr(_address);
  }

  public: ssize_t Send(const void *_buf, size_t _size)
  {
    return send(this->fd, _buf, _size, 0);
  }

  /// \brief Receive data
  /// \param[out] _buf Buffer that receives the data.
  /// \param[in] _size Size of the buffer.
  /// \param[in] _timeoutMS Milliseconds to wait for data.
  public: ssize_t Recv(void *_buf, const size_t _size, uint32_t _timeoutMs)
  {
    fd_set fds;
    struct timeval tv;

    FD_ZERO(&fds);
    FD_SET(this->fd, &fds);

    tv.tv_sec = _timeoutMs / 1000;
    tv.tv_usec = (_timeoutMs % 1000) * 1000UL;

    if (select(this->fd+1, &fds, NULL, NULL, &tv) != 1)
    {
        return -1;
    }

    #ifdef _WIN32
    return recv(this->fd, reinterpret_cast<char *>(_buf), _size, 0);
    #else
    return recv(this->fd, _buf, _size, 0);
    #endif
  }

  /// \brief Socket handle
  private: int fd;
};

// Private data class
class ArduPilotSimPrivate
{
  /// \brief keep track of controller update sim-time.
  public: std::chrono::duration<double> lastControllerUpdateTime;

  /// \brief Controller update mutex.
  public: std::mutex mutex;

  /// \brief Ardupilot Socket for receive motor command on gazebo
  public: ArduPilotSocketPrivate socket_in;

  /// \brief Ardupilot Socket to send state to Ardupilot
  public: ArduPilotSocketPrivate socket_out;

  /// \brief Ardupilot address
  public: std::string fdm_addr;

  /// \brief The Ardupilot listen address
  public: std::string listen_addr;

  /// \brief Ardupilot port for receiver socket
  public: uint16_t fdm_port_in;

  /// \brief Ardupilot port for sender socket
  public: uint16_t fdm_port_out;

  /// \brief false before ardupilot controller is online
  /// to allow gazebo to continue without waiting
  public: bool arduPilotOnline;

  /// \brief number of times ArduCotper skips update
  public: int connectionTimeoutCount;

  /// \brief number of times ArduCotper skips update
  /// before marking ArduPilot offline
  public: int connectionTimeoutMaxCount;
};

/////////////////////////////////////////////////
class ArduPilotSim {
    private: std::unique_ptr<ArduPilotSimPrivate> dataPtr;

    public: ArduPilotSim()
    : dataPtr(new ArduPilotSimPrivate())
    {
        this->dataPtr->arduPilotOnline = false;
        this->dataPtr->connectionTimeoutCount = 0;
    }

    /////////////////////////////////////////////////
    public: ~ArduPilotSim()
    {
    }

    /////////////////////////////////////////////////
    public: void Load()
    {
        // Controller time control.
        this->dataPtr->lastControllerUpdateTime;

        // Initialise ardupilot sockets
        if (!InitArduPilotSockets())
        {
            return;
        }
        std::cout << "Sockets initialized" << "\n";

        // Missed update count before we declare arduPilotOnline status false
        this->dataPtr->connectionTimeoutMaxCount = 10;

        std::cout << "ArduPilot ready to fly. The force will be with you\n";
    }

    /////////////////////////////////////////////////
    public: void Update(const std::chrono::duration<double>& sim_time)
    {
        std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

        // Update the control surfaces and publish the new state.
        if (sim_time > this->dataPtr->lastControllerUpdateTime)
        {
            this->ReceiveMotorCommand(sim_time);
            if (this->dataPtr->arduPilotOnline)
            {
            this->SendState(sim_time);
            }
        }

        this->dataPtr->lastControllerUpdateTime = sim_time;
    }

    /////////////////////////////////////////////////
    bool InitArduPilotSockets() const
    {
        this->dataPtr->fdm_addr = "127.0.0.1";
        this->dataPtr->listen_addr = "127.0.0.1";
        this->dataPtr->fdm_port_in = 9002;
        this->dataPtr->fdm_port_out = 9002;

        if (!this->dataPtr->socket_in.Bind(this->dataPtr->listen_addr.c_str(),
            this->dataPtr->fdm_port_in))
        {
            std::cout << "Failed to bind with " << this->dataPtr->listen_addr
                << ":" << this->dataPtr->fdm_port_in << " aborting.\n";
            return false;
        }

        if (!this->dataPtr->socket_out.Connect(this->dataPtr->fdm_addr.c_str(),
            this->dataPtr->fdm_port_out))
        {
            std::cout << "Failed to connect with " << this->dataPtr->fdm_addr
                << ":" << this->dataPtr->fdm_port_out << " aborting.\n";
            return false;
        }

        return true;
    }

    /////////////////////////////////////////////////
    void ReceiveMotorCommand(const std::chrono::duration<double>& sim_time)
    {
        // Added detection for whether ArduPilot is online or not.
        // If ArduPilot is detected (receive of fdm packet from someone),
        // then socket receive wait time is increased from 1ms to 1 sec
        // to accomodate network jitter.
        // If ArduPilot is not detected, receive call blocks for 1ms
        // on each call.
        // Once ArduPilot presence is detected, it takes this many
        // missed receives before declaring the FCS offline.

        ServoPacket pkt;
        uint32_t waitMs;
        if (this->dataPtr->arduPilotOnline)
        {
            // increase timeout for receive once we detect a packet from
            // ArduPilot FCS.
            waitMs = 1000;
        }
        else
        {
            // Otherwise skip quickly and do not set control force.
            waitMs = 1;
        }
        ssize_t recvSize =
            this->dataPtr->socket_in.Recv(&pkt, sizeof(ServoPacket), waitMs);

        // Drain the socket in the case we're backed up
        int counter = 0;
        ServoPacket last_pkt;
        while (true)
        {
            // last_pkt = pkt;
            const ssize_t recvSize_last =
            this->dataPtr->socket_in.Recv(&last_pkt, sizeof(ServoPacket), 0ul);
            if (recvSize_last == -1)
            {
               break;
            }
            counter++;
            pkt = last_pkt;
            recvSize = recvSize_last;
        }
        if (counter > 0)
        {
            std::cout << "Drained n packets: " << counter << std::endl;
        }

        if (recvSize == -1)
        {
            // didn't receive a packet
            // gzdbg << "no packet\n";
            using namespace std::chrono_literals;
            std::this_thread::sleep_for(100ms);
            if (this->dataPtr->arduPilotOnline)
            {
                std::cout << "Broken ArduPilot connection, count ["
                        << this->dataPtr->connectionTimeoutCount
                        << "/" << this->dataPtr->connectionTimeoutMaxCount
                        << "]\n";
                if (++this->dataPtr->connectionTimeoutCount >
                    this->dataPtr->connectionTimeoutMaxCount)
                {
                    this->dataPtr->connectionTimeoutCount = 0;
                    this->dataPtr->arduPilotOnline = false;
                    std::cout << "Broken ArduPilot connection, resetting motor control.\n";
                }
            }
        }
        else
        {
            // @DEBUG_INFO
            std::cout << "magic       : " << pkt.magic << "\n";
            std::cout << "frame_rate  : " << pkt.frame_rate << "\n";
            std::cout << "frame_count : " << pkt.frame_count << "\n";
            std::cout << "pwm: ";
            for (auto i=0; i<4; ++i)
                std::cout << pkt.pwm[i] << ", ";
            std::cout << "\n\n";

            if (!this->dataPtr->arduPilotOnline)
            {
                std::cout << "ArduPilot controller online detected.\n";
                // made connection, set some flags
                this->dataPtr->connectionTimeoutCount = 0;
                this->dataPtr->arduPilotOnline = true;
            }
        }
    }

    /////////////////////////////////////////////////
    void SendState(const std::chrono::duration<double>& sim_time) const
    {
        // require the duration since sim start in seconds 
        auto sim_time_s = std::chrono::duration_cast<std::chrono::seconds>(sim_time);

        // fill a default output message - only update the timestamp.
        Json::Value root;

        Json::Value timestamp(sim_time_s.count());
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

        // Output the JSON UTF-8 string to the socket
        std::ostringstream oss;
        oss << root << "\n";
        // auto bytes_sent = this->dataPtr->socket_out.Send(oss.str().c_str(), oss.str().size());
        // std::cout << "Sent " << bytes_sent << " of " << oss.str().size() << " bytes to SITL\n";
    }
};

int main(int argc, const char* argv[])
{
    try
    {
        // create and initialise simulation
        ArduPilotSim ardupilot_sim;
        ardupilot_sim.Load();

        // loop at 50 Hz
        std::chrono::milliseconds update_duration(1000/50);
        auto sim_start_time = std::chrono::steady_clock::now();        
        auto last_update_time = sim_start_time; 

        while (true) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_update_time > update_duration) {
                
                auto sim_duration = now - sim_start_time;

                ardupilot_sim.Update(sim_duration);

                // std::cout << "sim_duration [ms]: " 
                //     << std::chrono::duration_cast<std::chrono::milliseconds>(sim_duration).count()
                //     << "\n";
                
                last_update_time += update_duration;
            }
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    return 0;
}
