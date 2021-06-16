#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>

#include "ouster/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

using namespace ouster;

// const int N_SCANS = 1;
const size_t UDP_BUF_SIZE = 65536;

std::atomic<bool> quit_now;

void FATAL(const char* msg) {
    std::cerr << msg << std::endl;
    std::exit(EXIT_FAILURE);
}

void thread_input() {
        // char ch = 0;
        while(1)
      {
            // std::cin >> ch;
            if (getchar() == '\n') break;
      }
}

int loop(std::string filename, sensor::sensor_info info, std::shared_ptr<ouster::sensor::client> handle) {

    /*
     * The sensor client consists of the network client and a library for
     * reading and working with data.
     *
     * The network client supports reading and writing a limited number of
     * configuration parameters and receiving data without working directly with
     * the socket APIs. See the `client.h` for more details. The minimum
     * required parameters are the sensor hostname/ip and the data destination
     * hostname/ip.
     */

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */

    // Raw metadata can be parsed into a `sensor_info` struct

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;
    std::vector<LidarScan> scans;

    // A ScanBatcher can be used to batch packets into scans
    sensor::packet_format pf = sensor::get_format(info);
    ScanBatcher batch_to_scan(info.format.columns_per_frame, pf);

    /*
     * The network client provides some convenience wrappers around socket APIs
     * to facilitate reading lidar and IMU data from the network. It is also
     * possible to configure the sensor offline and read data directly from a
     * UDP socket.
     */
    std::cerr << "Capturing points... ";

    std::ofstream out_file;
    out_file.open(filename);
    out_file << std::fixed << std::setprecision(4);

    // buffer to store raw packet data
    std::unique_ptr<uint8_t[]> packet_buf(new uint8_t[UDP_BUF_SIZE]);
    LidarScan scan{w, h};
    // std::cout << w << std::endl;
    // out_file  << " " << w << std::endl;
    // std::vector<LidarScan::ts_t> times = ouster::LidarScan::timestamps();
    while (!quit_now) {
        
        // wait until sensor data is available
        sensor::client_state st = sensor::poll_client(*handle);

        // check for error status
        if (st & sensor::CLIENT_ERROR)
            FATAL("Sensor client returned error state!");

        // check for lidar data, read a packet and add it to the current batch
        if (st & sensor::LIDAR_DATA) {
            if (!sensor::read_lidar_packet(*handle, packet_buf.get(), pf))
                FATAL("Failed to read a packet of the expected size!");

            // batcher will return "true" when the current scan is complete
            if (batch_to_scan(packet_buf.get(), scan)) {
                std::vector<LidarScan::ts_t> times = scan.timestamps();
                
                // LidarScan provides access to azimuth block data and headers
                auto n_invalid = std::count_if(
                    scan.headers.begin(), scan.headers.end(),
                    [](const LidarScan::BlockHeader& h) {
                        return h.status != 0xffffffff;
                    });
                // retry until we receive a full scan
                if (n_invalid == 0) {
                    scans.push_back(scan);
                    auto reshaped1 = Eigen::Map<const Eigen::Array<LidarScan::raw_t, -1, 1>>(
                    scan.field(LidarScan::RANGE).data(), info.format.columns_per_frame * info.format.pixels_per_column);
                    auto final_ = reshaped1.cast<int>();
                        out_file << "Frame: " << scan.frame_id << std::endl;
                        for (size_t u = 0; u < h; u++) {
                            out_file << "Time: " << times[u].count() << std::endl;
                            for (size_t v = 0; v < w; v++) {
                                size_t i = u * w + v;
                                auto corrected_range = final_.row(i);
                                out_file  << corrected_range << std::endl;
                        }
                    }
                }
            }
        }
    }
    std::cerr << "ok" << std::endl;
    out_file.close();
    return EXIT_SUCCESS;
}


int main(int argc, char* argv[]) {
    if (argc != 4) {
        std::cerr << "Version: " << ouster::CLIENT_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: ouster_client_example <sensor_hostname> "
                     "<data_destination_ip> "
                     "<filename>"
                  << std::endl;

        return EXIT_FAILURE;
    }
    std::cerr << "Ouster client example " << ouster::CLIENT_VERSION
              << std::endl;
    /*
     * The sensor client consists of the network client and a library for
     * reading and working with data.
     *
     * The network client supports reading and writing a limited number of
     * configuration parameters and receiving data without working directly with
     * the socket APIs. See the `client.h` for more details. The minimum
     * required parameters are the sensor hostname/ip and the data destination
     * hostname/ip.
     */
    const std::string sensor_hostname = argv[1];
    const std::string data_destination = argv[2];
    const std::string filename = argv[3];

    std::cerr << "Connecting to \"" << sensor_hostname << "\"... ";


    auto handle = sensor::init_client(sensor_hostname, 7502, 7503);
    if (!handle) FATAL("Failed to connect");
    std::cerr << "ok" << std::endl;

    /*
     * Configuration and calibration parameters can be queried directly from the
     * sensor. These are required for parsing the packet stream and calculating
     * accurate point clouds.
     */
    std::cerr << "Gathering metadata..." << std::endl;
    auto metadata = sensor::get_metadata(*handle);

    // Raw metadata can be parsed into a `sensor_info` struct
    sensor::sensor_info info = sensor::parse_metadata(metadata);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    std::cerr << "  Firmware version:  " << info.fw_rev
              << "\n  Serial number:     " << info.sn
              << "\n  Product line:      " << info.prod_line
              << "\n  Scan dimensions:   " << w << " x " << h << std::endl;


    for (int i = 0; i < 1; i++) {
        quit_now = false;
        std::cout << filename << std::endl;
        std::thread t_1(thread_input);
        std::thread t(loop, filename, info, handle);
        t_1.join();
        quit_now = true;
        t.join();
    }


    return EXIT_SUCCESS;
}


