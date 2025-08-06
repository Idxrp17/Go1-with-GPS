#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>   // Needed for open()

using namespace UNITREE_LEGGED_SDK;

const double R = 6378137.0; // Earth radius in meters

// Open Arduino serial port
int openSerial(const char* dev) {
    int fd = open(dev, O_RDWR | O_NOCTTY);
    if (fd == -1) return -1;
    termios tty{};
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tcsetattr(fd, TCSANOW, &tty);
    return fd;
}

// Read line from serial
std::string readLine(int fd) {
    std::string s;
    char c;
    while (read(fd, &c, 1) > 0) {
        if (c == '\n') break;
        s += c;
    }
    return s;
}

// Haversine helpers
double hav(double a){ return sin(a/2)*sin(a/2); }
double distGPS(double la1,double lo1,double la2,double lo2){
    double p1 = la1 * M_PI / 180, p2 = la2 * M_PI / 180;
    return 2 * R * asin(sqrt(hav((la2-la1)*M_PI/180) +
                             cos(p1)*cos(p2)*hav((lo2-lo1)*M_PI/180)));
}
double bearing(double la1,double lo1,double la2,double lo2){
    la1 *= M_PI/180; lo1 *= M_PI/180; la2 *= M_PI/180; lo2 *= M_PI/180;
    double d = lo2 - lo1;
    double x = sin(d) * cos(la2);
    double y = cos(la1) * sin(la2) - sin(la1) * cos(la2) * cos(d);
    return fmod((atan2(x,y)*180/M_PI+360), 360);
}

// Convert relative (dx, dy) to GPS lat/lon
std::pair<double,double> rel2gps(double la,double lo,double dx,double dy){
    return { la + (dy/R)*(180/M_PI),
             lo + (dx/(R*cos(la*M_PI/180)))*(180/M_PI) };
}

// Load relative waypoints
std::vector<std::pair<double,double>> loadRel(const std::string& f){
    std::vector<std::pair<double,double>> v;
    std::ifstream in(f);
    double dx, dy;
    char c;
    while (in >> dx >> c >> dy)
        v.push_back({dx, dy});
    return v;
}

// Go1 robot wrapper
class Go1 {
public:
    Go1()
        : safe(LeggedType::Go1),
          udp(8090, "192.168.123.161", 8082, 0) // 3.8.0 constructor format
    {
        udp.InitCmdData(cmd);
    }

    void move(double vx, double vy, double wz) {
        cmd.mode = 0;
        cmd.gaitType = 1;
        cmd.velocity[0] = vx;
        cmd.velocity[1] = vy;
        cmd.yawSpeed = wz;
        udp.SetSend(cmd);
    }

    Safety safe;
    UDP udp;
    HighCmd cmd;
    LowState st;
};

int main() {
    auto rel = loadRel("waypoints.txt");
    if (rel.empty()) {
        std::cerr << "No waypoints loaded!\n";
        return -1;
    }

    int gps = openSerial("/dev/ttyACM0"); // Arduino Uno is usually ACM0
    if (gps < 0) {
        std::cerr << "Failed to open Arduino GPS serial port\n";
        return -1;
    }

    Go1 go;
    go.udp.SetDisconnectTime(5.0);

    double lat0 = 0, lon0 = 0;

    // Wait for valid GPS fix
    std::cout << "Waiting for GPS fix...\n";
    while (true) {
        std::string l = readLine(gps);
        double lat, lon, s, a;
        int sat;
        if (sscanf(l.c_str(), "%lf,%lf,%lf,%lf,%d",
                   &lat, &lon, &s, &a, &sat) == 5 && sat >= 4) {
            lat0 = lat; lon0 = lon;
            break;
        }
    }
    std::cout << "GPS fix acquired! Starting navigation...\n";

    // Convert to absolute GPS waypoints
    std::vector<std::pair<double,double>> wps;
    for (auto &p : rel)
        wps.push_back(rel2gps(lat0, lon0, p.first, p.second));

    size_t i = 0;
    while (i < wps.size()) {
        std::string l = readLine(gps);
        double lat, lon, s, a;
        int sat;
        if (sscanf(l.c_str(), "%lf,%lf,%lf,%lf,%d",
                   &lat, &lon, &s, &a, &sat) != 5) continue;

        go.udp.Recv();
        go.udp.GetRecv(go.st);
        double yaw = go.st.imu.rpy[2] * 180 / M_PI;

        double d = distGPS(lat, lon, wps[i].first, wps[i].second);
        double b = bearing(lat, lon, wps[i].first, wps[i].second);
        double err = fmod((b - yaw + 540), 360) - 180;

        if (d < 1) { // Reached waypoint
            go.move(0, 0, 0);
            i++;
            sleep(2);
            continue;
        }
        if (err > 10) go.move(0, 0, 0.3);
        else if (err < -10) go.move(0, 0, -0.3);
        else go.move(0.2, 0, 0);

        usleep(20000);
    }

    go.move(0, 0, 0);
    std::cout << "Navigation complete.\n";
}
