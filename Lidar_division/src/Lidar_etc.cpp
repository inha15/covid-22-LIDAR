#include "Lidar_declare.h"

using namespace std;

Fps::Fps(): m_count(0){ prev_clock = 0; };


void Fps::update(){
    double tmp = ros::Time::now().toSec();        
    cur_clock = tmp;
    interval = cur_clock - prev_clock;
    prev_clock = cur_clock;
    m_fps = 1 / interval;
    m_count++;
        
    cout << "Interval: " << interval << " sec";
    cout << "\tFPS: " << m_fps << " frame/sec" << endl;
    cout << "Loop " << m_count << endl;
}