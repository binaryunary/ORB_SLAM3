/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza. Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of
 * Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <gps_common/GPSFix.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sys/signal.h>

#include <opencv2/core/core.hpp>

#include "GPSPos.h"
#include "System.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace gps_common;

class MessageStreamMonitor
{
  public:
    MessageStreamMonitor(ORB_SLAM3::System *slam) : active(false), slam(slam)
    {
    }

    void messageReceived()
    {
        std::unique_lock<std::mutex> lock(last_message_received_mutex);
        if (!active)
        {
            active = true;
        }
        last_message_received = ros::Time::now();
        lock.unlock();
    }

    void checkIfStreamEnded()
    {
        ros::Duration no_new_message_duration(1); // 1.0 seconds

        while (ros::ok())
        {
            std::unique_lock<std::mutex> lock(last_message_received_mutex);
            ros::Time last_received_copy = last_message_received;
            lock.unlock();

            if ((ros::Time::now() - last_received_copy) > no_new_message_duration && active)
            {
                cout << "No new messages received for more than 1 second, assuming stream ended." << endl;
                cout << "Shutting down" << endl;
                slam->SaveData();
                slam->Shutdown();
            }

            ros::Duration(1).sleep();
        }
    }

  private:
    bool active;
    ORB_SLAM3::System *slam;
    ros::Time last_message_received;
    std::mutex last_message_received_mutex;
};

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM3::System *pSLAM, MessageStreamMonitor *pMonitor) : mpSLAM(pSLAM), mpMonitor(pMonitor)
    {
    }

    void GrabImage(const ImageConstPtr &msg, const GPSFixConstPtr &gps);

    ORB_SLAM3::System *mpSLAM;
    MessageStreamMonitor *mpMonitor;
};

class CustomSigIntHandler
{
  public:
    CustomSigIntHandler(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM)
    {
        instance = this;
    }

    void sigIntHandler(int sig)
    {
        cout << "Custom SIGINT handler called" << endl;
        mpSLAM->SaveData();
        mpSLAM->Shutdown();
        ros::shutdown();
        cout << "Custom SIGINT handler finished" << endl;
    }

    static void staticSigIntHandler(int sig)
    {
        if (instance)
        {
            instance->sigIntHandler(sig);
        }
    }

    ORB_SLAM3::System *mpSLAM;

  private:
    static CustomSigIntHandler *instance;
};

// Initialize the static instance pointer to nullptr
CustomSigIntHandler *CustomSigIntHandler::instance = nullptr;

int main(int argc, char **argv)
{
    cout << "+++ GPS +++" << endl;

    ros::init(argc, argv, "Mono", ros::init_options::NoSigintHandler);

    if (argc < 5)
    {
        cerr << endl
             << "Usage: rosrun orbslam mono_gps <path_to_vocabulary> <path_to_settings> <'mapping'|'localization'> "
                "<out_dir> [<active_map>]"
             << endl;
        ros::shutdown();
        return 1;
    }

    std::string vocPath = argv[1];
    std::string settingsPath = argv[2];
    bool activateLocalizationMode = std::string(argv[3]) == "localization";
    std::string outDir = argv[4];
    int activeMap = 0;

    cout << "Localization mode activated: " << activateLocalizationMode << endl;

    if (argc == 6)
    {
        activeMap = std::stoi(argv[5]);
        cout << "Active map: " << activeMap << endl;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(vocPath, settingsPath, ORB_SLAM3::System::MONOCULAR, true, activateLocalizationMode, outDir,
                           activeMap);
    MessageStreamMonitor messageStreamMonitor(&SLAM);
    CustomSigIntHandler customSigIntHandler(&SLAM);
    signal(SIGINT, CustomSigIntHandler::staticSigIntHandler);

    ros::start();

    ImageGrabber igb(&SLAM, &messageStreamMonitor);

    ros::NodeHandle nodeHandler;
    message_filters::Subscriber<Image> image_sub(nodeHandler, "/camera/image_raw", 1);
    message_filters::Subscriber<GPSFix> gps_sub(nodeHandler, "/gps/gps", 1);

    typedef sync_policies::ApproximateTime<Image, GPSFix> SyncPolicy;
    Synchronizer<SyncPolicy> sync(SyncPolicy(30), image_sub, gps_sub);

    sync.registerCallback(&ImageGrabber::GrabImage, &igb);

    // std::thread monitor_thread(&MessageStreamMonitor::checkIfStreamEnded, &messageStreamMonitor);

    cout << "Calling ros::spin()...";
    ros::spin();

    // cout << "ros::spin() finished, joining monitor thread..." << endl;
    // monitor_thread.join();

    cout << "calling ros::shutdon()" << endl;
    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const ImageConstPtr &image, const GPSFixConstPtr &gps)
{
    // mpMonitor->messageReceived();

    // cout << "image: " << image->header.stamp << ", gps:" << gps->header.stamp << endl;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    GPSPos gpsPos = {gps->latitude, gps->longitude, gps->altitude};

    mpSLAM->TrackMonocularGPS(cv_ptr->image, cv_ptr->header.stamp.toSec(), gpsPos);
}
