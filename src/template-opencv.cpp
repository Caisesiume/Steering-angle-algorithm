/*
 * Copyright (C) 2020  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Include the single-file, header-only middleware libcluon to create high-performance microservices
#include "cluon-complete.hpp"
// Include the OpenDLV Standard Message Set that contains messages that are usually exchanged for automotive or robotic applications
#include "opendlv-standard-message-set.hpp"

// Include the GUI and image processing header files from OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ctime>
#include <iostream>
#include <fstream>

auto calculateSteering(double rightIR, double leftIR, int rightCones, int leftCones, double steering)
{
    double incrementSteering = 0.045;
    steering = 0;

    if (rightIR <= 0.007)
    {
        steering = steering + incrementSteering;
    }
    if (leftIR <= 0.007)
    {
        steering = steering - incrementSteering;
    }

    if (rightCones == 0)
    {
        steering = -0.15;
    }
    if (leftCones == 0)
    {
        steering = 0.15;
    }
    return steering;
}

int32_t main(int32_t argc, char **argv)
{
    int32_t retCode{1};
    int yellowCones = 6;
    int blueCones = 0;
    double leftIR;
    double rightIR;
    double steering = 0.0;
    int directionEstablished = 0;
    std::string carDirection;
    int numberClockwise = 0;
    int numberCounterclockwise = 0;

    

    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) ||
        (0 == commandlineArguments.count("name")) ||
        (0 == commandlineArguments.count("width")) ||
        (0 == commandlineArguments.count("height")))
    {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else
    {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid())
        {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            opendlv::proxy::VoltageReading infrared;
            std::mutex infraredMutex;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env)
            {
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
            };
            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            ///Infrared sensor

            auto onVoltageReading = [&infrared, &infraredMutex, &rightIR, &leftIR, &yellowCones, &blueCones, &steering](cluon::data::Envelope &&env)
            {
                std::lock_guard<std::mutex> lck(infraredMutex);
                infrared = cluon::extractMessage<opendlv::proxy::VoltageReading>(std::move(env));
                if (env.senderStamp() == 3)
                {
                    rightIR = infrared.voltage();
                }
                else if (env.senderStamp() == 1)
                {
                    leftIR = infrared.voltage();
                }
            };

            od4.dataTrigger(opendlv::proxy::VoltageReading::ID(), onVoltageReading);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning())
            {
                // OpenCV data structure to hold an image.
                cv::Mat img;

                // Wait for a notification of a new frame.
                sharedMemory->wait();

                // Lock the shared memory.
                sharedMemory->lock();
                {
                    // Copy the pixels from the shared memory into our own data structure.
                    cv::Mat wrapped(HEIGHT, WIDTH, CV_8UC4, sharedMemory->data());
                    img = wrapped.clone();
                }
                
                std::pair<bool, cluon::data::TimeStamp> pair = sharedMemory->getTimeStamp();
                cluon::data::TimeStamp sampleT = pair.second;
                int64_t tStamp = cluon::time::toMicroseconds(sampleT);
                std::string ts = std::to_string(tStamp);
                std::string sampleTimeVar = "Sample Time: " + ts;
                sharedMemory->unlock();

                

                // HSV values reference: https://www.codespeedy.com/splitting-rgb-and-hsv-values-in-an-image-using-opencv-python/
                // Solution partly inspired by: https://stackoverflow.com/questions/9018906/detect-rgb-color-interval-with-opencv-and-c
                // AND: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/

                // Cone color detection

                using namespace cv;
                using namespace std;

                cv::Mat originalImg;
                img.copyTo(originalImg);

                cv::Mat hsvIMG;
                img.copyTo(hsvIMG);

                // Draw box around unnecessary part of car(to avoid conflicts with inrange below)
                cv::rectangle(img, cv::Point(150, 385), cv::Point(500, 500), cv::Scalar(0, 0, 0), CV_FILLED);
                // Draw box in region above cones, to avoid conflicts with irrelevant objects.
                cv::rectangle(img, cv::Point(0, 0), cv::Point(650, 250), cv::Scalar(0, 0, 0), CV_FILLED);

                cv::cvtColor(img, hsvIMG, cv::COLOR_BGR2HSV);

                cv::Mat justYellowColor;
                cv::Mat justYellowColor2;
                cv::Mat justBlueColor;

                inRange(hsvIMG, cv::Scalar(12, 20, 20), cv::Scalar(70, 100, 250), justYellowColor); // Yellow(low, high) - Yellow cones
                inRange(hsvIMG, cv::Scalar(8, 20, 20), cv::Scalar(11, 100, 250), justYellowColor2); // Yellow(low, high) - Yellow cones copy for lower ranges
                inRange(hsvIMG, cv::Scalar(80, 125, 8), cv::Scalar(135, 255, 210), justBlueColor); // Blue(low, high) - Blue cones

                justYellowColor = justYellowColor | justYellowColor2;

                cv::Rect bounding_rect;                   
                vector<vector<cv::Point>> yellowcontours; // Vector for storing yellow contours
                vector<vector<cv::Point>> bluecontours;   // Vector for storing blue contours
                cv::findContours(justBlueColor, bluecontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(justYellowColor, yellowcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

             

                cv::Rect boundRectangleYellow;
                cv::Rect boundRectangleBlue;

                cv::Rect largestboundRectangleBlue;
                cv::Rect largestboundRectangleYellow;

                int amountOfYellowCones = 0;
                int amountOfBlueCones = 0;

                int largestAreaYellow = 0;
                int largestAreaBlue = 0;

                for (unsigned int i = 0; i < bluecontours.size(); i++)
                {
                    boundRectangleBlue = cv::boundingRect(bluecontours[i]);
                    if (boundRectangleBlue.area() > 80)
                    {                                                                                                   
                        cv::rectangle(img, boundRectangleBlue.tl(), boundRectangleBlue.br(), cv::Scalar(0, 255, 0), 3); //<-- Light green rectangles

                        if (boundRectangleBlue.area() > largestAreaBlue)
                        {
                            largestAreaBlue = boundRectangleBlue.area();
                            largestboundRectangleBlue = cv::boundingRect(bluecontours[i]);
                        }

                        if (boundRectangleBlue.area() > 120)
                        {   
                            amountOfBlueCones += 1;
                        }
                    }
                }

                for (unsigned int i = 0; i < yellowcontours.size(); i++)
                {
                    boundRectangleYellow = cv::boundingRect(yellowcontours[i]);
                    if (boundRectangleYellow.area() > 80)
                    {                                                                                                       
                                                                                                                            
                        cv::rectangle(img, boundRectangleYellow.tl(), boundRectangleYellow.br(), cv::Scalar(6, 82, 58), 3); //<-- Dark green rectangles

                        if (boundRectangleYellow.area() > largestAreaYellow)
                        {
                            largestAreaYellow = boundRectangleYellow.area();
                            largestboundRectangleYellow = cv::boundingRect(yellowcontours[i]);
                        }

                        if (boundRectangleYellow.area() > 120)
                        {
                            amountOfYellowCones += 1;
                        }
                    }
                }

                if (directionEstablished < 10 && (largestboundRectangleBlue.x != 0) && (boundRectangleYellow.x != 0))
                {
                    if (largestboundRectangleBlue.x < largestboundRectangleYellow.x)
                    {
                        numberClockwise++;
                    }
                    else if (largestboundRectangleBlue.x > largestboundRectangleYellow.x)
                    {
                        numberCounterclockwise++;
                    }

                    if (numberClockwise > numberCounterclockwise)
                    {
                        carDirection = "Clockwise";
                    }
                    else if (numberCounterclockwise > numberClockwise)
                    {
                        carDirection = "Counter-Clockwise";
                    }

                    directionEstablished++;
                }

                if (carDirection == "Clockwise")
                {
                    steering = calculateSteering(rightIR, leftIR, amountOfYellowCones, amountOfBlueCones, steering);
                }
                if (carDirection == "Counter-Clockwise")
                {
                    steering = calculateSteering(rightIR, leftIR, amountOfBlueCones, amountOfYellowCones, steering);
                }
                std::cout << "Group_02;" << tStamp << ";" << steering << std::endl;

                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0, 0, 255));

                {
                    std::lock_guard<std::mutex> lck(gsrMutex);

                }


                if (VERBOSE)
                {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}
