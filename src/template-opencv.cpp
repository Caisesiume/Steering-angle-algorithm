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

//include due to features changed from highgui.hpp
/*
#include "opencv2/imgcodecs.hpp"
*/

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{1};
    // Parse the command line parameters as we require the user to specify some mandatory information on startup.
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("cid")) ||
         (0 == commandlineArguments.count("name")) ||
         (0 == commandlineArguments.count("width")) ||
         (0 == commandlineArguments.count("height")) ) {
        std::cerr << argv[0] << " attaches to a shared memory area containing an ARGB image." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OD4 session> --name=<name of shared memory area> [--verbose]" << std::endl;
        std::cerr << "         --cid:    CID of the OD4Session to send and receive messages" << std::endl;
        std::cerr << "         --name:   name of the shared memory area to attach" << std::endl;
        std::cerr << "         --width:  width of the frame" << std::endl;
        std::cerr << "         --height: height of the frame" << std::endl;
        std::cerr << "Example: " << argv[0] << " --cid=253 --name=img --width=640 --height=480 --verbose" << std::endl;
    }
    else {
        // Extract the values from the command line parameters
        const std::string NAME{commandlineArguments["name"]};
        const uint32_t WIDTH{static_cast<uint32_t>(std::stoi(commandlineArguments["width"]))};
        const uint32_t HEIGHT{static_cast<uint32_t>(std::stoi(commandlineArguments["height"]))};
        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        // Attach to the shared memory.
        std::unique_ptr<cluon::SharedMemory> sharedMemory{new cluon::SharedMemory{NAME}};
        if (sharedMemory && sharedMemory->valid()) {
            std::clog << argv[0] << ": Attached to shared memory '" << sharedMemory->name() << " (" << sharedMemory->size() << " bytes)." << std::endl;

            // Interface to a running OpenDaVINCI session where network messages are exchanged.
            // The instance od4 allows you to send and receive messages.
            cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

            opendlv::proxy::GroundSteeringRequest gsr;
            std::mutex gsrMutex;
            auto onGroundSteeringRequest = [&gsr, &gsrMutex](cluon::data::Envelope &&env){
                // The envelope data structure provide further details, such as sampleTimePoint as shown in this test case:
                // https://github.com/chrberger/libcluon/blob/master/libcluon/testsuites/TestEnvelopeConverter.cpp#L31-L40
                std::lock_guard<std::mutex> lck(gsrMutex);
                gsr = cluon::extractMessage<opendlv::proxy::GroundSteeringRequest>(std::move(env));
                std::cout << "lambda: groundSteering = " << gsr.groundSteering() << std::endl;
            };

            od4.dataTrigger(opendlv::proxy::GroundSteeringRequest::ID(), onGroundSteeringRequest);

            // Endless loop; end the program by pressing Ctrl-C.
            while (od4.isRunning()) {
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
                // TODO: Here, you can add some code to check the sampleTimePoint when the current frame was captured.
                //std::time_t currentTime = std::time(0);
                //std::tm* timeNow = std::localtime(&currentTime);
                
                //std::string timeVar = "Current Time: "+std::to_string(timeNow->tm_year +1900)+'-'+std::to_string(timeNow->tm_mon +1)+'-'+std::to_string(timeNow->tm_mday)+'T'+std::to_string(timeNow->tm_hour)+':'+std::to_string(timeNow->tm_min)+':'+std::to_string(timeNow->tm_sec)+'Z';
                //std::string dateVar = std::to_string(timeNow->tm_year) + '-' + std::to_string(timeNow->tm_mon) + '-' + std::to_string(timeNow->tm_mday);

               // std::cout << timeVar << std::endl;

                //std::cout << "Current Time: " << (timeNow->tm_year) << '-' << (timeNow->tm_mon +1) << '-' << (timeNow->tm_mday) << ' ' << (timeNow->tm_hour) << ':' << (timeNow->tm_min) << ':' << (timeNow->tm_sec) << std::endl;
                /*std::pair<bool, cluon::data::TimeStamp > pair = sharedMemory->getTimeStamp();
                    cluon::data::TimeStamp sampleT = pair.second;
                    int64_t tStamp = cluon::time::toMicroseconds(sampleT);
                    std::string ts = std::to_string(tStamp);
                    std::string sampleTimeVar = "Sample Time: " + ts;
                    std::cout << "Sample Time: " << ts << std::endl;*/ //commented out for performance
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                //cv::putText(img, label, Point(20, 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 170, 0), 2.0);
               // cv::putTest(img, ("Current Time: " + (timeNow->tm_year) + '-' + (timeNow->tm_mon +1) + '-' + (timeNow->tm_mday) + ' ' + (timeNow->tm_hour) + ':' + (timeNow->tm_min) + ':' + (timeNow->tm_sec), (10, 100), FONT_HERSHEY_PLAIN,1, (210, 155, 155), 4, cv::LINE_8));

               // cv::putText(img, timeVar, cv::Point(20,10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 200, 190)); 
               // cv::putText(img, sampleTimeVar, cv::Point(20, 50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 200, 190)); 
                
                
                
                
                //HSV values reference: https://www.codespeedy.com/splitting-rgb-and-hsv-values-in-an-image-using-opencv-python/
                //Solution partly inspired by: https://stackoverflow.com/questions/9018906/detect-rgb-color-interval-with-opencv-and-c
                //AND: https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/

                
                //Cone color detection
                
                using namespace cv;
                using namespace std;
                
    
    cv::Mat originalImg;
    img.copyTo(originalImg);

    cv::Mat hsvIMG;
    img.copyTo(hsvIMG);


    //Draw box around unnecessary part of car(to avoid conflicts with inrange below)
        cv::rectangle(img, cv::Point(150, 385), cv::Point(500, 500), cv::Scalar(0,0,0), CV_FILLED);
    //Draw box in region above cones, to avoid conflicts with irrelevant objects.
        cv::rectangle(img, cv::Point(0,0), cv::Point(650, 250), cv::Scalar(0,0,0), CV_FILLED);
    

    cv::cvtColor(img, hsvIMG, cv::COLOR_BGR2HSV);

    cv::Mat justYellowColor;
    cv::Mat justBlueColor;
    
    inRange(hsvIMG, cv::Scalar(15,20,20), cv::Scalar(70, 100, 250), justYellowColor);//Yellow(low, high) - Yellow cones
    inRange(hsvIMG, cv::Scalar(80,125,8), cv::Scalar(135, 255, 210), justBlueColor); //Blue(low, high) - Blue cones
        

    cv::Rect bounding_rect; //z-- Not used atm
    vector<vector<cv::Point>> yellowcontours; // Vector for storing yellow contours
    vector<vector<cv::Point>> bluecontours; // Vector for storing blue contours
    //vector<Vec4i> hierarchy;
    //findContours( justYellowColor, contours, hierarchy, CV_RETR_FLOODFILL, CHAIN_APPROX_SIMPLE );
    cv::findContours(justBlueColor, yellowcontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::findContours(justYellowColor, bluecontours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
   /* for( unsigned int i = 0; i< contours.size(); i++ )
    {         
       // double area = contourArea(contours[i]); //z-- idk if works, test later
       // if (area > 0) {
            // Find the bounding rectangle for biggest contour
            bounding_rect=cv::boundingRect(yellowcontours[i]);
            bounding_rect=cv::boundingRect(bluecontours[i]);
       // }
        
    }*/
    cv::drawContours(img, yellowcontours, -1, cv::Scalar(0, 255, 0), 3);
    cv::drawContours(img, bluecontours, -1, cv::Scalar(0, 255, 0), 3);
    //cv::rectangle(img, bounding_rect,  cv::Scalar(0,255,0),2, 8,0);
    cv::imshow( "Original Img", originalImg ); //<--Original Img(not changed)
    cv::imshow("justYellowColor", justYellowColor); //<-- (Just yellow)White if within HSV values, black if not.
    cv::imshow("justBlueColor", justBlueColor); //<-- (Just blue)White if within HSV values, black if not.
    //cv::imshow( "Attempt3", img ); //<-- duplicate of tmp/img
    //cv::imshow("hsvImg", hsvIMG); //<-- HSV(pink/purple) video feed



                // Example: Draw a red rectangle and display image.
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                 //   std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;


                 /* //Maybe useful later on
                    for (size_t i = 0; i < contours.size(); ++i){
                    cv::Rect boundRect = cv::boundingRect(contours[i]);
                    cv::rectangle(img, boundRect.tl(), boundRect.br(), (0,0,0), 3);
                }
                 */
                }

                // Display image on your screen.
                if (VERBOSE) {
                    cv::imshow(sharedMemory->name().c_str(), img);
                    cv::waitKey(1);
                }
            }
        }
        retCode = 0;
    }
    return retCode;
}

