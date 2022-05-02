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
                std::time_t currentTime = std::time(0);
                std::tm* timeNow = std::localtime(&currentTime);
                
                std::string timeVar = "Current Time: "+std::to_string(timeNow->tm_year +1900)+'-'+std::to_string(timeNow->tm_mon +1)+'-'+std::to_string(timeNow->tm_mday)+'T'+std::to_string(timeNow->tm_hour)+':'+std::to_string(timeNow->tm_min)+':'+std::to_string(timeNow->tm_sec)+'Z';
                //std::string dateVar = std::to_string(timeNow->tm_year) + '-' + std::to_string(timeNow->tm_mon) + '-' + std::to_string(timeNow->tm_mday);

                std::cout << timeVar << std::endl;

                //std::cout << "Current Time: " << (timeNow->tm_year) << '-' << (timeNow->tm_mon +1) << '-' << (timeNow->tm_mday) << ' ' << (timeNow->tm_hour) << ':' << (timeNow->tm_min) << ':' << (timeNow->tm_sec) << std::endl;
                std::pair<bool, cluon::data::TimeStamp > pair = sharedMemory->getTimeStamp();
                    cluon::data::TimeStamp sampleT = pair.second;
                    int64_t tStamp = cluon::time::toMicroseconds(sampleT);
                    std::string ts = std::to_string(tStamp);
                    std::string sampleTimeVar = "Sample Time: " + ts;
                    std::cout << "Sample Time: " << ts << std::endl;
                sharedMemory->unlock();

                // TODO: Do something with the frame.
                //cv::putText(img, label, Point(20, 20), FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 170, 0), 2.0);
               // cv::putTest(img, ("Current Time: " + (timeNow->tm_year) + '-' + (timeNow->tm_mon +1) + '-' + (timeNow->tm_mday) + ' ' + (timeNow->tm_hour) + ':' + (timeNow->tm_min) + ':' + (timeNow->tm_sec), (10, 100), FONT_HERSHEY_PLAIN,1, (210, 155, 155), 4, cv::LINE_8));

                cv::putText(img, timeVar, cv::Point(20,10), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 200, 190)); 
                //cv::putText(img, "Hansen, Robin", cv::Point(20, 30), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 200, 190)); 
                cv::putText(img, sampleTimeVar, cv::Point(20, 50), cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 200, 190)); 
                
                
                
                
                    //attempt 2
                /*
                    cv::Mat target = cv::imread("target.png");
                    cv::Mat background;
                    target.copyTo(background);

                    cv::cvtColor(target, target, cv::COLOR_BGR2HSV);

                    cv::Mat mask;
                    cv::inRange(target, cv::Scalar(30, 85, 183), cv::Scalar(44, 175, 209), mask);

                    std::vector<std::vector<cv::Point >> contours;
                    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
                    cv::drawContours(background, contours, -1, (0, 255, 0), 3);

                    cv::imshow("contours", background);
                    //cv::imshow("yellow", mask);
                    // end attempt 2
                */

                 //attempt 1 - picture
                //target pic
                /*
                cv::Mat target = cv::imread("target.png");
                cv::cvtColor(target, target, cv::COLOR_BGR2HSV);

                cv::Mat mask;
                cv::inRange(target, cv::Scalar(30, 85, 183), cv::Scalar(44, 175, 209), mask);
                */
                // end target pic



               /* 
                //attempt 1 - video feed

                cv::Scalar lowValue = cv::Scalar(26, 233, 182);
                cv::Scalar highValue = cv::Scalar(53, 229, 177);

                cv::Mat copyOfImg;
                img.copyTo(copyOfImg);
                cv::cvtColor(img, img, cv::COLOR_BGR2HSV);


                cv::Mat justYellowColor;
                cv::inRange(img, lowValue, highValue, justYellowColor);
                

                //cv::inRange(img, cv::Scalar(250, 250, 200), cv::Scalar(250, 230, 0), justYellowColor); //duplicate
                std::vector < std::vector < cv::Point >> contours;
                cv::findContours(justYellowColor, contours, cv::RET_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

                //cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE); //<-- Causes build issues
                
                
                cv::drawContours(img, contours, -1, cv::Scalar(0, 255, 0), 3); //<-- Draw contours, not rects

                
                for (size_t i = 0; i < contours.size(); ++i){
                    cv::Rect boundRect = cv::boundingRect(contours[i]);
                    cv::rectangle(img, boundRect.tl(), boundRect.br(), (0,0,0), 3);
                }



                cv::imshow("copyOfImg", copyOfImg);
                cv::imshow("contours", img);
                
                
                //cv::imshow("justYellow", justYellowColor); //remove comment
                // end attempt 1
*/
                



                
                //Attempt 3
                using namespace cv;
                using namespace std;
                //Read input image in gray scale
    //Mat src;
    //target = imread("target.jpg", CV_LOAD_IMAGE_COLOR); //<-- If loading image


    //Mat gray;
    //cvtColor(src, gray, CV_BGR2GRAY); //<-- Gray instead of HSV
    //threshold(gray, gray,200, 255,THRESH_BINARY_INV); //Threshold the gray
    //imshow("gray",gray);int largest_area=0;
    //int largest_contour_index=0;
    
    cv::cvtColor(img, img, cv::COLOR_BGR2HSV);

    Mat justYellowColor;
    //inRange(img, cv::Scalar(30, 85, 183), cv::Scalar(44, 175, 209), justYellowColor);
    inRange(img, cv::Scalar(0,0,0), cv::Scalar(255, 255, 255), justYellowColor); //For testing

    Rect bounding_rect;
    vector<vector<Point>> contours; // Vector for storing contour
    vector<Vec4i> hierarchy;
    //findContours( justYellowColor, contours, hierarchy, CV_RETR_FLOODFILL, CHAIN_APPROX_SIMPLE );
    findContours(justYellowColor, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    // iterate through each contour.
    for( unsigned int i = 0; i< contours.size(); i++ )
    {          
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        
    }
    Scalar color( 255,255,255);  // color of the contour in the
    //Draw the contour and rectangle
    //drawContours( img, contours, color, CV_FILLED,8,hierarchy, 3); //<-- Tried, produces error
    drawContours(img, contours, -1, Scalar(0, 255, 0), 3);
    rectangle(img, bounding_rect,  Scalar(0,255,0),2, 8,0);
    imshow( "Attempt3", img );    



    


                //


                // Example: Draw a red rectangle and display image.
                cv::rectangle(img, cv::Point(50, 50), cv::Point(100, 100), cv::Scalar(0,0,255));

                // If you want to access the latest received ground steering, don't forget to lock the mutex:
                {
                    std::lock_guard<std::mutex> lck(gsrMutex);
                    std::cout << "main: groundSteering = " << gsr.groundSteering() << std::endl;
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

