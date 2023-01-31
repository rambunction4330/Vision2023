/**
 * Author: Aiden Lambert
 * Author Quote: the coding team do be coding and the coding do be do being. Doo be doo be doo. --Adi Jain
 */

#include <iostream>

#include <ntcore_cpp.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include "tag16h5.h"

#include "argparse.h"
#include "VisualizationSystem.h"

#include "incbin.h"
#include "perf.h"
#include "ThreadedCaptureSystem.h"
#include "Detection.h"
#include "PositionEstimation.h"

#include "NetworkSystem.h"
#include "terminal.h"

INCBIN(FieldImage, "assets/field23.png");

float tagsize = 0.1524; // 6 inches -> meters
float decimate = 0.2f;
float sigma = 0.0f;
int nthreads = 2;
int cameraID = 0;
int decisionMarginCutoff = 30;
bool noGUI = false;
const char *cameraName = "noname";

const double fieldLength = 16.535239036755;
const double fieldHeight = 8.0137;

static void drawDebugInfo(cv::Mat &frame, apriltag_detection *det) {
    line(frame, cv::Point(det->p[0][0], det->p[0][1]),
         cv::Point(det->p[1][0], det->p[1][1]),
         cv::Scalar(0, 0xff, 0), 2);
    line(frame, cv::Point(det->p[0][0], det->p[0][1]),
         cv::Point(det->p[3][0], det->p[3][1]),
         cv::Scalar(0, 0, 0xff), 2);
    line(frame, cv::Point(det->p[1][0], det->p[1][1]),
         cv::Point(det->p[2][0], det->p[2][1]),
         cv::Scalar(0xff, 0, 0), 2);
    line(frame, cv::Point(det->p[2][0], det->p[2][1]),
         cv::Point(det->p[3][0], det->p[3][1]),
         cv::Scalar(0xff, 0, 0), 2);

    std::string text = std::to_string(det->id);

    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;

    double fontscale = 0.7;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                        &baseline);

    putText(frame, text, cv::Point(det->c[0] - textsize.width / 2,
                                   det->c[1] + textsize.height / 2),
            fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
}

static void undistortImage(cv::Mat &frame, cv::Mat cameraMatrix, cv::InputArray distanceCoefficients) {
    cv::Mat temp;
    cv::undistort(frame, temp, cameraMatrix, distanceCoefficients);
    temp.copyTo(frame);
}

// register perf here
REGISTER_PERF(total)
REGISTER_PERF(detection)
REGISTER_PERF(capture)
REGISTER_PERF(undistortion)
REGISTER_PERF(position_estimation)
REGISTER_PERF(gui_display)

int main(int argc, const char **argv) {

    struct argparse_option options[] = {
            OPT_HELP(),
            OPT_FLOAT((char) NULL, "tag-size", (void *) &tagsize,
                      "width of apriltag in meters. WARNING: The default frc-standard value is 6 inches. Your tags should be 6 inches",
                      NULL, 0, 0),
            OPT_FLOAT((char) NULL, "decimate", (void *) &decimate,
                      "decimation constant. See the frc/apriltag docs for more information", NULL, 0, 0),
            OPT_FLOAT((char) NULL, "sigma", (void *) &sigma,
                      "gaussian blur constant. See the frc/apriltag docs for more information", NULL, 0, 0),
            OPT_INTEGER('j', "nthreads", (void *) &nthreads, "number of threads to run for apriltag detection", NULL, 0,
                        0),
            OPT_INTEGER((char) NULL, "camera-id", (void *) &cameraID, "The ID of the webcam to use. Defaults to 0",
                        NULL, 0, 0),
            OPT_INTEGER((char) NULL, "decision-margin-cutoff", (void *) &decisionMarginCutoff,
                        "The decision margin is a measure of the quality of the binary decoding process: the "
                        "average difference between the intensity of a data bit versus "
                        "the decision threshold. Higher numbers roughly indicate better decodes. "
                        "The cutoff specifies the minimum value for the decision margin", NULL, 0, 0),
            OPT_STRING('n', "name", (void *) &cameraName, "user-defined camera name", NULL, 0, 0),
            OPT_BOOLEAN((char) NULL, "no-gui", (void *) &noGUI, "whether or not to disable GUI output for the program",
                        NULL, 0, 0),
            OPT_END(),
    };

    static const char *const usages[] = {
            "vision [args]",
            NULL,
    };

    struct argparse argparse;
    argparse_init(&argparse, options, usages, 0);
    argparse_describe(&argparse,
                      "The main program of the 2023 vision suite. Detects the location of AprilTags through a camera.\n",
                      "\nBe sure to run this in the source directory containing the data folder.");
    argc = argparse_parse(&argparse, argc, argv);

    ThreadedCaptureSystem captureSystem;
    captureSystem.start(cameraID);

    cv::FileStorage fs;

    std::string configFilename = std::string("data/calibrations/") + cameraName + "_info.yml";
    if (!fs.open(configFilename, cv::FileStorage::READ)) {
        std::cerr << "failed to open " + configFilename << std::endl;
        std::exit(EXIT_FAILURE);
    }
    cv::Mat cameraMatrix = fs["camera_matrix"].mat();
    cv::Mat distortionCoefficients = fs["distance_coefficients"].mat();

    apriltag_detector *apriltagDetector = apriltag_detector_create();
    apriltag_detector_add_family(apriltagDetector, tag16h5_create());

    apriltagDetector->quad_decimate = decimate;
    apriltagDetector->quad_sigma = sigma;
    apriltagDetector->nthreads = nthreads;
    apriltagDetector->refine_edges = true;
    apriltagDetector->decode_sharpening = 0.40;

    VisualizationSystem guiSystem;
    VisualizationSystem::Texture cameraTexture;
    VisualizationSystem::Texture fieldTexture;
    cv::Mat fieldImageMatDefaultData;

// load field image

    if (!noGUI) {
        guiSystem.init();

        cameraTexture = guiSystem.createTexture();
        fieldTexture = guiSystem.createTexture();

        cv::Mat temp = cv::imdecode(
            std::vector<unsigned char>(gFieldImageData, gFieldImageData + gFieldImageSize), cv::IMREAD_UNCHANGED);
        

        double scale_percent = 30;
        int width = (int)(temp.cols * scale_percent / 100);
        int height = (int)(temp.rows * scale_percent / 100);

        cv::resize(temp, fieldImageMatDefaultData, cv::Size(width, height));
    }

    cv::Mat frame;

    terminal::initAlternateScreen();
    terminal::goToCorner();

    std::chrono::high_resolution_clock::time_point lastUpdatedTime;
    NetworkSystem networkSystem;
    networkSystem.init();
    int32_t networkTableLastUpdateTime = nt::Now();

    while (noGUI || !guiSystem.windowShouldClose()) {
        CLEAR_PERF(total)
        CLEAR_PERF(detection)
        CLEAR_PERF(capture)
        CLEAR_PERF(undistortion)
        CLEAR_PERF(position_estimation)
        CLEAR_PERF(gui_display)

        BEGIN_PERF(total)

        BEGIN_PERF(capture)
//         while(captureSystem.getLastUpdatedTime() <= lastUpdatedTime) {
// #ifndef __arm__
//             __builtin_ia32_pause();
// #else
//             __asm__ __volatile__("yield");
// #endif
//         }
//         lastUpdatedTime = captureSystem.getLastUpdatedTime();
        captureSystem.read(frame);
        networkTableLastUpdateTime = nt::Now();
        END_PERF(capture)


        BEGIN_PERF(undistortion)
        //undistortImage(frame, cameraMatrix, distortionCoefficients);
        END_PERF(undistortion)

        BEGIN_PERF(detection)
        zarray_t* allDetections;
        int detectedTagBits = 0;
        std::array<apriltag_detection_t*, 8> detections = runDetection(apriltagDetector, frame, decisionMarginCutoff, &allDetections,
                     &detectedTagBits);
        END_PERF(detection)

        BEGIN_PERF(position_estimation)
        cv::Point3d position;
        double theta;
        runPositionEstimation(detections, detectedTagBits, cameraMatrix, tagsize, &position, &theta);
        END_PERF(position_estimation)

        BEGIN_PERF(gui_display)
        for(int i = 1; i <= 8; i++) {
            if(detectedTagBits & (1 << i)) {
                drawDebugInfo(frame, detections[i- 1]);
            }
        }

        if (!noGUI) {
            guiSystem.begin();
            ImGui::DockSpaceOverViewport();
        }
        END_PERF(gui_display);

        cv::Mat fieldImageAnnotated;
        int numDetections = __builtin_popcount(detectedTagBits);
        cv::Mat cameraPosition(position);

        networkSystem.update(cv::Point2d(position.x, position.y), theta, networkTableLastUpdateTime);

        if (!noGUI) {
            BEGIN_PERF(gui_display);
            ImGui::Begin("Camera");
            guiSystem.updateTexture(&cameraTexture, frame);
            ImGui::Image((ImTextureID) cameraTexture.ID, ImGui::GetContentRegionAvail());
            ImGui::End();

            ImGui::Begin("Field Diagram");
            fieldImageMatDefaultData.copyTo(fieldImageAnnotated);
            if (numDetections) {
                // draw the robot's position
                int imageWidth = fieldImageAnnotated.cols;
                int imageHeight = fieldImageAnnotated.rows;

                int xPosition = (int) ((float) imageWidth * (cameraPosition.at<double>(0, 0) / (float) fieldLength));
                int yPosition = imageHeight -
                                (int) ((float) imageHeight * (cameraPosition.at<double>(0, 1) / (float) fieldHeight));
                std::stringstream stringstream;
                stringstream << cv::Point(xPosition, yPosition);
                ImGui::Text("%s", stringstream.str().c_str());
                cv::circle(fieldImageAnnotated, cv::Point(xPosition, yPosition), 5, cv::Scalar(0xff, 0, 0xff), -1);

            }
            guiSystem.updateTexture(&fieldTexture, fieldImageAnnotated);
            ImGui::Image((void *) fieldTexture.ID, ImGui::GetContentRegionAvail());
            ImGui::End();

            ImGui::Begin("Settings");
            bool settingsModified = false;
            if (ImGui::SliderFloat("quad-decimate", &decimate, 0.0f, 1.0f)) {
                settingsModified = true;
                apriltagDetector->quad_decimate = decimate;
            }

            if (ImGui::SliderFloat("sigma", &sigma, 0.0f, 1.0f)) {
                settingsModified = true;
                apriltagDetector->quad_sigma = sigma;
            }
            ImGui::End();

            ImGui::Begin("Camera Position Estimation");
            if (numDetections) {
                std::stringstream ss;
                ss << cameraPosition * 39.3701;

                ImGui::Text("%s", ss.str().c_str());
            }
            ImGui::Text("angle: %f degrees", theta * (180 / M_PI));
            ImGui::End();

            guiSystem.end();
            END_PERF(gui_display)
        }

        apriltag_detections_destroy(allDetections);


        if (cv::waitKey(1) == 'q') {
            break;
        }

        END_PERF(total)
        terminal::clear();

        std::cout << "position: " << position * 39.3701 << std::endl;
        std::cout << "rotation: " << theta * (180.0 / M_PI) << " deg" << std::endl;

        PRINT_PERF(total)
        PRINT_PERF(detection)
        PRINT_PERF(capture)
        // PRINT_PERF(undistortion)
        PRINT_PERF(position_estimation)
        PRINT_PERF(gui_display)
    }
    terminal::exitAlternateScreen();

    if (!noGUI) {
        guiSystem.destruct();
    }

    return 0;
}
