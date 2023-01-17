#include <iostream>

#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include "tag16h5.h"
#include "apriltag_pose.h"

#include "argparse.h"
#include "VisualizationSystem.h"
#include "FieldData.h"

#include "incbin.h"

INCBIN(FieldImage, "assets/field23.png");

float tagsize = 0.1524; // 6 inches -> meters
float decimate = 0.2f;
float sigma = 0.0f;
int nthreads = 2;
int cameraID = 0;
bool noGUI = false;
const char *cameraName = "noname";

const double fieldLength = 16.535239036755;
const double fieldHeight = 8.0137;

static void drawDebugInfo(cv::Mat& frame, apriltag_detection* det) {
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

static void undistortImage(cv::Mat& frame, cv::Mat cameraMatrix, cv::InputArray distanceCoefficients) {
    cv::Mat temp;
    cv::undistort(frame, temp, cameraMatrix, distanceCoefficients);
    temp.copyTo(frame);
}

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

    cv::VideoCapture camera(cameraID);
    if (!camera.isOpened()) {
        std::cerr << "failed to open camera!!!" << std::endl;
        std::exit(EXIT_FAILURE);
    }

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

    VisualizationSystem guiSystem;
    VisualizationSystem::Texture cameraTexture;
    VisualizationSystem::Texture fieldTexture;
    cv::Mat fieldImageMatDefaultData;
    if (!noGUI) {
        guiSystem.init();

        cameraTexture = guiSystem.createTexture();
        fieldTexture = guiSystem.createTexture();


        // load field image
        fieldImageMatDefaultData = cv::imdecode(std::vector<unsigned char>(gFieldImageData, gFieldImageData + gFieldImageSize), cv::IMREAD_UNCHANGED);
    }

    cv::Mat frame, gray;
    
    uint32_t detectedTagsBits = 0;
    std::vector<cv::Point2d> imagePoints;
    std::vector<cv::Point3d> objectPoints;

    while (noGUI || !guiSystem.windowShouldClose()) {
        camera.read(frame);
        undistortImage(frame, cameraMatrix, distortionCoefficients);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t imHeader = {
                .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };

        detectedTagsBits = 0;
        zarray_t *detections = apriltag_detector_detect(apriltagDetector, &imHeader);

        if (!noGUI) {
            guiSystem.begin();
            ImGui::DockSpaceOverViewport();
        }

        // We shouldn't be iterating over more than 8 detections because there are only 8 in the field
        // In fact, realistically, the number shouldn't be more than 4 as the camera shouldn't have 360 degree vision
        int numDetections = 0;
        int currentImageIndex = 0;
        imagePoints.clear();
        objectPoints.clear();
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            
            if(det->id > 8) {
//                std::cerr << "WARNING: A tag of id " << det->id << " detected but should not be on the game field!" << std::endl;
                continue;
            }
            
            if(detectedTagsBits & (1 << det->id)) {
                std::cerr << "WARNING: multiple tags of id " << det->id << " found!" << std::endl;
                continue;
            } else {
                detectedTagsBits |= (1 << det->id);
            }

            // the tags between 5 and 8 are all rotated 180 degrees compared to the ones with ids 1 through 4
            // so we need to account for this my multiplying the y direction offset of the corner from the center by -1
            int fieldOrientationMultiplier = det->id <= 4 && det->id >=0 ? 1 : -1;
            double centerToEdge = inToM(3.0); // perpendicular distance from center of apriltag to edge is 3 inches

            // construct image and object points
            for(int j = 0; j < 4; j++) {
                imagePoints.emplace_back();
                objectPoints.emplace_back();
            }
            imagePoints[currentImageIndex * 4 + 0] = cv::Point2d(det->p[0][0], det->p[0][1]);
            objectPoints[currentImageIndex * 4 + 0] = aprilTagFieldPoints[det->id] + cv::Point3d(0.0, fieldOrientationMultiplier * centerToEdge, -centerToEdge);
            imagePoints[currentImageIndex * 4 + 1] = cv::Point2d(det->p[1][0], det->p[1][1]);
            objectPoints[currentImageIndex * 4 + 1] = aprilTagFieldPoints[det->id] + cv::Point3d(0.0, -fieldOrientationMultiplier * centerToEdge, -centerToEdge);
            imagePoints[currentImageIndex * 4 + 2] = cv::Point2d(det->p[2][0], det->p[2][1]);
            objectPoints[currentImageIndex * 4 + 2] = aprilTagFieldPoints[det->id] + cv::Point3d(0.0, -fieldOrientationMultiplier * centerToEdge, centerToEdge);
            imagePoints[currentImageIndex * 4 + 3] = cv::Point2d(det->p[3][0], det->p[3][1]);
            objectPoints[currentImageIndex * 4 + 3] = aprilTagFieldPoints[det->id] + cv::Point3d(0.0, fieldOrientationMultiplier * centerToEdge, centerToEdge);

            if(!noGUI) {
                drawDebugInfo(frame, det);
            }
            currentImageIndex++;
            numDetections++;
        }

        cv::Mat rvec, tvec, cameraPosition;
        cv::Mat fieldImageAnnotated;

        if(numDetections > 0) {
            cv::solvePnPRansac(objectPoints, imagePoints, cameraMatrix, distortionCoefficients, rvec, tvec);
            std::vector<cv::Point2d> outPoints;
            cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distortionCoefficients, outPoints);

            cv::Mat rotMat;
            cv::Rodrigues(rvec, rotMat);
            cameraPosition = -rotMat.inv() * tvec;

            if(!noGUI) {
                int pointIndex = 0;
                for (auto &point: outPoints) {
                    cv::circle(frame, point, 5, cv::Scalar(0, 0, 0xff), -1);
                    cv::putText(frame, std::to_string(pointIndex), point, cv::FONT_HERSHEY_COMPLEX, 1.0f,
                                cv::Scalar(0, 0xff, 0));
                    pointIndex++;
                }
            }

        }

        if (!noGUI) {
            ImGui::Begin("Camera");
            guiSystem.updateTexture(&cameraTexture, frame);
            ImGui::Image((ImTextureID) cameraTexture.ID, ImGui::GetContentRegionAvail());
            ImGui::End();

            ImGui::Begin("Field Diagram");
            fieldImageMatDefaultData.copyTo(fieldImageAnnotated);
            if(numDetections) {
                // draw the robot's position
                int imageWidth = fieldImageAnnotated.cols;
                int imageHeight = fieldImageAnnotated.rows;

                int xPosition = (int) ((float) imageWidth * (cameraPosition.at<double>(0, 0) / (float) fieldLength));
                int yPosition = imageHeight -
                                (int) ((float) imageHeight * (cameraPosition.at<double>(0, 1) / (float) fieldHeight));
                std::stringstream stringstream;
                stringstream << cv::Point(xPosition, yPosition);
                ImGui::Text("%s", stringstream.str().c_str());
                cv::circle(fieldImageAnnotated, cv::Point(xPosition, yPosition), 20, cv::Scalar(0xff, 0, 0xff), -1);

            }
            guiSystem.updateTexture(&fieldTexture, fieldImageAnnotated);
            ImGui::Image((void *) fieldTexture.ID, ImGui::GetContentRegionAvail());
            ImGui::End();

            ImGui::Begin("Settings");
            bool settingsModified = false;
            if(ImGui::SliderFloat("quad-decimate", &decimate, 0.0f, 1.0f)) {
                settingsModified = true;
                apriltagDetector->quad_decimate = decimate;
            }

            if(ImGui::SliderFloat("sigma", &sigma, 0.0f, 1.0f)) {
               settingsModified = true;
               apriltagDetector->quad_sigma = sigma;
            }
            ImGui::End();

            ImGui::Begin("Camera Position Estimation");
            if(numDetections) {
                std::stringstream ss;
                ss << cameraPosition * 39.3701;

                ImGui::Text("%s", ss.str().c_str());
            }
            ImGui::End();

            guiSystem.end();
        }

        apriltag_detections_destroy(detections);


        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    if (!noGUI) {
        guiSystem.destruct();
    }

    return 0;
}
