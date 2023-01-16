#include <iostream>

#include <opencv2/opencv.hpp>
#include <apriltag.h>
#include "tag16h5.h"
#include "apriltag_pose.h"

#include "argparse.h"
#include "VisualizationSystem.h"

float tagsize = 0.1524; // 6 inches -> meters
float decimate = 0.2f;
float sigma = 0.0f;
int nthreads = 2;
int cameraID = 0;
bool noGUI = false;
const char *cameraName = "noname";

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
    cv::Mat distanceCoefficients = fs["distance_coefficients"].mat();

    apriltag_detector *apriltagDetector = apriltag_detector_create();
    apriltag_detector_add_family(apriltagDetector, tag16h5_create());

    apriltagDetector->quad_decimate = decimate;
    apriltagDetector->quad_sigma = sigma;
    apriltagDetector->nthreads = nthreads;
    apriltagDetector->refine_edges = true;

    VisualizationSystem guiSystem;
    VisualizationSystem::Texture cameraTexture;
    if (!noGUI) {
        guiSystem.init();
        cameraTexture = guiSystem.createTexture();
    }

    cv::Mat frame, gray;
    while (true) {
        camera.read(frame);
        cv::Mat undistorted;
        cv::undistort(frame, undistorted, cameraMatrix, distanceCoefficients);
        undistorted.copyTo(frame);

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        image_u8_t imHeader = {
                .width = gray.cols,
                .height = gray.rows,
                .stride = gray.cols,
                .buf = gray.data
        };

        zarray_t *detections = apriltag_detector_detect(apriltagDetector, &imHeader);

        if (!noGUI) {
            guiSystem.begin();
            ImGui::DockSpaceOverViewport();
        }

        // Draw detection outlines
        ImGui::Begin("Info");
        for (int i = 0; i < zarray_size(detections); i++) {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            if(det->id > 8)
                continue;

            std::string text = "";
            if(!noGUI) {
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

                text = std::to_string(det->id);
            }

            apriltag_detection_info_t info;
            info.det = det;
            info.tagsize = tagsize;
            info.fx = cameraMatrix.at<double>(0, 0);
            info.fy = cameraMatrix.at<double>(1, 1);
            info.cx = cameraMatrix.at<double>(0, 2);
            info.cy = cameraMatrix.at<double>(1, 2);


            apriltag_pose_t pose;
            double err = estimate_tag_pose(&info, &pose);

            double t_x = matd_get(pose.t, 0, 0);
            double t_y = matd_get(pose.t, 1, 0);
            double t_z = matd_get(pose.t, 2, 0);

            cv::Mat RMat = cv::Mat(pose.R->nrows, pose.R->ncols, CV_64F, pose.R->data);

            double thetaX;
            double thetaY;
            double thetaZ;
            {
                double r32 = RMat.at<double>(2, 1);
                double r33 = RMat.at<double>(2, 2);
                double r31 = RMat.at<double>(2, 0);
                double r21 = RMat.at<double>(1, 0);
                double r11 = RMat.at<double>(0, 0);

                thetaX = std::atan2(r32, r33);
                thetaY = std::atan2(-r31, std::sqrt(r32 * r32 + r33 * r33));
                thetaZ = std::atan2(r21, r11);
            }


            if(!noGUI) {
                int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;

                double fontscale = 0.7;
                int baseline;
                cv::Size textsize = cv::getTextSize(text, fontface, fontscale, 2,
                                                    &baseline);
                text += "(" + std::to_string(t_x) + ", " + std::to_string(t_y) + ", " + std::to_string(t_z) + ")";

                putText(frame, text, cv::Point(det->c[0] - textsize.width / 2,
                                               det->c[1] + textsize.height / 2),
                        fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);

                if(ImGui::CollapsingHeader((std::string("TAG_") + std::to_string(det->id)).c_str())) {
                    ImGui::Text("POSITION: (%f, %f, %f)", t_x, t_y, t_z);
                    ImGui::Text("ROTATION: (%f, %f, %f)", thetaX * (180.0 / M_PI), thetaY * (180.0 / M_PI), thetaZ * (180.0 / M_PI));
                }
            }


        }
        ImGui::End();


        if (!noGUI) {
            ImGui::Begin("Camera");
            guiSystem.updateTexture(&cameraTexture, frame);
            ImGui::Image((ImTextureID) cameraTexture.ID, ImGui::GetContentRegionAvail());

            if (!noGUI) {
                ImGui::End();
            }
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
