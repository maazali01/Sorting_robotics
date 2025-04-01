#include <webots/Robot.hpp>
#include <webots/Camera.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <vector>
#include <cmath>
#include <webots/Motor.hpp>

#define TIME_STEP 32
#define GREEN_THRESHOLD 90  // Adjusted for green box detection
#define MIN_GREEN_PIXELS 200 // Minimum pixels required to detect green box
#define RED_THRESHOLD 90  // Adjusted for red object detection
#define MIN_RED_PIXELS 200 // Minimum pixels required to detect red object
#define BLUE_THRESHOLD 150  // Adjusted for blue object detection
#define MIN_BLUE_PIXELS 230 // Minimum pixels required to detect blue object

using namespace webots;

int main() {
    Robot robot;
    Camera *camera = robot.getCamera("camera");
    DistanceSensor *distanceSensor = robot.getDistanceSensor("distance sensor");
    Motor *shoulderPanMotor = robot.getMotor("shoulder_pan_joint");
    Motor *shoulderLiftMotor = robot.getMotor("shoulder_lift_joint");
    Motor *elbowMotor = robot.getMotor("elbow_joint");
    Motor *wrist1Motor = robot.getMotor("wrist_1_joint");
    Motor *wrist2Motor = robot.getMotor("wrist_2_joint");
    Motor *wrist3Motor = robot.getMotor("wrist_3_joint");
    Motor *finger1Motor = robot.getMotor("finger_1_joint_1");
    Motor *finger2Motor = robot.getMotor("finger_2_joint_1");
    Motor *fingerMiddleMotor = robot.getMotor("finger_middle_joint_1");
    
    if (!camera) {
        std::cerr << "Error: Camera not found!" << std::endl;
        return 1;
    }
    if (!distanceSensor) {
        std::cerr << "Error: Distance sensor not found!" << std::endl;
        return 1;
    }
    
    camera->enable(TIME_STEP);
    distanceSensor->enable(TIME_STEP);
    int width = 240;
    int height = 240;
    
    std::cout << "Camera and Distance Sensor started successfully." << std::endl;
    
    while (robot.step(TIME_STEP) != -1) {
        const unsigned char *image = camera->getImage();
        if (image) {
            int green_pixel_count = 0;
            int red_pixel_count = 0;
            int blue_pixel_count = 0;
            
            for (int y = 0; y < height; y++) {
                for (int x = 0; x < width; x++) {
                    int r = camera->imageGetRed(image, width, x, y);
                    int g = camera->imageGetGreen(image, width, x, y);
                    int b = camera->imageGetBlue(image, width, x, y);
                    
                    if (g > GREEN_THRESHOLD && r < g * 0.6 && b < g * 0.6) {
                        green_pixel_count++;
                    }
                    
                    if (r > RED_THRESHOLD && g < r * 0.6 && b < r * 0.6) {
                        red_pixel_count++;
                    }
                    
                    if (b > BLUE_THRESHOLD && r < b * 0.6 && g < b * 0.6) {
                        blue_pixel_count++;
                    }
                }
            }
            
            bool greenBoxDetected = (green_pixel_count > MIN_GREEN_PIXELS);
            bool redObjectDetected = (red_pixel_count > MIN_RED_PIXELS);
            bool blueObjectDetected = (blue_pixel_count > MIN_BLUE_PIXELS);
            double distance = distanceSensor->getValue();
            
            if (greenBoxDetected) {
                std::cout << "Green box detected! Picking up..." << std::endl;
                finger1Motor->setPosition(0.85);
                finger2Motor->setPosition(0.85);
                fingerMiddleMotor->setPosition(0.85);
                robot.step(1000);
                
                std::cout << "Moving arm to green basket..." << std::endl;
                // shoulderPanMotor->setPosition(-1.0);
                shoulderLiftMotor->setPosition(-2.5);
                elbowMotor->setPosition(-1.4);
                // wrist1Motor->setPosition(-0.5);
                robot.step(1000);
                
                std::cout << "Releasing green box..." << std::endl;
                finger1Motor->setPosition(0.0);
                finger2Motor->setPosition(0.0);
                fingerMiddleMotor->setPosition(0.0);
                robot.step(1000);
            }
            else if (redObjectDetected) {
                std::cout << "Red object detected! Picking up..." << std::endl;
                finger1Motor->setPosition(0.85);
                finger2Motor->setPosition(0.85);
                fingerMiddleMotor->setPosition(0.85);
                robot.step(1000);
                
                std::cout << "Moving arm to red basket..." << std::endl;
                shoulderPanMotor->setPosition(-1.5);
                // shoulderLiftMotor->setPosition(-1.8);
                // elbowMotor->setPosition(-1.2);
                // wrist1Motor->setPosition(-0.3);
                robot.step(1000);
                
                std::cout << "Releasing red object..." << std::endl;
                finger1Motor->setPosition(0.0);
                finger2Motor->setPosition(0.0);
                fingerMiddleMotor->setPosition(0.0);
                robot.step(1000);
            }
            else if (blueObjectDetected) {
                std::cout << "Blue object detected! Picking up..." << std::endl;
                finger1Motor->setPosition(0.85);
                finger2Motor->setPosition(0.85);
                fingerMiddleMotor->setPosition(0.85);
                robot.step(1000);
                
                std::cout << "Moving arm to blue basket..." << std::endl;
                shoulderPanMotor->setPosition(-2.7);
                // shoulderLiftMotor->setPosition(-2.2);
                // elbowMotor->setPosition(-1.8);
                // wrist1Motor->setPosition(-0.7);
                robot.step(1000);
                
                std::cout << "Releasing blue object..." << std::endl;
                finger1Motor->setPosition(0.0);
                finger2Motor->setPosition(0.0);
                fingerMiddleMotor->setPosition(0.0);
                robot.step(1000);
            }

            shoulderPanMotor->setPosition(0.0);
            shoulderLiftMotor->setPosition(0.0);
            elbowMotor->setPosition(0.0);
            wrist1Motor->setPosition(0.0);
            wrist2Motor->setPosition(0.0);
            wrist3Motor->setPosition(0.0);
            robot.step(2000);
        }
    }
    return 0;
}
