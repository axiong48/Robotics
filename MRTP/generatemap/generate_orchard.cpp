#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

// ================= CONFIGURATION =================
// 1. FILES
const std::string MAP_FILE = "orchard_map.png";
const std::string OUTPUT_FILE = "generated_orchard.sdf";

// 2. MODELS
const std::string TREE_URI = "model://orchard/orchard_tree.dae";
const std::string GROUND_URI = "model://orchard/orchard_world.dae";

// 3. CALIBRATION
const double RESOLUTION = 0.1;         // Meters per pixel
const double BASE_TREE_DIAMETER = 3.0; // Width of the unscaled .dae model

// 4. COLLISION SETTINGS (The "Invisible Trunk")
// Robot hits this cylinder, but drives through the visual leaves
const double TRUNK_COLLISION_RADIUS = 0.2;
const double TRUNK_COLLISION_HEIGHT = 1.0;
// =================================================

int main()
{
    std::cout << "--- Starting C++ Orchard Generator ---" << std::endl;

    // 1. Load Map
    cv::Mat img = cv::imread(MAP_FILE, cv::IMREAD_GRAYSCALE);
    if (img.empty())
    {
        std::cerr << "Error: Could not open " << MAP_FILE << std::endl;
        return -1;
    }

    // 2. Process Image
    // Threshold: Dark trees (0) -> White blobs (255)
    cv::Mat binary_img;
    cv::threshold(img, binary_img, 127, 255, cv::THRESH_BINARY_INV);

    // Find Contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::cout << "Found " << contours.size() << " trees. Generating SDF..." << std::endl;

    // 3. Prepare Output File
    std::ofstream sdf(OUTPUT_FILE);
    if (!sdf.is_open())
    {
        std::cerr << "Error: Could not write to " << OUTPUT_FILE << std::endl;
        return -1;
    }

    // Write Header
    sdf << "<?xml version='1.0'?>\n";
    sdf << "<sdf version='1.7'>\n";
    sdf << "  <world name='procedural_orchard'>\n";

    // Physics & Sun
    sdf << "    <physics type='ode'><max_step_size>0.001</max_step_size></physics>\n";
    sdf << "    <include><uri>model://sun</uri></include>\n\n";

    // Custom Ground
    sdf << "    <model name='orchard_ground'>\n";
    sdf << "      <static>true</static>\n";
    sdf << "      <pose>0 0 0 0 0 0</pose>\n";
    sdf << "      <link name='link'>\n";
    sdf << "        <collision name='col'><geometry><mesh><uri>" << GROUND_URI << "</uri></mesh></geometry></collision>\n";
    sdf << "        <visual name='vis'><geometry><mesh><uri>" << GROUND_URI << "</uri></mesh></geometry></visual>\n";
    sdf << "      </link>\n";
    sdf << "    </model>\n\n";

    // Center Offset
    double center_x = img.cols / 2.0;
    double center_y = img.rows / 2.0;
    int valid_trees = 0;

    // 4. Loop Through Contours
    for (size_t i = 0; i < contours.size(); ++i)
    {
        // Skip noise
        if (cv::contourArea(contours[i]) < 3.0)
            continue;

        double w, h, angle;
        cv::Point2f center;

        // SMART SCALING: Check if tree is Round or Oval
        if (contours[i].size() >= 5)
        {
            // Fit Ellipse for oval shapes
            cv::RotatedRect box = cv::fitEllipse(contours[i]);
            center = box.center;
            w = box.size.width;
            h = box.size.height;
            angle = box.angle; // Degrees
        }
        else
        {
            // Fallback for tiny blobs
            float r;
            cv::minEnclosingCircle(contours[i], center, r);
            w = r * 2;
            h = r * 2;
            angle = 0.0;
        }

        // Convert to World Coordinates
        double world_x = (center.x - center_x) * RESOLUTION;
        double world_y = -(center.y - center_y) * RESOLUTION; // Flip Y for Gazebo
        double world_yaw = angle * (M_PI / 180.0);            // Degrees to Radians

        // Calculate Scale
        double scale_x = (w * RESOLUTION) / BASE_TREE_DIAMETER;
        double scale_y = (h * RESOLUTION) / BASE_TREE_DIAMETER;
        double scale_z = (scale_x + scale_y) / 2.0;

        // Clamp Scale (Safety)
        if (scale_x < 0.1)
            scale_x = 0.1;
        if (scale_x > 5.0)
            scale_x = 5.0;
        if (scale_y < 0.1)
            scale_y = 0.1;
        if (scale_y > 5.0)
            scale_y = 5.0;

        // --- WRITE MODEL ---
        sdf << "    <model name='tree_" << i << "'>\n";
        sdf << "      <static>true</static>\n";
        sdf << "      <pose>" << world_x << " " << world_y << " 0 0 0 " << world_yaw << "</pose>\n";
        sdf << "      <link name='link'>\n";

        // VISUAL: The Full Tree Mesh (Leaves + Trunk)
        sdf << "        <visual name='visual'>\n";
        sdf << "          <geometry><mesh>\n";
        sdf << "            <uri>" << TREE_URI << "</uri>\n";
        sdf << "            <scale>" << scale_x << " " << scale_y << " " << scale_z << "</scale>\n";
        sdf << "          </mesh></geometry>\n";
        sdf << "        </visual>\n";

        // COLLISION: Invisible Cylinder (So robot drives through leaves)
        sdf << "        <collision name='trunk_collision'>\n";
        sdf << "          <pose>0 0 " << (TRUNK_COLLISION_HEIGHT / 2.0) << " 0 0 0</pose>\n";
        sdf << "          <geometry>\n";
        sdf << "            <cylinder>\n";
        sdf << "              <radius>" << TRUNK_COLLISION_RADIUS << "</radius>\n";
        sdf << "              <length>" << TRUNK_COLLISION_HEIGHT << "</length>\n";
        sdf << "            </cylinder>\n";
        sdf << "          </geometry>\n";
        sdf << "        </collision>\n";

        sdf << "      </link>\n";
        sdf << "    </model>\n";

        valid_trees++;
    }

    // Close File
    sdf << "  </world>\n";
    sdf << "</sdf>\n";
    sdf.close();

    std::cout << "--- Success! Generated " << valid_trees << " trees in " << OUTPUT_FILE << " ---" << std::endl;
    return 0;
}