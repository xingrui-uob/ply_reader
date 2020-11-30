/**
 * Demo program for simulation library
 * A virtual camera generates simulated point clouds
 * No visual output, point clouds saved to file
 *
 * three different demo modes:
 * 0 - static camera, 100 poses
 * 1 - circular camera flying around the scene, 16 poses
 * 2 - camera translates between 2 poses using slerp, 20 poses
 * pcl_sim_terminal_demo 2 ../../../../kmcl/models/table_models/meta_model.ply
 */

#include <pcl/common/time.h> // for getTime
#include "sim_io.h"
#include "glob.h"
#include <cmath>
#include <boost/program_options.hpp>
#include <iostream>

using namespace Eigen;
using namespace pcl;
using namespace pcl::console;
using namespace pcl::io;
using namespace pcl::simulation;

SimExample::Ptr simexample;
namespace po = boost::program_options;
std::string pose_file, ply_path;
std::string out_path, intr_file, depth_path;

int load_pose(std::string filename, std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> &poses)
{
    std::ifstream file(filename);
    if (!file.is_open())
        return false;

    Eigen::Matrix4d init_pose;
    float m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23;
    while (file >> m00 >> m01 >> m02 >> m03 >> m10 >> m11 >> m12 >> m13 >> m20 >> m21 >> m22 >> m23)
    {
        Eigen::Matrix4d mat;
        mat << m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, 0, 0, 0, 1;
        Eigen::Matrix3d rot = mat.topLeftCorner(3, 3);
        rot *= Eigen::AngleAxisd(-M_PI, Eigen::Vector3d::UnitZ()).matrix();
        rot *= Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitY()).matrix();
        rot *= Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()).matrix();
        mat.topLeftCorner(3, 3) = rot;
        Eigen::Isometry3d pose;
        pose = mat;
        // Eigen::Matrix3d rotation = mat.topLeftCorner(3, 3);
        // pose *= rotation;
        // pose.translation() = mat.topRightCorner(3, 1);
        poses.push_back(pose);
    }

    return poses.size();
}

// Output the simulated output to file:
void write_sim_output(const std::string &fname_root)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_out(new pcl::PointCloud<pcl::PointXYZRGB>);

    // Read Color Buffer from the GPU before creating PointCloud:
    // By default the buffers are not read back from the GPU
    // simexample->rl_->getColorBuffer();
    simexample->rl_->getDepthBuffer();

    // simexample->write_score_image(simexample->rl_->getScoreBuffer(), std::string(fname_root + "_score.png"));
    // simexample->write_rgb_image(simexample->rl_->getColorBuffer(), std::string(fname_root + "_rgb.png"));
    // simexample->write_depth_image(simexample->rl_->getDepthBuffer(), std::string(fname_root));
    simexample->write_depth_image_uint(simexample->rl_->getDepthBuffer(), fname_root);

    // Demo interacton with RangeImage:
    pcl::RangeImagePlanar rangeImage;
    simexample->rl_->getRangeImagePlanar(rangeImage);
}

int main(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()(
        "help", "produce a help message")(
        "ply-path", po::value<std::string>(&ply_path)->required(), "set path to the ply files.")(
        "pose-file", po::value<std::string>(&pose_file)->required(), "set path to the pose files.")(
        "intr-file", po::value<std::string>(&intr_file)->required(), "set path to the intrinsic file.")(
        "out-path", po::value<std::string>(&out_path)->required(), "set path to the output files.");

    po::variables_map vm;

    try
    {
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << desc << "\n";
            return 1;
        }

        po::notify(vm);
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << "\n";
        std::cout << desc << "\n";
        return -1;
    }

    simexample = SimExample::Ptr(new SimExample(argc, argv, intr_file, ply_path));
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    load_pose(pose_file, poses);

    for (std::size_t i = 0; i < poses.size(); i += 1)
    {
        std::stringstream ss;
        ss << out_path << i << ".png"; // << ".pcd";
        simexample->doSim(poses[i]);

        write_sim_output(ss.str());
    }

    return 0;
}