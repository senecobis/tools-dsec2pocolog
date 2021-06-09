#ifndef dsec2pocolog_TYPES_HPP
#define dsec2pocolog_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/samples/Frame.hpp>

namespace dsec2pocolog
{
    struct Config
    {
        std::string meta_filename;//the meta yaml file
        std::string event_camera_idx; //index of the camera in the calib [0, 1, 2, 3]
        std::string rgb_camera_idx;  //index of the camare in the calib [0, 1, 2, 3]

        base::samples::frame::frame_size_t in_events_frame_size; // input event image resolution
        base::samples::frame::frame_size_t out_image_size; // desired output image resolution

        unsigned int events_pkgsize;

        /** Path to folders, all path folders are relative to the root folder **/
        std::string root_folder; // path root folder to the dataset
        std::string output_folder; // path to output folder
        std::string images_folder; //folder where the images are
        std::string disparity_events_folder; //disparity for event camera
        std::string disparity_images_folder; //disparity for standard camera

        /** File name, all path files are relative to the root folder **/
        std::string img_ts_filename; //file with timestamps
        std::string events_filename; //HDF5 file for events
        std::string imu_filename; //HDF5 file for imu
        std::string disparity_ts_filename; //file with timestamps
        std::string cam_to_imu_filename; //T_cam_imu
        std::string cam_to_cam_filename; //calibration file
    };
}

#endif

