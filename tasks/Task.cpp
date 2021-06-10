/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <iostream>
#include <iomanip>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include <glob.h>

#include <blosc_filter.h>
#include <hdf5/serial/hdf5.h>
#include <hdf5/serial/H5Cpp.h>
#include <boost/filesystem.hpp>

/** Base types **/
#include <base/samples/IMUSensors.hpp>
#include <base/samples/EventArray.hpp>

/** Frame helper **/
#include <frame_helper/FrameHelper.h>

#include "Task.hpp"

using namespace H5;
using namespace dsec2pocolog;
namespace fs = boost::filesystem;

Task::Task(std::string const& name)
    : TaskBase(name)
{

}

Task::~Task()
{

}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Set the event frame member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

    char *version, *date;
    int r = register_blosc(&version, &date);
    printf("Blosc version info: %s (%s)\n", version, date);


    this->config = _config.value();

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** Read Meta data **/
    fs::path meta_fname = fs::path(config.root_folder)/ fs::path(config.meta_filename);
    YAML::Node map = YAML::LoadFile(meta_fname.string());

    for(YAML::const_iterator it=map.begin(); it!=map.end(); ++it)
    {
        const std::string &key=it->first.as<std::string>();
        YAML::Node attributes = it->second;
        if (key.compare("sequence") == 0)
        {
            YAML::Node start_time = attributes["name"];
            this->starting_time = base::Time::fromString(start_time.as<std::string>(), base::Time::Resolution::Seconds);

        }
    }

    std::cout<<"DSEC Dataset Starting Time: "<<this->starting_time.toString()<<std::endl;

    /** Read the event camera calibration file **/
    fs::path calib_fname = fs::path(config.root_folder)/ fs::path(config.cam_to_cam_filename);
    this->event_cam_calib = readCameraInfo(calib_fname.string(),this->config.event_camera_idx );
    std::cout<<"COEFF:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"Model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->event_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->event_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"K_:"<<this->event_cam_calib.K_<<std::endl;
    std::cout<<"Rect:"<<this->event_cam_calib.Rect<<std::endl;
    std::cout<<"T:"<<this->event_cam_calib.T_dd<<std::endl;
    this->rgb_cam_calib = readCameraInfo(calib_fname.string(),this->config.rgb_camera_idx );
    std::cout<<"COEFF:"<<this->rgb_cam_calib.D<<std::endl;
    std::cout<<"Model:"<<this->rgb_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->rgb_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->rgb_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->rgb_cam_calib.K<<std::endl;
    std::cout<<"K_:"<<this->rgb_cam_calib.K_<<std::endl;
    std::cout<<"Rect:"<<this->rgb_cam_calib.Rect<<std::endl;
    std::cout<<"T:"<<this->rgb_cam_calib.T_dd<<std::endl;

    /** Read images timestamps **/
    fs::path img_ts_fname = fs::path(config.root_folder)/ fs::path(config.img_ts_filename);
    std::ifstream infile;
    infile.open(img_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<img_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    double ts;
    while (infile >> ts)
    {
        this->image_ts.push_back(ts);
    }
    infile.close();

    /** Read depthmaps timestamps **/
    fs::path disp_ts_fname = fs::path(config.root_folder)/ fs::path(config.disparity_ts_filename);
    infile.open(disp_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<disp_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    while (infile >> ts)
    {
        this->disp_ts.push_back(ts);
    }
    infile.close();

    /** Read Events data **/
    fs::path events_fname = fs::path(config.root_folder)/ fs::path(config.events_filename);
    try
    {
        std::vector<double> data;
        readH5Dataset(events_fname.string(), "events/t", data);
        this->events.t = std::vector<double> (data);
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/x", data);
        this->events.x = std::vector<double> (data.begin(), data.end());
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/y", data);
        this->events.y = std::vector<double> (data.begin(), data.end());
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/p", data);
        this->events.p = std::vector<double> (data.begin(), data.end());
        data.resize(0);

        readH5Dataset(events_fname.string(), "t_offset", data);
        this->events.offset = std::vector<double> (data.begin(), data.end());
        data.resize(0);
    }
    // catch failure caused by the H5File operations
    catch( FileIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSet operations
    catch( DataSetIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataSpaceIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataTypeIException error )
    {
        error.printErrorStack();
        return -1;
    }
    
    /** Read IMU data **/
    fs::path imu_fname = fs::path(config.root_folder)/ fs::path(config.imu_filename);
    try
    {
        std::vector<double> data;
        readH5Dataset(imu_fname.string(), "imu", data);
        this->imu.values.resize(data.size()/6);
        size_t idx = 0;
        for (std::vector<base::Vector6d>::iterator it = this->imu.values.begin();
                it != this->imu.values.end(); ++it)
        {
            base::Vector6d &meas = *it;
            double *ptr = &data[idx];
            meas = Eigen::Map<base::Vector6d>(ptr, 6);
            idx += 6;
        }
        data.resize(0);

        readH5Dataset(imu_fname.string(), "t", data);
        this->imu.t = std::vector<double> (data.begin(), data.end());
        data.resize(0);
    }
    // catch failure caused by the H5File operations
    catch( FileIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSet operations
    catch( DataSetIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataSpaceIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataTypeIException error )
    {
        error.printErrorStack();
        return -1;
    }


    /** Name for the all the images **/
    fs::path path_images = fs::path(config.root_folder)/ fs::path(config.images_folder)/fs::path("*.png");
    std::cout<<path_images.string()<<std::endl;
    glob::glob glob_img(path_images.string());
    while (glob_img)
    {
        std::string path = (fs::path(config.root_folder)/fs::path(config.images_folder)/fs::path(glob_img.current_match())).string();
        this->img_fname.push_back(path);
        glob_img.next();
    }
    std::sort(this->img_fname.begin(), this->img_fname.end());

    /** Name for the all the disparity images **/
    fs::path path_disp_images = fs::path(config.root_folder)/ fs::path(config.disparity_images_folder)/fs::path("*.png");
    std::cout<<path_disp_images.string()<<std::endl;
    glob::glob glob_disp_i(path_disp_images.string());
    while (glob_disp_i)
    {
        std::string path = (fs::path(config.root_folder)/fs::path(config.disparity_images_folder)/fs::path(glob_disp_i.current_match())).string();
        this->disp_img_fname.push_back(path);
        glob_disp_i.next();
    }
    std::sort(this->disp_img_fname.begin(), this->disp_img_fname.end());

    /** Name for the all the disparity event frames **/
    fs::path path_disp_events = fs::path(config.root_folder)/ fs::path(config.disparity_events_folder)/fs::path("*.png");
    std::cout<<path_disp_events.string()<<std::endl;
    glob::glob glob_disp_e(path_disp_events.string());
    while (glob_disp_e)
    {
        std::string path = (fs::path(config.root_folder)/fs::path(config.disparity_events_folder)/fs::path(glob_disp_e.current_match())).string();
        this->disp_event_fname.push_back(path);
        glob_disp_e.next();
    }
    std::sort(this->disp_event_fname.begin(), this->disp_event_fname.end());

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    std::cout<<"t_size:"<<this->events.t.size()<<std::endl;
    std::cout<<"x_size:"<<this->events.x.size()<<std::endl;
    std::cout<<"y_size:"<<this->events.y.size()<<std::endl;
    std::cout<<"offset_size:"<<this->events.offset.size()<<std::endl;
    std::cout<<"imu:"<<this->imu.values.size()<<std::endl;
    std::cout<<"imu time:"<<this->imu.t.size()<<std::endl;
    std::cout << std::fixed;
    std::cout << std::setprecision(6);
    float t_offset = this->events.offset[0];
    std::cout<<"events first time["<<this->events.t.size()<<"]: "<<this->events.t[0]+t_offset<<" last:"<<this->events.t[this->events.t.size()-1]+t_offset<<std::endl;
    std::cout<<"imu first time["<<this->imu.t.size()<<"]: "<<this->imu.t[0]<<" last:"<<this->imu.t[this->imu.t.size()-1]<<std::endl;
    std::cout<<"image first time["<<this->image_ts.size()<<"]: "<<this->image_ts[0]<<" last:"<<this->image_ts[this->image_ts.size()-1]<<std::endl;
    std::cout<<"disparity first time["<<this->disp_ts.size()<<"]: "<<this->disp_ts[0]<<" last:"<<this->disp_ts[this->disp_ts.size()-1]<<std::endl;
    std::cout<<"Number of RGB images: "<<this->img_fname.size()<<std::endl;
    std::cout<<"Number of disparity for RGB images: "<<this->disp_img_fname.size()<<std::endl;
    std::cout<<"Number of disparity for event images: "<<this->disp_event_fname.size()<<std::endl;

    /** Output port variables **/
    ::base::samples::IMUSensors imu_msg;
    ::base::samples::EventArray events_msg;

    /** Write the Events **/
    for (size_t i=0; i<this->events.t.size(); ++i)
    {
        ::base::samples::Event ev(
            this->events.x[i], this->events.y[i],
            this->starting_time + ::base::Time::fromMicroseconds(this->events.t[i] + t_offset),
            (uint8_t)this->events.p[i]);

        if (events_msg.events.size() == 0)
        {
            events_msg.time = ev.ts;
        }
        events_msg.events.push_back(ev);

        if (i%this->config.events_pkgsize == 0)
        {
            std::cout<<"events ["<<events_msg.events.size()<<"] at"<<events_msg.time.toString()<<std::endl;
            events_msg.height = this->event_cam_calib.height;
            events_msg.width = this->event_cam_calib.width;
            this->_events.write(events_msg);
            events_msg.events.clear();
        }

    }

    /** Write the IMU **/
    ::base::Time first_ev_time = this->starting_time + ::base::Time::fromMicroseconds(this->events.t[0] + t_offset);
    ::base::Time last_ev_time = this->starting_time + ::base::Time::fromMicroseconds(this->events.t[this->events.t.size()-1] + t_offset);
    for (size_t i=0; i<this->imu.t.size(); ++i)
    {
        ::base::Vector6d &values = this->imu.values[i];
        ::base::samples::IMUSensors imusamples;
        imusamples.time = this->starting_time + ::base::Time::fromMicroseconds(this->imu.t[i]);
        imusamples.acc << GRAVITY * values[0], GRAVITY * values[1], GRAVITY * values[2]; //[m/s^2]
        imusamples.gyro << values[3], values[4], values[5]; //[rad/s]
        if (imusamples.time >= first_ev_time)
        {
            std::cout<<"IMU at"<<imusamples.time.toString()<<std::endl;
            this->_imu.write(imusamples);
        }
        if (imusamples.time > last_ev_time)
            break;
    }

    /** Write the images **/
    auto it_img =this->img_fname.begin();
    auto it_ts =this->image_ts.begin();
    std::cout<<"Writing images... ";
    while(it_img != this->img_fname.end() && it_ts != this->image_ts.end())
    {
        /** Read the image file **/
        cv::Mat img, orig_img = cv::imread(*it_img, cv::IMREAD_COLOR);

        /** Resize the image to the desired in the config **/
        cv::resize(orig_img, img, cv::Size(this->event_cam_calib.width, this->event_cam_calib.height), 0, 0);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *img_msg_ptr = this->img_msg.write_access();
        img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(img, *img_msg_ptr);

        /** Write into the port **/
        img_msg_ptr->time =  this->starting_time + ::base::Time::fromMicroseconds(*it_ts);
        img_msg_ptr->received_time = img_msg_ptr->time;
        this->img_msg.reset(img_msg_ptr);
        _frame.write(this->img_msg);

        ++it_img;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::readH5Dataset(std::string fname, std::string dataset, std::vector<double> &data)
{
    H5File file( fname.c_str(), H5F_ACC_RDONLY );
    DataSet dset = file.openDataSet(dataset.c_str());
    std::cout<<"Reading dataset "<<dataset<<std::endl;

    //Get dataspace of the dataset.
    DataSpace dataspace = dset.getSpace();

    // Get the number of dimensions in the dataspace.
    int rank = dataspace.getSimpleExtentNdims();

    if (rank == 0)
    {
        // for single value datasets
        // create a vector the same size as the dataset
        data.resize(1);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;
        dset.read(data.data(), PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL);
    
    }
    else if (rank == 1)
    {
        // for array datasets
        // Get the dimension size of each dimension in the dataspace and display them.
        hsize_t dims_out[1];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        std::cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) << std::endl;
        std::cout<<"ndims :"<<ndims<<std::endl;
        // Define the memory dataspace
        DataSpace memspace (1,dims_out);

        // create a vector the same size as the dataset
        data.resize(dims_out[0]);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;

        // pass pointer to the array (or vector) to read function, along with the data type and space.
        dset.read(data.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);
    }
    else if (rank == 2)
    {
        hsize_t dims_out[2];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        std::cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) <<" x " <<(unsigned long) (dims_out[1])<< std::endl;
        std::cout<<"ndims :"<<ndims<<std::endl;
        // Define the memory dataspace
        DataSpace memspace (rank,dims_out);

        // create a vector the same size as the dataset
        data.resize(dims_out[0]*dims_out[1]);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;

        // pass pointer to the array (or vector) to read function, along with the data type and space.
        dset.read(data.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);
    }

    // close the HDF5 file
    file.close();
}

CameraCalib Task::readCameraInfo(std::string calib_fname, std::string cam_id)
{
    CameraCalib calib;


    /* Lambda methods **/
    auto intrinsics = [cam_id, &calib] (const std::string &key,  YAML::Node &attributes)
    {
        //std::cout<<"KEY: "<<key<<"\nATTRIBUTES: "<<attributes<<std::endl;
        YAML::Node event_cam = attributes["cam" + cam_id];
        YAML::Node event_dist = event_cam["distortion_coeffs"];
        calib.D = cv::Vec4d(event_dist[0].as<double>(),
                            event_dist[1].as<double>(),
                            event_dist[2].as<double>(),
                            event_dist[3].as<double>());
        calib.distortion_model = event_cam["distortion_model"].as<std::string>();
        calib.width = event_cam["resolution"][0].as<int>();
        calib.height = event_cam["resolution"][1].as<int>();
        YAML::Node cam_matrix = event_cam["camera_matrix"];
        calib.K = cv::Mat_<double>::eye(3, 3);
        calib.K.at<double>(0,0) = cam_matrix[0].as<double>();
        calib.K.at<double>(0,2) = cam_matrix[2].as<double>();
        calib.K.at<double>(1,1) = cam_matrix[1].as<double>();
        calib.K.at<double>(1,2) = cam_matrix[3].as<double>();
        /** Get  rectified intrinsics **/
        event_cam = attributes["camRect" + cam_id];
        cam_matrix = event_cam["camera_matrix"];
        calib.K_ = cv::Mat_<double>::eye(3, 3);
        calib.K_.at<double>(0,0) = cam_matrix[0].as<double>();
        calib.K_.at<double>(0,2) = cam_matrix[2].as<double>();
        calib.K_.at<double>(1,1) = cam_matrix[1].as<double>();
        calib.K_.at<double>(1,2) = cam_matrix[3].as<double>();
    };
    
    auto extrinsics = [cam_id, &calib] (const std::string &key,  YAML::Node &attributes)
    {
        YAML::Node rect_matrix = attributes["R_rect" + cam_id];
        calib.Rect = cv::Mat_<double>::eye(3, 3);
        calib.Rect.at<double>(0,0) = rect_matrix[0][0].as<double>();
        calib.Rect.at<double>(0,1) = rect_matrix[0][1].as<double>();
        calib.Rect.at<double>(0,2) = rect_matrix[0][2].as<double>();
        calib.Rect.at<double>(1,0) = rect_matrix[1][0].as<double>();
        calib.Rect.at<double>(1,1) = rect_matrix[1][1].as<double>();
        calib.Rect.at<double>(1,2) = rect_matrix[1][2].as<double>();
        calib.Rect.at<double>(2,0) = rect_matrix[2][0].as<double>();
        calib.Rect.at<double>(2,1) = rect_matrix[2][1].as<double>();
        calib.Rect.at<double>(2,2) = rect_matrix[2][2].as<double>();
    };
    auto disparity = [cam_id, &calib] (const std::string &key,  YAML::Node &attributes, std::string id)
    {
        calib.T_dd = cv::Mat_<double>::eye(4, 4);
        YAML::Node M = attributes[id];
        calib.T_dd.at<double>(0,0) = M[0][0].as<double>();
        calib.T_dd.at<double>(0,1) = M[0][1].as<double>();
        calib.T_dd.at<double>(0,2) = M[0][2].as<double>();
        calib.T_dd.at<double>(0,3) = M[0][3].as<double>();
        calib.T_dd.at<double>(1,0) = M[1][0].as<double>();
        calib.T_dd.at<double>(1,1) = M[1][1].as<double>();
        calib.T_dd.at<double>(1,2) = M[1][2].as<double>();
        calib.T_dd.at<double>(1,3) = M[1][3].as<double>();
        calib.T_dd.at<double>(2,0) = M[2][0].as<double>();
        calib.T_dd.at<double>(2,1) = M[2][1].as<double>();
        calib.T_dd.at<double>(2,2) = M[2][2].as<double>();
        calib.T_dd.at<double>(2,3) = M[2][3].as<double>();
        calib.T_dd.at<double>(3,0) = M[3][0].as<double>();
        calib.T_dd.at<double>(3,1) = M[3][1].as<double>();
        calib.T_dd.at<double>(3,2) = M[3][2].as<double>();
        calib.T_dd.at<double>(3,3) = M[3][3].as<double>();
    };

    YAML::Node calibmap = YAML::LoadFile(calib_fname);
    YAML::const_iterator it=calibmap.begin();
    for(YAML::const_iterator it=calibmap.begin(); it!=calibmap.end(); ++it)
    {
        const std::string &key=it->first.as<std::string>();
        YAML::Node attributes = it->second;
        if (key.compare("intrinsics") == 0)
        {
            /** Read intrinsics **/
            intrinsics(key, attributes);
        }
        else if (key.compare("extrinsics") == 0)
        {
            /** Read Extrinsics **/
            extrinsics(key, attributes);
        }
        else if (key.compare("disparity_to_depth") == 0)
        {
            /** Read the disprity to depth info **/
            if(cam_id.compare("0") == 0)
                disparity(key, attributes, "cams_03");
            else
                disparity(key, attributes, "cams_12");

        }
    }



    //YAML::Node calibmap = YAML::LoadFile(calib_fname);
    //YAML::const_iterator it=calibmap.begin();
    //const std::string &key=it->first.as<std::string>();

    ///** Get intrinsics **/
    //YAML::Node attributes = it->second;
    //std::cout<<"KEY: "<<key<<"\nATTRIBUTES: "<<attributes<<std::endl;
    //YAML::Node event_cam = attributes["cam" + cam_id];
    //YAML::Node event_dist = event_cam["distortion_coeffs"];
    //std::cout<<"CONCAT: "<<"cam" + cam_id<<std::endl;

    //std::cout<<"DIST0: "<<event_dist[0]<<","<<event_dist[1]<<","<<event_dist[2]<<","<<event_dist[3]<<std::endl;
    //calib.D = cv::Vec4d(event_dist[0].as<double>(),
    //                    event_dist[1].as<double>(),
    //                    event_dist[2].as<double>(),
    //                    event_dist[3].as<double>());
    //calib.distortion_model = event_cam["distortion_model"].as<std::string>();
    //calib.width = event_cam["resolution"][0].as<int>();
    //calib.height = event_cam["resolution"][1].as<int>();
    //YAML::Node cam_matrix = event_cam["camera_matrix"];
    //calib.K = cv::Mat_<double>::eye(3, 3);
    //calib.K.at<double>(0,0) = cam_matrix[0].as<double>();
    //calib.K.at<double>(0,2) = cam_matrix[2].as<double>();
    //calib.K.at<double>(1,1) = cam_matrix[1].as<double>();
    //calib.K.at<double>(1,2) = cam_matrix[3].as<double>();
    ///** Get  rectified intrinsics **/
    //event_cam = attributes["camRect" + cam_id];
    //cam_matrix = event_cam["camera_matrix"];
    //calib.K_ = cv::Mat_<double>::eye(3, 3);
    //calib.K_.at<double>(0,0) = cam_matrix[0].as<double>();
    //calib.K_.at<double>(0,2) = cam_matrix[2].as<double>();
    //calib.K_.at<double>(1,1) = cam_matrix[1].as<double>();
    //calib.K_.at<double>(1,2) = cam_matrix[3].as<double>();

    //std::cout<<"*******************************"<<std::endl;

    //++it;
    ///** Get extrinsics **/
    //attributes = it->second;
    //YAML::Node rect_matrix = attributes["R_rect" + cam_id];
    //std::cout<<"RECT0: "<<rect_matrix[0][0]<<std::endl;
    //calib.Rect = cv::Mat_<double>::eye(3, 3);
    //calib.Rect.at<double>(0,0) = rect_matrix[0][0].as<double>();
    //calib.Rect.at<double>(0,1) = rect_matrix[0][1].as<double>();
    //calib.Rect.at<double>(0,2) = rect_matrix[0][2].as<double>();
    //calib.Rect.at<double>(1,0) = rect_matrix[1][0].as<double>();
    //calib.Rect.at<double>(1,1) = rect_matrix[1][1].as<double>();
    //calib.Rect.at<double>(1,2) = rect_matrix[1][2].as<double>();
    //calib.Rect.at<double>(2,0) = rect_matrix[2][0].as<double>();
    //calib.Rect.at<double>(2,1) = rect_matrix[2][1].as<double>();
    //calib.Rect.at<double>(2,2) = rect_matrix[2][2].as<double>();

    return calib;
}