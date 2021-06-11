/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <iostream>
#include <iomanip>
#include <fstream>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

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

    /** Set the img member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

    /** Set the disp event member **/
    ::base::samples::frame::Frame *disp_img = new ::base::samples::frame::Frame();
    this->disp_img_msg.reset(disp_img);
    disp_img = nullptr;

    /** Set the disp event member **/
    ::base::samples::frame::Frame *disp_event = new ::base::samples::frame::Frame();
    this->disp_event_msg.reset(disp_event);
    disp_event = nullptr;

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
    this->event_cam_calib = Task::readCameraInfo(calib_fname.string(), this->config.event_camera_idx);
    this->rgb_cam_calib = Task::readCameraInfo(calib_fname.string(), this->config.event_camera_idx);

    std::cout<<"COEFF:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"Model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->event_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->event_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"Kr:"<<this->event_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:"<<this->event_cam_calib.Rr<<std::endl;
    std::cout<<"Q:"<<this->event_cam_calib.Q<<std::endl;
    std::cout<<"T:"<<this->event_cam_calib.Tij<<std::endl;

    std::cout<<"COEFF:"<<this->rgb_cam_calib.D<<std::endl;
    std::cout<<"Model:"<<this->rgb_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->rgb_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->rgb_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->rgb_cam_calib.K<<std::endl;
    std::cout<<"Kr:"<<this->rgb_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:"<<this->rgb_cam_calib.Rr<<std::endl;
    std::cout<<"Q:"<<this->rgb_cam_calib.Q<<std::endl;
    std::cout<<"T:"<<this->rgb_cam_calib.Tij<<std::endl;

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


    this->convertData();

    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();
}

void Task::convertData()
{
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

        /** Resize to the event image size to have the same **/
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
    
    /** Write the disparity at RGB camera frame **/
    auto it_disp =this->disp_img_fname.begin();
    it_ts =this->disp_ts.begin();
    std::cout<<"Writing disparity in images...";
    while(it_disp != this->disp_img_fname.end() && it_ts != this->disp_ts.end())
    {
        /** Read the disp image file **/
        cv::Mat disp, orig_disp = cv::imread(*it_disp, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
        
        /** Resize to the event image size to have the same **/
        cv::resize(orig_disp, disp, cv::Size(this->event_cam_calib.width, this->event_cam_calib.height), 0, 0, cv::INTER_NEAREST);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *disp_img_msg_ptr = this->disp_img_msg.write_access();
        disp_img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(disp, *disp_img_msg_ptr);

        /** Write into the port **/
        disp_img_msg_ptr->time =  this->starting_time + ::base::Time::fromMicroseconds(*it_ts);
        disp_img_msg_ptr->received_time = disp_img_msg_ptr->time;
        this->disp_img_msg.reset(disp_img_msg_ptr);
        _disp_img.write(this->disp_img_msg);

        ++it_disp;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;

    /** Write the disparity at event camera frame **/
    it_disp =this->disp_event_fname.begin();
    it_ts =this->disp_ts.begin();
    std::cout<<"Writing disparity in events...";
    while(it_disp != this->disp_event_fname.end() && it_ts != this->disp_ts.end())
    {
        /** Read the disp image file **/
        cv::Mat disp = cv::imread(*it_disp, cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *disp_event_msg_ptr = this->disp_event_msg.write_access();
        disp_event_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(disp, *disp_event_msg_ptr);

        /** Write into the port **/
        disp_event_msg_ptr->time =  this->starting_time + ::base::Time::fromMicroseconds(*it_ts);
        disp_event_msg_ptr->received_time = disp_event_msg_ptr->time;
        this->disp_event_msg.reset(disp_event_msg_ptr);
        _disp_events.write(this->disp_event_msg);

        ++it_disp;
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
