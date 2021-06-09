/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <iomanip>
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>

#include <blosc_filter.h>
#include <hdf5/serial/hdf5.h>
#include <hdf5/serial/H5Cpp.h>
#include <boost/filesystem.hpp>

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
    std::cout << std::setprecision(3);
    float t_offset = this->events.offset[0];
    std::cout<<"events first time: "<<this->events.t[0]+t_offset<<" last:"<<this->events.t[this->events.t.size()-1]+t_offset<<std::endl;
    std::cout<<"imu first time: "<<this->imu.t[0]<<" last:"<<this->imu.t[this->imu.t.size()-1]<<std::endl;
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
