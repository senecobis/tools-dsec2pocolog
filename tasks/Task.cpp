/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

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

    fs::path events_fname = fs::path(config.root_folder)/ fs::path(config.events_filename);

    try
    {
        std::vector<double> data;
        readH5Datasets(events_fname.string(), "events/t", data);
        std::vector<double> t(data);
        data.resize(0);
        readH5Datasets(events_fname.string(), "events/x", data);
        std::vector<uint16_t> x(data.begin(), data.end());
        data.resize(0);
        readH5Datasets(events_fname.string(), "events/y", data);
        std::vector<uint16_t> y(data.begin(), data.end());
        data.resize(0);
        readH5Datasets(events_fname.string(), "events/p", data);
        std::vector<uint8_t> p(data.begin(), data.end());
        data.resize(0);
        readH5Datasets(events_fname.string(), "t_offset", data);
        std::vector<double> offset_data(data.begin(), data.end());
        data.resize(0);

        float t_offset = offset_data[0];

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

void Task::readH5Datasets(std::string fname, std::string dataset, std::vector<double> &data)
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
        dset.read(data.data(), PredType::NATIVE_DOUBLE,    generator.starte
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

    // close the HDF5 file
    file.close();
}
