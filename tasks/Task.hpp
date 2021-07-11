/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef DSEC2POCOLOG_TASK_TASK_HPP
#define DSEC2POCOLOG_TASK_TASK_HPP

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/parser.h>

#include <opencv2/core/mat.hpp>

#include <base/samples/Frame.hpp>
#include <base/samples/DistanceImage.hpp>
#include "dsec2pocolog/TaskBase.hpp"

#include <cmath>

namespace dsec2pocolog{

    struct CameraCalib
    {
        cv::Mat K; // intrinsics
        cv::Mat Kr; //rectified K
        cv::Vec4d D; //distortion
        cv::Mat Rr;// rect matrix
        std::string distortion_model; //model
        int height, width; //image size
        cv::Mat Q; //disparity to depth as in https://docs.opencv.org/4.5.2/d9/d0c/group__calib3d.html#ga1bc1152bd57d63bc524204f21fde6e02
        cv::Mat Tij; //Transformation matrix in DSEC can be T_10, T21, T32
    };

    struct Event
    {
        /** Variable **/
        std::vector<double> t;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> p;
        std::vector<double> offset;
    };

    struct IMU
    {
        std::vector<base::Vector6d> values;
        std::vector<double> t;
    };

    /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

       The corresponding C++ class can be edited in tasks/Task.hpp and
       tasks/Task.cpp, and will be put in the dsec2pocolog namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','dsec2pocolog::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */

    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:

        /** Mean gravity value at Earth surface [m/s^2] **/
        static constexpr float GRAVITY = 9.81;

        /** Comfiguration **/
        dsec2pocolog::Config config;

        /** Variable **/
        ::base::Time starting_time;
        dsec2pocolog::Event events;
        dsec2pocolog::IMU imu;
        std::vector<double> image_ts;
        std::vector<double> disp_ts;
        std::vector<std::string> img_fname;
        std::vector<std::string> disp_img_fname;
        std::vector<std::string> disp_event_fname;

        /** Calibration information **/
        CameraCalib event_cam_calib;
        CameraCalib rgb_cam_calib;

        /** Output ports **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> img_msg;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> disp_img_msg;
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> disp_event_msg;
        RTT::extras::ReadOnlyPointer<base::samples::DistanceImage> depth_msg;

        void convertData();

        void writeEvents(float &t_offset);

        void writeIMU(float &t_offset);

        void writeRGB();

        void writeDisparityRGB();

        void writeDepthEvent();

        base::samples::DistanceImage disparityToDepth(cv::Mat &disp);

        cv::Mat RGBToEventFrame(cv::Mat &frame,  cv::Mat &P, int &height, int &width);

        void readH5Dataset(std::string fname, std::string dataset, std::vector<double> &data);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "dsec2pocolog::Task");

        /** Default deconstructor of Task
         */
	    ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** DSEC use ruamel python stupid library to wirte the yaml **/
        static CameraCalib readCameraInfo(std::string calib_fname, std::string cam_id)
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
                calib.Kr = cv::Mat_<double>::eye(3, 3);
                calib.Kr.at<double>(0,0) = cam_matrix[0].as<double>();
                calib.Kr.at<double>(0,2) = cam_matrix[2].as<double>();
                calib.Kr.at<double>(1,1) = cam_matrix[1].as<double>();
                calib.Kr.at<double>(1,2) = cam_matrix[3].as<double>();
            };
 
            auto extrinsics = [cam_id, &calib] (const std::string &key,  YAML::Node &attributes)
            {
                YAML::Node rect_matrix = attributes["R_rect" + cam_id];
                calib.Rr = cv::Mat_<double>::eye(3, 3);
                calib.Rr.at<double>(0,0) = rect_matrix[0][0].as<double>();
                calib.Rr.at<double>(0,1) = rect_matrix[0][1].as<double>();
                calib.Rr.at<double>(0,2) = rect_matrix[0][2].as<double>();
                calib.Rr.at<double>(1,0) = rect_matrix[1][0].as<double>();
                calib.Rr.at<double>(1,1) = rect_matrix[1][1].as<double>();
                calib.Rr.at<double>(1,2) = rect_matrix[1][2].as<double>();
                calib.Rr.at<double>(2,0) = rect_matrix[2][0].as<double>();
                calib.Rr.at<double>(2,1) = rect_matrix[2][1].as<double>();
                calib.Rr.at<double>(2,2) = rect_matrix[2][2].as<double>();

                calib.Tij = cv::Mat_<double>::eye(4, 4);
                if ((cam_id.compare("0") == 0) || (cam_id.compare("1") == 0)) 
                {
                    /** Left cameras **/
                    YAML::Node T = attributes["T_10"];
                    calib.Tij.at<double>(0,0) = T[0][0].as<double>();
                    calib.Tij.at<double>(0,1) = T[0][1].as<double>();
                    calib.Tij.at<double>(0,2) = T[0][2].as<double>();
                    calib.Tij.at<double>(0,3) = T[0][3].as<double>();
                    calib.Tij.at<double>(1,0) = T[1][0].as<double>();
                    calib.Tij.at<double>(1,1) = T[1][1].as<double>();
                    calib.Tij.at<double>(1,2) = T[1][2].as<double>();
                    calib.Tij.at<double>(1,3) = T[1][3].as<double>();
                    calib.Tij.at<double>(2,0) = T[2][0].as<double>();
                    calib.Tij.at<double>(2,1) = T[2][1].as<double>();
                    calib.Tij.at<double>(2,2) = T[2][2].as<double>();
                    calib.Tij.at<double>(2,3) = T[2][3].as<double>();
                    calib.Tij.at<double>(3,0) = T[3][0].as<double>();
                    calib.Tij.at<double>(3,1) = T[3][1].as<double>();
                    calib.Tij.at<double>(3,2) = T[3][2].as<double>();
                    calib.Tij.at<double>(3,3) = T[3][3].as<double>();

                    if (cam_id.compare("1") == 0)
                        calib.Tij = calib.Tij.inv();

                }
                else if ((cam_id.compare("2") == 0) || (cam_id.compare("3") == 0)) 
                {
                    /** Left cameras **/
                    YAML::Node T = attributes["T_32"];
                    calib.Tij.at<double>(0,0) = T[0][0].as<double>();
                    calib.Tij.at<double>(0,1) = T[0][1].as<double>();
                    calib.Tij.at<double>(0,2) = T[0][2].as<double>();
                    calib.Tij.at<double>(0,3) = T[0][3].as<double>();
                    calib.Tij.at<double>(1,0) = T[1][0].as<double>();
                    calib.Tij.at<double>(1,1) = T[1][1].as<double>();
                    calib.Tij.at<double>(1,2) = T[1][2].as<double>();
                    calib.Tij.at<double>(1,3) = T[1][3].as<double>();
                    calib.Tij.at<double>(2,0) = T[2][0].as<double>();
                    calib.Tij.at<double>(2,1) = T[2][1].as<double>();
                    calib.Tij.at<double>(2,2) = T[2][2].as<double>();
                    calib.Tij.at<double>(2,3) = T[2][3].as<double>();
                    calib.Tij.at<double>(3,0) = T[3][0].as<double>();
                    calib.Tij.at<double>(3,1) = T[3][1].as<double>();
                    calib.Tij.at<double>(3,2) = T[3][2].as<double>();
                    calib.Tij.at<double>(3,3) = T[3][3].as<double>();

                    if (cam_id.compare("3") == 0)
                        calib.Tij = calib.Tij.inv();
                }

            };
            auto disparity = [cam_id, &calib] (const std::string &key,  YAML::Node &attributes, std::string id)
            {
                calib.Q = cv::Mat_<float>::eye(4, 4);
                YAML::Node M = attributes[id];
                calib.Q.at<float>(0,0) = M[0][0].as<float>();
                calib.Q.at<float>(0,1) = M[0][1].as<float>();
                calib.Q.at<float>(0,2) = M[0][2].as<float>();
                calib.Q.at<float>(0,3) = M[0][3].as<float>();
                calib.Q.at<float>(1,0) = M[1][0].as<float>();
                calib.Q.at<float>(1,1) = M[1][1].as<float>();
                calib.Q.at<float>(1,2) = M[1][2].as<float>();
                calib.Q.at<float>(1,3) = M[1][3].as<float>();
                calib.Q.at<float>(2,0) = M[2][0].as<float>();
                calib.Q.at<float>(2,1) = M[2][1].as<float>();
                calib.Q.at<float>(2,2) = M[2][2].as<float>();
                calib.Q.at<float>(2,3) = M[2][3].as<float>();
                calib.Q.at<float>(3,0) = M[3][0].as<float>();
                calib.Q.at<float>(3,1) = M[3][1].as<float>();
                calib.Q.at<float>(3,2) = M[3][2].as<float>();
                calib.Q.at<float>(3,3) = M[3][3].as<float>();
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
                    /** Read the disparity to depth info **/
                    if(cam_id.compare("0") == 0)
                        disparity(key, attributes, "cams_03");
                    else
                        disparity(key, attributes, "cams_12");

                }
            }

            return calib;
        }

        template <typename P, typename V > 
        static void drawEventsOnImage (const std::vector<P> &points, const std::vector<V> &values, cv::Mat &img,
                                    cv::Vec3b &color_positive, cv::Vec3b &color_negative, const std::string &method = "nn")
        {
            auto clip = [](const int n, const int lower, const int upper)
            {
                return std::max(lower, std::min(n, upper));
            };

            cv::Size s = img.size();
            auto it_x = points.begin();
            auto it_p = values.begin();
            while(it_x != points.end() && it_p != values.end())
            {
                /** Get the color based on the polarity **/
                auto color = color_negative;
                if ((*it_p))
                {
                    auto color = color_positive;
                }

                if (method.compare("nn") == 0)
                {
                    cv::Point2i x_int = *it_x;
                    x_int.x = clip(x_int.x, 0, s.width - 1);
                    x_int.y = clip(x_int.x, 0, s.height - 1);

                    img.at<cv::Vec3b>(x_int) = color;
                    
                }
                else if (method.compare("bilinear"))
                {
                    int x0 = floor(it_x->x);
                    int y0 = floor(it_x->y);
                    int x1 = x0 + 1;
                    int y1 = x1 + 1;

                    /** compute the voting weights. Note: assign weight 0 if the point is out of the image **/
                    float wa = (x1 - it_x->x) * (y1 - it_x->y) * (x0 < s.width) * (y0 < s.height) * (x0 >= 0) * (y0 >= 0);
                    float wb = (x1 - it_x->x) * (it_x->y - y0) * (x0 < s.width) * (y1 < s.height) * (x0 >= 0) * (y1 >= 0);
                    float wc = (it_x->x - x0) * (y1 - it_x->y) * (x1 < s.width) * (y0 < s.height) * (x1 >= 0) * (y0 >= 0);
                    float wd = (it_x->x - x0) * (it_x->y - y0) * (x1 < s.width) * (y1 < s.height) * (x1 >= 0) * (y1 >= 0);

                    x0 = clip(x0, 0, s.width - 1);
                    x1 = clip(x1, 0, s.width - 1);
                    y0 = clip(y0, 0, s.height - 1);
                    y1 = clip(y1, 0, s.height - 1);
 
                    img.at<cv::Vec3b>(x0, y0) = color;
                    img.at<cv::Vec3b>(x0, y1) = color;
                    img.at<cv::Vec3b>(x1, y0) = color;
                    img.at<cv::Vec3b>(x1, y1) = color;
                }
                ++it_x;
                ++it_p;
            }
            return;
        }

        static std::string type2str(int type)
        {
            std::string r;

            uchar depth = type & CV_MAT_DEPTH_MASK;
            uchar chans = 1 + (type >> CV_CN_SHIFT);

            switch ( depth ) {
                case CV_8U:  r = "8U"; break;
                case CV_8S:  r = "8S"; break;
                case CV_16U: r = "16U"; break;
                case CV_16S: r = "16S"; break;
                case CV_32S: r = "32S"; break;
                case CV_32F: r = "32F"; break;
                case CV_64F: r = "64F"; break;
                default:     r = "User"; break;
            }

            r += "C";
            r += (chans+'0');

            return r;
        };
    };
}

#endif

