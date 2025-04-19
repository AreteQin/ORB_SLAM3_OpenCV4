//
// Created by qin on 17/04/25.
//

#ifndef GPSTYPES_H
#define GPSTYPES_H
#include <string>
#include <array>
#include <Eigen/Core>
#include "SerializationUtils.h"

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/array.hpp>

namespace ORB_SLAM3
{
    namespace GPS
    {
        // Standard ROS header
        class Header
        {
            friend class boost::serialization::access;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int /* version */)
            {
                ar & seq;
                ar & stamp;
                ar & frame_id;
            }

        public:
            uint32_t seq{0};
            double stamp{0.0}; // seconds since epoch
            std::string frame_id;
        };

        // NavSat status as in sensor_msgs/NavSatStatus
        class NavSatStatus
        {
            friend class boost::serialization::access;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int /* version */)
            {
                ar & status;
                ar & service;
            }

        public:
            enum Status : int8_t
            {
                STATUS_NO_FIX = -1,
                STATUS_FIX = 0,
                STATUS_SBAS_FIX = 1,
                STATUS_GBAS_FIX = 2
            };

            enum Service : uint16_t
            {
                SERVICE_GPS = 1 << 0,
                SERVICE_GLONASS = 1 << 1,
                SERVICE_COMPASS = 1 << 2,
                SERVICE_GALILEO = 1 << 3
            };

            int8_t status{STATUS_NO_FIX};
            uint16_t service{0};
        };

        // NavSatFix message equivalent
        class NavSatFix
        {
            friend class boost::serialization::access;

            template <class Archive>
            void serialize(Archive& ar, const unsigned int /* version */)
            {
                ar & header;
                ar & status;
                ar & latitude;
                ar & longitude;
                ar & altitude;
                ar & boost::serialization::make_array(position_covariance.data(), position_covariance.size());
                ar & position_covariance_type;
            }

        public:
            Header header;
            NavSatStatus status;
            double latitude{0.0}; // degrees
            double longitude{0.0}; // degrees
            double altitude{0.0}; // meters
            Eigen::Matrix<double, 3, 3> position_covariance = Eigen::Matrix<double, 3, 3>::Zero();
            uint8_t position_covariance_type{0};
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        };
    } // namespace MSG
} // namespace ORB_SLAM3
#endif //GPSTYPES_H
