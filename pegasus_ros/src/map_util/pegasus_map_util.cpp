#include <boost/ptr_container/ptr_vector.hpp>
#include "ros/ros.h"
#include "geodesy/utm.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "tf/tf.h"

typedef std::vector<std::string> string_vector;


class PegasusMapUtil {
  private:
    struct PegasusDrone {
      ros::Subscriber m_localPoseSub;
      ros::Subscriber m_globalGpsSub;
      ros::Subscriber m_globalCompassSub;
      std::string m_localPoseTopic; // the origin local frames of different drones in GPS
      std::string m_globalGpsTopic; // the gps coordinated of different drones
      std::string m_globalCompassTopic; // the gps coordinated of different drones
      geometry_msgs::PoseStamped m_localPose;
      sensor_msgs::NavSatFix m_navFix; 
      std_msgs::Float64 m_compass;
      geometry_msgs::Quaternion m_orientation;
      geometry_msgs::Pose m_globalPose;
      geometry_msgs::Pose m_globalOriginPose;

      void computeHeading() {
        /* 0 north, 90  east, 180, west, 270, south */
        tfScalar heading = m_compass.data;

        /* convert to ENU yaw 
         * east 0 west 90 south 180 north 270 */
        tfScalar yaw = (heading < 90.0f) ? 270.0f + heading: heading - 90.0f;
        
        tf::Quaternion q;
        /* Set roll pitch yaw */
        q.setRPY(0.0f, 0.0f, yaw);
        tf::quaternionTFToMsg(q, m_orientation);
      }

      void computePose() {
        geodesy::UTMPose utmPose;
        /* convert from wgs 84 to UTM */
        geographic_msgs::GeoPose geoPose = geodesy::toMsg(m_navFix, m_orientation);
        geodesy::fromMsg(geoPose, utmPose);

        m_globalPose = geodesy::toGeometry(utmPose);

      }

      void computeGlobalOrigin() {
        /* compute m_globalOriginPose
         * from m_globalPose <- the navsatfix and heading 
         * m_localPose <- the current pose
         */
        // TODO
      }
    };

    ros::NodeHandle m_handle;
    geometry_msgs::PoseStamped m_mapOrigin;
    ros::Subscriber m_mapOriginSub;
    std::string m_mapOriginTopic; // the origin of the map in GPS

    boost::ptr_vector<PegasusDrone> m_drones;

  public:
    PegasusMapUtil(const std::string& mapOriginTopic,
        const string_vector& localPoseTopics,
        const string_vector& globalGpsTopics,
        const string_vector& globalCompassTopics);

    void MapOriginCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
        PegasusDrone* drone);

    void GlobalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg,
        PegasusDrone* drone);

    void GlobalCompassCallback(const std_msgs::Float64::ConstPtr& msg,
        PegasusDrone* drone);

};

PegasusMapUtil::PegasusMapUtil(const std::string& mapOriginTopic,
    const string_vector& localPoseTopics,
    const string_vector& globalGpsTopics,
    const string_vector& globalCompassTopics) {

  m_mapOriginTopic = mapOriginTopic;

  if (localPoseTopics.size() != globalGpsTopics.size()) {
    ROS_FATAL("Subscription error.");
    exit(1);
  }

  m_mapOriginSub = m_handle.subscribe<geometry_msgs::PoseStamped>
    (m_mapOriginTopic,
     1,
     boost::bind(&PegasusMapUtil::MapOriginCallback, this, _1)
    );

  for(auto i = 0u; i < globalGpsTopics.size(); i++) {
    auto drone = new PegasusDrone;

    drone->m_localPoseTopic = localPoseTopics[i];

    drone->m_localPoseSub = m_handle.subscribe<geometry_msgs::PoseStamped>
      (
       drone->m_localPoseTopic,
       1,
       boost::bind(&PegasusMapUtil::LocalPoseCallback, this, _1, drone)
      );

    drone->m_globalGpsTopic = globalGpsTopics[i];

    drone->m_globalGpsSub = m_handle.subscribe<sensor_msgs::NavSatFix>
      (drone->m_globalGpsTopic,
       1,
       boost::bind(&PegasusMapUtil::GlobalGpsCallback, this, _1, drone)
      );

    drone->m_globalCompassTopic = globalCompassTopics[i];

    drone->m_globalCompassSub = m_handle.subscribe<std_msgs::Float64>
      (drone->m_globalCompassTopic,
       1,
       boost::bind(&PegasusMapUtil::GlobalCompassCallback, this, _1, drone)
      );
    ROS_INFO_STREAM("Created drone " << drone);

    m_drones.push_back(drone);
  }
}

void PegasusMapUtil::MapOriginCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  m_mapOriginSub.shutdown();
  m_mapOrigin = *msg;
  ROS_INFO_STREAM("Map origin:" << m_mapOrigin);
}

void PegasusMapUtil::LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
    PegasusDrone* drone) {
  drone->m_localPoseSub.shutdown();
  drone->m_localPose = *msg;
  ROS_INFO_STREAM("[" << drone << "] local pose:" << drone->m_localPose);
}


void PegasusMapUtil::GlobalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg,
    PegasusDrone* drone) {
  if (msg->status.status >= msg->status.STATUS_FIX) {
    drone->m_globalGpsSub.shutdown();
    drone->m_navFix = *msg;
    ROS_INFO_STREAM("[" << drone << "] " << "Got a fix: " << *msg);
  }
}

void PegasusMapUtil::GlobalCompassCallback(const std_msgs::Float64::ConstPtr& msg,
    PegasusDrone* drone) {
  drone->m_globalCompassSub.shutdown();
  drone->m_compass = *msg;
  drone->computeHeading();
  ROS_INFO_STREAM("[" << drone << "] " << "Got heading: " << *msg);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "pegasus_map_util");

  PegasusMapUtil pegasusMapUtil(
      "/local_xy_origin",
      {
      "/uav0/mavros/local_position/pose",
      "/uav1/mavros/local_position/pose",
      "/uav2/mavros/local_position/pose",
      },
      {
      "/uav0/mavros/global_position/global",
      "/uav1/mavros/global_position/global",
      "/uav2/mavros/global_position/global",
      },
      {
      "/uav0/mavros/global_position/compass_hdg",
      "/uav1/mavros/global_position/compass_hdg",
      "/uav2/mavros/global_position/compass_hdg",
      }
      );

  ros::spin();
  return 0;
}
