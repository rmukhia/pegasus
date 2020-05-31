#include <boost/ptr_container/ptr_vector.hpp>
#include "ros/ros.h"
#include "geodesy/utm.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Quaternion.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Float64.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

typedef std::vector<std::string> string_vector;


class PegasusMapUtil {
  private:
    struct PegasusDrone {
      std::string m_localPoseTopic; // the origin local frames of different drones in GPS
      std::string m_globalGpsTopic; // the gps coordinated of different drones
      std::string m_globalCompassTopic; // the gps coordinated of different drones
      std::string m_childFrameId; // the local frame id
      ros::Subscriber m_localPoseSub;
      ros::Subscriber m_globalGpsSub;
      ros::Subscriber m_globalCompassSub;
      std_msgs::Float64 m_compass;
      geometry_msgs::Pose m_localPose;
      sensor_msgs::NavSatFix m_navFix;
      geometry_msgs::Quaternion m_orientation;
      geometry_msgs::Pose m_globalPoseUTM;
      geometry_msgs::Pose m_globalOriginPoseUTM;

      void ComputeHeading() {
        /* 0 north, 90  east, 180, west, 270, south */
        tf2Scalar heading = m_compass.data;

        /* convert to ENU yaw
         * east 0 west 90 south 180 north 270 */
        tf2Scalar yaw = (heading < 90.0f) ? 270.0f + heading: heading - 90.0f;
        yaw = yaw * M_PI/ 180.0f;

        ROS_INFO_STREAM("heading w.r.t. east " << yaw);

        tf2::Quaternion q;
        /* Set roll pitch yaw */
        q.setRPY(0.0f, 0.0f, yaw);

        //tf2::convert<tf2::Quaternion, geometry_msgs::Quaternion>(q, m_orientation);

        m_orientation = tf2::toMsg(q);
        //tf::quaternionTFToMsg(q, m_orientation);
      }

      void ComputePose() {
        geodesy::UTMPose utmPose;
        /* convert from wgs 84 to UTM */
        geographic_msgs::GeoPose geoPose = geodesy::toMsg(m_navFix, m_orientation);
        geodesy::fromMsg(geoPose, utmPose);

        m_globalPoseUTM = geodesy::toGeometry(utmPose);

        ROS_INFO_STREAM("[Local Position]" << m_localPose);
        ROS_INFO_STREAM("[Global Map Position]" << m_globalPoseUTM);

      }

      void ComputeGlobalOrigin() {
        /* compute m_globalOriginPoseUTM
         * from m_globalPoseUTM <- the navsatfix and heading
         * m_localPose <- the current pose */

        /* apply the local offset, get the transform to the local origin */

        m_globalOriginPoseUTM.position.x = m_globalPoseUTM.position.x - m_localPose.position.x;
        m_globalOriginPoseUTM.position.y = m_globalPoseUTM.position.y - m_localPose.position.y;
        m_globalOriginPoseUTM.position.z = m_globalPoseUTM.position.z - m_localPose.position.z;

        // we are only interested in yaw
        tf2Scalar roll, pitch, lyaw, gyaw;

        tf2::Matrix3x3 mLocal(tf2::Quaternion(
            m_localPose.orientation.x,
            m_localPose.orientation.y,
            m_localPose.orientation.z,
            m_localPose.orientation.w
        ));

        tf2::Matrix3x3 mGlobal(tf2::Quaternion(
            m_globalPoseUTM.orientation.x,
            m_globalPoseUTM.orientation.y,
            m_globalPoseUTM.orientation.z,
            m_globalPoseUTM.orientation.w
        ));

        mLocal.getRPY(roll,pitch, lyaw);
        mGlobal.getRPY(roll, pitch, gyaw);

        tf2Scalar originYaw = gyaw - lyaw;

        tf2::Quaternion originQ;
        /* Set roll pitch yaw */
        originQ.setRPY(0.0f, 0.0f, originYaw);

        tf2::convert(originQ, m_globalOriginPoseUTM.orientation);
        ROS_INFO_STREAM("[Local Map Origin]" << m_globalOriginPoseUTM);
      }


      void BroadcastTransform(tf2_ros::TransformBroadcaster br, geometry_msgs::Pose mapOriginUTM) {
        tf2::Transform mapT, childMapT;

        tf2::convert(mapOriginUTM, mapT);
        tf2::convert(m_globalOriginPoseUTM, childMapT);

        tf2::Transform transform = mapT.inverseTimes(childMapT);

        geometry_msgs::TransformStamped transformStamped;

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = m_childFrameId;

        tf2::convert(transform, transformStamped.transform);

        br.sendTransform(transformStamped);
      }
    };

    tf2_ros::TransformBroadcaster m_br;
    ros::NodeHandle m_handle;
    geometry_msgs::Pose m_mapOrigin;
    geometry_msgs::Pose m_mapOriginUTM;
    ros::Subscriber m_mapOriginSub;
    std::string m_mapOriginTopic; // the origin of the map in GPS

    boost::ptr_vector<PegasusDrone> m_drones;

  public:
    PegasusMapUtil(const std::string& mapOriginTopic,
        const string_vector& localPoseTopics,
        const string_vector& globalGpsTopics,
        const string_vector& globalCompassTopics,
        const string_vector& childFrameIds);

    void Compute();

    void BroadcastTransforms();

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
    const string_vector& globalCompassTopics,
    const string_vector& childFrameIds) {

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


    drone->m_childFrameId = childFrameIds[i];

    m_drones.push_back(drone);


  }
}

void PegasusMapUtil::Compute() {
  ROS_INFO_STREAM("Started computing....");

  geodesy::UTMPose utmPose;
  geographic_msgs::GeoPose geoPose;
  geoPose.position.latitude = m_mapOrigin.position.y;
  geoPose.position.longitude = m_mapOrigin.position.x;
  geoPose.position.altitude = m_mapOrigin.position.z;
  geoPose.orientation = m_mapOrigin.orientation;

  geodesy::fromMsg(geoPose, utmPose);

  m_mapOriginUTM = geodesy::toGeometry(utmPose);
  ROS_INFO_STREAM("[Global Map Origin]" << m_mapOriginUTM);

  for(auto& drone: m_drones) {
    drone.ComputeHeading();
    drone.ComputePose();
    drone.ComputeGlobalOrigin();
  }
}

void PegasusMapUtil::BroadcastTransforms() {
  ROS_INFO_STREAM("Broadcasting transformations....");
  for(auto& drone: m_drones) {
    drone.BroadcastTransform(m_br, m_mapOriginUTM);
  }
}

void PegasusMapUtil::MapOriginCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  m_mapOriginSub.shutdown();
  m_mapOrigin = msg->pose;
}

void PegasusMapUtil::LocalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg,
    PegasusDrone* drone) {
  drone->m_localPoseSub.shutdown();
  drone->m_localPose = msg->pose;
}


void PegasusMapUtil::GlobalGpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg,
    PegasusDrone* drone) {
  if (msg->status.status >= msg->status.STATUS_FIX) {
    drone->m_globalGpsSub.shutdown();
    drone->m_navFix = *msg;
  }
}

void PegasusMapUtil::GlobalCompassCallback(const std_msgs::Float64::ConstPtr& msg,
    PegasusDrone* drone) {
  drone->m_globalCompassSub.shutdown();
  drone->m_compass = *msg;
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
      },
      {
      "map_0",
      "map_1",
      "map_2",
      }
      );

  ros::Rate rate(10);
  /* f = 1 / time_period
   * 1/10 per sec
   * 5 / (1/10) iterations for 5 seconds
   * 50 iterations
   */

  auto ite = 0u;

  while (ros::ok()) {
    ros::spinOnce();

    if (ite == 50) {
      pegasusMapUtil.Compute();
    }
    else if (ite > 50) {
      pegasusMapUtil.BroadcastTransforms();
      rate = ros::Rate(0.25);
    }

    ite++;

    rate.sleep();
  }

  return 0;
}
