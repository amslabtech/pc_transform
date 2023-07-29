#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

class PcEularTransform{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
		/*subscriber*/
		ros::Subscriber sub_;
		/*publisher*/
		ros::Publisher pub_;
		/*parameter*/
		std::string publish_frame_;
		double x_m_, y_m_, z_m_;
		double x_deg_, y_deg_, z_deg_;

	public:
		PcEularTransform();
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		void publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc);
		double degToRad(double deg);
};

PcEularTransform::PcEularTransform()
	: nh_private_("~")
{
	// rpy_transform.cppと同じ
}

void PcEularTransform::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// rpy_transform.cppと同じ
}

void PcEularTransform::transformPC(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
	Eigen::Quaternionf rotation =
		Eigen::AngleAxisf(degToRad(x_deg_), Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(degToRad(y_deg_), Eigen::Vector3f::UnitY())
    		* Eigen::AngleAxisf(degToRad(z_deg_), Eigen::Vector3f::UnitZ());
	Eigen::Vector3f offset(x_m_, y_m_, z_m_);

	pcl::transformPointCloud(*pc, *pc, offset, rotation);
}

void PcEularTransform::publication(pcl::PointCloud<pcl::PointXYZI>::Ptr pc)
{
	// rpy_transform.cppと同じ
}

double PcEularTransform::degToRad(double deg)
{
	// rpy_transform.cppと同じ
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pc_eular_transform");

	PcEularTransform pc_eular_transform;

	ros::spin();
}
