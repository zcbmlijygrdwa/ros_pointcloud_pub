#include "utility.h"

#include <string>
#include "/home/zhenyu/cpp_util/cpp_util.hpp"


ros::Publisher pubLidarRaw;

void readTimeStamp(std::vector<double>& time_stamp, std::string timestamp_filename)
{
    std::vector<std::string> lines = readLinesFromTxt(timestamp_filename);
    double t = 0.0;
    for(std::string l : lines)
    {
        std::vector<std::string> data = splitString(l, " ");
        //time_stamp.push_back(std::stod(data[1]));
        time_stamp.push_back(t); //fake
        t += 0.1;
    }
}

void readData(pcl::PointCloud<pcl::PointXYZ>& cloud, std::string filename)
{
    printv(filename);

    static const size_t BufferSize = 4*sizeof(float);
    unsigned char buffer[BufferSize];
    FILE *ptr;

    ptr = fopen(filename.c_str(),"rb");  // r for read, b for binary
    const size_t fileSize = fread(buffer, sizeof(unsigned char), BufferSize, ptr);

    printf("File size = %d bytes\n", fileSize);
    printf("Size of each item in bytes = %d\n", sizeof(unsigned char));

int count = 0;
    while(fread(buffer,sizeof(buffer),1,ptr)==1)
    {
        //for(int i = 0; i<4; i++)
        //    printf("%f ", ((float*)buffer)[i]); // prints a series of bytes
        pcl::PointXYZ point;
        point.x = ((float*)buffer)[0];
        point.y = ((float*)buffer)[1];
        point.z = ((float*)buffer)[2];
        cloud.push_back(point);
        count++    ;
    }
    printv(count);
}

int main(int argc, char** argv)
{
    std::vector<double> time_stamp;
    readTimeStamp(time_stamp, "/home/zhenyu/datasets/kitti/2011_09_26_drive_0046_sync/2011_09_26/2011_09_26_drive_0046_sync/velodyne_points/timestamps.txt");
    if(time_stamp.size()<=0)
    {
        std::cout<<"Error : empty time_stamp data.."<<std::endl;
        return 0;
    }


    ros::init(argc, argv, "pc_pub");
    ros::NodeHandle nh;

    int pc_count = 0;

    ROS_INFO("\033[1;32m---->\033[0m Lidar Raw publishing Started.");

    ros::Rate rate(20);
    std::string path = "/home/zhenyu/datasets/kitti/2011_09_26_drive_0046_sync/2011_09_26/2011_09_26_drive_0046_sync/velodyne_points/data/";
    std::string file_name = "";
    std::string postfix = ".bin";
    pubLidarRaw = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    while (ros::ok())
    {
        ros::spinOnce();

        if(pc_count<10)
        {
            file_name = "000000000"+std::to_string(pc_count);
        }
        else if(pc_count<100)
        {
            file_name = "00000000"+std::to_string(pc_count);
        }
        else if(pc_count<1000)
        {
            file_name = "0000000"+std::to_string(pc_count);
        }
        else if(pc_count<10000)
        {
            file_name = "000000"+std::to_string(pc_count);
        }
        else
        {
            file_name = "00000"+std::to_string(pc_count);
        }

        

        if (pubLidarRaw.getNumSubscribers() != 0 && pc_count<time_stamp.size()){
            printv(pc_count);

            pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
            //readData(laserCloudIn, "/home/zhenyu/datasets/kitti/2011_09_26_drive_0046_sync/2011_09_26/2011_09_26_drive_0046_sync/velodyne_points/data/0000000054.bin");
            readData(laserCloudIn, path+file_name+postfix);
            std::cout<<"after read, cloud size = "<<laserCloudIn.size()<<std::endl;
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(laserCloudIn, cloudMsgTemp);
            printv(time_stamp.size());
            double t = time_stamp[pc_count];
            printv(t);
            //cloudMsgTemp.header.stamp = ros::Time().fromSec(t);
            cloudMsgTemp.header.stamp = ros::Time().now();
            cloudMsgTemp.header.frame_id = "velodyne";
            pubLidarRaw.publish(cloudMsgTemp);
            pc_count++;
        }   

        rate.sleep();
    }

    return 0;
}
