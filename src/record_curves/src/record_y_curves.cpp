#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
using namespace std;
class DataRecorder
{
public:
    DataRecorder()
    {
        // 初始化节点句柄
        ros::NodeHandle nh;

        // 订阅主题
        subscriber_ = nh.subscribe("/hat_x_topic", 0, &DataRecorder::callback, this);

        // 创建（或覆盖）CSV文件
        file_.open("data.csv", std::ofstream::out | std::ofstream::trunc);
        file_ << "data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11],data[12],data[13],data[14],data[15],data[16],data[17],data[18],data[19],data[20],data[21],data[22],data[23],data[24],data[25],data[26]\n"; // 写入CSV文件的标题行
    }

    ~DataRecorder()
    {
        file_.close();
    }

    void callback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        // cout << msg->data.size() << endl;
        if (msg->data.size() >= 27)
        {
            file_ << msg->data[0] << "," << msg->data[1] << "," << msg->data[2] << ","
                  << msg->data[3] << "," << msg->data[4] << "," << msg->data[5] << "," << msg->data[6] << "," << msg->data[7] << "," << msg->data[8] << "," << msg->data[9] << "," << msg->data[10] << "," << msg->data[11] << ","
                  << msg->data[12] << "," << msg->data[13] << "," << msg->data[14] << "," << msg->data[15] << "," << msg->data[16] << "," << msg->data[17] << "," << msg->data[18] << "," << msg->data[19] << "," << msg->data[20] << ","
                  << msg->data[21] << "," << msg->data[22] << "," << msg->data[23] << "," << msg->data[24] << "," << msg->data[25] << "," << msg->data[26] << "\n";
        }
    }

private:
    ros::Subscriber subscriber_;
    std::ofstream file_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_recorder");
    DataRecorder recorder;
    ros::spin();
    return 0;
}
