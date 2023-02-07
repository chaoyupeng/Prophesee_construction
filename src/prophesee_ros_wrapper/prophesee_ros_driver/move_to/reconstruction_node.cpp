#include <prophesee_event_msgs/EventArray.h>
#include <ros/ros.h>
//目前只需要这两个包
#include <pure_event_reconstruction.h>
//我们要做的就是把viewer编程一个 event和high pass filter 的连接器

//下面这样写错了
//下面是初始化函数 
//初始化参数可能不需要写到里面去

// PropheseeEventReconsturction::PropheseeEventReconsturction():
//     //不能这样写 原来这是一个class
//     //我们要写成一个main函数
//     //topic 的名字

//     nh_("~"),
//     std::string camera_name("");
//     //这里需要补充
//     const std::string topic_cd_event_buffer="/prophesee/" + camera_name + "/cd_events_buffer";
//     //the above is our topic name
//     //new way of declaring variables

// {//同样创建一个 reconstruction node
//     //ros::init(argc,argv,"reconstruction_node");
//     //ros::NodeHandle nh;
    
//     //pure_event_reconstruction::High_pass_filter high_pass_filter(nh,nh_private);
//     //我们不需要判断是不是bag了
//     VLOG(1)<<"This is running in real-time.";
//     //我们定义的这个类里面的这个节点需要对我们那个topic进行听取
//     //constexpr int EVENT_SUB_QUEUE_SIZE=1000;
//     //ros::Subscriber event_sub=nh.subscribe(topic_cd_event_buffer,EVENT_SUB_QUEUE_SIZE,&pure_event_reconstruction::High_pass_filter::eventsCallback);
//     //what does the last parameter do
//     //first parameter是topicname 第二个参数是queuesize，then callback func
//     constexpr int EVENT_SUB_QUEUE_SIZE=1000;
//     sub_cam_reconstruction=nh_.subscribe(topic_cd_event_buffer,EVENT_SUB_QUEUE_SIZE,&pure_event_reconstruction::High_pass_filter::eventsCallback,&high_pass_filter);
//     //我们只需要写一个eventcallback 然后将内容输入到里面去就好了
//     //这里不需要书写return什么的只需要我们对那个topic进行subscribe
// }

// //下面是销毁函数

// PropheseeEventReconsturction::~PropheseeEventReconsturction(){
//     //我们不需要判断是否初始化窗口 因为我们是不需要的
// }



// int main(int argc,char **argv)
// {
//     //初始化节点
// ros::init(argc,argv,"prophesee_event_reconstruction");

// //这个是对我们定义的类进行实体化
// PropheseeEventReconstruction pr;

// while (ros::ok()&& !pr.isInitialized())
// {
//     //如果初始化好了 那么就开始运作
//     ros::spinOnce();
// }

// while(ros::ok())
// {
//     ros::spinPnce();
//     //原本这里该是showdata 显示event 现在这里需要是我们的reconstruct函数
    
// }

// ros::shutdown();
// return 0;


// }



int main(int argc, char *argv[])
{

 
    //有个参数服务器 这里面的参数所有的节点都可以进行访问
    //跟字典一样 比如“camera_name”:255, 当我请求camera_name的时候 返回的值就是255
    //getParam第一个参数就是字典里面的键值，当我们获取到里面的信息之后，我们就把这个值放入到
    //第二个新定义的变量里面去 然后我们就可以直接利用这个变量了
    //下面这个就是纯粹把东西连接起来

    ros::init(argc,argv,"reconstruction_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    std::string camera_name("");
    nh.getParam("camera_name",camera_name);
    const std::string topic_cd_event_buffer="/prophesee/" + camera_name + "/cd_events_buffer";
    //这样就得到这个topic的名字了
    pure_event_reconstruction::High_pass_filter high_pass_filter(nh,nh_private);
    VLOG(1)<<"Running in real-time mode";
    constexpr int EVENT_SUB_QUEUE_SIZE=1000;
    ros::Subscriber event_sub=nh.subscribe(topic_cd_event_buffer,EVENT_SUB_QUEUE_SIZE,&pure_event_reconstruction::High_pass_filter::eventsCallback,&high_pass_filter);
    ros::spin();
    ros::shutdown();
    return 0;
}