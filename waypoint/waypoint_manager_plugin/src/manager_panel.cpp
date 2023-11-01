#include "waypoint_manager_plugin/manager_panel.h"

namespace waypoint_manager_plugin
{
    ManagerPanel::ManagerPanel(QWidget* parent) : Panel(parent), _ui(new Ui::ManagerUI())
    {
        _ui->setupUi(this);
    }

    ManagerPanel::~ManagerPanel() = default;

    void ManagerPanel::onInitialize()
    {
        connect(_ui->init_pose_button, SIGNAL(clicked()), this, SLOT(onInitPoseButtonClicked()));
        connect(_ui->wp_index, SIGNAL(valueChanged(int)), this , SLOT(onWpIndexEdited(int)));

        parentWidget()->setVisible(true);

        _wpManager_set_client = _nh.serviceClient<waypoint_manager_msgs::waypoint_manager_set>("/naviton/waypoint/wpManager/set");
        _nowWp_sub = _nh.subscribe("/naviton/waypoint/wpManager/nowWp", 10, &ManagerPanel::nowWp_cb, this);
        _initialPose_pub =  _nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        _initialPose.pose.covariance[0]=0.25;
        _initialPose.pose.covariance[7]=0.25;
        _initialPose.pose.covariance[14]=0.25/2.0;
        _initialPose.pose.covariance[21]=0.06853891945200942/4.0;
        _initialPose.pose.covariance[28]=0.06853891945200942/4.0;
        _initialPose.pose.covariance[35]=0.06853891945200942/4.0;
    }

    void ManagerPanel::onEnable()
    {
        show();
        parentWidget()->show();
    }

    void ManagerPanel::onDisable()
    {
        hide();
        parentWidget()->hide();
    }

    void ManagerPanel::onInitPoseButtonClicked()
    {
        _initialPose_pub.publish(_initialPose);
    }

    void ManagerPanel::onWpIndexEdited(int wp_index)
    {
        if(_nowWpIndex == wp_index) return;
        waypoint_manager_msgs::waypoint_manager_set srv;
        srv.request.index = wp_index;
        _wpManager_set_client.call(srv);
    }

    void ManagerPanel::nowWp_cb(waypoint_msgs::waypointConstPtr msg)
    {
        if(msg->index == _nowWpIndex) return;
        
        _nowWpIndex = msg->index;
        _ui->wp_index->setValue(_nowWpIndex);

        _initialPose.header = msg->pose.header;
        _initialPose.pose.pose = msg->pose.pose;
    }
}

PLUGINLIB_EXPORT_CLASS(waypoint_manager_plugin::ManagerPanel, rviz::Panel )
