#include "nvt_plugin/core_panel.h"

namespace nvt_plugin
{
    CorePanel::CorePanel(QWidget* parent) : Panel(parent), _ui(new Ui::CoreUI())
    {
        _ui->setupUi(this);
    }

    CorePanel::~CorePanel() = default;

    void CorePanel::onInitialize()
    {
        connect(_ui->start_button, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(_ui->pause_button, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));

        parentWidget()->setVisible(true);

        _nvt_start_client = _nh.serviceClient<std_srvs::Empty>("/naviton/core/start");
        _nvt_pause_client = _nh.serviceClient<std_srvs::Empty>("/naviton/core/pause");
        _nowWp_sub = _nh.subscribe("/naviton/waypoint/wpManager/nowWp", 10, &CorePanel::nowWp_cb, this);
    }

    void CorePanel::onEnable()
    {
        show();
        parentWidget()->show();
    }

    void CorePanel::onDisable()
    {
        hide();
        parentWidget()->hide();
    }

    void CorePanel::onStartButtonClicked()
    {
        std_srvs::Empty srv;
        _nvt_start_client.call(srv);
    }

    void CorePanel::onPauseButtonClicked()
    {
        std_srvs::Empty srv;
        _nvt_pause_client.call(srv);
    }

    void CorePanel::nowWp_cb(waypoint_msgs::waypointConstPtr msg)
    {
        _ui->wp_index->display((int)msg->index);
    }
}

PLUGINLIB_EXPORT_CLASS(nvt_plugin::CorePanel, rviz::Panel )
