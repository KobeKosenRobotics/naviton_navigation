#include "waypoint_manager_plugin/recorder_panel.h"

namespace waypoint_manager_plugin
{
    RecorderPanel::RecorderPanel(QWidget* parent) : Panel(parent), _ui(new Ui::RecorderUI())
    {
        _ui->setupUi(this);
    }

    RecorderPanel::~RecorderPanel() = default;

    void RecorderPanel::onInitialize()
    {
        connect(_ui->start_button, SIGNAL(clicked()), this, SLOT(onStartButtonClicked()));
        connect(_ui->end_button, SIGNAL(clicked()), this, SLOT(onEndButtonClicked()));
        _ui->file_name->setPlaceholderText("FILE NAME");
        connect(_ui->file_name,  SIGNAL(textChanged(const QString &)), this, SLOT(onFileNameEdited()));

        parentWidget()->setVisible(true);

        _wpRecorder_start_client = _nh.serviceClient<waypoint_manager_msgs::waypoint_recorder_start>("/naviton/waypoint/wpRecorder/start");
        _wpRecorder_end_client = _nh.serviceClient<waypoint_manager_msgs::waypoint_recorder_end>("/naviton/waypoint/wpRecorder/end");
    
        _file_name = "";
    }

    void RecorderPanel::onEnable()
    {
        show();
        parentWidget()->show();
    }

    void RecorderPanel::onDisable()
    {
        hide();
        parentWidget()->hide();
    }

    void RecorderPanel::onStartButtonClicked()
    {
        waypoint_manager_msgs::waypoint_recorder_start srv;
        _wpRecorder_start_client.call(srv);
    }

    void RecorderPanel::onEndButtonClicked()
    {
        waypoint_manager_msgs::waypoint_recorder_end srv;
        
        if(_file_name != "")
        {
            srv.request.save = true;
            srv.request.file_name = _file_name;
        }
        else
        {
            srv.request.save = false;
        }

        _wpRecorder_end_client.call(srv);
    }

    void RecorderPanel::onFileNameEdited()
    {
        if(_ui->file_name->text().isEmpty())
            _file_name = "";
        else
            _file_name = _ui->file_name->text().toStdString();
    }
}

PLUGINLIB_EXPORT_CLASS(waypoint_manager_plugin::RecorderPanel, rviz::Panel )
