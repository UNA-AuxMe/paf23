#include "acting_panels/manual_control.hpp"
#include <pluginlib/class_list_macros.hpp>

#include "rviz/visualization_manager.h"
#include "rviz/display_group.h"

#include <thread>
#include <unordered_map>
#include <boost/bind.hpp>
PLUGINLIB_EXPORT_CLASS(manual_control::manualControl, rviz::Panel)

#define STEPSIZE_THROTTLE 0.1
#define STEPSIZE_STEER 0.1

namespace manual_control
{

    manualControl::manualControl(QWidget *parent)
        : rviz::Panel(parent),
          ui_(std::make_shared<Ui::manual_control>())
    {
        // Extend the widget with all attributes and children from UI file
        ui_->setupUi(this);

        this->steer = 0.0;
        this->throttle = 0.0;
        this->override = false;

        connect(ui_->btn_set_override, SIGNAL(clicked()), this, SLOT(btn_override()));

        connect(ui_->btn_left, SIGNAL(clicked()), this, SLOT(btn_left()));
        connect(ui_->btn_right, SIGNAL(clicked()), this, SLOT(btn_right()));
        connect(ui_->btn_center, SIGNAL(clicked()), this, SLOT(btn_center()));
        connect(ui_->btn_reverse, SIGNAL(clicked()), this, SLOT(btn_reverse()));
        connect(ui_->btn_forward, SIGNAL(clicked()), this, SLOT(btn_forward()));
        connect(ui_->btn_stop, SIGNAL(clicked()), this, SLOT(btn_stop()));
        this->steerPublisher = this->nh_.advertise<std_msgs::Float32>("/paf/hero/manual_steer", 10);
        this->throttlePublisher = this->nh_.advertise<std_msgs::Float32>("/paf/hero/manual_throttle", 10);
        this->overridePublisher = this->nh_.advertise<std_msgs::Bool>("/paf/hero/switch_manual_override", 10);
    }

    void manualControl::onInitialize()
    {
        // Through this we can continuously update the size
        connect(vis_manager_, &rviz::VisualizationManager::preUpdate, this, &manualControl::onUpdate);
    }

    void manualControl::onUpdate()
    {
        ui_->layoutWidget->resize(this->width(), this->height());
    }

    void manualControl::btn_override()
    {
        this->override = !this->override;
        auto msg = std_msgs::Bool();
        msg.data = this->override;
        this->overridePublisher.publish(msg);
        if (this->override)
        {
            ui_->override_label->setText(QString("Override is ON"));
        }
        else
        {
            ui_->override_label->setText(QString("Override is OFF"));
        }
    }

    void manualControl::btn_left()
    {
        this->steer -= STEPSIZE_STEER;
        this->sendSteer();
    }

    void manualControl::btn_right()
    {
        this->steer += STEPSIZE_STEER;
        this->sendSteer();
    }

    void manualControl::btn_center()
    {
        this->steer = 0.0;
        this->sendSteer();
    }

    void manualControl::btn_reverse()
    {
        this->throttle -= STEPSIZE_THROTTLE;
        this->sendThrottle();
    }

    void manualControl::btn_forward()
    {
        this->throttle += STEPSIZE_THROTTLE;
        this->sendThrottle();
    }

    void manualControl::btn_stop()
    {
        this->throttle = 0.0;
        this->sendThrottle();
    }

    void manualControl::sendSteer()
    {
        auto msg = std_msgs::Float32();
        msg.data = this->steer;
        this->steerPublisher.publish(msg);
        ui_->steering_label->setText(QString::number(this->steer) + QString(" ")); // TODO: Put unit here
    }
    void manualControl::sendThrottle()
    {
        auto msg = std_msgs::Float32();
        msg.data = this->throttle;
        this->throttlePublisher.publish(msg);
        ui_->throttle_label->setText(QString::number(this->throttle) + QString(" ")); // TODO: Put unit here
    }

    /**
     *  Save all configuration data from this panel to the given
     *  Config object. It is important here that you call save()
     *  on the parent class so the class id and panel name get saved.
     */
    void manualControl::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    /**
     *  Load all configuration data for this panel from the given Config object.
     */
    void manualControl::load(const rviz::Config &config)
    {
        rviz::Panel::load(config);
    }
} // namespace subswarm_panel