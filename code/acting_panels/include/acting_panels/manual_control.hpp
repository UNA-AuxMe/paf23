#pragma once

#include <ros/ros.h>
#include <rviz/panel.h>

#include <unordered_map>
#include <QPalette>

/**
 *  Include header generated from ui file
 *  Note that you will need to use add_library function first
 *  in order to generate the header file from ui.
 */
#include <ui_manual_control.h>

// Other ROS dependencies
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <thread>

namespace manual_control
{

    /**
     *  Here we declare our new subclass of rviz::Panel. Every panel which
     *  can be added via the Panels/Add_New_Panel menu is a subclass of
     *  rviz::Panel.
     */

    class manualControl : public rviz::Panel
    {
        /**
         * This class uses Qt slots and is a subclass of QObject, so it needs
         * the Q_OBJECT macro.
         */
        Q_OBJECT

    public:
        /**
         *  QWidget subclass constructors usually take a parent widget
         *  parameter (which usually defaults to 0).  At the same time,
         *  pluginlib::ClassLoader creates instances by calling the default
         *  constructor (with no arguments). Taking the parameter and giving
         *  a default of 0 lets the default constructor work and also lets
         *  someone using the class for something else to pass in a parent
         *  widget as they normally would with Qt.
         */

        manualControl(QWidget *parent = 0);
        void onInitialize() override;
        void onUpdate();

        /**
         *  Now we declare overrides of rviz::Panel functions for saving and
         *  loading data from the config file.  Here the data is the topic name.
         */
        virtual void save(rviz::Config config) const;
        virtual void load(const rviz::Config &config);

        /**
         *  Next come a couple of public Qt Slots.
         */
    public Q_SLOTS:

        /**
         *  Here we declare some internal slots.
         */
    private Q_SLOTS:
        void btn_override();

        void btn_left();
        void btn_right();
        void btn_center();

        void btn_reverse();
        void btn_forward();
        void btn_stop();

        void sendSteer();
        void sendThrottle();

        /**
         *  Finally, we close up with protected member variables
         */
    protected:
        // UI pointer
        std::shared_ptr<Ui::manual_control> ui_;

        double steer;
        double throttle;
        bool override;

        ros::Publisher steerPublisher;
        ros::Publisher throttlePublisher;
        ros::Publisher overridePublisher;
        ros::NodeHandle nh_;
    };
} // namespace