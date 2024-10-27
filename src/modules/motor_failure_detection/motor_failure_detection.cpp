// #include <px4_platform_common/module.h>
// #include <px4_platform_common/log.h>
// #include <drivers/drv_hrt.h>
// #include <uORB/uORB.h>
// #include <uORB/topics/vehicle_local_position.h>
// #include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/vehicle_status.h>
// #include <cerrno>

// extern "C" __EXPORT int motor_failure_detection_main(int argc, char *argv[]);

// class MotorFailureDetection : public ModuleBase<MotorFailureDetection>
// {
// public:
//     MotorFailureDetection() = default;
//     ~MotorFailureDetection() = default;

//     static int task_spawn(int argc, char *argv[]);
//     static MotorFailureDetection *instantiate(int argc, char *argv[]);
//     static int custom_command(int argc, char *argv[]);
//     static int print_usage(const char *reason = nullptr);

//     void run() override;

// private:
//     int _local_pos_sub{-1};
//     int _sensor_combined_sub{-1};
// };

// int MotorFailureDetection::task_spawn(int argc, char *argv[])
// {
//     _task_id = px4_task_spawn_cmd("motor_failure_detection",
//                                   SCHED_DEFAULT,
//                                   SCHED_PRIORITY_DEFAULT,
//                                   2000,
//                                   (px4_main_t)&run_trampoline,
//                                   (char *const *)argv);

//     if (_task_id < 0) {
//         PX4_ERR("task start failed");
//         return -errno;
//     }

//     return PX4_OK;
// }

// MotorFailureDetection *MotorFailureDetection::instantiate(int argc, char *argv[])
// {
//     return new MotorFailureDetection();
// }

// int MotorFailureDetection::custom_command(int argc, char *argv[])
// {
//     return print_usage("unknown command");
// }

// int MotorFailureDetection::print_usage(const char *reason)
// {
//     if (reason) {
//         PX4_WARN("%s\n", reason);
//     }

//     PRINT_MODULE_DESCRIPTION(
//         R"DESCR_STR(
// ### Description
// This module detects motor failure using altitude data.
// )DESCR_STR");

//     PRINT_MODULE_USAGE_NAME("motor_failure_detection", "template");
//     PRINT_MODULE_USAGE_COMMAND("start");
//     PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

//     return 0;
// }

// void MotorFailureDetection::run()
// {
//     _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
//     _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

//     vehicle_local_position_s local_pos;
//     sensor_combined_s sensor_data;

//     while (!should_exit()) {
//         if (orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos) == PX4_OK) {
//             const float ALTITUDE_THRESHOLD = 1.0f; // Example threshold value
//             if (local_pos.z > ALTITUDE_THRESHOLD) {
//                 PX4_INFO("Failure detection kicked in");
//             } else {
//                 PX4_INFO("No failure detected");
//             }
//             PX4_INFO("Altitude: %f", (double)local_pos.z);
//         }

//         if (orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_data) == PX4_OK) {
//             PX4_INFO("Accelerometer: [%f, %f, %f]", (double)sensor_data.accelerometer_m_s2[0], (double)sensor_data.accelerometer_m_s2[1], (double)sensor_data.accelerometer_m_s2[2]);
//             PX4_INFO("Gyroscope: [%f, %f, %f]", (double)sensor_data.gyro_rad[0], (double)sensor_data.gyro_rad[1], (double)sensor_data.gyro_rad[2]);
//         //     PX4_INFO("Barometer: %f", (double)sensor_data.baro_alt_meter);
//         }

//         usleep(100000); // 100 ms
//     }

//     orb_unsubscribe(_local_pos_sub);
//     orb_unsubscribe(_sensor_combined_sub);
// }

// int motor_failure_detection_main(int argc, char *argv[])
// {
//     return MotorFailureDetection::main(argc, argv);
// }


// #include <px4_platform_common/module.h>
// #include <px4_platform_common/log.h>
// #include <drivers/drv_hrt.h>
// #include <uORB/uORB.h>
// #include <uORB/topics/vehicle_local_position.h>
// #include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/vehicle_status.h>
// #include <uORB/topics/vehicle_local_position_setpoint.h>
// #include <uORB/topics/vehicle_command.h>
// // #include <px4_platform_common/px4_custom_mode.h>
// #include <cerrno>

// extern "C" __EXPORT int motor_failure_detection_main(int argc, char *argv[]);

// class MotorFailureDetection : public ModuleBase<MotorFailureDetection>
// {
// public:
//     MotorFailureDetection() = default;
//     ~MotorFailureDetection() = default;

//     static int task_spawn(int argc, char *argv[]);
//     static MotorFailureDetection *instantiate(int argc, char *argv[]);
//     static int custom_command(int argc, char *argv[]);
//     static int print_usage(const char *reason = nullptr);

//     void run() override;

// private:
//     int _local_pos_sub{-1};
//     int _sensor_combined_sub{-1};
//     orb_advert_t _setpoint_pub{nullptr};
//     orb_advert_t _vehicle_command_pub{nullptr};

//     void publish_setpoint(float x, float y, float z);
//     void set_offboard_mode();
// };

// int MotorFailureDetection::task_spawn(int argc, char *argv[])
// {
//     _task_id = px4_task_spawn_cmd("motor_failure_detection",
//                                   SCHED_DEFAULT,
//                                   SCHED_PRIORITY_DEFAULT,
//                                   2000,
//                                   (px4_main_t)&run_trampoline,
//                                   (char *const *)argv);

//     if (_task_id < 0) {
//         PX4_ERR("task start failed");
//         return -errno;
//     }

//     return PX4_OK;
// }

// MotorFailureDetection *MotorFailureDetection::instantiate(int argc, char *argv[])
// {
//     return new MotorFailureDetection();
// }

// int MotorFailureDetection::custom_command(int argc, char *argv[])
// {
//     return print_usage("unknown command");
// }

// int MotorFailureDetection::print_usage(const char *reason)
// {
//     if (reason) {
//         PX4_WARN("%s\n", reason);
//     }

//     PRINT_MODULE_DESCRIPTION(
//         R"DESCR_STR(
// ### Description
// This module detects motor failure using altitude data and navigates to a safe waypoint.
// )DESCR_STR");

//     PRINT_MODULE_USAGE_NAME("motor_failure_detection", "template");
//     PRINT_MODULE_USAGE_COMMAND("start");
//     PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

//     return 0;
// }

// void MotorFailureDetection::publish_setpoint(float x, float y, float z)
// {
//     vehicle_local_position_setpoint_s setpoint{};
//     setpoint.x = x;
//     setpoint.y = y;
//     setpoint.z = z;
//     setpoint.timestamp = hrt_absolute_time();

//     if (_setpoint_pub == nullptr) {
//         _setpoint_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &setpoint);
//     } else {
//         orb_publish(ORB_ID(vehicle_local_position_setpoint), _setpoint_pub, &setpoint);
//     }

//     PX4_INFO("Navigating to setpoint: [%f, %f, %f]", (double)x, (double)y, (double)z);
// }

// void MotorFailureDetection::set_offboard_mode()
// {
//     vehicle_command_s cmd{};
//     cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
//     cmd.param1 = 1; // custom mode
//     cmd.param2 = 1; // main mode: offboard
//     cmd.param3 = 0; // sub-mode: not used for offboard
//     cmd.target_system = 1;
//     cmd.target_component = 1;
//     cmd.source_system = 1;
//     cmd.source_component = 1;
//     cmd.timestamp = hrt_absolute_time();

//     if (_vehicle_command_pub == nullptr) {
//         _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
//     } else {
//         orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
//     }

//     PX4_INFO("Set to offboard mode");
// }

// void MotorFailureDetection::run()
// {
//     _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
//     _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));

//     vehicle_local_position_s local_pos;
//     sensor_combined_s sensor_data;

//     while (!should_exit()) {
//         if (orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos) == PX4_OK) {
//             const float ALTITUDE_THRESHOLD = -5.0f; // Example threshold value
//             if (local_pos.z > ALTITUDE_THRESHOLD) {
//                 PX4_INFO("Fucking failure detection kicked in!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
//                 set_offboard_mode();
//                 publish_setpoint(100.0f, 0.0f, -10.0f); // Navigate to (0, 0, 10)
//             } else {
//                 PX4_INFO("No failure detected");
//             }
//             PX4_INFO("Altitude: %f", (double)local_pos.z);
//         }

//         if (orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &sensor_data) == PX4_OK) {
//             PX4_INFO("Accelerometer: [%f, %f, %f]", (double)sensor_data.accelerometer_m_s2[0], (double)sensor_data.accelerometer_m_s2[1], (double)sensor_data.accelerometer_m_s2[2]);
//             PX4_INFO("Gyroscope: [%f, %f, %f]", (double)sensor_data.gyro_rad[0], (double)sensor_data.gyro_rad[1], (double)sensor_data.gyro_rad[2]);
//         }

//         usleep(100000); // 100 ms
//     }

//     orb_unsubscribe(_local_pos_sub);
//     orb_unsubscribe(_sensor_combined_sub);
// }

// int motor_failure_detection_main(int argc, char *argv[])
// {
//     return MotorFailureDetection::main(argc, argv);
// }


#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <cerrno>
#include <px4_custom_mode.h>

extern "C" __EXPORT int motor_failure_detection_main(int argc, char *argv[]);

class MotorFailureDetection : public ModuleBase<MotorFailureDetection>
{
public:
    MotorFailureDetection() = default;
    ~MotorFailureDetection() = default;

    static int task_spawn(int argc, char *argv[]);
    static MotorFailureDetection *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    int _local_pos_sub{-1};
    int _sensor_combined_sub{-1};
    int _vehicle_status_sub{-1};
    int _command_ack_sub{-1};
    orb_advert_t _setpoint_pub{nullptr};
    orb_advert_t _vehicle_command_pub{nullptr};

    bool _offboard_mode_enabled{false};
    bool _failure_detected{false};
    hrt_abstime _last_setpoint_publish{0};

    void publish_setpoint(float x, float y, float z, float vx = 0.0f, float vy = 0.0f, float vz = 0.0f);
    bool set_offboard_mode();
    bool check_mode_switch();
};

int MotorFailureDetection::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("motor_failure_detection",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_DEFAULT,
                                  2000,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        PX4_ERR("task start failed");
        return -errno;
    }

    return PX4_OK;
}

MotorFailureDetection *MotorFailureDetection::instantiate(int argc, char *argv[])
{
    return new MotorFailureDetection();
}

int MotorFailureDetection::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int MotorFailureDetection::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module detects motor failure using altitude data and navigates to a safe waypoint.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("motor_failure_detection", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void MotorFailureDetection::publish_setpoint(float x, float y, float z, float vx, float vy, float vz)
{
    vehicle_local_position_setpoint_s setpoint{};
    setpoint.x = x;
    setpoint.y = y;
    setpoint.z = z;
    setpoint.vx = vx;
    setpoint.vy = vy;
    setpoint.vz = vz;
    setpoint.timestamp = hrt_absolute_time();

    if (_setpoint_pub == nullptr) {
        _setpoint_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &setpoint);
    } else {
        orb_publish(ORB_ID(vehicle_local_position_setpoint), _setpoint_pub, &setpoint);
    }

    _last_setpoint_publish = setpoint.timestamp;
    PX4_INFO("Published setpoint: pos[%f, %f, %f], vel[%f, %f, %f]",
             (double)x, (double)y, (double)z,
             (double)vx, (double)vy, (double)vz);
}

bool MotorFailureDetection::set_offboard_mode()
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1; // custom mode
    cmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
    cmd.param3 = 0;
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.timestamp = hrt_absolute_time();

    if (_vehicle_command_pub == nullptr) {
        _vehicle_command_pub = orb_advertise(ORB_ID(vehicle_command), &cmd);
    } else {
        orb_publish(ORB_ID(vehicle_command), _vehicle_command_pub, &cmd);
    }

    return check_mode_switch();
}

bool MotorFailureDetection::check_mode_switch()
{
    // Wait for command acknowledgment
    vehicle_command_ack_s ack;
    bool ack_received = false;
    hrt_abstime start_time = hrt_absolute_time();

    while (hrt_elapsed_time(&start_time) < 1000000) { // Wait up to 1 second
        if (orb_copy(ORB_ID(vehicle_command_ack), _command_ack_sub, &ack) == PX4_OK) {
            if (ack.command == vehicle_command_s::VEHICLE_CMD_DO_SET_MODE) {
                ack_received = true;
                break;
            }
        }
        px4_usleep(10000);
    }

    if (!ack_received) {
        PX4_ERR("Mode switch acknowledgment not received");
        return false;
    }

    if (ack.result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
        PX4_ERR("Mode switch denied");
        return false;
    }

    PX4_INFO("Successfully switched to offboard mode");
    return true;
}

void MotorFailureDetection::run()
{
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
    _command_ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));

    vehicle_local_position_s local_pos;
//     sensor_combined_s sensor_data;
    vehicle_status_s vehicle_status;

    const float ALTITUDE_THRESHOLD = -11.0f;
    const float SETPOINT_PUBLISH_PERIOD_US = 100000; // 100ms (10Hz)

    while (!should_exit()) {
        // Check if we need to publish setpoints to maintain offboard mode
        if (_failure_detected &&
            _offboard_mode_enabled &&
            (hrt_elapsed_time(&_last_setpoint_publish) > SETPOINT_PUBLISH_PERIOD_US)) {
            publish_setpoint(100.0f, 100.0f, 0.0f, 2.0f, 2.0f, 0.0f); // Include desired velocities
        }

        // Check vehicle position
        if (orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos) == PX4_OK) {
            if (local_pos.z > ALTITUDE_THRESHOLD ) {
                PX4_INFO("Failure detection triggered at altitude: %f", (double)local_pos.z);
                _failure_detected = true;

                // Try to switch to offboard mode
                if (set_offboard_mode()) {
                    _offboard_mode_enabled = true;
                    // Initial setpoint with both position and velocity
                    publish_setpoint(100.0f, 100.0f, 0.0f, 2.0f, 2.0f, 0.0f);
                }
            }
            PX4_INFO("Current altitude: %f", (double)local_pos.z);
        }

        // Monitor vehicle status
        if (orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status) == PX4_OK) {
            if (_offboard_mode_enabled &&
                vehicle_status.nav_state != vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
                PX4_WARN("Offboard mode lost, attempting to re-enable");
                set_offboard_mode();
            }
        }

        px4_usleep(10000); // 10ms
    }

    orb_unsubscribe(_local_pos_sub);
    orb_unsubscribe(_sensor_combined_sub);
    orb_unsubscribe(_vehicle_status_sub);
    orb_unsubscribe(_command_ack_sub);
}

int motor_failure_detection_main(int argc, char *argv[])
{
    return MotorFailureDetection::main(argc, argv);
}
