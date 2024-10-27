#include <px4_platform_common/module.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <px4_custom_mode.h>
#include <cerrno>

extern "C" __EXPORT int waypoint_test_main(int argc, char *argv[]);

class WaypointTest : public ModuleBase<WaypointTest>
{
public:
    WaypointTest() = default;
    ~WaypointTest() = default;

    static int task_spawn(int argc, char *argv[]);
    static WaypointTest *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    int _local_pos_sub{-1};
    int _vehicle_status_sub{-1};
    orb_advert_t _setpoint_pub{nullptr};
    orb_advert_t _vehicle_command_pub{nullptr};

    void publish_setpoint(float x, float y, float z);
    void set_offboard_mode();
    bool arm_vehicle(bool force = false);
    void takeoff();
};

int WaypointTest::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("waypoint_test",
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

WaypointTest *WaypointTest::instantiate(int argc, char *argv[])
{
    return new WaypointTest();
}

int WaypointTest::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int WaypointTest::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
This module tests waypoint navigation by setting offboard mode and publishing a setpoint.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("waypoint_test", "template");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

void WaypointTest::publish_setpoint(float x, float y, float z)
{
    vehicle_local_position_setpoint_s setpoint{};
    setpoint.x = x;
    setpoint.y = y;
    setpoint.z = z;
    setpoint.timestamp = hrt_absolute_time();

    if (_setpoint_pub == nullptr) {
        _setpoint_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &setpoint);
    } else {
        orb_publish(ORB_ID(vehicle_local_position_setpoint), _setpoint_pub, &setpoint);
    }

    PX4_INFO("Navigating to setpoint: [%f, %f, %f]", (double)x, (double)y, (double)z);
}

void WaypointTest::set_offboard_mode()
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
    cmd.param1 = 1; // custom mode
    cmd.param2 = PX4_CUSTOM_MAIN_MODE_OFFBOARD;
    cmd.param3 = 0; // sub-mode: not used for offboard
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

    PX4_INFO("Set to offboard mode");
}

bool WaypointTest::arm_vehicle(bool force)
{
    vehicle_command_s cmd{};
    cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    cmd.param1 = 1.0; // arm
    cmd.param2 = force ? 21196.0f : 0.0f; // force arming if requested
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

    PX4_INFO("Arming vehicle");

    // Wait for acknowledgment
    vehicle_command_ack_s ack{};
    int ack_sub = orb_subscribe(ORB_ID(vehicle_command_ack));
    bool success = false;
    hrt_abstime start_time = hrt_absolute_time();

    while (hrt_absolute_time() - start_time < 1000000) { // 1 second timeout
        if (orb_copy(ORB_ID(vehicle_command_ack), ack_sub, &ack) == PX4_OK) {
            if (ack.command == vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM && ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
                success = true;
                break;
            }
        }
        usleep(10000); // 10 ms
    }

    orb_unsubscribe(ack_sub);
    return success;
}

void WaypointTest::takeoff()
{
    publish_setpoint(0.0f, 0.0f, -10.0f); // Take off to 10 meters altitude
}

void WaypointTest::run()
{
    _local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
    _vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));

    vehicle_local_position_s local_pos;
    vehicle_status_s vehicle_status;

    bool takeoff_initiated = false;

    while (!should_exit()) {
        if (orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &local_pos) == PX4_OK) {
            PX4_INFO("Current position: [x: %f, y: %f, z: %f]", (double)local_pos.x, (double)local_pos.y, (double)local_pos.z);
        }

        if (orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &vehicle_status) == PX4_OK) {
            if (vehicle_status.arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
                if (arm_vehicle(true)) { // Force arming
                    PX4_INFO("Vehicle armed");
                } else {
                    PX4_ERR("Failed to arm vehicle");
                }
            }

            if (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED && !takeoff_initiated) {
                set_offboard_mode();
                takeoff();
                takeoff_initiated = true;
                PX4_INFO("Takeoff initiated");
            }

            if (takeoff_initiated && local_pos.z <= -9.5f) { // Check if the vehicle has reached the takeoff altitude
                publish_setpoint(10.0f, 10.0f, -10.0f); // Navigate to (10, 10, -10)
                PX4_INFO("Navigating to waypoint");
                takeoff_initiated = false; // Reset the flag to avoid re-initiating takeoff
            }
        }

        usleep(100000); // 100 ms
    }

    orb_unsubscribe(_local_pos_sub);
    orb_unsubscribe(_vehicle_status_sub);
}

int waypoint_test_main(int argc, char *argv[])
{
    return WaypointTest::main(argc, argv);
}
