#include <stdlib.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define STOP_DURATION 2.0
#define THRESHOLD3 800.0

int main(int argc, char *argv[]) {
    wb_robot_init();

    assert(argc == 3);

    const int timestep = (int)wb_robot_get_basic_time_step();

    double speed;
    sscanf(argv[1], "%lf", &speed);

    double timer;
    sscanf(argv[2], "%lf", &timer);

    WbDeviceTag belt_motor = wb_robot_get_device("belt_motor");
    wb_motor_set_position(belt_motor, INFINITY);
    wb_motor_set_velocity(belt_motor, speed);

    WbDeviceTag ds1 = wb_robot_get_device("ds1");
    WbDeviceTag ds2 = wb_robot_get_device("ds2");
    WbDeviceTag ds3 = wb_robot_get_device("ds3");
    wb_distance_sensor_enable(ds1, timestep);
    wb_distance_sensor_enable(ds2, timestep);
    wb_distance_sensor_enable(ds3, timestep);

    WbDeviceTag emitter = wb_robot_get_device("emitter");
    WbDeviceTag arm_emitter = wb_robot_get_device("arm_emitter");

    WbDeviceTag receiver = wb_robot_get_device("receiver");
    wb_receiver_enable(receiver, timestep);

    WbDeviceTag arm_receiver = wb_robot_get_device("arm_receiver");
    wb_receiver_enable(arm_receiver, timestep);

    const double THRESHOLD = 800.0;
    int flag = 0;
    double stop_start_time = -1.0;
    int go_down_sent = 0;

    while (wb_robot_step(timestep) != -1) {
        double distance1 = wb_distance_sensor_get_value(ds1);
        double distance2 = wb_distance_sensor_get_value(ds2);
        double distance3 = wb_distance_sensor_get_value(ds3);
        double current_time = wb_robot_get_time();

        // Réception du message du bras
        if (wb_receiver_get_queue_length(arm_receiver) > 0) {
            const char* msg = (const char*)wb_receiver_get_data(arm_receiver);
            printf("[Convoyeur] Message reçu du bras : %s\n", msg);
            if (strcmp(msg, "START_CONV") == 0) {
                wb_motor_set_velocity(belt_motor, speed);
                printf("[Convoyeur] Convoyeur relancé après relâchement.\n");
            }
            wb_receiver_next_packet(arm_receiver);
        }

        // Réception d'autres messages (caméra)
        if (wb_receiver_get_queue_length(receiver) > 0) {
            const char* msg = (const char*)wb_receiver_get_data(receiver);
            printf("[Convoyeur] Message reçu : %s\n", msg);
            wb_receiver_next_packet(receiver);
        }

        // Logique d'arrêt et reprise
        if (distance1 < THRESHOLD && flag == 0) {
            wb_motor_set_velocity(belt_motor, 0.0);
            stop_start_time = current_time;
            flag = 1;
            const char* stop_msg = "STOP";
            wb_emitter_send(emitter, stop_msg, strlen(stop_msg) + 1);
            printf("[Convoyeur] STOP envoyé à la caméra.\n");
        }

        if (flag == 1 && current_time - stop_start_time >= STOP_DURATION) {
            wb_motor_set_velocity(belt_motor, speed);
        }

        if (flag == 1 && distance2 < THRESHOLD && flag == 1) {
            flag = 0;
            stop_start_time = -1.0;
        }

        if (!go_down_sent && distance3 < THRESHOLD3) {
            wb_motor_set_velocity(belt_motor, 0.0);
            int go_down_signal = 1;
            wb_emitter_send(arm_emitter, &go_down_signal, sizeof(int));
            printf("[Convoyeur] Signal GO_DOWN (entier) envoyé au bras.\n");
            go_down_sent = 1;
        }

        // Réinitialisation du flag go_down_sent après reprise du convoyeur
        if (go_down_sent && wb_motor_get_velocity(belt_motor) == speed) {
            go_down_sent = 0;
        }

        if (timer > 0 && current_time >= timer) {
            wb_motor_set_velocity(belt_motor, 0.0);
            wb_robot_step(timestep);
            break;
        }
    }

    wb_robot_cleanup();
    return EXIT_SUCCESS;
}
