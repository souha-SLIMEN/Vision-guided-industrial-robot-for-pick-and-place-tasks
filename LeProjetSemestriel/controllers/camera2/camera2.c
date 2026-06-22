#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <winsock2.h>

#define TIME_STEP 32

int main() {
  wb_robot_init();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP);
  wb_camera_recognition_enable(camera, TIME_STEP);

  WbDeviceTag emitter = wb_robot_get_device("emitter");
  WbDeviceTag receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, TIME_STEP);

  // --- TCP vers YOLO ---
  WSADATA wsa;
  WSAStartup(MAKEWORD(2,2), &wsa);

  SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);

  struct sockaddr_in server;
  server.sin_addr.s_addr = inet_addr("127.0.0.1");
  server.sin_family = AF_INET;
  server.sin_port = htons(5050);

  if (connect(sock, (struct sockaddr *)&server, sizeof(server)) != 0) {
    printf("Erreur de connexion au serveur YOLO\n");
    return 1;
  }
  printf("Connecté au serveur YOLO\n");

  double last_angle = -999.0;  // sauvegarde de l’angle reçu

  while (wb_robot_step(TIME_STEP) != -1) {

    while (wb_receiver_get_queue_length(receiver) > 0) {

      const char *msg = (const char *)wb_receiver_get_data(receiver);

      if (strcmp(msg, "STOP") == 0) {

        // --- Capture image ---
        int w = wb_camera_get_width(camera);
        int h = wb_camera_get_height(camera);
        const unsigned char *img = wb_camera_get_image(camera);

        int size = w * h * 3;
        unsigned char *buffer = malloc(size);

        for (int y = 0; y < h; y++)
          for (int x = 0; x < w; x++) {
            buffer[(y*w + x)*3 + 0] = wb_camera_image_get_red(img, w, x, y);
            buffer[(y*w + x)*3 + 1] = wb_camera_image_get_green(img, w, x, y);
            buffer[(y*w + x)*3 + 2] = wb_camera_image_get_blue(img, w, x, y);
          }

        int w_net = htonl(w);
        int h_net = htonl(h);
        int size_net = htonl(size);

        send(sock, (char *)&w_net, sizeof(int), 0);
        send(sock, (char *)&h_net, sizeof(int), 0);
        send(sock, (char *)&size_net, sizeof(int), 0);
        send(sock, (char *)buffer, size, 0);

        free(buffer);

        // --- Recevoir ANGLE YOLO ---
        double angle_deg = -999.0;
        int r = recv(sock, (char *)&angle_deg, sizeof(double), 0);
        if (r == sizeof(double)) {
          last_angle = angle_deg;
          printf("Angle YOLO reçu = %.2f°\n", angle_deg);
        }

        // --- Envoi position + angle au bras ---
        int count = wb_camera_recognition_get_number_of_objects(camera);
        const WbCameraRecognitionObject *objects = wb_camera_recognition_get_objects(camera);

        if (count > 0) {

          double angle_rad = last_angle * M_PI / 180.0; // conversion
          printf("Angle en radians = %.3f\n", angle_rad);

          double data[4];
          data[0] = objects[0].position[0];
          data[1] = objects[0].position[1];
          data[2] = objects[0].position[2];
          data[3] = angle_rad;

          wb_emitter_send(emitter, data, sizeof(data));

          printf("ENVOI AU BRAS → x=%.3f  y=%.3f  z=%.3f  angle(rad)=%.3f\n",
                 data[0], data[1], data[2], data[3]);
        }
      }

      wb_receiver_next_packet(receiver);
    }
  }

  closesocket(sock);
  WSACleanup();
  wb_robot_cleanup();
  return 0;
}