#include <webots/robot.h>
#include <webots/receiver.h>
#include <webots/motor.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <webots/emitter.h>
#define TIME_STEP 32
#define L1 0.613
#define L2 0.637

enum State { WAITING, TRANSLATING, WAITING2, DESCENDING, GRASPING, ASCENDING, TRANSLATING_BACK, RELEASING };

static const double BASE_SHOULDER_PAN  = 0.0;
static const double BASE_SHOULDER_LIFT = -0.785;
static const double BASE_ELBOW         = 0.785;
static const double BASE_WRIST1        = -1.57;
static const double BASE_WRIST2        = -1.57;
static const double BASE_WRIST3        = 0.0;

static const double BASE_CURV = BASE_SHOULDER_LIFT + BASE_ELBOW;

static double Cx0, Cy0;

static double clamp(double v) {
  if (v > 1.0) return 1.0;
  if (v < -1.0) return -1.0;
  return v;
}

static void move_horiz(double X, WbDeviceTag lift, WbDeviceTag elbow,
                       double *theta_out, double *phi_out) {
  double xC = Cx0 - X;
  double yC = Cy0;

  double R = sqrt(xC*xC + yC*yC);
  double gamma = atan2(yC,xC);
  double num = R*R + L1*L1 - L2*L2;
  double den = 2.0*R*L1;
  double beta = acos(clamp(num / den));
  double theta = gamma - beta;
  double Bx = L1*cos(theta);
  double By = L1*sin(theta);
  double psi = atan2(yC - By, xC - Bx);
  double phi = psi - theta;

  wb_motor_set_position(lift, theta);
  wb_motor_set_position(elbow, phi);

  if(theta_out)*theta_out=theta;
  if(phi_out)*phi_out=phi;
}

static void move_vert(double Z, WbDeviceTag lift, WbDeviceTag elbow,
                      double *theta_out, double *phi_out) {
  double xC = Cx0;
  double yC = Cy0 - Z;

  double R = sqrt(xC * xC + yC * yC);
  double gamma = atan2(yC,xC);
  double num = R*R + L1*L1 - L2*L2;
  double den = 2.0 * R * L1;
  double beta = acos(clamp(num / den));
  double theta = gamma - beta;
  double Bx = L1 * cos(theta);
  double By = L1 * sin(theta);
  double psi = atan2(yC - By, xC - Bx);
  double phi = psi - theta;

  wb_motor_set_position(lift, theta);
  wb_motor_set_position(elbow, phi);

  if(theta_out)*theta_out=theta;
  if(phi_out)*phi_out=phi;
}

#define STEPS 40

int main() {
  wb_robot_init();

  // Déclarations nécessaires
  int i, counter = 0;
  int translation_back_counter = 0;
  double translation_back_start_x = 0.0;

  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");

  WbDeviceTag shoulder_pan = wb_robot_get_device("shoulder_pan_joint");
  WbDeviceTag lift = wb_robot_get_device("shoulder_lift_joint");
  WbDeviceTag elbow = wb_robot_get_device("elbow_joint");
  WbDeviceTag wrist1 = wb_robot_get_device("wrist_1_joint");
  WbDeviceTag wrist2 = wb_robot_get_device("wrist_2_joint");
  WbDeviceTag wrist3 = wb_robot_get_device("wrist_3_joint");
  
  WbDeviceTag emitter = wb_robot_get_device("emitter");


  WbDeviceTag cam_receiver = wb_robot_get_device("receiver");  // caméra
  wb_receiver_enable(cam_receiver, TIME_STEP);

  WbDeviceTag conv_receiver = wb_robot_get_device("conv_receiver"); // convoyeur
  wb_receiver_enable(conv_receiver, TIME_STEP);

  if (conv_receiver == 0) {
    printf("[Bras] ERREUR : conv_receiver non trouvé !\n");
  } else {
    printf("[Bras] Récepteur conv_receiver activé.\n");
  }

  // Position initiale
  wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
  wb_motor_set_position(lift, BASE_SHOULDER_LIFT);
  wb_motor_set_position(elbow, BASE_ELBOW);
  wb_motor_set_position(wrist1, BASE_WRIST1);
  wb_motor_set_position(wrist2, BASE_WRIST2);
  wb_motor_set_position(wrist3, BASE_WRIST3);

  for(i=0; i < 20; i++)
    wb_robot_step(TIME_STEP);

  double Bx0 = L1 * cos(BASE_SHOULDER_LIFT);
  double By0 = L1 * sin(BASE_SHOULDER_LIFT);
  Cx0 = Bx0 + L2 * cos(BASE_SHOULDER_LIFT + BASE_ELBOW);
  Cy0 = By0 + L2 * sin(BASE_SHOULDER_LIFT + BASE_ELBOW);

  enum State state = WAITING;
  double object_x = 0.0;
  double object_angle = 0.0;

  while(wb_robot_step(TIME_STEP) != -1) {
    if(wb_receiver_get_queue_length(cam_receiver) > 0) {
      const double *pos = (const double *)wb_receiver_get_data(cam_receiver);
      object_x = pos[1];
      object_angle = pos[3];
      wb_receiver_next_packet(cam_receiver);
      printf("Objet détecté en x = %f\n", object_x);
    }

    if(wb_receiver_get_queue_length(conv_receiver) > 0) {
      int *signal = (int *)wb_receiver_get_data(conv_receiver);
      printf("[Bras] Signal reçu : %d\n", *signal);
      if (*signal == 1) {
        state = DESCENDING;
        printf("[Bras] Lancement de la descente.\n");
      }
      wb_receiver_next_packet(conv_receiver);
    }

    switch(state) {
      case WAITING:{
      if(object_x != 0.0) {
        // Réinitialiser les positions de départ
        double Bx0 = L1 * cos(BASE_SHOULDER_LIFT);
        double By0 = L1 * sin(BASE_SHOULDER_LIFT);
        Cx0 = Bx0 + L2 * cos(BASE_SHOULDER_LIFT + BASE_ELBOW);
        Cy0 = By0 + L2 * sin(BASE_SHOULDER_LIFT + BASE_ELBOW);
    
        state = TRANSLATING;
        printf("Début translation horizontal pour x = %f\n", object_x);
      }
      break;}


      case TRANSLATING: {
        double X_START = 0.0;
        double X_END = -object_x;
        double theta, phi;
        double wrist1_cmd;

        for(int k=0; k<=STEPS; k++) {
          double s = (double)k / STEPS;
          double X = X_START + s * (X_END - X_START);
          move_horiz(X, lift, elbow, &theta, &phi);
          wrist1_cmd = BASE_WRIST1 + BASE_CURV - (theta + phi);
          wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
          wb_motor_set_position(wrist1, wrist1_cmd);
          wb_motor_set_position(wrist2, BASE_WRIST2);
          wb_motor_set_position(wrist3, BASE_WRIST3);
          if(wb_robot_step(TIME_STEP) == -1) break;
        }

        Cx0 = Cx0 - X_END;

        printf("Translation horizontale terminée, bras en attente du signal de descente\n");
        state = WAITING2;
        break;
      }

      case WAITING2:{
        wb_motor_set_position(wrist3, object_angle);
        if(wb_receiver_get_queue_length(conv_receiver) > 0) {
          int *signal = (int *)wb_receiver_get_data(conv_receiver);
          if (*signal == 1) {
            wb_receiver_next_packet(conv_receiver);
            state = DESCENDING;
            printf("[Bras] Signal GO_DOWN reçu, lancement de la descente.\n");
          } else {
            wb_receiver_next_packet(conv_receiver);
          }
        }
        break;}

      case DESCENDING: {
        double Z_START = 0.0;
        double Z_END = -0.37;
        double theta, phi;
        double wrist1_cmd;

        for(int k=0; k<=STEPS; k++) {
          double s = (double)k / STEPS;
          double Z = Z_START + s * (Z_END - Z_START);
          move_vert(Z, lift, elbow, &theta, &phi);
          wrist1_cmd = BASE_WRIST1 + BASE_CURV - (theta + phi);
          wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
          wb_motor_set_position(wrist1, wrist1_cmd);
          wb_motor_set_position(wrist2, BASE_WRIST2);
          /*wb_motor_set_position(wrist3, BASE_WRIST3);*/
          if(wb_robot_step(TIME_STEP) == -1) break;
        }
        state = GRASPING;
        printf("[Bras] Descente terminée, saisie de la pièce\n");
        break;
      }

      case GRASPING: {
        // Fermeture de la pince (ajustez la valeur selon votre modèle)
        counter = 8;
        printf("Grasping can\n");
        for (i = 0; i < 3; ++i)
            wb_motor_set_position(hand_motors[i], 0.7);
        wb_robot_step(TIME_STEP);
        state = ASCENDING;
        printf("[Bras] Pièce saisie, lancement de la remontée\n");
        break;
      }

      case ASCENDING: {
        double Z_START = -0.4;
        double Z_END = 0.0;
        double theta, phi;
        double wrist1_cmd;

        // Sauvegarder la position horizontale actuelle
        double old_Cx0 = Cx0;
        double old_Cy0 = Cy0;

        // Remontée verticale
        for(int k=0; k<=STEPS; ++k) {
          double s = (double)k / STEPS;
          double Z = Z_START + s * (Z_END - Z_START);
          move_vert(Z, lift, elbow, &theta, &phi);
          wrist1_cmd = BASE_WRIST1 + BASE_CURV - (theta + phi);
          wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
          wb_motor_set_position(wrist1, wrist1_cmd);
          wb_motor_set_position(wrist2, BASE_WRIST2);
          /*wb_motor_set_position(wrist3, BASE_WRIST3);*/
          if(wb_robot_step(TIME_STEP) == -1) break;
        }

        // Restaurer les positions horizontales
        Cx0 = old_Cx0;
        Cy0 = old_Cy0;

        // Préparer la translation arrière
        translation_back_counter = STEPS;

        state = TRANSLATING_BACK;
        printf("[Bras] Remontée terminée, début translation arrière\n");
        break;
      }

      case TRANSLATING_BACK: {
        double X_START = -object_x;
        double X_END = 0.0;
        double theta, phi;
        double wrist1_cmd;

        if (translation_back_counter > 0) {
          double s = (double)(STEPS - translation_back_counter) / STEPS;
          double X = X_END - s * (X_END - X_START);
          move_horiz(-X, lift, elbow, &theta, &phi);
          wrist1_cmd = BASE_WRIST1 + BASE_CURV - (theta + phi);
          wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
          wb_motor_set_position(wrist1, wrist1_cmd);
          wb_motor_set_position(wrist2, BASE_WRIST2);
          wb_motor_set_position(wrist3, BASE_WRIST3);
          wb_robot_step(TIME_STEP);
          translation_back_counter--;
        } else {
          Cx0 = X_END;
          object_x = 0;
          printf("[Bras] Retour horizontal terminé, début relâchement\n");
          state = RELEASING;
        }
        break;
      }

          case RELEASING: {
      // Rotation de shoulder_pan de -pi (180°)
      wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN - M_PI);
      wb_robot_step(TIME_STEP * STEPS);
    
      // Ouverture de la pince
      for(int i=0; i<3; ++i) {
        wb_motor_set_position(hand_motors[i], 0.0);
      }
      wb_robot_step(TIME_STEP * 10);
    
      // Rotation retour de shoulder_pan à la position initiale
      wb_motor_set_position(shoulder_pan, BASE_SHOULDER_PAN);
      wb_robot_step(TIME_STEP * STEPS);
    
      printf("[Bras] Pièce relâchée, retour à l'attente\n");
      wb_emitter_send(emitter, "START_CONV", strlen("START_CONV") + 1);
      state = WAITING;
      break;
    }

    }
  }

  wb_robot_cleanup();
  return 0;
}
