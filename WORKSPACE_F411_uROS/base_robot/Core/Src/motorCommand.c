/*
 * MotorCommand.c
 */

#include "motorCommand.h"

static TIM_HandleTypeDef    TimHandle;
static TIM_OC_InitTypeDef   sConfigOC;

RobotState robot_state = STOPPED;
//=================================================================
//			PWM INIT
// TIMER 3 (PWM)  : CH1 et CH2
// ENABLE : Sortie Logique (GPIO) PA7
//=================================================================

void motorCommand_Init(void)
{
	unsigned int uwPrescalerValue = 0;

	/* Compute the prescaler value to have TIM4 counter clock equal to 10MHz */
	  uwPrescalerValue = (unsigned int) ((SystemCoreClock / 10000000) - 1);
	  TimHandle.Instance = TIM3;
	  TimHandle.Init.Period = 200 - 1; // 100MHz/200=50kHz
	  TimHandle.Init.Prescaler = uwPrescalerValue;
	  TimHandle.Init.ClockDivision = 0;
	  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;

	  HAL_TIM_Base_Init(&TimHandle);

	  sConfigOC.OCMode = TIM_OCMODE_PWM1;
	  sConfigOC.Pulse = 0x5;// Specifies the pulse value to be loaded into the Capture Compare Register. This parameter can be a number between Min_Data = 0x0000 and Max_Data = 0xFFFF */

	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	 HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_1);
	 HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfigOC, TIM_CHANNEL_2);

	 // CHANGEMENT DU RAPPORT CYCLIQUE
	 __HAL_TIM_SetCompare(&TimHandle, TIM_CHANNEL_1, 100);	// 100 : moteurs au repos
	 __HAL_TIM_SetCompare(&TimHandle, TIM_CHANNEL_2, 100);

	  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_1);	// MOTOR RIGHT
	  HAL_TIM_PWM_Start(&TimHandle, TIM_CHANNEL_2); // MOTOR LEFT

	  // ENABLE MOTEUR (SI INVERSEUR)
	  //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, 0);
	  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 0);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, 0);
}

//=================================================================
//			SET DUTY CYCLE LEFT
//=================================================================
void motorLeft_SetDuty(int duty)
{
	__HAL_TIM_SetCompare(&TimHandle, TIM_CHANNEL_1, duty);
}
//=================================================================
//			SET DUTY CYCLE RIGHT
//=================================================================
void motorRight_SetDuty(int duty)
{
	__HAL_TIM_SetCompare(&TimHandle, TIM_CHANNEL_2, duty);
}
//=================================================================

/* N.B: centrer le rapport cyclique
ce qui place les moteurs au repos si duty = 100 */

void onMoveForward(int index,int consigne) {
	if(index==1){
		motorRight_SetDuty(consigne+100);
	}
	else if(index==2){
		motorLeft_SetDuty(consigne+100);
	}
    robot_state = MOVING_FORWARD;
}

void onMoveBackward(int index,int consigne) {
	if(index==1){
		motorRight_SetDuty(-(consigne+100));
	}
	else if(index==2){
		motorLeft_SetDuty(-(consigne+100));
	}
    robot_state = MOVING_BACKWARD;
}

void onMoveLeft(int index, int consigne) {
    if(index==1){
   		motorRight_SetDuty(consigne+100);
    }
   	else if(index==2){
   		motorLeft_SetDuty(-(consigne+100));
   	}
    robot_state = TURNING_LEFT;
}

void onMoveRight(int index, int consigne) {
    if(index==1){
   		motorRight_SetDuty(-(consigne+100));
   	}
   	else if(index==2){
    	motorLeft_SetDuty(consigne+100);
   	}
    robot_state = TURNING_RIGHT;
}

void stopMoving(int index) {
	if(index==1){
		motorRight_SetDuty(100);
	}
	else if(index==2){
		motorLeft_SetDuty(100);
	}
    robot_state = STOPPED;
}

/**
 * @brief Détecte la présence d'un obstacle à l'avant.
 *
 * @return 0 si un obstacle est détecté à gauche,
 *         1 si un obstacle est détecté à droite,
 *        -1 si aucun obstacle n'est détecté.
 */
int checkFrontObstacle(void) {
    int sensorValues[2];      // Tableau pour stocker les lectures des capteurs IR
    int threshold = 2000;     // Seuil pour détecter un obstacle (en unités du capteur)

    // Récupère les valeurs des capteurs IR avant
    captDistIR_Get(sensorValues);

    // Vérifie les valeurs et détecte les obstacles
    for (int i = 0; i < 2; i++) {
        if (sensorValues[i] > threshold) {
            return i;  // Retourne l'indice du capteur (0 = gauche, 1 = droite)
        }
    }

    // Aucun obstacle détecté
    return -1;
}

/**
 * @brief Détecte la présence d'un obstacle à l'arrière.
 *
 * @return 1 si un obstacle est détecté,
 *         0 si aucun obstacle n'est détecté.
 */
int checkRearObstacle(void) {
    uint16_t range = VL53L0X_readRangeContinuousMillimeters(); // Lecture du capteur de distance arrière
    uint16_t threshold = 100;                                 // Seuil de détection (en mm)

    // Vérifie si la lecture est valide et si un obstacle est détecté
    if (range > 0 && range < threshold) {
        return 1;  // Obstacle détecté
    }

    // Aucun obstacle détecté
    return 0;
}

static int vitess_send = 0; // Variable pour stocker la vitesse calculée
static char last_command = '\0'; // Dernière commande reçue

/**
 * @brief Traite les données de commande à partir du buffer fourni.
 *
 * @param buffer Pointeur vers la chaîne contenant la commande.
 * @return char La dernière commande valide ou '\0' si aucune commande n'est disponible.
 */
char process_command_data(char* buffer) {
    if (buffer != NULL) {
        char first_char = buffer[0];

        // Si le premier caractère est 'v', vérifier la commande précédente
        if (first_char == 'v') {
            return (last_command != '\0') ? last_command : '\0';
        } else {
            last_command = first_char;
            return last_command;
        }
    }

    return '\0';
}

/**
 * @brief Traite les données de vitesse à partir du buffer.
 *
 * @param buffer Pointeur vers la chaîne contenant les données de vitesse.
 * @return int La valeur entière de la vitesse calculée.
 */
int process_vitess_data(char* buffer) {
    if (buffer != NULL) {
        vitess_send = 0; // Réinitialiser la valeur de vitesse

        for (int i = 0; buffer[i] != '\0'; i++) {
            get_vitess(buffer[i]); // Traiter chaque caractère pour calculer la vitesse
        }
    }
    // Retourner la vitesse calculée
    return vitess_send;
}

/**
 * @brief Traite un caractère pour le convertir en une valeur de vitesse.
 *
 * @param c Caractère à analyser (doit être un chiffre).
 */
void get_vitess(char c) {
    // Vérifier si le caractère est un chiffre ('0' à '9')
    if (c >= '0' && c <= '9') {
        // Convertir le caractère en entier et mettre à jour la valeur de vitesse
        vitess_send = vitess_send * 10 + (c - '0');
    }
}
