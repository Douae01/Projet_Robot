#include "main.h"
#include "motorCommand.h"
#include "quadEncoder.h"
#include "captDistIR.h"
#include "VL53L0X.h"
#include "groveLCD.h"

#define ARRAY_LEN 100
#define SAMPLING_PERIOD_ms 5
#define FIND_MOTOR_INIT_POS 1

//################################################
#define EX1 1
#define EX2 2
#define EX3 3

#define SYNCHRO_EX EX1
//################################################
//################################################
// PARAMETRE A MODIFIER EN FONCTION DU N° ROBOT
#define ROS_DOMAIN_ID 6
//################################################
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

extern I2C_HandleTypeDef hi2c1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

// Déclaration des objets synchronisants !! Ne pas oublier de les créer
xSemaphoreHandle xSemaphore = NULL;
xQueueHandle qh = NULL;
xQueueHandle qhL = NULL;
xQueueHandle qhR = NULL;

struct AMessage // Exemple de type de message pour queues, on peut mettre ce qu'on veut
{
	char command;
	int data;
};


void SystemClock_Config(void);
void microros_task(void *argument);
uint16_t mes_vl53=0;

int Left_first_index_reached = 0;
int Right_first_index_reached = 0;

//========================================================================
#if SYNCHRO_EX == EX1

// Fonctionnement du moteur en différents modes
int currentMode = 0;
int lastMode = 0;

#define Te 5
#define tau_L 230
#define tau_R 210
#define Ti_L (0.1*tau_L)
#define Ti_R (0.1*tau_R)
#define Ki_L (Te/Ti_L)
#define Ki_R (Te/Ti_R)
#define Kp_L 0.01
#define Kp_R 0.01

int tab_speed[100];
int speed;
int obstacle_detected ;
char command_received ;
int speed_received ;

int speed_L=0; // vitesse roue gauche
// Fonction de contrôle pour la roue gauche
static void control_motorLeft(void *pvParameters)
{
	struct AMessage pxLMessage;
	int consigne=0; // La vitesse à laquelle je souhaite rouler
	int err_L; // erreur : La différence entre ce que je souhaite et ce que j’ai réellement
	float proportionalComponent_L;
	static float integralComponent_L=0.0;
    int commande = 0;
    int start = 0;
    int tmp = 0;

	for (;;)
	{
		// Synchronisation de l’asservissement
		xQueueReceive( qhL,  &( pxLMessage ) , portMAX_DELAY );
		consigne= pxLMessage.data;

		if(pxLMessage.command=='f'){
			onMoveForward(2,consigne);
		}
		else if(pxLMessage.command=='b'){
			onMoveBackward(2,consigne);
		}
		else if(pxLMessage.command=='r'){
			onMoveRight(2,consigne);
		}
		else if(pxLMessage.command=='l'){
			onMoveleft(2,consigne);
		}

		// Vitesse moteur gauche
		speed_L = quadEncoder_GetSpeedL();
		err_L=consigne-speed_L;
		proportionalComponent_L=Kp_L*(float)err_L;
		integralComponent_L=integralComponent_L+Kp_L*Ki_L*(float)err_L;
		commande = (int)(proportionalComponent_L+integralComponent_L);

		if(pxLMessage.command=='s'){
			stopMoving(2);
		}else {
			motorLeft_SetDuty(commande+100);
		}

		// Libère un sémaphore
		xSemaphoreGive( xSemaphore );
	}
}

int speed_R=0; // vitesse roue droite
// Fonction de contrôle pour la roue droite
static void control_motorRight(void *pvParameters)
{
	struct AMessage pxRMessage;
	int consigne=0; // La vitesse à laquelle je souhaite rouler
	int err_R; // erreur : La différence entre ce que je souhaite et ce que j’ai réellement
	float proportionalComponent_R;
	static float integralComponent_R=0.0;
    int commande = 0;
    int start = 0;
    int tmp = 0;

	for (;;)
	{
		// Synchronisation de l’asservissement
		xQueueReceive( qhR,  &( pxRMessage ) , portMAX_DELAY );
		consigne= pxRMessage.data;

		if(pxRMessage.command=='f'){
			onMoveForward(1,consigne);
		}
		else if(pxRMessage.command=='b'){
			onMoveBackward(1,consigne);
		}
		else if(pxRMessage.command=='r'){
			onMoveRight(1,consigne);
		}
		else if(pxRMessage.command=='l'){
			onMoveleft(1,consigne);
		}

		// Vitesse moteur droite
		speed_R = quadEncoder_GetSpeedR();
		err_R=consigne-speed_R;
		proportionalComponent_R=Kp_R*(float)err_R;
		integralComponent_R=integralComponent_R+Kp_R*Ki_R*(float)err_R;
		commande = (int)(proportionalComponent_R+integralComponent_R);

		if(pxRMessage.command=='s'){
			stopMoving(1);
		}else {
			motorRight_SetDuty(commande+100);
		}

		// Libère un sémaphore
		xSemaphoreGive( xSemaphore );
	}
}

// Générateur d'ordres pour le contrôle des moteurs du robot
static void obstacleDetectionTask(void *pvParameters)
{
    struct AMessage pxMessage;
    int frontObstacle , rearObstacle;
    vTaskDelay(5); // Initial delay for system setup

    for (;;)
    {
        // Initialisation des drapeaux d'obstacle
        obstacle_detected = 0;
        frontObstacle = checkFrontObstacle();
        rearObstacle = checkRearObstacle();

        // Mode Manuel
        if (currentMode == 0)
        {
            if (frontObstacle>=0)
            {
                // Obstacle détecté à l'avant
                printf("Mode Manuel:\nObstacle Avant");
                pxMessage.command = 's'; // Stop
                obstacle_detected = 1;
            }
            else if (rearObstacle)
            {
                // Obstacle détecté à l'arrière
                printf("Mode Manuel:\nObstacle Arrière");
                pxMessage.command = 's'; // Stop
                obstacle_detected = 2;
            }
            else
            {
                // Pas d'obstacle, envoyer les commandes utilisateur
                pxMessage.command = command_received;  // Commande reçue ('f', 'b', 'l', 'r')
                pxMessage.data = speed_received;       // Vitesse associée
            }
        }

        // Mode Aléatoire
        else if (currentMode == 1)
        {
            if (frontObstacle>=0)
            {
                // Obstacle détecté à l'avant : éviter
                printf("Mode Aléatoire:\nObstacle détecté à l'avant - ");
                if (frontObstacle == 0)
                {
                    printf("côté gauche.\n");
                    pxMessage.command = 'r'; // Tourner à droite
                    pxMessage.data = 200;   // Vitesse de rotation
                }
                else if (frontObstacle == 1)
                {
                    printf("côté droit.\n");
                    pxMessage.command = 'l'; // Tourner à gauche
                    pxMessage.data = 200;    // Vitesse de rotation
                }
                obstacle_detected = 1;
            }
            else if (rearObstacle)
            {
                // Obstacle détecté à l'arrière : avancer
                printf("Mode Aléatoire:\nObstacle Arrière");
                pxMessage.command = 'f'; // Avancer
                pxMessage.data = 200;    // Vitesse d'avance
                obstacle_detected = 2;
            }
            else
            {
                // Pas d'obstacle : avancer aléatoirement
                pxMessage.command = 'f'; // Avancer
                pxMessage.data = 300;    // Vitesse d'avance
            }
        }

        /* Mode Tracking
        else if (currentMode == 2)
        {
            if (obstacle_detected_forward)
            {
                // Obstacle détecté à l'avant : s'arrêter
                groveLCD_clear();
                groveLCD_term_printf("Mode Tracking:\nObstacle Avant");
                pxMessage.command = 's'; // Stop
            }
            else
            {
                // Suivre la couleur détectée par la caméra
                int trackingCommand = detectAndTrackColor(); // Fonction pour détecter et suivre une couleur
                pxMessage.command = trackingCommand;         // Commande basée sur la couleur détectée
                pxMessage.data = 200;                        // Vitesse associée au tracking
            }
        }*/

        // Commandes pour les moteurs gauche et droit
        xQueueSend(qhL, (void *)&pxMessage, portMAX_DELAY); // Commande moteur gauche
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        xQueueSend(qhR, (void *)&pxMessage, portMAX_DELAY); // Commande moteur droit
        xSemaphoreTake(xSemaphore, portMAX_DELAY);

        // Pause avant la prochaine itération
        vTaskDelay(SAMPLING_PERIOD_ms);
    }
}


static void displayTask(void *pvParameters)
{
	struct AMessage receivedMessage;
	for(;;) {
        if (currentMode != lastMode)
        {
            groveLCD_clear();

            switch (currentMode)
            {
                case 0:
                    groveLCD_term_printf("Mode Manuel");
                    break;
                case 1:
                    groveLCD_term_printf("Mode Aléatoire");
                    break;
                case 2:
                    groveLCD_term_printf("Mode Suivi");
                    break;
                default:
                    groveLCD_term_printf("Mode Inconnu");
                    break;
            }
            lastMode = currentMode;
        }
	}
}

//========================================================
#elif SYNCHRO_EX == EX2

static void task_C( void *pvParameters )
{
	for (;;)
	{
		 printf("TASK C \n\r");
		xSemaphoreTake( xSemaphore, portMAX_DELAY );
	}
}

static void task_D( void *pvParameters )
{
	for (;;)
	{
		 printf("TASK D \n\r");
		xSemaphoreGive( xSemaphore );
	}
}

//========================================================
#elif SYNCHRO_EX == EX3

static void task_E( void *pvParameters )
{
	struct AMessage pxMessage;
	pxMessage.command='a';
	pxMessage.data=10;
	vTaskDelay(1000);
	for (;;)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
	    printf("TASK E \r\n");
		xQueueSend( qh, ( void * ) &pxMessage,  portMAX_DELAY );
		xSemaphoreTake( xSemaphore, portMAX_DELAY );

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 0);
		vTaskDelay(SAMPLING_PERIOD_ms);
	}
}

static void task_F(void *pvParameters)
{
	struct AMessage pxRxedMessage;

	for (;;)
	{
		xQueueReceive( qh,  &( pxRxedMessage ) , portMAX_DELAY );
		 printf("TASK F \r\n");
		xSemaphoreGive( xSemaphore );
	}
}
#endif


//=========================================================================
int main(void)
{
  int ret=0;
  int tab_dist[2];

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  motorCommand_Init();
  quadEncoder_Init();
  captDistIR_Init();

  HAL_Delay(500);

  // Affichage via UART2 sur Terminal série $ minicom -D /dev/ttyACM0
  printf("hello\r\n"); // REM : ne pas oublier le \n

  VL53L0X_init();

  ret = VL53L0X_validateInterface();
  if(ret ==0)
  {
	  printf("VL53L0X OK\r\n");
  }
  else
  {
	  printf("!! PROBLEME VL53L0X !!\r\n");
  }
  VL53L0X_startContinuous(0);

  int a, b;
  groveLCD_begin(16,2,0); // !! cette fonction prend du temps
  HAL_Delay(100);
  groveLCD_display();
  a=5; b=2;
  groveLCD_term_printf("%d+%d=%d",a,b,a+b);
  groveLCD_setCursor(0,0);
  groveLCD_term_printf("hello");


  HAL_Delay(50);

#if FIND_MOTOR_INIT_POS

  int16_t speed=0;
// RECHERCHE DE LA POSITION INITIALE ( 1er signal 'index' du capteur )
// Evite une erreur pour une mesure de vitesse 
	Left_first_index_reached = 0;
	 while( Left_first_index_reached != 1 )
	 {
		motorLeft_SetDuty(130);
		HAL_Delay(SAMPLING_PERIOD_ms);
		speed = quadEncoder_GetSpeedL();
	 }

	Right_first_index_reached = 0;
	 while( Right_first_index_reached != 1 )
	 {
		motorRight_SetDuty(130);
		HAL_Delay(SAMPLING_PERIOD_ms);
		speed = quadEncoder_GetSpeedR();
	 }

	 motorLeft_SetDuty(50);
	 motorRight_SetDuty(50);
	 HAL_Delay(200);

	 speed = quadEncoder_GetSpeedL();
	 speed = quadEncoder_GetSpeedR();
#endif

  osKernelInitialize();

  xTaskCreate( microros_task, ( const portCHAR * ) "microros_task", 3000 /* stack size */, NULL,  25, NULL );
#if SYNCHRO_EX == EX1
	xTaskCreate( control_motorLeft, ( const portCHAR * ) "control motor Left", 128 /* stack size */, NULL, 27, NULL );
	xTaskCreate( control_motorRight, ( const portCHAR * ) "control motor Right", 128 /* stack size */, NULL, 26, NULL );
	xTaskCreate( obstacleDetectionTask, ( signed portCHAR * ) "obstacle Detection Task", 128 /* stack size */, NULL, 28, NULL );
	xTaskCreate( displayTask, ( const portCHAR * ) "task LCD", 128 /* stack size */, NULL, 24, NULL );
#elif SYNCHRO_EX == EX2
	xTaskCreate( task_C, ( signed portCHAR * ) "task C", 128 /* stack size */, NULL, 28, NULL );
	xTaskCreate( task_D, ( signed portCHAR * ) "task D", 128 /* stack size */, NULL, 27, NULL );
#elif SYNCHRO_EX == EX3
	xTaskCreate( task_E, ( signed portCHAR * ) "task E", 128 /* stack size */, NULL, 30, NULL );
	xTaskCreate( task_F, ( signed portCHAR * ) "task F", 128 /* stack size */, NULL, 29, NULL );
#endif

	vSemaphoreCreateBinary(xSemaphore);
	xSemaphoreTake( xSemaphore, portMAX_DELAY );

	qh = xQueueCreate( 1, sizeof(struct AMessage ) );

#if SYNCHRO_EX == EX1
	qhL = xQueueCreate( 1, sizeof(struct AMessage ) ); //queue gauche
	qhR = xQueueCreate( 1, sizeof(struct AMessage ) ); //queue droite
#endif

  osKernelStart();

  while (1)
  {}

}
//=========================================================================
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);


void subscription_callback(const void * msgin)
{
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;

  // Process message
  printf("Received from HOST: %s\n\r", msg->data);
}


void microros_task(void *argument)
{
  rmw_uros_set_custom_transport( true, (void *) &huart1, cubemx_transport_open,  cubemx_transport_close,  cubemx_transport_write, cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app
  rclc_support_t support;
  rcl_allocator_t allocator;
  allocator = rcl_get_default_allocator();

  // create node
  rcl_node_t node;
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  rclc_node_init_default(&node, "STM32_Node","", &support);

  // create publisher
  rcl_publisher_t publisher;
  std_msgs__msg__String sensor_dist_back_msg;
  rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/sensor/dist_back");

  // create subscriber
  rcl_subscription_t subscriber;
  std_msgs__msg__String str_msg;
  rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),"/command/move");
  // Add subscription to the executor
  rclc_executor_t executor;
  rclc_executor_init(&executor, &support.context, 1, &allocator); // ! 'NUMBER OF HANDLES' A MODIFIER EN FONCTION DU NOMBRE DE TOPICS
  rclc_executor_add_subscription(&executor, &subscriber, &str_msg, &subscription_callback, ON_NEW_DATA);

  str_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
  str_msg.data.size = 0;
  str_msg.data.capacity = ARRAY_LEN;

  for(;;)
  {
	  sprintf(str_msg.data.data, "from STM32 : mes_vl53 : #%d", (int32_t)mes_vl53);
	  str_msg.data.size = strlen(str_msg.data.data);

      rcl_ret_t ret = rcl_publish(&publisher, &str_msg, NULL);

		if (ret != RCL_RET_OK)
		{
		  printf("Error publishing (line %d)\n\r", __LINE__);
		}
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    osDelay(10);
  }
}

//=========================================================================
/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
}

//=========================================================================
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {}
}
//=========================================================================
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
