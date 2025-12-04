# Guide Développeur - STM32H723 DM-MC02

## Table des matières

1. [Présentation de la carte](#1-présentation-de-la-carte)
2. [Configuration système](#2-configuration-système)
3. [IMU BMI088](#3-imu-bmi088)
4. [Ports UART](#4-ports-uart)
5. [Communication RS-485](#5-communication-rs-485)
6. [Bus CAN (FDCAN)](#6-bus-can-fdcan)
7. [Activation des périphériques](#7-activation-des-périphériques)
8. [Exemples de code](#8-exemples-de-code)

---

## 1. Présentation de la carte

### Caractéristiques principales

| Composant | Spécification |
|-----------|---------------|
| **MCU** | STM32H723VGT6 (Cortex-M7 @ 550 MHz) |
| **Boîtier** | LQFP100 |
| **Flash** | 1 MB |
| **RAM** | 564 KB (DTCM: 128KB, AXI SRAM: 128KB, SRAM1-2: 32KB, SRAM4: 16KB) |
| **Flash externe** | W25Q64JV (8 MB QSPI) |
| **IMU embarqué** | BMI088 (Accéléromètre + Gyroscope 6 axes) |
| **OS** | RT-Thread RTOS |

### Interfaces de communication

| Interface | Quantité | Description |
|-----------|----------|-------------|
| **FDCAN** | 3 | Bus CAN à 1 Mbps |
| **UART** | 5 | Dont 2 avec RS-485 |
| **SPI** | 3 | Pour IMU et périphériques |
| **USB** | 1 | USB OTG HS (CDC) |

---

## 2. Configuration système

### Configuration de l'horloge

```
Horloge système (SYSCLK): 550 MHz
Source: HSE (24 MHz) + PLL
├── AHB: 275 MHz (diviseur /2)
├── APB1: 137.5 MHz (diviseur /2)
├── APB2: 137.5 MHz (diviseur /2)
├── APB3: 137.5 MHz (diviseur /2)
└── APB4: 137.5 MHz (diviseur /2)
```

### Configuration PLL principale

| Paramètre | Valeur |
|-----------|--------|
| PLLM | 3 |
| PLLN | 68 |
| PLLP | 1 |
| PLLQ | 4 |
| PLLR | 2 |
| PLLFRACN | 6144 |

---

## 3. IMU BMI088

### Présentation

Le BMI088 est un capteur inertiel 6 axes haute performance composé de :
- **Accéléromètre** : ±3g, ±6g, ±12g, ±24g
- **Gyroscope** : ±125°/s à ±2000°/s

### Configuration matérielle

L'IMU est connecté via le bus **SPI2** :

| Signal | Pin | Description |
|--------|-----|-------------|
| SCK | PB13 | Horloge SPI |
| MOSI | PC1 | Master Out Slave In |
| MISO | PC2 | Master In Slave Out |
| CS_ACC | - | Chip Select Accéléromètre (GPIO) |
| CS_GYRO | - | Chip Select Gyroscope (GPIO) |

### Paramètres SPI2

```c
// Configuration SPI2 pour BMI088
Mode: Master, Full Duplex
Prescaler: 32 (3.75 Mbps)
CPOL: HIGH
CPHA: 2-edge
Data Size: 8 bits
```

### Activation du driver BMI088

Pour activer le support BMI088 dans RT-Thread :

```bash
# Dans le répertoire du projet
scons --menuconfig

# Naviguer vers:
# RT-Thread online packages → sensors drivers → BMI088
```

Ou modifier `.config` :

```
CONFIG_PKG_USING_BMI088=y
```

### Exemple de lecture BMI088

```c
#include <rtthread.h>
#include <rtdevice.h>

/* Structure de données BMI088 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} bmi088_data_t;

/* Registres BMI088 */
#define BMI088_ACC_CHIP_ID      0x00
#define BMI088_ACC_DATA         0x12
#define BMI088_GYR_CHIP_ID      0x00
#define BMI088_GYR_DATA         0x02

/* Initialisation SPI pour BMI088 */
static struct rt_spi_device *spi_dev_acc;
static struct rt_spi_device *spi_dev_gyro;

int bmi088_init(void)
{
    /* Configuration du bus SPI2 */
    struct rt_spi_configuration cfg;
    cfg.data_width = 8;
    cfg.mode = RT_SPI_MODE_3 | RT_SPI_MSB;  /* CPOL=1, CPHA=1 */
    cfg.max_hz = 3750000;  /* 3.75 MHz */

    /* Attacher les périphériques SPI */
    rt_hw_spi_device_attach("spi2", "spi20", GPIOX, GPIO_PIN_X);  /* ACC CS */
    rt_hw_spi_device_attach("spi2", "spi21", GPIOX, GPIO_PIN_X);  /* GYRO CS */

    spi_dev_acc = (struct rt_spi_device *)rt_device_find("spi20");
    spi_dev_gyro = (struct rt_spi_device *)rt_device_find("spi21");

    if (spi_dev_acc && spi_dev_gyro)
    {
        rt_spi_configure(spi_dev_acc, &cfg);
        rt_spi_configure(spi_dev_gyro, &cfg);
        return RT_EOK;
    }
    return -RT_ERROR;
}

/* Lecture des données accéléromètre */
int bmi088_read_accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t tx_buf[8] = {BMI088_ACC_DATA | 0x80, 0};  /* Bit 7 = 1 pour lecture */
    uint8_t rx_buf[8];

    rt_spi_transfer(spi_dev_acc, tx_buf, rx_buf, 8);

    /* Les données commencent à l'index 2 (dummy byte) */
    *ax = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    *ay = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);
    *az = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);

    return RT_EOK;
}

/* Lecture des données gyroscope */
int bmi088_read_gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t tx_buf[7] = {BMI088_GYR_DATA | 0x80, 0};
    uint8_t rx_buf[7];

    rt_spi_transfer(spi_dev_gyro, tx_buf, rx_buf, 7);

    *gx = (int16_t)((rx_buf[2] << 8) | rx_buf[1]);
    *gy = (int16_t)((rx_buf[4] << 8) | rx_buf[3]);
    *gz = (int16_t)((rx_buf[6] << 8) | rx_buf[5]);

    return RT_EOK;
}
```

---

## 4. Ports UART

### Vue d'ensemble des ports UART

| UART | TX Pin | RX Pin | Baudrate | Usage recommandé |
|------|--------|--------|----------|------------------|
| **USART1** | PA9 | PA10 | 115200 | Console/Debug RT-Thread |
| **USART2** | PD5 | PD6 | 115200 | RS-485 #1 |
| **USART3** | PD8 | PD9 | 115200 | RS-485 #2 |
| **UART7** | PE8 | PE7 | 115200 | Communication générale |
| **USART10** | PE3 | PE2 | 115200 | Communication générale |

### Configuration des paramètres UART

Tous les ports sont configurés par défaut en :
- **Baudrate** : 115200
- **Data bits** : 8
- **Stop bits** : 1
- **Parity** : None
- **Flow control** : None

### Schéma de câblage

```
                    STM32H723
                   ┌──────────┐
    USART1 TX ────►│ PA9      │
    USART1 RX ◄────│ PA10     │  ► Console Debug
                   │          │
    USART2 TX ────►│ PD5      │
    USART2 RX ◄────│ PD6      │  ► RS-485 #1
    USART2 DE ────►│ PD4      │
                   │          │
    USART3 TX ────►│ PD8      │
    USART3 RX ◄────│ PD9      │  ► RS-485 #2
    USART3 DE ────►│ PB14     │
                   │          │
    UART7 TX  ────►│ PE8      │
    UART7 RX  ◄────│ PE7      │  ► Générale
                   │          │
    USART10 TX ───►│ PE3      │
    USART10 RX ◄───│ PE2      │  ► Générale
                   └──────────┘
```

### Activation des drivers UART

Dans `menuconfig` :

```
RT-Thread Components → Device Drivers → Using UART device drivers

Hardware Drivers Config → On-chip Peripheral Drivers → Enable UART
├── Enable UART1 (Console)
├── Enable UART2
├── Enable UART3
├── Enable UART7
└── Enable UART10
```

### Exemple d'utilisation UART

```c
#include <rtthread.h>
#include <rtdevice.h>

#define UART_DEVICE_NAME    "uart7"

static rt_device_t uart_device;

/* Callback de réception */
static rt_err_t uart_rx_callback(rt_device_t dev, rt_size_t size)
{
    /* Signaler le thread de réception */
    rt_sem_release(&rx_sem);
    return RT_EOK;
}

int uart_init(void)
{
    /* Rechercher le périphérique */
    uart_device = rt_device_find(UART_DEVICE_NAME);
    if (uart_device == RT_NULL)
    {
        rt_kprintf("Cannot find device %s\n", UART_DEVICE_NAME);
        return -RT_ERROR;
    }

    /* Ouvrir en lecture/écriture avec interruption RX */
    rt_device_open(uart_device, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    /* Configurer le callback de réception */
    rt_device_set_rx_indicate(uart_device, uart_rx_callback);

    return RT_EOK;
}

/* Envoi de données */
void uart_send(const char *data, rt_size_t len)
{
    rt_device_write(uart_device, 0, data, len);
}

/* Réception de données */
rt_size_t uart_receive(char *buffer, rt_size_t len)
{
    return rt_device_read(uart_device, 0, buffer, len);
}

/* Modification du baudrate */
void uart_set_baudrate(rt_uint32_t baudrate)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = baudrate;
    rt_device_control(uart_device, RT_DEVICE_CTRL_CONFIG, &config);
}
```

---

## 5. Communication RS-485

### Configuration matérielle

La carte dispose de 2 ports RS-485 avec contrôle DE (Driver Enable) automatique :

| Port RS-485 | UART | TX | RX | DE (Driver Enable) |
|-------------|------|----|----|-------------------|
| **RS-485 #1** | USART2 | PD5 | PD6 | PD4 |
| **RS-485 #2** | USART3 | PD8 | PD9 | PB14 |

### Schéma de connexion RS-485

```
                                 Transceiver RS-485
    STM32H723                    (ex: MAX485)
   ┌──────────┐                 ┌──────────────┐
   │          │                 │              │
   │   PD5 TX ├────────────────►│ DI           │
   │          │                 │              │        ┌────┐
   │   PD6 RX │◄────────────────┤ RO           │ A ────►│    │
   │          │                 │              │        │Bus │
   │   PD4 DE ├────────────────►│ DE    ┌──────┤ B ────►│485 │
   │          │                 │ /RE ──┘      │        └────┘
   └──────────┘                 └──────────────┘

   Note: Le pin DE contrôle automatiquement la direction
   - DE HIGH = Transmission
   - DE LOW  = Réception
```

### Mode de fonctionnement

Le STM32H723 supporte le mode RS-485 natif avec gestion automatique du signal DE :

```c
/* Configuration RS-485 dans CubeMX (déjà configuré) */
HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0);
HAL_RS485Ex_Init(&huart3, UART_DE_POLARITY_HIGH, 0, 0);
```

### Exemple d'utilisation RS-485

```c
#include <rtthread.h>
#include <rtdevice.h>

#define RS485_1_DEVICE    "uart2"
#define RS485_2_DEVICE    "uart3"

static rt_device_t rs485_dev;

/* Initialisation RS-485 */
int rs485_init(const char *device_name)
{
    rs485_dev = rt_device_find(device_name);
    if (rs485_dev == RT_NULL)
    {
        rt_kprintf("RS485: Cannot find %s\n", device_name);
        return -RT_ERROR;
    }

    /* Ouvrir avec support DMA si disponible */
    rt_device_open(rs485_dev, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    /* Configuration baudrate pour RS-485 (typiquement 9600 ou 115200) */
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    config.baud_rate = BAUD_RATE_115200;
    rt_device_control(rs485_dev, RT_DEVICE_CTRL_CONFIG, &config);

    return RT_EOK;
}

/* Envoi Modbus RTU (exemple) */
int rs485_send_modbus(uint8_t slave_addr, uint8_t function,
                       uint16_t reg_addr, uint16_t reg_count)
{
    uint8_t frame[8];
    uint16_t crc;

    frame[0] = slave_addr;
    frame[1] = function;
    frame[2] = (reg_addr >> 8) & 0xFF;
    frame[3] = reg_addr & 0xFF;
    frame[4] = (reg_count >> 8) & 0xFF;
    frame[5] = reg_count & 0xFF;

    /* Calcul CRC16 Modbus */
    crc = modbus_crc16(frame, 6);
    frame[6] = crc & 0xFF;         /* CRC Low */
    frame[7] = (crc >> 8) & 0xFF;  /* CRC High */

    /* Envoi - le DE est géré automatiquement par le hardware */
    rt_device_write(rs485_dev, 0, frame, 8);

    return RT_EOK;
}

/* Calcul CRC16 Modbus */
uint16_t modbus_crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}
```

### Protocole Modbus RTU

Structure d'une trame Modbus RTU :

```
┌──────────┬──────────┬───────────────┬───────────┐
│ Adresse  │ Fonction │    Données    │  CRC16    │
│ (1 byte) │ (1 byte) │ (N bytes)     │ (2 bytes) │
└──────────┴──────────┴───────────────┴───────────┘
```

Codes de fonction courants :
| Code | Fonction |
|------|----------|
| 0x01 | Read Coils |
| 0x02 | Read Discrete Inputs |
| 0x03 | Read Holding Registers |
| 0x04 | Read Input Registers |
| 0x05 | Write Single Coil |
| 0x06 | Write Single Register |
| 0x0F | Write Multiple Coils |
| 0x10 | Write Multiple Registers |

---

## 6. Bus CAN (FDCAN)

### Vue d'ensemble

La carte dispose de **3 bus FDCAN** configurés à **1 Mbps** :

| Bus CAN | RX Pin | TX Pin | RAM Offset | Usage |
|---------|--------|--------|------------|-------|
| **FDCAN1** | PD0 | PD1 | 0x000 | Principal |
| **FDCAN2** | PB5 | PB6 | 0x406 | Secondaire |
| **FDCAN3** | PD12 | PD13 | 0x812 | Auxiliaire |

### Schéma de connexion CAN

```
    STM32H723                    Transceiver CAN
   ┌──────────┐                  (ex: TJA1050)
   │          │                 ┌──────────────┐
   │  PD0 RX  │◄────────────────┤ RXD          │
   │          │                 │              │        ┌─────┐
   │  PD1 TX  ├────────────────►│ TXD    CANH  ├───────►│     │
   │          │                 │              │        │ Bus │
   │          │                 │        CANL  ├───────►│ CAN │
   └──────────┘                 └──────────────┘        └─────┘

   Note: Résistance de terminaison 120Ω entre CANH et CANL
   aux extrémités du bus
```

### Configuration FDCAN

Paramètres configurés par défaut :

| Paramètre | Valeur |
|-----------|--------|
| Baudrate nominal | 1 Mbps |
| Mode | Classic CAN (non FD) |
| Prescaler | 24 |
| Time Quantum | 200 ns |
| RX FIFO Elements | 32 |
| TX FIFO Elements | 32 |
| Standard Filters | 1 |

### Calcul du timing CAN

```
Clock FDCAN = 120 MHz (PLL2)
Prescaler = 24
Time Quantum = 24 / 120 MHz = 200 ns

Nominal Bit Time:
├── Sync Segment: 1 TQ
├── Propagation + Phase1: 2 TQ
└── Phase2: 2 TQ
Total: 5 TQ = 1 µs → 1 Mbps
```

### Activation des drivers CAN

Dans `menuconfig` :

```
RT-Thread Components → Device Drivers → Using CAN device drivers

Hardware Drivers Config → On-chip Peripheral Drivers → Enable CAN
├── Enable CAN1
├── Enable CAN2
└── Enable CAN3
```

### Exemple d'utilisation CAN

```c
#include <rtthread.h>
#include <rtdevice.h>

#define CAN_DEVICE_NAME    "can1"

static rt_device_t can_dev;

/* Structure de message CAN */
struct rt_can_msg
{
    rt_uint32_t id;         /* ID du message */
    rt_uint32_t ide;        /* 0: Standard, 1: Extended */
    rt_uint32_t rtr;        /* 0: Data, 1: Remote */
    rt_uint32_t len;        /* Longueur des données (0-8) */
    rt_uint32_t priv;       /* Privé */
    rt_uint32_t hdr;        /* Index hardware */
    rt_uint8_t data[8];     /* Données */
};

/* Initialisation CAN */
int can_init(void)
{
    can_dev = rt_device_find(CAN_DEVICE_NAME);
    if (can_dev == RT_NULL)
    {
        rt_kprintf("Cannot find %s device\n", CAN_DEVICE_NAME);
        return -RT_ERROR;
    }

    /* Ouvrir le périphérique en interruption */
    rt_device_open(can_dev, RT_DEVICE_FLAG_INT_TX | RT_DEVICE_FLAG_INT_RX);

    /* Configuration: 1 Mbps */
    rt_device_control(can_dev, RT_CAN_CMD_SET_BAUD, (void *)CAN1MBaud);

    /* Mode normal */
    rt_device_control(can_dev, RT_CAN_CMD_SET_MODE, (void *)RT_CAN_MODE_NORMAL);

    return RT_EOK;
}

/* Configuration d'un filtre */
int can_set_filter(rt_uint32_t id, rt_uint32_t mask)
{
    struct rt_can_filter_item items[1] =
    {
        {
            .id = id,
            .ide = 0,           /* Standard ID */
            .rtr = 0,           /* Data frame */
            .mode = 0,          /* Mask mode */
            .mask = mask,
            .hdr = -1,          /* Auto */
        }
    };

    struct rt_can_filter_config cfg =
    {
        .count = 1,
        .actived = 1,
        .items = items,
    };

    rt_device_control(can_dev, RT_CAN_CMD_SET_FILTER, &cfg);
    return RT_EOK;
}

/* Envoi d'un message CAN */
int can_send(rt_uint32_t id, rt_uint8_t *data, rt_uint8_t len)
{
    struct rt_can_msg msg = {0};

    msg.id = id;
    msg.ide = RT_CAN_STDID;     /* Standard ID (11 bits) */
    msg.rtr = RT_CAN_DTR;       /* Data frame */
    msg.len = len;
    rt_memcpy(msg.data, data, len);

    rt_size_t size = rt_device_write(can_dev, 0, &msg, sizeof(msg));

    return (size == sizeof(msg)) ? RT_EOK : -RT_ERROR;
}

/* Réception d'un message CAN */
int can_receive(struct rt_can_msg *msg, rt_int32_t timeout)
{
    rt_size_t size = rt_device_read(can_dev, 0, msg, sizeof(struct rt_can_msg));
    return (size == sizeof(struct rt_can_msg)) ? RT_EOK : -RT_ERROR;
}

/* Thread de réception CAN */
void can_rx_thread_entry(void *parameter)
{
    struct rt_can_msg rxmsg;

    while (1)
    {
        if (can_receive(&rxmsg, RT_WAITING_FOREVER) == RT_EOK)
        {
            rt_kprintf("CAN RX: ID=0x%03X, Len=%d, Data=",
                       rxmsg.id, rxmsg.len);
            for (int i = 0; i < rxmsg.len; i++)
                rt_kprintf("%02X ", rxmsg.data[i]);
            rt_kprintf("\n");
        }
    }
}

/* Exemple d'utilisation pour contrôle moteur */
int motor_set_speed(rt_uint8_t motor_id, int16_t speed)
{
    rt_uint8_t data[4];

    /* Format: ID de base + ID moteur */
    rt_uint32_t can_id = 0x200 + motor_id;

    /* Données: Vitesse sur 16 bits (Big Endian) */
    data[0] = (speed >> 8) & 0xFF;
    data[1] = speed & 0xFF;
    data[2] = 0x00;
    data[3] = 0x00;

    return can_send(can_id, data, 4);
}
```

### Protocoles CAN courants pour robotique

#### Protocole DJI Motor (M3508/M2006)

```c
/* ID de commande pour moteurs DJI */
#define CAN_ID_MOTOR_CMD_1_4    0x200   /* Moteurs 1-4 */
#define CAN_ID_MOTOR_CMD_5_8    0x1FF   /* Moteurs 5-8 */

/* Structure de commande (4 moteurs par trame) */
typedef struct {
    int16_t current[4];  /* Courant commandé (-16384 à +16384) */
} motor_cmd_t;

/* Envoi commande 4 moteurs */
int dji_motor_set_current(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    uint8_t data[8];

    data[0] = (m1 >> 8) & 0xFF;
    data[1] = m1 & 0xFF;
    data[2] = (m2 >> 8) & 0xFF;
    data[3] = m2 & 0xFF;
    data[4] = (m3 >> 8) & 0xFF;
    data[5] = m3 & 0xFF;
    data[6] = (m4 >> 8) & 0xFF;
    data[7] = m4 & 0xFF;

    return can_send(CAN_ID_MOTOR_CMD_1_4, data, 8);
}
```

#### Feedback moteur

```c
/* ID de retour moteur */
#define CAN_ID_MOTOR_FB_BASE    0x201   /* 0x201-0x208 */

/* Structure de feedback */
typedef struct {
    uint16_t angle;          /* Position (0-8191) */
    int16_t  speed_rpm;      /* Vitesse en RPM */
    int16_t  current;        /* Courant mesuré */
    uint8_t  temperature;    /* Température */
} motor_feedback_t;

/* Parse feedback */
void parse_motor_feedback(struct rt_can_msg *msg, motor_feedback_t *fb)
{
    fb->angle       = (msg->data[0] << 8) | msg->data[1];
    fb->speed_rpm   = (msg->data[2] << 8) | msg->data[3];
    fb->current     = (msg->data[4] << 8) | msg->data[5];
    fb->temperature = msg->data[6];
}
```

---

## 7. Activation des périphériques

### Utilisation de menuconfig

```bash
# Dans le répertoire du projet
scons --menuconfig
```

### Options de configuration

```
RT-Thread Components
└── Device Drivers
    ├── Using UART device drivers
    ├── Using CAN device drivers
    ├── Using SPI Bus/Device device drivers
    └── Using Sensor device drivers

Hardware Drivers Config
└── On-chip Peripheral Drivers
    ├── Enable GPIO
    ├── Enable UART
    │   ├── Enable UART1 (console)
    │   ├── Enable UART2 (RS485)
    │   ├── Enable UART3 (RS485)
    │   ├── Enable UART7
    │   └── Enable UART10
    ├── Enable CAN
    │   ├── Enable CAN1
    │   ├── Enable CAN2
    │   └── Enable CAN3
    └── Enable SPI
        ├── Enable SPI1
        └── Enable SPI2 (BMI088)

RT-Thread online packages
└── sensors drivers
    └── BMI088
```

### Régénération du projet

Après modification de la configuration :

```bash
# Mettre à jour les packages
pkgs --update

# Régénérer le projet CMake
scons --target=cmake

# Compiler
mkdir build && cd build
cmake ..
make
```

---

## 8. Exemples de code

### Application complète : Lecture IMU + Envoi CAN

```c
#include <rtthread.h>
#include <rtdevice.h>

/* Configuration */
#define IMU_SAMPLE_RATE_HZ      1000
#define CAN_DEVICE              "can1"
#define CAN_ID_IMU_DATA         0x100

/* Variables globales */
static rt_device_t can_dev;
static struct rt_semaphore imu_sem;

/* Thread IMU */
static void imu_thread_entry(void *param)
{
    int16_t ax, ay, az, gx, gy, gz;
    uint8_t can_data[8];

    /* Initialisation IMU */
    bmi088_init();

    while (1)
    {
        /* Lecture données */
        bmi088_read_accel(&ax, &ay, &az);
        bmi088_read_gyro(&gx, &gy, &gz);

        /* Préparation trame CAN */
        can_data[0] = (ax >> 8) & 0xFF;
        can_data[1] = ax & 0xFF;
        can_data[2] = (ay >> 8) & 0xFF;
        can_data[3] = ay & 0xFF;
        can_data[4] = (az >> 8) & 0xFF;
        can_data[5] = az & 0xFF;
        can_data[6] = 0;
        can_data[7] = 0;

        /* Envoi CAN */
        can_send(CAN_ID_IMU_DATA, can_data, 8);

        /* Attente période */
        rt_thread_mdelay(1000 / IMU_SAMPLE_RATE_HZ);
    }
}

/* Point d'entrée */
int main(void)
{
    rt_thread_t imu_tid;

    /* Initialisation CAN */
    can_init();

    /* Création thread IMU */
    imu_tid = rt_thread_create("imu",
                               imu_thread_entry,
                               RT_NULL,
                               1024,
                               10,
                               10);
    if (imu_tid != RT_NULL)
        rt_thread_startup(imu_tid);

    return 0;
}
```

### Application : Communication RS-485 Modbus

```c
#include <rtthread.h>
#include <rtdevice.h>

#define RS485_DEVICE    "uart2"
#define MODBUS_ADDR     0x01

static rt_device_t rs485;

/* Thread esclave Modbus */
static void modbus_slave_thread(void *param)
{
    uint8_t rx_buffer[256];
    uint8_t tx_buffer[256];
    rt_size_t len;

    while (1)
    {
        /* Réception requête */
        len = rt_device_read(rs485, 0, rx_buffer, sizeof(rx_buffer));

        if (len > 0 && rx_buffer[0] == MODBUS_ADDR)
        {
            /* Vérifier CRC */
            if (modbus_check_crc(rx_buffer, len))
            {
                /* Traiter la requête */
                len = modbus_process_request(rx_buffer, len, tx_buffer);

                /* Envoyer réponse */
                if (len > 0)
                    rt_device_write(rs485, 0, tx_buffer, len);
            }
        }

        rt_thread_mdelay(1);
    }
}

int main(void)
{
    /* Initialisation RS-485 */
    rs485 = rt_device_find(RS485_DEVICE);
    rt_device_open(rs485, RT_DEVICE_FLAG_RDWR | RT_DEVICE_FLAG_INT_RX);

    /* Démarrer thread Modbus */
    rt_thread_t tid = rt_thread_create("modbus",
                                        modbus_slave_thread,
                                        RT_NULL,
                                        2048,
                                        15,
                                        10);
    if (tid != RT_NULL)
        rt_thread_startup(tid);

    return 0;
}
```

---

## Annexes

### A. Tableau récapitulatif des pins

| Périphérique | Pin | Fonction | AF |
|--------------|-----|----------|-----|
| USART1_TX | PA9 | Console TX | AF7 |
| USART1_RX | PA10 | Console RX | AF7 |
| USART2_TX | PD5 | RS485-1 TX | AF7 |
| USART2_RX | PD6 | RS485-1 RX | AF7 |
| USART2_DE | PD4 | RS485-1 DE | AF7 |
| USART3_TX | PD8 | RS485-2 TX | AF7 |
| USART3_RX | PD9 | RS485-2 RX | AF7 |
| USART3_DE | PB14 | RS485-2 DE | AF7 |
| UART7_TX | PE8 | UART TX | AF7 |
| UART7_RX | PE7 | UART RX | AF7 |
| USART10_TX | PE3 | UART TX | AF11 |
| USART10_RX | PE2 | UART RX | AF4 |
| FDCAN1_TX | PD1 | CAN1 TX | AF9 |
| FDCAN1_RX | PD0 | CAN1 RX | AF9 |
| FDCAN2_TX | PB6 | CAN2 TX | AF9 |
| FDCAN2_RX | PB5 | CAN2 RX | AF9 |
| FDCAN3_TX | PD13 | CAN3 TX | AF5 |
| FDCAN3_RX | PD12 | CAN3 RX | AF5 |
| SPI2_SCK | PB13 | IMU CLK | AF5 |
| SPI2_MOSI | PC1 | IMU MOSI | AF5 |
| SPI2_MISO | PC2 | IMU MISO | AF5 |

### B. Ressources

- [Documentation RT-Thread](https://www.rt-thread.org/document/site/)
- [STM32H723 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [Boutique DaMiao](https://item.taobao.com/item.htm?id=814954787248)

### C. Historique des révisions

| Version | Date | Auteur | Description |
|---------|------|--------|-------------|
| 1.0 | 2024-12 | - | Version initiale |
