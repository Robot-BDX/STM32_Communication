# Guide API BDX_SIm - DM-MC02

## Table des matières

1. [Vue d'ensemble](#1-vue-densemble)
2. [Architecture système](#2-architecture-système)
3. [Installation et configuration](#3-installation-et-configuration)
4. [API STM32 (Firmware)](#4-api-stm32-firmware)
5. [API Python (Jetson)](#5-api-python-jetson)
6. [Protocole de communication](#6-protocole-de-communication)
7. [Exemples d'utilisation](#7-exemples-dutilisation)
8. [Dépannage](#8-dépannage)

---

## 1. Vue d'ensemble

### Objectif

Cette API fournit une interface temps-réel haute performance pour contrôler le robot BDX_SIm via la carte DM-MC02. Elle est optimisée pour l'exécution de politiques de reinforcement learning avec :

- **Boucle de contrôle 600 Hz** (période 1.667 ms)
- **Latence < 200 µs** via UART @ 2Mbps
- **14 articulations** (format standard BDX_SIm)
- Lecture IMU haute fréquence (BMI088 @ 800 Hz)
- Communication bidirectionnelle temps-réel avec Jetson Orin Nano

### Spécifications clés

| Paramètre | Valeur |
|-----------|--------|
| Fréquence contrôle | 600 Hz |
| Latence communication | < 200 µs |
| Interface Jetson | UART @ 2 Mbps |
| Nombre d'articulations | 14 |
| Bus moteurs RS-485 | 2 (1 Mbps) |
| IMU | BMI088 @ 800 Hz |

### Composants

```
┌─────────────────────────────────────────────────────────────────┐
│                     JETSON ORIN NANO                            │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │             Python API (bdx_robot.py)                     │ │
│  │  • BDXRobot class - 600 Hz control loop                   │ │
│  │  • Lock-free state access                                 │ │
│  │  • Policy execution helpers                               │ │
│  └───────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────┘
                              │ UART @ 2Mbps (/dev/ttyTHS1)
                              │ Protocole binaire 600 Hz
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                     STM32H723 DM-MC02                           │
│  ┌───────────────────────────────────────────────────────────┐ │
│  │           BDX API (bdx_api.h/c)                           │ │
│  │  • Timer interrupt @ 600 Hz                               │ │
│  │  • DMA-based UART communication                           │ │
│  │  • Double-buffered state (lock-free)                      │ │
│  └───────────────────────────────────────────────────────────┘ │
│         │                    │                    │             │
│  ┌──────▼─────┐      ┌──────▼─────┐      ┌──────▼─────┐       │
│  │   BMI088   │      │  RS-485 #1 │      │  RS-485 #2 │       │
│  │   Driver   │      │  (USART2)  │      │  (USART3)  │       │
│  │   (SPI2)   │      │ Joints 0-5 │      │ Joints 6-13│       │
│  └────────────┘      └────────────┘      └────────────┘       │
└─────────────────────────────────────────────────────────────────┘
```

### Mapping des articulations (BDX_SIm Standard)

| Index | Nom | Bus | Type moteur |
|-------|-----|-----|-------------|
| 0 | left_hip_yaw | RS-485 #1 | Unitree Go1 |
| 1 | left_hip_roll | RS-485 #1 | Unitree A1 |
| 2 | left_hip_pitch | RS-485 #1 | Unitree A1 |
| 3 | left_knee | RS-485 #1 | Unitree A1 |
| 4 | left_ankle | RS-485 #1 | Unitree Go1 |
| 5 | neck_pitch | RS-485 #1 | Unitree Go1 |
| 6 | head_pitch | RS-485 #2 | Dynamixel XH540 |
| 7 | head_yaw | RS-485 #2 | Dynamixel XH540 |
| 8 | head_roll | RS-485 #2 | Dynamixel XH540 |
| 9 | right_hip_yaw | RS-485 #2 | Unitree Go1 |
| 10 | right_hip_roll | RS-485 #2 | Unitree A1 |
| 11 | right_hip_pitch | RS-485 #2 | Unitree A1 |
| 12 | right_knee | RS-485 #2 | Unitree A1 |
| 13 | right_ankle | RS-485 #2 | Unitree Go1 |

---

## 2. Architecture système

### Fichiers de l'API

```
stm32h723-dm-mc02/
├── api/
│   ├── bdx_api.h           # API principale (header)
│   └── bdx_api.c           # API principale (600 Hz loop)
├── drivers/
│   ├── bmi088/
│   │   ├── bmi088.h        # Driver IMU (header)
│   │   └── bmi088.c        # Driver IMU (implémentation)
│   ├── rs485/
│   │   ├── rs485_motor.h   # Driver RS-485 (header)
│   │   └── rs485_motor.c   # Driver RS-485 (implémentation)
│   └── protocol/
│       ├── bdx_protocol.h  # Protocole UART 600 Hz (header)
│       └── bdx_protocol.c  # Protocole (CRC-8, DMA)
└── python_api/
    ├── __init__.py
    ├── bdx_robot.py        # API Python temps-réel
    └── requirements.txt
```

### Flux de données 600 Hz

```
          ┌──────────────────────────────────────────────────┐
          │               Jetson (Policy)                    │
          │                                                  │
          │   observation ──► policy() ──► action            │
          │       ▲                          │               │
          └───────│──────────────────────────│───────────────┘
                  │ State (136 bytes)        │ Cmd (60 bytes)
                  │ 600 Hz                   │ 600 Hz
          ┌───────│──────────────────────────│───────────────┐
          │       │                          ▼               │
          │   ┌───┴───┐              ┌───────────┐           │
          │   │ State │◄────────────►│ Setpoints │           │
          │   │Buffer │              │  Buffer   │           │
          │   └───────┘              └─────┬─────┘           │
          │       ▲                        │                 │
          │       │      600 Hz Timer ISR  │                 │
          │       │      ┌─────────────────┼───────┐         │
          │       │      │                 ▼       │         │
          │   ┌───┴──────┴──┐        ┌─────────┐   │         │
          │   │  Read IMU   │        │  Send   │   │         │
          │   │  & Motors   │        │ Commands│   │         │
          │   └─────────────┘        └─────────┘   │         │
          │                                        │         │
          │               STM32H723                │         │
          └────────────────────────────────────────┴─────────┘
```

### Configuration matérielle

| Composant | Interface | Configuration |
|-----------|-----------|---------------|
| **IMU BMI088** | SPI2 | CS_ACC: PC0, CS_GYRO: PC3, 800 Hz ODR |
| **RS-485 Bus 1** | USART2 | DE: PD4, 1 Mbps, Joints 0-5 |
| **RS-485 Bus 2** | USART3 | DE: PB14, 1 Mbps, Joints 6-13 |
| **Jetson UART** | USART1 | 2 Mbps, DMA TX/RX |
| **Timer Control** | TIM6 | 600 Hz interrupt |

---

## 3. Installation et configuration

### Côté STM32 (Firmware)

#### 1. Configuration du timer 600 Hz

Dans CubeMX, configurer TIM6 :
- Clock source: Internal
- Prescaler: (APB clock / 600 Hz) - 1
- Period: Ajuster pour obtenir exactement 600 Hz
- Activer l'interruption Update

#### 2. Configuration UART Jetson

```c
// USART1 @ 2 Mbps
huart1.Init.BaudRate = 2000000;
huart1.Init.WordLength = UART_WORDLENGTH_8B;
huart1.Init.StopBits = UART_STOPBITS_1;
huart1.Init.Parity = UART_PARITY_NONE;
huart1.Init.Mode = UART_MODE_TX_RX;
huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
```

#### 3. Initialisation dans main.c

```c
#include "bdx_api.h"

/* Instance système globale */
static bdx_system_t bdx_sys;

int main(void)
{
    /* Configuration par défaut */
    bdx_config_t config;
    bdx_get_default_config(&config);

    /* Personnaliser le mapping moteur si nécessaire */
    config.motor_map[JOINT_LEFT_KNEE].gear_ratio = 6.33f;
    config.motor_map[JOINT_LEFT_KNEE].inverted = true;

    /* Initialisation */
    if (bdx_init(&bdx_sys, &config) != RT_EOK) {
        rt_kprintf("BDX init failed!\n");
        return -1;
    }

    /* Démarrer la boucle de contrôle 600 Hz */
    bdx_start(&bdx_sys);

    /* Enregistrer les commandes MSH (debug) */
    bdx_register_msh_commands(&bdx_sys);

    return 0;
}

/* Timer interrupt callback - 600 Hz */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        bdx_timer_callback(&bdx_sys);
    }
}

/* UART DMA callbacks */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        bdx_uart_rx_callback(&bdx_sys);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        bdx_uart_tx_callback(&bdx_sys);
    }
}
```

### Côté Jetson (Python)

#### 1. Installation

```bash
# Copier les fichiers Python
cp -r python_api /path/to/your/project/

# Installer les dépendances
pip3 install pyserial numpy
```

#### 2. Configuration UART

```bash
# Désactiver le terminal série sur UART (si activé)
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Vérifier le port UART
ls -la /dev/ttyTHS*
```

#### 3. Test de connexion

```python
from bdx_robot import BDXRobot

robot = BDXRobot('/dev/ttyTHS1')
if robot.connect():
    print("Connected!")
    print(f"RX frames: {robot.stats.rx_frames}")
    robot.disconnect()
```

---

## 4. API STM32 (Firmware)

### Types de données

#### Robot State (double-buffered)

```c
typedef struct {
    /* IMU data */
    float imu_accel[3];             /* Linear acceleration (m/s²) */
    float imu_gyro[3];              /* Angular velocity (rad/s) */
    float imu_temperature;          /* Temperature (°C) */

    /* Joint feedback */
    float joint_positions[14];      /* Current positions (rad) */
    float joint_velocities[14];     /* Current velocities (rad/s) */
    float joint_currents[14];       /* Motor currents (mA) */

    /* Foot contacts */
    bool contact_left;
    bool contact_right;

    /* Timing */
    uint32_t timestamp_us;          /* Microsecond timestamp */
    uint32_t sequence;              /* Frame sequence number */
} bdx_robot_state_t;
```

#### Joint Setpoints

```c
typedef struct {
    float positions[14];    /* Target positions (rad) */
    uint8_t flags;          /* Control flags */
    uint16_t sequence;      /* Command sequence */
    uint32_t timestamp_us;  /* When received */
} bdx_setpoints_t;
```

#### Motor Mapping

```c
typedef struct {
    uint8_t bus_id;         /* RS-485 bus (0 or 1) */
    uint8_t motor_id;       /* Motor ID on bus */
    float gear_ratio;       /* Gear ratio */
    float offset;           /* Zero position offset (rad) */
    bool inverted;          /* Invert direction */
} bdx_motor_map_t;
```

### Fonctions principales

#### Initialisation

```c
/* Obtenir la configuration par défaut */
void bdx_get_default_config(bdx_config_t *config);

/* Initialiser le système */
rt_err_t bdx_init(bdx_system_t *sys, const bdx_config_t *config);

/* Démarrer/arrêter la boucle 600 Hz */
rt_err_t bdx_start(bdx_system_t *sys);
rt_err_t bdx_stop(bdx_system_t *sys);
```

#### Lecture des données (lock-free)

```c
/* État complet du robot */
void bdx_get_state(bdx_system_t *sys, bdx_robot_state_t *state);

/* Statistiques système */
void bdx_get_stats(bdx_system_t *sys, bdx_stats_t *stats);

/* État système */
bdx_sys_state_t bdx_get_sys_state(bdx_system_t *sys);

/* Vérifier communication active */
bool bdx_is_comm_active(bdx_system_t *sys);
```

#### Contrôle moteur

```c
/* Emergency stop - désactive immédiatement tous les moteurs */
void bdx_estop(bdx_system_t *sys);

/* Effacer l'arrêt d'urgence */
rt_err_t bdx_clear_estop(bdx_system_t *sys);
```

#### Callbacks (à implémenter)

```c
/* Timer 600 Hz (appeler depuis HAL_TIM_PeriodElapsedCallback) */
void bdx_timer_callback(bdx_system_t *sys);

/* UART DMA callbacks */
void bdx_uart_rx_callback(bdx_system_t *sys);
void bdx_uart_tx_callback(bdx_system_t *sys);
```

---

## 5. API Python (Jetson)

### Classe principale : BDXRobot

```python
from bdx_robot import BDXRobot, JointID, SystemState
import numpy as np

# Création et connexion
robot = BDXRobot('/dev/ttyTHS1')
robot.connect()

# Ou avec context manager
with BDXRobot('/dev/ttyTHS1') as robot:
    # Boucle de contrôle...
    pass
```

### Lecture des données (lock-free)

```python
# État complet du robot
state = robot.get_state()

# IMU
print(f"Accel: {state.imu_accel}")  # numpy array [ax, ay, az]
print(f"Gyro: {state.imu_gyro}")    # numpy array [gx, gy, gz]

# Articulations
print(f"Positions: {state.joint_positions}")    # 14 floats (rad)
print(f"Velocities: {state.joint_velocities}")  # 14 floats (rad/s)

# Contacts
print(f"Left foot: {state.contact_left}")
print(f"Right foot: {state.contact_right}")

# Statut
print(f"System state: {state.system_state.name}")
print(f"Timestamp: {state.timestamp_us} µs")
```

### Envoi de commandes

```python
# Position des 14 articulations (radians)
positions = np.zeros(14)
positions[JointID.LEFT_KNEE] = 0.5
positions[JointID.RIGHT_KNEE] = 0.5

# Envoyer avec moteurs activés
robot.set_joint_positions(positions, enable=True)

# Désactiver les moteurs
robot.disable()

# Arrêt d'urgence
robot.emergency_stop()
```

### Conversion pour RL Policy

```python
# Convertir l'état en observation pour la policy
state = robot.get_state()
observation = state.to_observation()

# observation shape: (36,)
# [imu_accel(3), imu_gyro(3), positions(14), velocities(14), contacts(2)]

# Exécuter la policy
action = policy(observation)  # shape: (14,)

# Envoyer l'action
robot.set_joint_positions(action, enable=True)
```

### Statistiques de communication

```python
stats = robot.get_stats()
print(f"TX frames: {stats.tx_frames}")
print(f"RX frames: {stats.rx_frames}")
print(f"RX errors: {stats.rx_errors}")
print(f"CRC errors: {stats.crc_errors}")
print(f"Avg latency: {stats.avg_latency_us} µs")
print(f"Max latency: {stats.max_latency_us} µs")
```

---

## 6. Protocole de communication

### Vue d'ensemble

- **UART @ 2 Mbps** pour latence minimale
- **CRC-8** (polynôme 0x07) pour validation rapide
- **Trames à taille fixe** pour timing déterministe
- **600 Hz bidirectionnel**

### Trame de commande (Jetson → STM32) : 60 bytes

```
┌───────┬──────┬──────────┬─────────────────────────┬───────┬─────┐
│ Start │ Type │ Sequence │   Joint Setpoints (14)  │ Flags │ CRC │
│  0xAA │ 0x01 │   2B     │       56 bytes          │  1B   │ 1B  │
└───────┴──────┴──────────┴─────────────────────────┴───────┴─────┘
```

| Champ | Taille | Description |
|-------|--------|-------------|
| Start | 1 | Marqueur 0xAA |
| Type | 1 | 0x01 = commande |
| Sequence | 2 | Numéro de séquence (little-endian) |
| Setpoints | 56 | 14 × float (positions en rad) |
| Flags | 1 | Bit 0: enable, Bit 1: estop |
| CRC | 1 | CRC-8 des 59 premiers bytes |

### Trame d'état (STM32 → Jetson) : 136 bytes

```
┌───────┬──────┬──────────┬───────────┬─────────────┬────────────┬────────────┬────────┬───────┬─────┐
│ Start │ Type │ Sequence │ Timestamp │  IMU Data   │ Positions  │ Velocities │ Status │ Error │ CRC │
│  0xAA │ 0x02 │   2B     │    4B     │   24 bytes  │  56 bytes  │  56 bytes  │   1B   │  1B   │ 1B  │
└───────┴──────┴──────────┴───────────┴─────────────┴────────────┴────────────┴────────┴───────┴─────┘
```

| Champ | Offset | Taille | Description |
|-------|--------|--------|-------------|
| Start | 0 | 1 | Marqueur 0xAA |
| Type | 1 | 1 | 0x02 = état |
| Sequence | 2 | 2 | Numéro de séquence |
| Timestamp | 4 | 4 | Timestamp µs (DWT) |
| IMU Accel | 8 | 12 | ax, ay, az (m/s²) |
| IMU Gyro | 20 | 12 | gx, gy, gz (rad/s) |
| Positions | 32 | 56 | 14 positions (rad) |
| Velocities | 88 | 56 | 14 vitesses (rad/s) |
| Status | 144 | 1 | État + flags |
| Error | 145 | 1 | Code erreur |
| CRC | 146 | 1 | CRC-8 |

### Flags de commande

| Bit | Nom | Description |
|-----|-----|-------------|
| 0 | ENABLE | Activer les moteurs |
| 1 | ESTOP | Arrêt d'urgence |
| 2 | CALIBRATE | Démarrer calibration |

### Bits de statut

| Bit | Nom | Description |
|-----|-----|-------------|
| 0-1 | STATE | État système (0-3) |
| 2 | IMU_OK | IMU opérationnel |
| 3 | CONTACT_L | Contact pied gauche |
| 4 | CONTACT_R | Contact pied droit |
| 5 | MOTORS_OK | Tous moteurs OK |

### CRC-8

Polynôme 0x07, valeur initiale 0x00.

```python
CRC8_TABLE = [0x00, 0x07, 0x0E, ...]  # Table pré-calculée

def crc8(data: bytes) -> int:
    crc = 0x00
    for byte in data:
        crc = CRC8_TABLE[crc ^ byte]
    return crc
```

---

## 7. Exemples d'utilisation

### Exemple 1 : Boucle de contrôle 600 Hz

```python
#!/usr/bin/env python3
"""
Boucle de contrôle temps-réel pour BDX_SIm.
"""
import time
import numpy as np
from bdx_robot import BDXRobot, BDX_CONTROL_FREQ_HZ

def my_policy(observation: np.ndarray) -> np.ndarray:
    """Votre policy RL ici."""
    # Exemple: garder la position actuelle
    positions = observation[6:20]  # Indices 6-19 = positions
    return positions

robot = BDXRobot('/dev/ttyTHS1')

if robot.connect():
    target_dt = 1.0 / BDX_CONTROL_FREQ_HZ  # 1.667 ms
    warmup_steps = 100

    try:
        step = 0
        while True:
            loop_start = time.time()

            # 1. Obtenir l'état (lock-free)
            state = robot.get_state()
            observation = state.to_observation()

            # 2. Exécuter la policy
            action = my_policy(observation)

            # 3. Envoyer les commandes
            if step >= warmup_steps:
                robot.set_joint_positions(action, enable=True)
            else:
                # Warmup: suivre la position actuelle
                robot.set_joint_positions(state.joint_positions, enable=False)

            step += 1

            # 4. Timing
            elapsed = time.time() - loop_start
            if elapsed < target_dt:
                time.sleep(target_dt - elapsed)

    except KeyboardInterrupt:
        print("\nArrêt...")

    robot.disable()
    robot.disconnect()

    # Afficher les stats
    stats = robot.get_stats()
    print(f"Avg latency: {stats.avg_latency_us} µs")
```

### Exemple 2 : Utiliser BDXController

```python
#!/usr/bin/env python3
"""
Utilisation du controller haut niveau.
"""
import numpy as np
from bdx_robot import BDXController

def sine_policy(observation: np.ndarray) -> np.ndarray:
    """Policy simple: oscillation sinusoïdale."""
    t = observation[-1] if len(observation) > 36 else 0
    action = np.zeros(14)
    action[3] = 0.5 * np.sin(t * 2 * np.pi)   # Left knee
    action[12] = 0.5 * np.sin(t * 2 * np.pi)  # Right knee
    return action

controller = BDXController('/dev/ttyTHS1')

if controller.connect():
    # Exécuter pendant 10 secondes
    controller.run(sine_policy, duration=10.0, warmup_steps=200)
    controller.disconnect()
```

### Exemple 3 : Enregistrement de données

```python
#!/usr/bin/env python3
"""
Enregistrer les données à 600 Hz.
"""
import time
import numpy as np
from bdx_robot import BDXRobot

data_buffer = []

def record_callback(state):
    """Callback appelé à chaque frame (600 Hz)."""
    data_buffer.append({
        'timestamp': state.timestamp_us,
        'imu_accel': state.imu_accel.copy(),
        'imu_gyro': state.imu_gyro.copy(),
        'positions': state.joint_positions.copy(),
        'velocities': state.joint_velocities.copy(),
        'contacts': [state.contact_left, state.contact_right]
    })

robot = BDXRobot('/dev/ttyTHS1')

if robot.connect():
    # Activer le callback
    robot.set_state_callback(record_callback)

    # Enregistrer pendant 5 secondes
    time.sleep(5.0)

    robot.disconnect()

    # Sauvegarder
    np.save('recording.npy', data_buffer)
    print(f"Enregistré {len(data_buffer)} frames ({len(data_buffer)/600:.2f} s)")
```

---

## 8. Dépannage

### Problèmes de connexion

#### Le port UART n'est pas accessible

```bash
# Vérifier les ports disponibles
ls -la /dev/ttyTHS*
ls -la /dev/ttyUSB*

# Vérifier les permissions
sudo chmod 666 /dev/ttyTHS1

# Ou ajouter l'utilisateur au groupe dialout
sudo usermod -a -G dialout $USER
```

#### Pas de réponse du STM32

1. Vérifier que le firmware est chargé et en cours d'exécution
2. Vérifier les connexions UART (TX/RX, GND)
3. Vérifier le baudrate (2 Mbps)
4. Utiliser la console MSH pour débugger

### Problèmes de timing

#### Latence élevée (> 500 µs)

- Vérifier que le DMA est correctement configuré
- Vérifier qu'il n'y a pas de contention sur le bus
- Réduire la charge CPU du Jetson

#### Frames manquées

- Vérifier les statistiques: `robot.stats.sequence_errors`
- Augmenter la priorité du thread de réception
- Vérifier qu'il n'y a pas de surcharge de la boucle de contrôle

### Commandes MSH (Debug STM32)

```
# Afficher le statut système
msh> bdx_status

# Afficher les positions articulaires
msh> bdx_joints

# Arrêt d'urgence
msh> bdx_estop

# Effacer l'arrêt d'urgence
msh> bdx_clear
```

### Statistiques de debug

```python
# Côté Python
stats = robot.get_stats()
print(f"TX: {stats.tx_frames}, RX: {stats.rx_frames}")
print(f"Errors: {stats.rx_errors}, CRC: {stats.crc_errors}")
print(f"Seq errors: {stats.sequence_errors}")
print(f"Latency: avg={stats.avg_latency_us}µs, max={stats.max_latency_us}µs")

# Vérifier le taux de frames
expected = 600  # Hz
actual = stats.rx_frames / elapsed_time
print(f"Frame rate: {actual:.1f} Hz (expected {expected})")
```

---

## Annexe : Références rapides

### Configuration par défaut

| Paramètre | Valeur |
|-----------|--------|
| Fréquence contrôle | 600 Hz |
| Période | 1.667 ms |
| Baudrate Jetson | 2 Mbps |
| Baudrate RS-485 | 1 Mbps |
| Timeout communication | 50 ms |
| IMU ODR | 800 Hz |

### Tailles de trames

| Trame | Taille | Direction |
|-------|--------|-----------|
| Command | 60 bytes | Jetson → STM32 |
| State | 136 bytes | STM32 → Jetson |

### Bande passante

- Commandes: 60 × 600 × 8 = 288 kbps
- États: 136 × 600 × 8 = 653 kbps
- Total: ~1 Mbps sur 2 Mbps disponibles

### Historique des versions

| Version | Date | Description |
|---------|------|-------------|
| 2.0 | 2024-12 | UART 2Mbps @ 600Hz, 14 joints |
| 1.0 | 2024-12 | Version initiale (USB CDC) |
