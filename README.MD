# Software for Embedded Systems (AVR)

This repository contains my coursework and projects from the **Software for Embedded Systems** module, focused on **bare-metal AVR programming**, **interrupts**, **hardware timers**, and **FreeRTOS** on the **ATmega328P**.

Each Continuous Assessment (CA) builds on the previous one, progressing from low-level hardware interaction to real-time operating system concepts.

All projects were developed in **C**, compiled using **avr-gcc**, and tested using **SimulIDE**, **AVR Dragon**, and **real hardware** where applicable.

---

## Hardware & Tools Used

- **Microcontroller:** ATmega328P  
- **Programming Language:** C  
- **Toolchain:** avr-gcc, Makefiles  
- **Debugging:** AVR Dragon, GDB  
- **Simulation:** SimulIDE  
- **RTOS:** FreeRTOS  

---

## Continuous Assessment (CA) Files Overview

---

### CA1 – Frequency Measurement Using Interrupts & Timers

**Description:**  
Measured the frequency of an external square-wave signal using hardware interrupts and Timer1 on the ATmega328P.  
Initial testing was done in SimulIDE, followed by validation on real hardware using a function generator.

**Demo & Explanation:**  
[![CA1 Demo](https://img.youtube.com/vi/SICio2jdrgs/0.jpg)](https://youtu.be/SICio2jdrgs)

---

### CA2 – Race Conditions & False Positives  
*(Nuclear Reactor Simulation)*

**Description:**  
Demonstrated how race conditions between interrupt and non-interrupt code can produce false alarms.  
A simulated nuclear reactor system was used to reproduce and then fix concurrency bugs using atomic sections.

**Demo & Explanation:**  
[![CA2 Demo](https://img.youtube.com/vi/HSYGz5UmGRE/0.jpg)](https://youtu.be/HSYGz5UmGRE)

---

### CA3 – Hidden Race Conditions at Assembly Level

**Description:**  
Showed that race conditions can still exist even when C code appears safe.  
Using the AVR Dragon and GDB, the false alarm was demonstrated at the **assembly level**.

**Demo & Explanation:**  
[![CA3 Demo](https://img.youtube.com/vi/TVwVEOjYdlM/0.jpg)](https://youtu.be/TVwVEOjYdlM)

---

### CA4 – RTOS-Based Nuclear Reactor Simulation

**Description:**  
Re-implemented the nuclear reactor simulation using **FreeRTOS**, combining interrupts, tasks, and semaphores to safely manage shared resources.

**Demo & Explanation:**  
[![CA4 Demo](https://img.youtube.com/vi/KElGwMslQd0/0.jpg)](https://youtu.be/KElGwMslQd0)

---

### CA5 – RTOS Queues (AVR Port)

**Description:**  
Ported an RTOS queue-based example from Shawn Hymel’s RTOS tutorial series (originally written for ESP32 + Arduino) to **bare-metal AVR** using **avr-gcc**.

**Demo & Explanation:**  
[![CA5 Demo](https://img.youtube.com/vi/0rY31SfOGNk/0.jpg)](https://youtu.be/0rY31SfOGNk)

---

## Full Video Playlist

**YouTube Playlist:**  
[![Playlist](https://img.youtube.com/vi/SICio2jdrgs/0.jpg)](https://www.youtube.com/playlist?list=PLw1CliiFCSWO-0DrH7VF5J_USj6C2Yf5h)

---

## Topics & Skills Covered

Through this module, I gained hands-on experience with:

- Installing and working within a **Debian Virtual Machine**
- Testing embedded hardware using **SimulIDE**
- **USART communication** between a microcontroller and a terminal
- Using **interrupts** and writing **Interrupt Service Routines (ISRs)**
- **Hardware timers** on the ATmega328P and their practical uses
- Identifying **race conditions** in embedded systems
- Preventing race conditions using **atomic operations**
- Debugging embedded systems with the **AVR Dragon**
- Debugging C programs using **GDB**
- **FreeRTOS** fundamentals
- Core FreeRTOS components:
  - Tasks  
  - Scheduler  
  - Semaphores  
  - Mutexes  
  - Queues  

