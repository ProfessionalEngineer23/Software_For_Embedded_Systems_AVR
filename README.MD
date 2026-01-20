# Software For Embedded Systems Module:

--
This repository contains my coursework and projects from the Software for Embedded Systems module, focused on bare-metal AVR programming, interrupts, timers, and FreeRTOS on the ATmega328P.

Each Continuous Assessment (CA) builds on the previous one, progressing from low-level hardware interaction to real-time operating system concepts.
All projects were developed in C, compiled with avr-gcc, and tested using SimulIDE, AVR Dragon, and real hardware where applicable.
--

## Topics/ Things I learned how to use:
- Installing Debian Virtual Machine
- Testing Hardware in SimulIDE
- USART communication to terminal from a microcontroller
- Using interrupts and creating interrupt service routines
- Hardware Timers on the ATmega328p and their uses
- Race conditions in embedded systems and how to prevent them
- Making functions/ variables atomic
- Debugging with the AVR Dragon 
- Debugging C code with GDB
- FreeRTOS
- Components of FreeRTOS like:
- - Tasks 
- - Scheduler 
- - Semaphores
- - Mutexes 
- - Queues

## Hardware/ Tools Used:
- Microcontroller: ATmega328P
- Language: C
- Toolchain: avr-gcc, Makefile
- Debugging: AVR Dragon, GDB
- Simulation: SimulIDE
- RTOS: FreeRTOS

## CA Overview:
- CA1 – Frequency Measurement Using Interrupts & Timers
- Demo and Explanation: https://youtu.be/SICio2jdrgs

- CA2 – Race Conditions & False Positives (Nuclear Reactor Simulation)
- Demo and Explanation: https://youtu.be/HSYGz5UmGRE

- CA3 – Hidden Race Conditions at Assembly Level
- Demo and Explanation: https://youtu.be/TVwVEOjYdlM

- CA4 – RTOS-Based Nuclear Reactor Simulation
- Demo and Explanation: https://youtu.be/KElGwMslQd0

- CA5 – RTOS Queues (AVR Port of Shawn Hymel’s RTOS Example)
- Demo and Explanation: https://youtu.be/0rY31SfOGNk

