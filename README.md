# High-Accuracy Indoor Localization using Wi-Fi RTT (ESP32-S3)

This repository contains the ESP32-S3 C source code developed for my thesis:

**_“High Accurate Indoor Localization System Using Wi-Fi RTT”_**

The project focuses on building a high-precision indoor ranging system based on **IEEE 802.11mc Fine Timing Measurement (FTM)** and **Channel State Information (CSI)**. A custom ranging framework was designed to fuse FTM and CSI data and further enhance ranging performance using a neural-network-based model.

---

## Project Summary

A **ranging framework based on ESP32-S3** was developed, achieving the following:

- Combined **FTM + CSI fusion** for robust and accurate distance estimation  
- Integrated with a **neural network model for ranging** to improve accuracy  
- Custom data pipeline to align and fuse physical-layer CSI with MAC-layer RTT  
- ESP-NOW-based synchronization mechanism for CSI trigger and multi-anchor deployment  
- Runs on low-cost ESP32-S3 hardware with real-time data logging capability  

This work **overcomes the limitations of the official ESP-IDF FTM example**, enabling synchronized CSI-assisted RTT measurements — a key requirement for machine-learning-based indoor localization.

---

## Files in This Repository

| File | Role | Description |
|------|-------|----------------|
| `initiator.c` | **FTM Initiator + CSI Logger** | Runs on the mobile device. Initiates FTM ranging, receives CSI packets, and logs fused RTT+CSI data for machine learning. *(Replaces the official ESP-IDF FTM Initiator example)* |
| `responder.c` | **FTM Responder + ESP-NOW Broadcaster** | Runs on anchor nodes. Acts as a Wi-Fi AP with FTM responder enabled and periodically sends ESP-NOW packets to trigger CSI collection. *(Replaces the official ESP-IDF FTM Responder example)* |

These two files **serve as a drop-in replacement** for the ESP-IDF FTM examples so that researchers can directly use them for FTM+CSI fusion ranging experiments.

---

## Why Replace the Official ESP-IDF FTM Example?

The official examples are only suitable for basic RTT tests. They lack:

| Limitation in Official ESP-IDF Example |
|----------------------------------------|
| No CSI integration |
| No CSI and FTM synchronization |
| No multi-anchor support |
| Not designed for dataset generation or AI/ML |


