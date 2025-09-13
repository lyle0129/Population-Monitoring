# Population-Monitoring
Quantifying Occupancy Dynamics for Emergency Evacuation   **Monitoring In-Building Population in Real-Time**  


Castillo | Soriano | Trillanes  

---

## 📌 Introduction  
This project focuses on **real-time monitoring of in-building population** to support **emergency evacuation efforts**. By integrating multiple sensors with an STM32 microcontroller and cloud-based services, the system aims to detect human activity, track inflow/outflow dynamics, and issue alerts during potential disaster scenarios.  

The system addresses key objectives:  
- Monitoring inflow and outflow of people  
- Mitigating risks and aiding rescue efforts during emergencies  
- Detecting disasters and ensuring seamless communication  

---

## ⚙️ Methodology  

### Hardware Components  
- **LD2450 mmWave Radar Sensor** – for population counting  
- **HCSR501 PIR Sensor** – for motion detection  
- **MQ2 Gas Sensor** – for hazardous gas detection  
- **ESP8266 WiFi Module** – for cloud connectivity  
- **STM32 Microcontroller** – core processing unit  

### Software & Integration  
- Register-level programming for sensor integration  
- UART & GPIO protocols for communication  
- ADC-based gas concentration conversion  
- Cloud integration with **ThingSpeak** for real-time alerts (email notifications)  
- Web application for data visualization:  
  👉 [Room Monitoring Web App](https://sites.google.com/up.edu.ph/room-monitoring)  

---

## 🖥️ Prototype Demonstration  
- Real-time tabulated sensor data  
- Visualization of room population dynamics  
- Email alert notifications sent to emergency contacts  
- Power resiliency feature for uninterrupted monitoring  

---

## 📊 Results and Discussion  
- **LD2450:** Successfully tracked population but encountered issues with negative counts and clustering.  
- **HCSR501:** Reliable motion detection with minor pauses during deployment.  
- **MQ2:** Effective in detecting gas presence; successful deployment testing with accurate PPM conversions.  
- **System Resiliency:** Maintained functionality even with power interruptions.  
- **Actuation System:** Email alerts were successfully delivered to users and emergency contacts.  

---

## ✅ Conclusion  
The system successfully:  
- Implemented sensor codes for real-time monitoring  
- Sent alerts via email through ThingSpeak  
- Ensured resiliency during power interruptions  
- Met project specifications for communication, monitoring, and alerting  

Findings confirm the ability to monitor:  
- **Room population count**  
- **Motion detection**  
- **Gas concentration levels**  

---

## 🚀 Recommendations  
- Deploy dual-node systems at all entrances for better accuracy  
- Address clustering of multiple people entering simultaneously  
- Explore **double-segmentation methods** for improved precision  
- Use faster communication (e.g., **I2C**) instead of cloud-based triggers for faster alerts  

---
