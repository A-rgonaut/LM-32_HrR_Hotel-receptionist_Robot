# HrR: Hotel-receptionist Robot

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Unity](https://img.shields.io/badge/unity-%23000000.svg?style=for-the-badge&logo=unity&logoColor=white)
![Neo4J](https://img.shields.io/badge/neo4j-008CC1?style=for-the-badge&logo=neo4j&logoColor=white)
![Docker](https://img.shields.io/badge/docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)

> Implementation and simulation of an intelligent robotic system for customer service and healthcare in the hotel.

## 📖 **Context**

This project was developed for the **Artificial Intelligence 2** examination of Prof. **Valeria Seidita** and **Robotics** examination of Prof. **Antonio Chella**, during the **2025/2026** Academic Year at the **Università degli Studi di Palermo**, **Computer Engineering (LM-32, 2035)** course.

## 👥 **Authors**
_Andrea Spinelli - Gabriele Bova - Paolo Manuele Gulotta - Vincenzo Zizzo_

## 🛠️ **Technologies Used**

* **Language:** Python 3.x
* **Knowledge Base:** Neo4J 
* **LLM Framework:** OpenAI API
* **Robotics Middleware:** ROS
* **Deployment:** Docker 

## 🚀 **Installation and Startup**

To run this project, you will need Python, a running instance of Neo4j, and the required libraries.

### Prerequisites

* **Python** 3.10 or higher.
* **Docker** (recommended for running Neo4j).
* **API Keys**: An OpenAI API Key (if using GPT) or local model setup.
* **Git** (to clone the repository).

### Instructions

**Important:** All commands must be executed from the project's **root directory**.

1.  **Clone the Repository**
    Open your terminal or command prompt and clone the repository to your local machine.
    ```bash
    git clone [https://github.com/A-rgonaut/LM-32_HrR_Hotel-receptionist_Robot.git](https://github.com/A-rgonaut/LM-32_HrR_Hotel-receptionist_Robot.git)
    cd LM-32_HrR_Hotel-receptionist_Robot
    ```

2.  **Environment Setup**
    Create a virtual environment and install dependencies.
    ```bash
    python -m venv venv
    source venv/bin/activate  # On Windows: venv\Scripts\activate
    pip install -r requirements.txt
    ```

3.  **Database Configuration (Neo4j)**
    Start the Neo4j database using Docker.
    ```bash
    docker-compose up -d
    ```
    *Ensure the database is populated. If there is a population script:*
    ```bash
    python -m scripts.populate_db
    ```

4.  **Configuration**
    Create a `.env` file in the root directory (based on `.env.example`) and add your API keys and DB credentials:
    ```env
    OPENAI_API_KEY=your_key_here
    NEO4J_URI=bolt://localhost:7687
    NEO4J_USERNAME=neo4j
    NEO4J_PASSWORD=password
    ```

5.  **Run the Robot Agent**
    Launch the main application to start the simulation and the interaction loop.
    ```bash
    #
    #TODO
    #
    #./startup.sh
    #python main.py 
    ```
    *(Note: Replace `main.py` with your actual entry point, e.g., `src/app.py`)*

## ✨ **Key Features**

The project integrates advanced robotics and symbolic artificial intelligence to create an autonomous agent capable of operating in non-deterministic environments.

### 1. **Distributed Unity-ROS2 Architecture**

* **Simulation Bridge:** The architecture decouples the physical-visual simulation layer from the logical-cognitive control layer. Unity serves as the engine for physics and sensors, while ROS2 hosts the intelligence stack. The two ecosystems communicate via the TCP/IP protocol.


* **Realistic Physics:** The robot is a digital replica of the Pepper model, modeled as a multi-body system with a real mass of approximately 28 kg. *ArticulationBody* components were used for wheel simulation to ensure joint stability and physical precision.



### 2. **Hybrid Neuro-Symbolic AI**

* **Form/Content Separation:** Integration with Large Language Models (LLM), specifically Google Gemini, is limited to *Natural Language Generation* (NLG) to transform structured data into empathetic responses. The model does not make operational decisions, reducing the risk of hallucinations.


* **Ontology and Graph Database:** Long-term memory is entrusted to the Neo4j graph database. The ontological schema is formally defined via Protégé (TBox), while Neo4j manages the dynamic population of instances (ABox), ensuring high performance on large volumes of data.



### 3. **Explainable AI (XAI) & Reasoning**

* 
**OWA (Open World Assumption) Management:** The system handles the ambiguities typical of open worlds by explicitly modeling validity and risk constraints within the ontology.


* **Decision Transparency:** An external Java module (based on OWLAPI and Openllet) generates formal explanations for every inference. The robot inhibits or proposes actions based on the presence of axioms in prohibition or permission categories, making every behavior intelligible.



### 4. **Robust Autonomous Navigation**

* **Custom Path Planning:** A custom implementation of the A* algorithm was developed, optimized for occupancy grids with 8-neighbor connectivity and differentiated costs for diagonal movements (Octile heuristic). It includes *Corner Cutting Prevention* to avoid collisions on edges.


* **AMCL Localization:** Localization uses the AMCL (Adaptive Monte Carlo Localization) probabilistic particle filter synchronized with simulated time. The filter dynamically adapts the number of particles based on system uncertainty.



### 5. **Biometric Monitoring and Safety**

* **Kalman Filter:** A dedicated module processes signals from smart wristbands (heart rate, blood pressure) via decoupled linear Kalman filters. This approach isolates faults on individual sensors and distinguishes between momentary artifacts and real clinical trends.


* **Priority Arbitration:** Control is managed by a subsumption arbitration system with fixed priorities. Emergency management (Scenario C) has absolute priority over everything, including battery charging, ensuring guest safety above any other function.
