# HrR: Hotel-receptionist Robot

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Neo4J](https://img.shields.io/badge/neo4j-008CC1?style=for-the-badge&logo=neo4j&logoColor=white)
![Docker](https://img.shields.io/badge/docker-2496ED?style=for-the-badge&logo=docker&logoColor=white)

> Implementation of an LLM and a robotic system for the simulation of an advanced mobile robotic agent for customer service and healthcare in the hospitality industry.

## 📖 **Context**

This project was developed for the **Artificial Intelligence 2** examination of Prof. **Valeria Seidita** and **Robotics** examination of Prof. **Antonio Chella**, during the **2025/2026** Academic Year at the **Università degli Studi di Palermo**, **Computer Engineering (LM-32, 2035)** course.

## 👥 **Authors**
_Andrea Spinelli - Gabriele Bova - Paolo Manuele Gulotta - Vincenzo Zizzo_

## 🛠️ **Technologies Used**

* **Language:** Python 3.x
* **Knowledge Base:** Neo4J (Graph Database)
* **LLM Framework:** LangChain / OpenAI API (o specifica il modello locale se usato, es. Llama)
* **Robotics Middleware:** QiBullet / NAOqi / ROS (specificare quale usato per la simulazione)
* **Deployment:** Docker (per il container Neo4j)

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
    python main.py
    ```
    *(Note: Replace `main.py` with your actual entry point, e.g., `src/app.py`)*

## 📂 **Project Structure**
The code is organized into a modular structure to ensure clarity and separation of concerns between the robotic control and the cognitive layer.

- **`data/`**: Contains initialization scripts for the Neo4j database (Cypher queries).
- **`src/`**: The core of the project.
  - **`llm_module/`**: Handles the interaction with the Large Language Model (Prompt Engineering, Chains).
  - **`knowledge_base/`**: Modules for connecting to and querying Neo4j.
  - **`robot_control/`**: Scripts for controlling the robot's movement and gestures (simulation or physical).
  - **`speech/`**: Speech-to-Text (STT) and Text-to-Speech (TTS) modules.
- **`docker-compose.yml`**: Configuration for spinning up the Neo4j container.
- **`requirements.txt`**: List of Python dependencies.
- **`config.yaml`**: Central configuration file for simulation parameters.

## ✨ **Key Features**

This project integrates a robotic system with a Large Language Model and a Knowledge Graph to create an intelligent hotel receptionist.

1.  **Conversational Intelligence (LLM)**
    * The robot uses an **LLM** (e.g., GPT-4) to understand natural language user queries, maintaining context and intent.
    * It generates human-like responses suitable for a hospitality setting.

2.  **Structured Knowledge Access (RAG with Neo4j)**
    * Unlike a standard chatbot, HrR is grounded in factual data stored in **Neo4j**.
    * It can query the graph database to retrieve real-time information about **room availability, hotel services, and guest bookings**, minimizing hallucinations.

3.  **Embodied Interaction**
    * The system simulates a physical presence (e.g., Pepper Robot), combining voice interaction with movement.
    * It handles task execution such as "Guiding the guest to the room" or "Check-in procedures".

4.  **Speech Pipeline**
    * Implements an end-to-end audio pipeline: **Speech-to-Text** to transcribe user voice and **Text-to-Speech** to vocalize the robot's response.