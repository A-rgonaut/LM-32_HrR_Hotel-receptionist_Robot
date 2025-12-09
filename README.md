# HrR: Hotel-receptionist Robot

![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)

> Implementation of an LLM and a robotic system for the simulation of an advanced mobile robotic agent for customer service and healthcare in the hospitality industry.

## 📖 **Context**

This project was developed for the **Artificial Intelligence 2** examination of Prof. **Valeria Seidita** and **Robotics** examination of Prof. **Antonio Chella**, during the **2025/2026** Academic Year at the **Università degli Studi di Palermo**, **Computer Engineering (LM-32, 2035)** course.

## 👥 **Authors**
_Andrea Spinelli - Gabriele Bova - Vincenzo Zizzo_

## 🛠️ **Technologies Used**

*   **Languages:** Python
*   **Databases:** Neo4J
*   **Other:** Git

## 🚀 **Installation and Startup**

To run this project, you will need Python and the required libraries. The setup involves creating an environment and running the analysis script.

### Prerequisites

*   **Python** 3.8 or higher.
*   **Required libraries**: indicated in the requirements.txt file
*   **Git** (to clone the repository).

### Instructions

**Important:** All commands must be executed from the project's **root directory** after installing the dependencies.

1.  **Clone the Repository**
    Open your terminal or command prompt and clone the repository to your local machine.
    ```bash
    git clone https://github.com/A-rgonaut/LM-32_Speech_Analysis_for_Depression_Assessment.git
    ```

2.  **Libraries**
    Install the required libraries
    ```bash
    pip install -r requirements.txt
    ```

3.  **Experiment Configuration**
    Before running any script, modify the `config.yaml` file. The `common.active_model` key determines which model will be used (`'svm'`, `'cnn'`, `'ssl'`). All other model-specific parameters are defined in their respective sections.

4.  **Data Preprocessing (Run once)**
    This script processes the raw datasets and generates the unified E1-DAIC-WOZ dataset required for training.
    ```bash
    python -m scripts.preprocessor
    ```

5.  **Feature Extraction (Optional)**
    This step is required **only** for the `ssl` model if `use_preextracted_features` is set to `true` in `config.yaml`.
    ```bash
    python -m scripts.feature_extraction
    ```

6.  **Model Training**
    The training script is unified. It will read `config.yaml`, identify the `active_model`, and launch the correct training procedure.
    ```bash
    python -m scripts.train
    ```

7.  **Model Testing**
    Similar to training, the testing script is unified and relies on the configuration in `config.yaml`. It loads the K saved models and runs an evaluation on the test set, reporting the mean metrics and standard deviation.
    ```bash
    python -m scripts.test
    ```

### Hyperparameter Search Workflow
When `hyperparameter_search_mode` is set to `true` for `cnn` or `ssl`, the script performs a full grid search:
-   For each combination of hyperparameters, a K-Fold Cross-Validation is performed.
-   The models from each fold are saved temporarily.
-   If the average F1-score of the current run exceeds the best score recorded so far, the temporary models are made permanent, and the models from the previous best run are deleted.
-   At the end of the search, the `saved_models/` directory will contain only the K models from the winning hyperparameter combination.

### Specific Workflow for the SSL Model
The `ssl` model follows a structured, three-phase workflow to first find the best architecture and then systematically compare different pre-trained models and their layers.
  
**Phase 1: Feature Extraction**
-   **Goal**: Pre-extract and save feature representations from all SSL models you wish to analyze.
-   **Action**:
    1.  In `config.yaml`, populate the `ssl.ssl_model_names` list with all desired models (e.g., `'facebook/wav2vec2-base-960h'`, `'microsoft/wavlm-base'`).
    2.  Run the extraction script:
        ```bash
        python -m scripts.feature_extraction
        ```

**Phase 2: Finding the Optimal Downstream Architecture**
-   **Goal**: Find the best sequential model (e.g., Transformer, BiLSTM) and its hyperparameters for processing the SSL features. This is done once using a strong reference model.
-   **Action**:
    1.  In `config.yaml`, set `active_model: 'ssl'`, `hyperparameter_search_mode: true` and `run_layer_sweep: false`.
    2.  Specify your single reference model using the **singular** key: `ssl_model_name: 'facebook/wav2vec2-base-960h'`. The `ssl_model_names` list is ignored in this mode.
    3.  Run the training script to start the grid search:
        ```bash
        python -m scripts.train
        ```
    4.  **Outcome**: This process will save the best-performing architecture's parameters to `saved_models/ssl/best_params_ssl.json`.

**Phase 3: Automated Sweep for Model and Layer Comparison**
-   **Goal**: Fairly compare the performance of each layer from each SSL model, using the optimal architecture found in Phase 2.
-   **Action**:
    1.  In `config.yaml`, set `hyperparameter_search_mode: false` and `run_layer_sweep: true`.
    2.  Ensure `ssl_model_names` contains the list of models for which you extracted features in Phase 1.
    3.  Populate `layers_to_use` with the layer indices you want to test (e.g., `[0, 1, ..., 12]`).
    4.  Run the training script:
        ```bash
        python -m scripts.train
        ```
    5.  Run the testing script: After training is complete, run the test script to evaluate all models on the held-out dev and test set.
        ```bash
        python -m scripts.test
        ```
    6. Run the results plotting script to visualize the performance across different models and layers.
        ```bash
        python -m scripts.plot_results
        ```

## Project Structure
The code is organized into a modular structure to ensure clarity, maintainability, and reusability.

- **`datasets/`**: Should contain the raw DAIC-WOZ and E-DAIC-WOZ datasets. The preprocessed `E1-DAIC-WOZ` dataset will also be generated here.
- **`features/`**: Directory where features extracted by SSL models (e.g., Wav2Vec2) are saved, if used.
- **`scripts/`**: Contains executable scripts for preprocessing, feature extraction, training, and testing the models.
- **`src/`**: The core of the project, organized as a Python package.
  - **`preprocessor.py`**: Handles the initial processing of audio and transcripts to create the unified E1-DAIC-WOZ dataset.
  - **`cnn_module/`**: Contains modules related to the 1D-CNN model (data loader, architecture).
  - **`svm_module/`**: Contains modules related to the SVM models (data loader, model logic).
  - **`ssl_module/`**: Contains modules for the first SSL-based model (chunk-based data loader, sequential architecture).
  - **`trainer.py`**: Unified Trainer class for all PyTorch models.
  - **`evaluator.py`**: Unified Evaluator class for all models.
  - **`utils.py`**: Contains shared utility functions (e.g., metrics calculation, seeding).
- **`saved_models/`**: Directory where trained models (`.pth`, `.pkl`) will be saved.
- **`results/`**: Directory where evaluation results (`.csv`) will be saved.
- **`config.yaml`**: Central configuration file to manage all experiment parameters.

## ✨ **Key Features**

This project presents a systematic and comparative study of three distinct machine learning paradigms for the automatic detection of depression from speech, using the **DAIC-WOZ** dataset. The complexity of the models progressively increases, providing a clear view of the evolution from classical methods to the current state-of-the-art.

1.  **Classical Baseline: Support Vector Machine (SVM)**
    *   This approach establishes a robust baseline using a traditional two-stage pipeline.
    *   It relies on **hand-crafted acoustic features** that capture engineered aspects of articulation, phonation (e.g., jitter, shimmer), and prosody (e.g., pitch, energy).
    *   Its purpose is to represent the classical methods that have long dominated the field and provide a solid, interpretable benchmark for comparison.

2.  **Supervised Deep Learning: 1-D Convolutional Neural Network (CNN)**
    *   This model serves as an "evolutionary bridge" by moving to an end-to-end approach.
    *   It operates directly on **segmented raw audio waveforms**, learning relevant features automatically instead of relying on manual engineering.
    *   The architecture is designed to capture hierarchical patterns in speech, from local phonetic textures to longer-term prosodic features. Its goal is to outperform the SVM by learning richer, more discriminative representations from the data itself.

3.  **State-of-the-Art Hierarchical Model with Self-Supervised Learning (SSL)**
    *   This is the project's most innovative contribution, designed to address the critical challenge of limited labeled data in clinical settings.
    *   **Feature Extractor**: It leverages a large, pre-trained **foundation model** (like `wav2vec 2.0`, `HuBERT`, etc.) to extract rich, high-level representations from audio. These models are pre-trained on vast amounts of unlabeled speech, allowing them to understand the fundamental structure of human speech.
    *   **Sequential Modeling**: The features from the foundation model are fed into a sequential module (a **Transformer Encoder** or **BiLSTM**) to model long-term temporal dependencies across an entire interview.
    *   **Hierarchical Pooling**: The model uses a sophisticated **Gated Attention Pooling** mechanism to intelligently aggregate information over time, allowing it to focus on the most relevant speech segments for a final diagnosis.
