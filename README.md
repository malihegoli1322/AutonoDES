# AutonoDES

**AutonoDES** stands for **A Discrete Event Simulation Platform for Safety Scrutiny of Autonomous Mining Systems**. This project aims to provide a user-friendly simulation software that enables safety analysis for mixed-fleet (manned and autonomous) mining operations.

The platform allows mining engineers and planners to run what-if analyses and examine how different policies or operational scenarios affect safety and productivity. Unlike general-purpose simulation tools, **AutonoDES** is designed specifically to support safety evaluations in autonomous haulage systems (AHS).

---

## 🚀 Key Features

- **Event-based simulation** of autonomous and manned truck operations
- **Time-based modeling** of resource constraints, randomness, and interactions
- **Collision analysis** across a realistic mine road network
- **Risk assessment** identifying high-probability collision zones
- **Scenario testing** with customizable fleet configurations and road layouts
- **Built for non-programmers** in mining operations

AutonoDES uses the **OpenCLSim** and **OpenTNSim** libraries to simulate truck traffic flow, perform accident analysis, and highlight safety-critical zones like intersections and junctions. Results include visualization of traffic flows, production charts, and operational Gantt charts.

> Note: The model is not real-time but supports batch-style execution where users input data, run simulations, and review output.

---

## 🧰 Technologies Used

- **Python 3.10+**
- **OpenCLSim** – event-driven cyclic logistics simulator
- **OpenTNSim** – network simulation tool for transport operations
- **Pandas**, **NumPy**, **Matplotlib**, and other common Python libraries

---

## 📁 Repository Structure

```bash
AutonoDES/
│
├── src/                     # Main simulation script (Python format)
│   └── AutonoDES_Final.py
│
├── data/                    # Sample datasets in Excel format
│   ├── sample_1.xlsx
│   ├── sample_2.xlsx
│   └── ...
│
├── result/                  # Simulation outputs (figures, charts)
│   ├── sample_network.png
│   ├── production_chart.pdf
│   └── operation_gantt.pdf
│
├── environment.yml          # Conda environment setup
├── README.md                # Project overview and usage guide
└── LICENSE                  # MIT License
```

---

## ⚙️ Getting Started

### 1. Clone the Repository

```bash
git clone https://github.com/malihegoli1322/AutonoDES.git
cd AutonoDES
```

### 2. Set Up the Environment

We recommend creating a Conda virtual environment:

```bash
conda env create -f environment.yml
conda activate openclsimenv
```

Alternatively, install required libraries manually, including `openclsim`, `opentnsim`, and others.

### 3. Run the Simulation

- Open `src/autonodes.py` in **VS Code**.
- Ensure that input files are placed in the `data/` folder.
- Input files **must follow the sample format and column structure** provided.

Then run the script normally in VS Code.

---

## 🔒 Data Confidentiality & Input Format

This repository includes only **sample data**. All original datasets used in the research are **confidential and excluded**.

Users can substitute their own datasets as long as they match the structure and format of the provided Excel templates.

---

## 📚 License & Citation

This repository is licensed under the MIT License.

If you use AutonoDES in your work, please cite the following paper:

> Goli, M., Moniri-Morad, A., Baart, F., Shishvan, M.S., and Sattarvand, J., (2025). *Improving Safety in Hauling Operations: Predicting and Analyzing Collision Probabilities with Discrete Event Simulation*. ESREL-SRA conference, Stavanger, Norway (Accepted).

Related Publications

> Moniri-Morad, A., Shishvan, M. S., Aguilar, M., Goli, M., & Sattarvand, J. (2024). *Powered haulage safety, challenges, analysis, and solutions in the mining industry; a comprehensive review*. Results in Engineering, 21, 101684.

> Goli, M., Moniri-Morad, A., Aguilar, M., Shishvan, M.S., and Sattarvand, J., (2025). *A simulation-based risk assessment model for comparative analysis of collisions in autonomous and non-autonomous haulage trucks*. Transportation Research Interdisciplinary Perspectives (Under Review).

This repository is licensed under the MIT License.


📄 Citation info will be updated with DOI upon publication.


---

## 🔭 Future Development

AutonoDES is an **ongoing project**. Planned improvements include:

- Simulation of mitigation strategies to reduce collision probability
- Integration of **maintenance schedules** and **random breakdowns**
- More realistic modeling of mixed-fleet operations

---

## 🤝 Connect with Me

- [LinkedIn – Malihe Goli](https://www.linkedin.com/in/malihe-goli-3a039031/)
- [GitHub – malihe-goli](https://github.com/malihegoli1322)

Feel free to reach out with feedback or collaboration ideas!


