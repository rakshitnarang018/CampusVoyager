# 🤖 Campus Voyager

An **AI-powered autonomous robot** that guides campus visitors with real-time navigation, personalized chat, and landmark descriptions — powered by OpenAI and Streamlit.

---

## 🛍️ Overview

**Camous Voyager** is an intelligent physical robot designed to give campus tours autonomously. Whether you're a new student, a guest, or a prospective visitor, this bot offers **real-time conversational assistance**, **location-aware guidance**, and **autonomous movement** across the campus.

---

## ✨ Features

* 🧠 **AI Chat Interface**: Chat with the bot in natural language using an intuitive Streamlit interface
* 🏫 **Campus Information**: Get details about buildings, departments, services, and more
* 🏆 **Autonomous Navigation**: Robot navigates campus routes without human control
* 📍 **Location-Aware Descriptions**: Describes landmarks as it reaches them
* 🎤 **Voice Announcements**: Speaks information during tours
* 🧰 **Custom Tour Routes**: Choose routes (academic, cultural, etc.) dynamically
* 🔄 **Obstacle Avoidance**: Navigates safely through crowds and dynamic environments
* 🗓️ **Event Info**: Shares updates about ongoing or upcoming campus events
* 🧹 **Customizable**: Easily update campus data, prompts, and navigation settings

---

## 💠 Tech Stack

| Component            | Tech/Library                  |
| -------------------- | ----------------------------- |
| UI & Interaction     | Streamlit, Python             |
| AI Responses         | OpenAI API, LangChain         |
| Context Handling     | LangChain Agents/Memory       |
| Robotics Integration | Custom hardware, YAML configs |
| Mapping & Vision     | OpenCV, onboard sensors       |
| Security             | dotenv for API keys           |

---

## ⚙️ Installation

### 1. Clone the repository

```bash
git clone https://github.com/<your-username>/Hellum-campus_tour_bot.git
cd Hellum-campus_tour_bot
```

### 2. Set up virtual environment

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

### 4. Add your OpenAI API key

Create a `.env` file in the root directory:

```
OPENAI_API_KEY=your_openai_api_key_here
```

---

## 🔧 Hardware Setup

1. Connect to the robot's onboard Raspberry Pi or system
2. Edit navigation parameters in:

   ```
   config/robot_config.yml
   ```
3. Calibrate sensors using:

```bash
python scripts/calibrate_sensors.py
```

---

## 🚀 Running the App

### Start the interface

```bash
streamlit run app.py
```

Visit [http://localhost:8501](http://localhost:8501) in your browser.

---

## 🤖 Navigation Modes

* **Autonomous Campus Tour**
  Select a predefined route; the robot will navigate and describe key locations.

* **Custom Destination Mode**
  Ask the bot for directions to any building or department — and it will take you there.

---

## 🧠 Customization

### ➕ Add New Campus Info

* Update files in `data/` directory with your own:

  * Department info
  * Landmark details
  * Rules and policies

### ⚙️ Modify Bot Behavior

* Edit prompts and templates in `chat_handler.py`
* Adjust system prompts to change personality or domain focus

### 🗺️ Update Navigation Routes

* Add/edit waypoints in `config/robot_config.yml`
* Add corresponding descriptions in the landmarks database

---

## 🌍 Deployment Options

| Platform              | Use Case                     |
| --------------------- | ---------------------------- |
| Onboard Hardware      | Robot operation on campus    |
| Streamlit Cloud       | Remote monitoring & demo use |
| Mobile App (optional) | Companion app for visitors   |

Ensure `.env` and credentials are securely managed for each environment.

---

## 🙌 Contributing

We welcome contributions to make Hellum Campus Tour Bot even better!

```bash
# Fork → Code → PR
git checkout -b feature/amazing-idea
git commit -m "Add: your feature"
git push origin feature/amazing-idea
```

Then, open a **Pull Request** 🚀

---

## 📄 License

Licensed under the [MIT License](LICENSE). Feel free to use, modify, and share.

---
---

> *Building the future of campus experiences — one intelligent bot at a time.* 🌐
