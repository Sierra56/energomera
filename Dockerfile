FROM python:3.11-slim-bookworm

RUN apt-get update && apt-get install -y usbutils && rm -rf /var/lib/apt/lists/*

WORKDIR /app

RUN pip install --no-cache-dir pyserial==3.5 paho-mqtt==2.1.0

COPY ce308-to-mqtt.py /app/ce308-to-mqtt.py

RUN useradd -m -u 1000 poller && chown -R poller:poller /app
USER poller

CMD ["python3", "-u", "/app/ce308-to-mqtt.py"]
