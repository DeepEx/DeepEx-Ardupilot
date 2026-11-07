# BlueOS extension: DeepEx VESC maintenance mode (VESC Tool over TCP <-> can0)
FROM python:3.11-slim
WORKDIR /app
COPY app.py vesc_can_tcp_bridge.py /app/
ENV WEB_PORT=9135 VESC_IFACE=can0 VESC_TCP_PORT=65102 VESC_TARGET=1 VESC_BRIDGE_ID=120
CMD ["python3", "-u", "app.py"]

LABEL version="0.1.0"
LABEL type="tool"
LABEL company='{"name": "DeepEx"}'
LABEL readme="https://raw.githubusercontent.com/DeepEx/DeepEx-Ardupilot/deepex/vesc-maintenance-extension/Tools/deepex/vesc-maintenance/README.md"
LABEL tags='["vesc", "can", "maintenance"]'
LABEL requirements="core >= 1.1"
LABEL permissions='{"NetworkMode": "host", "HostConfig": {"NetworkMode": "host", "RestartPolicy": {"Name": "unless-stopped"}}}'
