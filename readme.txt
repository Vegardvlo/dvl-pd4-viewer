# DVL PD4 Viewer

- Parses Nortek/Teledyne PD4 frames (TCP/Serial input).
- Shows BT/WT speeds, 2D/3D speeds (m/s & m/min), altitude, status with colors.
- Forwards values-only JSON arrays via TCP/UDP in user-selected order.

## Run (Windows)
pip install -r requirements.txt
python pd4_viewer.py

## Build .exe
pyinstaller --onefile --noconsole --name DVL_PD4_Viewer pd4_viewer.py
