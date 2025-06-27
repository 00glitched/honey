import serial
import time
import csv
import os
from collections import defaultdict, deque
import matplotlib.pyplot as plt

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
MAXIT = 5000
PERIODO_PROMEDIO = 600  # 10 minutos
VENTANA_TIEMPO = 86400  # 24 horas en segundos

init_time = time.time()
last_save_time = init_time
usb = serial.Serial(PORT, BAUDRATE)

filename = "datos_promediados.csv"
header_written = False
count = 0

acumulador = defaultdict(list)
registro_tiempo = deque()
registro_datos = defaultdict(lambda: deque())

plt.ion()
fig, ax = plt.subplots()
lineas = {}

# Variable(s) que deseas graficar en tiempo real (solo promedios)
VARIABLES_A_GRAFICAR = ["TEMP", "HUM"]  # Puedes cambiar esto

def parse_data(raw_data):
    """Convierte 'NOMBRE1:valor1,NOMBRE2:valor2,...' en dict"""
    pairs = raw_data.split(",")
    data_dict = {}
    for pair in pairs:
        if ":" in pair:
            key, val = pair.split(":")
            try:
                data_dict[key.strip()] = float(val.strip())
            except ValueError:
                pass
    return data_dict

def save_stats(data_acum, timestamp, filename):
    global header_written
    if not data_acum:
        return

    stats = {}
    for key, values in data_acum.items():
        stats[f"{key}_avg"] = sum(values) / len(values)
        stats[f"{key}_min"] = min(values)
        stats[f"{key}_max"] = max(values)

    with open(filename, "a", newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not header_written:
            headers = ["Tiempo"] + list(stats.keys())
            writer.writerow(headers)
            header_written = True
        row = [f"{timestamp:.2f}"] + [f"{stats[k]:.3f}" for k in stats]
        writer.writerow(row)

    # Guardar en memoria para graficar
    registro_tiempo.append(timestamp)
    for var in VARIABLES_A_GRAFICAR:
        key = f"{var}_avg"
        if key in stats:
            registro_datos[var].append(stats[key])

    # Mantener solo las últimas 24 horas (~144 puntos)
    while registro_tiempo and (timestamp - registro_tiempo[0]) > VENTANA_TIEMPO:
        registro_tiempo.popleft()
        for var in VARIABLES_A_GRAFICAR:
            if registro_datos[var]:
                registro_datos[var].popleft()

    actualizar_grafico()

def actualizar_grafico():
    ax.clear()
    for var in VARIABLES_A_GRAFICAR:
        ax.plot(list(registro_tiempo), list(registro_datos[var]), label=var)
    ax.set_xlabel("Tiempo [s]")
    ax.set_ylabel("Valor promedio")
    ax.set_title("Promedios cada 10 minutos (últimas 24 h)")
    ax.legend()
    ax.grid(True)
    plt.draw()
    plt.pause(0.01)

noSTOP = True

while noSTOP:
    raw = usb.readline().decode('utf-8').strip()
    timestamp = time.time() - init_time

    if raw:
        data_dict = parse_data(raw)
        for k, v in data_dict.items():
            acumulador[k].append(v)

    if (time.time() - last_save_time) >= PERIODO_PROMEDIO:
        save_stats(acumulador, timestamp, filename)
        acumulador = defaultdict(list)
        last_save_time = time.time()

    count += 1
    if count > MAXIT:
        noSTOP = False
