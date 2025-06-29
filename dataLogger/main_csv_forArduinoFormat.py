import serial
import time
import csv
import os
from collections import defaultdict

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
MAXIT = 10000
PERIODO_PROMEDIO = 60  # 10 minutos = 600

init_time = time.time()
last_save_time = init_time
usb = serial.Serial(PORT, BAUDRATE)

filename = "datos.csv"
header_written = False
console_header_printed = False
count = 0

acumulador = defaultdict(list)

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

def save_average_and_print(data_acum, timestamp, filename):
    global header_written, console_header_printed
    if not data_acum:
        return

    promedios = {key: sum(vals)/len(vals) for key, vals in data_acum.items()}
    headers = ["Tiempo"] + list(promedios.keys())
    valores = [f"{timestamp:.2f}"] + [f"{promedios[k]:.3f}" for k in promedios]

    # Guardar en archivo CSV
    with open(filename, "a", newline='') as csvfile:
        writer = csv.writer(csvfile)
        if not header_written:
            writer.writerow(headers)
            header_written = True
        writer.writerow(valores)

    # Imprimir encabezado y valores por consola
    if not console_header_printed:
        print(",".join(headers))
        console_header_printed = True
    print(",".join(valores))

noSTOP = True

while noSTOP:
    raw = usb.readline().decode('utf-8').strip()
    timestamp = time.time() - init_time

    if raw:
        data_dict = parse_data(raw)
        for k, v in data_dict.items():
            acumulador[k].append(v)

    if (time.time() - last_save_time) >= PERIODO_PROMEDIO:
        save_average_and_print(acumulador, timestamp, filename)
        acumulador = defaultdict(list)
        last_save_time = time.time()

    count += 1
    if count > MAXIT:
        noSTOP = False
