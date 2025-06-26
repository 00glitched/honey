import pandas as pd
import matplotlib.pyplot as plt

# Reemplaza 'tu_archivo.csv' con el nombre real de tu archivo
df = pd.read_csv('temperature.csv')

# Reemplaza 'columna_x' y 'columna_y' con los nombres de las columnas
# que quieres graficar
columna_x = 'Time'
columna_y = 'Temp'

# Crea la gráfica
plt.plot(df[columna_x], df[columna_y], marker='o')  # 'o' es para puntos, puedes cambiarlo

# Agrega etiquetas a los ejes
plt.xlabel(columna_x)
plt.ylabel(columna_y)
plt.title('Gráfico de ' + columna_x + ' vs ' + columna_y)

# Muestra la gráfica
plt.grid(True)  # Añade una grilla
plt.show()
