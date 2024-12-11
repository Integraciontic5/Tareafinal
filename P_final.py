import socket
import sqlite3
from datetime import datetime
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
import sys
from PyQt5.uic import loadUi
from pyqtgraph import PlotWidget

ESP32_HOST = "0.0.0.0"  # Cambia esto por la IP del ESP32
ESP32_PORT = 1234        # Puerto usado por el ESP32
db_filename = "datos_sensores.db"

# Función para inicializar la base de datos
def inicializar_db():
    conn = sqlite3.connect(db_filename)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS sensores (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp TEXT NOT NULL,
            datos TEXT NOT NULL
        )
    ''')
    conn.commit()
    conn.close()

# Función para guardar los datos en la base de datos
def guardar_datos(datos):
    conn = sqlite3.connect(db_filename)
    cursor = conn.cursor()
    cursor.execute('''
        INSERT INTO sensores (timestamp, datos) 
        VALUES (?, ?)
    ''', (datetime.now().isoformat(), datos))
    conn.commit()
    conn.close()

class ServerThread(QThread):
    new_data_signal = pyqtSignal(str)

    def run(self):
        # Configuración del servidor TCP
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.bind((ESP32_HOST, ESP32_PORT))
            s.listen()
            print("El servidor está esperando conexiones en el puerto", ESP32_PORT)

            conn, addr = s.accept()  # Acepta una conexión
            with conn:
                print('Conectado por', addr)
                while True:
                    data = conn.recv(1024)  # Recibe hasta 1024 bytes del cliente
                    if not data:
                        break  # Sale del bucle si no hay datos (conexión cerrada)
                    mensaje = data.decode('utf-8')
                    print("Datos recibidos:", mensaje)  # Muestra el mensaje recibido

                    # Intenta guardar el mensaje recibido en la base de datos
                    try:
                        guardar_datos(mensaje)  # Guarda los datos en la base de datos SQLite
                    except Exception as e:
                        print("Error al guardar los datos en la base de datos:", e)

                    # Emitir los datos para actualizar la interfaz
                    self.new_data_signal.emit(mensaje)

                    # Enviar una respuesta de confirmación al ESP32
                    respuesta = "ACK"
                    conn.sendall(respuesta.encode('utf-8'))

class MainWindow(QMainWindow):
    def __init__(self):
        super(MainWindow, self).__init__()
        loadUi("interfazz.ui", self)

        print("Interfaz cargada correctamente.")
        self.socket = None
        self.monitoring = False
        self.esp32_connection = None
        self.initUI()

        # Renombrar botones
        self.rename_buttons()

        # Inicializa la base de datos
        inicializar_db()

        # Crea el hilo para el servidor
        self.server_thread = ServerThread()
        self.server_thread.new_data_signal.connect(self.update_graph_and_data)
        self.server_thread.start()

    def initUI(self):
        # Crear el PlotWidget dinámicamente
        self.graphWidget = PlotWidget(self.centralwidget)
        self.graphWidget.setObjectName("plotWidget")
        self.graphWidget.setGeometry(100, 100, 400, 300)  # Ajusta según el diseño
        self.graphWidget.setTitle("Temperatura y Humedad")
        self.graphWidget.setBackground("w")
        self.graphWidget.addLegend()

        # Configurar gráficos
        self.temp_plot = self.graphWidget.plot(pen="r", name="Temperatura (C)")
        self.hum_plot = self.graphWidget.plot(pen="b", name="Humedad (%)")

        # Variables para almacenar datos
        self.temp_data = []
        self.hum_data = []

        # Configurar el temporizador para actualizar los datos
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_graph)

        # Conectar los botones a sus funciones
        self.pushButton.clicked.connect(self.start_monitoring)
        self.pushButton_2.clicked.connect(self.stop_monitoring)
        self.pushButton_4.clicked.connect(self.send_start_command)
        self.pushButton.setText("Stop")
        self.pushButton_4.setText("Start")
        self.pushButton_2.setText("mod 1")
        self.pushButton_3.setText("mod 2")

    def rename_buttons(self):
        # Cambiar los nombres de los botones
        self.pushButton.setObjectName("btnStop")
        self.pushButton_2.setObjectName("btnMode1")
        self.pushButton_3.setObjectName("btnMode2")
        self.pushButton_4.setObjectName("btnStart")

    def start_monitoring(self):
        # Inicia el monitoreo
        if not self.monitoring:
            self.monitoring = True
            self.timer.start(1000)  # Actualiza cada segundo

    def stop_monitoring(self):
        # Detiene el monitoreo
        if self.monitoring:
            self.monitoring = False
            self.timer.stop()
        self.send_stop_command()

    def update_graph(self):
        # Actualiza el gráfico con los últimos datos de temperatura y humedad
        self.temp_plot.setData(self.temp_data)
        self.hum_plot.setData(self.hum_data)

    def update_graph_and_data(self, data):
        # Este método maneja los datos recibidos y decide cómo procesarlos.
        try:
            # Si el mensaje contiene 'Temperatura:' y 'Humedad:', es el tipo de datos actuales
            if "Temperatura:" in data and "Humedad:" in data:
                # Eliminar etiquetas y unidades
                parsed_data = data.replace("Temperatura:", "").replace("Humedad:", "")
                parsed_data = parsed_data.replace("°C", "").replace("%", "")
                
                # Convertir los valores de temperatura y humedad
                temp, hum = map(float, parsed_data.split(","))
                
                # Añadir los datos actuales a las listas
                self.temp_data.append(temp)
                self.hum_data.append(hum)
                
                # Limitar la cantidad de datos mostrados (100 puntos máximo)
                if len(self.temp_data) > 100:
                    self.temp_data.pop(0)
                    self.hum_data.pop(0)
                
                # Actualizar el gráfico
                self.update_graph()
            
            # Si el mensaje contiene 'T_Avg:', 'T_Max:', 'T_Min:', es el tipo de datos de promedios, máximos y mínimos
            elif "T_Avg:" in data and "T_Max:" in data and "T_Min:" in data:
                # Eliminar etiquetas y unidades
                parsed_data = data.replace("T_Avg:", "").replace("T_Max:", "").replace("T_Min:", "")
                parsed_data = parsed_data.replace("H_Avg:", "").replace("H_Max:", "").replace("H_Min:", "")
                parsed_data = parsed_data.replace("°C", "").replace("%", "")
                
                # Separar los valores utilizando la coma como delimitador
                values = parsed_data.split(",")
                
                # Asignar cada valor a la variable correspondiente
                # Asignar cada valor a la variable correspondiente
                temp_avg = float(values[0].strip())  # T_Avg
                temp_max = float(values[1].strip())  # T_Max
                temp_min = float(values[2].strip())  # T_Min
                hum_avg = float(values[3].strip())  # H_Avg
                hum_max = float(values[4].strip())  # H_Max
                hum_min = float(values[5].strip())  # H_Min
                # Mostrar los valores promedio en los QLCDNumber
                self.lcdNumber.display(int(round(temp_max)))
                self.lcdNumber_2.display(int(round(temp_min)))
                self.lcdNumber_3.display(int(round(temp_avg)))
                self.lcdNumber_4.display(int(round(hum_max)))
                self.lcdNumber_5.display(int(round(hum_min)))
                self.lcdNumber_6.display(int(round(hum_avg)))
                
                # Guardar los datos en la base de datos
                guardar_datos(data)
            
            else:
                print("Mensaje recibido no reconocido:", data)

        except ValueError:
            print("Error al parsear los datos:", data)
        except Exception as e:
            print(f"Error al procesar los datos: {e}")

    def send_start_command(self):
        # Enviar comando de inicio a la ESP32
        self.send_command_to_esp32("START")

    def send_stop_command(self):
        # Enviar comando de parada a la ESP32
        self.send_command_to_esp32("STOP")

    def send_command_to_esp32(self, command):
        if self.esp32_connection:
            try:
                self.esp32_connection.sendall(command.encode('utf-8'))
                print(f"Comando enviado a la ESP32: {command}")
            except Exception as e:
                print(f"Error al enviar el comando a la ESP32: {e}")
        else:
            print("No hay conexión con la ESP32 para enviar comandos.")

# Inicializa la aplicación
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
