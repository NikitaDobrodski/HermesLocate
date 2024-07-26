"""socket_server.py"""

import socket
import logging

class SocketServer:
    def __init__(self, host, port):
        """Инициализация сервера с указанным хостом и портом."""
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.clients = set()
        self.running = True

        logging.basicConfig(level=logging.INFO,
                            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                            handlers=[
                                logging.FileHandler("/home/adam/Documents/HermesLocate/logs/server.log"),
                                logging.StreamHandler()
                            ])

    def start(self):
        """Запуск сервера."""
        self.sock.bind((self.host, self.port))
        logging.info(f"Server listening on {self.host}:{self.port}")
        print(f"Server listening on {self.host}:{self.port}")

        while self.running:
            try:
                data, client_address = self.sock.recvfrom(1024)
                if client_address not in self.clients:
                    logging.info(f"New client connected: {client_address}")
                    print(f"New client connected: {client_address}")
                    self.clients.add(client_address)
                logging.info(f"Received data from {client_address}: {data}")
                print(f"Received data from {client_address}: {data.decode('utf-8')}")
            except Exception as e:
                logging.error(f"Error receiving from client: {e}")

    def receive_data(self):
        """Получение данных от клиента."""
        try:
            data, _ = self.sock.recvfrom(1024)
            return data.decode('utf-8')
        except Exception as e:
            logging.error(f"Error receiving data: {e}")
            return None

    def send_coordinates(self, x, y):
        """Рассылает координаты всем клиентам."""
        message = f"{x:.6f} {y:.6f}"
        self.send_message(message)

    def send_message(self, message):
        """Рассылает сообщение всем клиентам."""
        for client in self.clients:
            try:
                self.sock.sendto(bytes(message, "utf-8"), client)
            except Exception as e:
                logging.error(f"Error broadcasting to client {client}: {e}")

    def stop(self):
        """Остановка сервера."""
        self.running = False
        self.sock.close()
        print("Server stopped.")


