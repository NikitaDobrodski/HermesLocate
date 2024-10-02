"""socket_server.py"""

import socket

class SocketServer:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.clients = set()
        self.running = True

    def start(self):
        self.sock.bind((self.host, self.port))
        print(f"Server listening on {self.host}:{self.port}")

        while self.running:
            try:
                data, client_address = self.sock.recvfrom(1024)
                if client_address not in self.clients:
                    print(f"New client connected: {client_address}")
                    self.clients.add(client_address)
                print(f"Received data from {client_address}: {data.decode('utf-8')}")
            except Exception as e:
                print(f"Error receiving from client: {e}")

    def receive_data(self):
        try:
            data, client_address = self.sock.recvfrom(1024)  # Get both data and address
            return data.decode('utf-8'), client_address  # Return both values
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None, None  # Return None for both if there's an error

    def send_message(self, message):
        for client in self.clients:
            try:
                self.sock.sendto(bytes(message, "utf-8"), client)
            except Exception as e:
                print(f"Error broadcasting to client {client}: {e}")

    def stop(self):
        self.running = False
        self.sock.close()
        print("Server stopped.")