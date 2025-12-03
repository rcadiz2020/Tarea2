import socket

HOST = '192.168.0.136'   # Escucha en todas las interfaces
PORT = 1234        # Puerto UDP

def main():
    # Socket IPv4 + UDP
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        # Opcional: reutilizar puerto si reinicias rápido
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))

        print(f"Servidor UDP escuchando en {HOST}:{PORT}")
        try:
            while True:
                # Recibe hasta 2048 bytes y la dirección del remitente
                data, addr = s.recvfrom(2048)
                if not data:
                    continue

                msg = data.decode('utf-8', errors='replace')
                print(f"Recibido de {addr[0]}:{addr[1]} -> {msg}")

                # Responde al mismo remitente
                respuesta = f"tu mensaje es: {msg}"
                s.sendto(respuesta.encode('utf-8'), addr)
        except KeyboardInterrupt:
            print("\nCerrando servidor UDP...")

if __name__ == "__main__":
    main()
