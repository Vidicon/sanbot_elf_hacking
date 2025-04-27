import socket

# Define the server's IP address and port number
HOST = '192.168.3.80'  # Replace with your server's IP address
# HOST = 'saradev.local'  # Replace with your server's IP address
PORT = 5000  # Port number 5000
# PORT = 5001  # Port number 5000

# Create a socket object
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    # Connect to the server
    s.connect((HOST, PORT))
    print(f"Connected to {HOST} on port {PORT}")
    
    while True:
        # Receive data from the server (1024 bytes at a time)
        data = s.recv(1024)
        
        # If data is received, print it
        if data:
            print("Received data:", data.hex())
        else:
            # No data means the server has closed the connection
            print("Connection closed by server")
            break

except Exception as e:
    print(f"Error: {e}")
finally:
    # Close the socket connection
    s.close()
